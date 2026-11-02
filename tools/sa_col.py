"""GTA SA map collision (COL1/COL2/COL3 inside models/gta3.img) placed in the world by the IPL instances: triangles with
their surface material, to measure the real road surface without driving (road_width.py).

Material ids are the lines of data/surfinfo.dat: 1-3 TARMAC*, 4-5 PAVEMENT*, 6 GRAVEL, 7 FUCKED_CONCRETE, 8 PAINTED_GROUND...
"""
import glob
import math
import os
import struct

import numpy as np

from sa_ipl import img_entries, all_insts


def _models_in_col(data):
    """Yield (name, model id, version, body bytes, body offset base) for each model of a .col file"""
    o = 0
    while o + 8 <= len(data):
        four = data[o:o + 4]
        if four not in (b'COLL', b'COL2', b'COL3', b'COL4'):
            break
        size = struct.unpack_from('<I', data, o + 4)[0]
        body = data[o + 8:o + 8 + size]
        name = body[:22].split(b'\0')[0].decode('latin-1').lower()
        mid = struct.unpack_from('<H', body, 22)[0]
        yield name, mid, four, body
        o += 8 + size


def _parse_body(four, body):
    """(vertices N x 3 float, faces M x 3 int, materials M, boxes [(min, max, material)])"""
    if four == b'COLL':
        o = 24 + 40
        ns = struct.unpack_from('<I', body, o)[0]; o += 4 + ns * 20
        o += 4                                              # unknown, always 0
        nb = struct.unpack_from('<I', body, o)[0]; o += 4
        boxes = []
        for i in range(nb):
            v = struct.unpack_from('<6f4B', body, o + i * 28)
            boxes.append((v[0:3], v[3:6], v[6]))
        o += nb * 28
        nv = struct.unpack_from('<I', body, o)[0]; o += 4
        verts = np.frombuffer(body, dtype='<f4', count=nv * 3, offset=o).reshape(-1, 3).astype(np.float32); o += nv * 12
        nf = struct.unpack_from('<I', body, o)[0]; o += 4
        raw = np.frombuffer(body, dtype='<u4', count=nf * 4, offset=o).reshape(-1, 4)
        return verts, raw[:, :3].astype(np.int64), (raw[:, 3] & 0xFF).astype(np.uint8), boxes
    # COL2/3/4: counts, flags, offsets (from the start of the body minus 4, i.e. FourCC + 4)
    h = 24 + 40
    ns, nb, nf, nl = struct.unpack_from('<HHHB', body, h)
    off_s, off_b, off_l, off_v, off_f = struct.unpack_from('<5I', body, h + 12)
    base = -4
    boxes = []
    if nb and off_b:
        for i in range(nb):
            v = struct.unpack_from('<6f4B', body, base + off_b + i * 28)
            boxes.append((v[0:3], v[3:6], v[6]))
    if not nf or not off_f or not off_v:
        return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.int64), np.zeros(0, np.uint8), boxes
    faces = np.frombuffer(body, dtype=np.uint8, count=nf * 8, offset=base + off_f).reshape(-1, 8)
    idx = faces[:, :6].copy().view('<u2').reshape(-1, 3).astype(np.int64)
    mats = faces[:, 6].copy()
    nv = int(idx.max()) + 1
    verts = np.frombuffer(body, dtype='<i2', count=nv * 3, offset=base + off_v).reshape(-1, 3).astype(np.float32) / 128.0
    return verts, idx, mats, boxes


def load_ide_names(game_dir):
    """model id -> lower-case name, from every IDE under data/maps"""
    names = {}
    for path in glob.glob(os.path.join(game_dir, 'data', 'maps', '**', '*.ide'), recursive=True):
        section = None
        for line in open(path, encoding='latin-1'):
            s = line.split('#')[0].strip()
            if not s:
                continue
            low = s.lower()
            if low in ('objs', 'tobj', 'anim', 'path', '2dfx', 'txdp', 'cars', 'peds', 'weap', 'hier'):
                section = low
                continue
            if low == 'end':
                section = None
                continue
            if section in ('objs', 'tobj', 'anim'):
                p = [t.strip() for t in s.split(',')]
                if len(p) >= 2 and p[0].lstrip('-').isdigit():
                    names[int(p[0])] = p[1].lower()
    return names


def quat_matrix(qx, qy, qz, qw, conjugate):
    if conjugate:
        qx, qy, qz = -qx, -qy, -qz
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]], dtype=np.float32)


def load_world_triangles(game_dir, keep=None, conjugate=True, img_name='models/gta3.img'):
    """World triangles (N x 3 x 3 float32) and materials (N) of every map instance; 'keep(x, y, radius)' filters instances.
    Boxes become two triangles for their top face."""
    names = load_ide_names(game_dir)
    cols = {}
    img = os.path.join(game_dir, img_name)
    with open(img, 'rb') as f:
        for name, off, size in img_entries(img):
            if not name.lower().endswith('.col'):
                continue
            f.seek(off)
            data = f.read(size)
            for mname, mid, four, body in _models_in_col(data):
                if mname not in cols:
                    try:
                        cols[mname] = _parse_body(four, body)
                    except (struct.error, ValueError):
                        pass
    tris, mats = [], []
    used = 0
    for inst in all_insts(game_dir, img_name):
        mid, mname, x, y, z, qx, qy, qz, qw, interior = inst[:10]
        if interior not in (0, 13):
            continue
        name = (mname or names.get(mid, '')).lower()
        col = cols.get(name)
        if col is None:
            continue
        verts, idx, mm, boxes = col
        if keep is not None:
            r = float(np.abs(verts).max()) if len(verts) else 10.0
            if not keep(x, y, r):
                continue
        R = quat_matrix(qx, qy, qz, qw, conjugate)
        t = np.array([x, y, z], np.float32)
        if len(idx):
            w = verts @ R.T + t
            tris.append(w[idx])
            mats.append(mm)
        for (mn, mx, m) in boxes:
            c = np.array([[mn[0], mn[1], mx[2]], [mx[0], mn[1], mx[2]], [mx[0], mx[1], mx[2]], [mn[0], mx[1], mx[2]]], np.float32)
            w = c @ R.T + t
            tris.append(np.stack([w[[0, 1, 2]], w[[0, 2, 3]]]))
            mats.append(np.array([m, m], np.uint8))
        used += 1
    if not tris:
        return np.zeros((0, 3, 3), np.float32), np.zeros(0, np.uint8), used, len(cols)
    return np.concatenate(tris), np.concatenate(mats), used, len(cols)


class TriGrid:
    """Triangles binned by 2D cell for vertical ray queries"""

    def __init__(self, tris, mats, cell=8.0):
        self.tris, self.mats, self.cell = tris, mats, cell
        mn = tris[:, :, :2].min(axis=1)
        mx = tris[:, :, :2].max(axis=1)
        self.cells = {}
        x0 = np.floor(mn[:, 0] / cell).astype(np.int64)
        y0 = np.floor(mn[:, 1] / cell).astype(np.int64)
        x1 = np.floor(mx[:, 0] / cell).astype(np.int64)
        y1 = np.floor(mx[:, 1] / cell).astype(np.int64)
        for i in range(len(tris)):
            if (x1[i] - x0[i]) * (y1[i] - y0[i]) > 400:
                continue                                    # huge ground triangles are rare; skip pathological ones
            for cx in range(x0[i], x1[i] + 1):
                for cy in range(y0[i], y1[i] + 1):
                    self.cells.setdefault((cx, cy), []).append(i)

    def surface(self, pts, zref, below=4.0, above=2.5):
        """For points (K x 2): the material and height of the highest triangle within [zref-below, zref+above]
        (-1 where none)"""
        cs = {(int(math.floor(p[0] / self.cell)), int(math.floor(p[1] / self.cell))) for p in pts}
        ids = sorted({i for c in cs for i in self.cells.get(c, ())})
        mat = np.full(len(pts), -1, np.int16)
        hz = np.full(len(pts), -1e9, np.float32)
        if not ids:
            return mat, hz
        T = self.tris[ids]                                  # K' x 3 x 3
        a, b, c = T[:, 0], T[:, 1], T[:, 2]
        v0 = (b - a)[:, :2]
        v1 = (c - a)[:, :2]
        den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
        ok = np.abs(den) > 1e-6
        P = np.asarray(pts, np.float32)
        d = P[:, None, :] - a[None, :, :2]                  # K x K' x 2
        with np.errstate(divide='ignore', invalid='ignore'):
            u = (d[..., 0] * v1[None, :, 1] - v1[None, :, 0] * d[..., 1]) / den[None]
            v = (v0[None, :, 0] * d[..., 1] - d[..., 0] * v0[None, :, 1]) / den[None]
        inside = ok[None] & (u >= -1e-4) & (v >= -1e-4) & (u + v <= 1 + 1e-4)
        z = a[None, :, 2] + u * (b - a)[None, :, 2] + v * (c - a)[None, :, 2]
        z = np.where(inside & (z > zref - below) & (z < zref + above), z, -1e9)
        best = z.argmax(axis=1)
        bz = z[np.arange(len(P)), best]
        hit = bz > -1e8
        mat[hit] = self.mats[np.array(ids)[best[hit]]]
        hz[hit] = bz[hit]
        return mat, hz
