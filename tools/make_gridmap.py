"""The proving grid: a new, empty, flat map area (after BeamNG's 'Gridmap Small Pure') for the vehicle physics tests.

A platform of 100 m tiles high above the sea south of Los Santos (nothing of the map reaches that height), textured with a
10 m grid (a heavier line every 50 m), tarmac collision. Written from scratch as a modloader mod: the tile's model (.dff),
texture (.txd), collision (.col), its object definition (.ide) and the tiles' placement (.ipl).

python tools/make_gridmap.py [game dir]      -> <game>/modloader/RealTrafficFix ProvingGrid/
"""
import math
import os
import struct
import sys

import numpy as np

MODEL_ID = 19400                 # free in the game's and the installed mods' .ide files (18631-19999 unused)
NAME = 'pg_grid'
TILE = 100.0                     # m
X0, X1 = -100.0, 2900.0          # the platform (world): 3 km east-west
Y0, Y1 = -2950.0, -1750.0        # 1.2 km north-south (10 lanes of the GA circuit)
Z = 300.0                        # its surface
DECKS = (300.0, 380.0, 460.0, 540.0, 620.0, 700.0)  # the platform 6 times, 80 m apart (60 GA cars: 10 lanes on each)
RW = 0x1803FFFF                  # RenderWare 3.6.0.3 (San Andreas)
TEX = 512                        # texture size: 50 m of ground (5 cells of 10 m)
TEX_M = 50.0
SURFACE_TARMAC = 1
# Obstacles the proving ground puts on its road (ProvingGround.h, track piece O): a bump for one side of the car (1 m
# wide, 3 m long, 0.18 m high, a sine) and a kicker across the lane (4 m wide, a 7 m ramp up to 0.45 m and a drop:
# the car hops). Origin: the middle of their base; length along y (the car's forward)
OBSTACLES = ((19401, 'pg_bump'), (19402, 'pg_kicker'))
OBST_TXD = 'pg_obst'


def chunk(kind, data):
    return struct.pack('<III', kind, len(data), RW) + data


def rw_string(s):
    b = s.encode('ascii') + b'\0'
    b += b'\0' * (-len(b) % 4)
    return chunk(0x02, b)


# --- Texture -------------------------------------------------------------------------------------------------------------

def grid_texture():
    rng = np.random.default_rng(1)
    img = np.full((TEX, TEX, 3), 168.0)
    img += rng.normal(0, 4, (TEX, TEX, 1))                        # a little grain
    px = TEX / (TEX_M / 10.0)                                     # pixels per 10 m
    for i in range(int(TEX_M / 10.0) + 1):
        c = int(round(i * px)) % TEX
        heavy = i == 0
        w = 6 if heavy else 3
        col = (40, 40, 40) if heavy else (85, 85, 85)
        for k in range(-w // 2, w - w // 2):
            img[(c + k) % TEX, :, :] = col
            img[:, (c + k) % TEX, :] = col
    return np.clip(img, 0, 255).astype(np.uint8)


def txd(img, name=NAME):
    levels = []
    cur = img.astype(np.float32)
    while True:
        h, w = cur.shape[:2]
        bgra = np.concatenate([cur[..., ::-1], np.full((h, w, 1), 255.0)], axis=2).astype(np.uint8)
        levels.append(bgra.tobytes())
        if w == 1:
            break
        cur = cur.reshape(h // 2, 2, w // 2, 2, 3).mean(axis=(1, 3))
    size = img.shape[0]
    name = name.encode().ljust(32, b'\0')
    data = struct.pack('<II', 9, 0x1106)                          # D3D9; linear mip-mapped filter, wrap U/V
    data += name + b'\0' * 32
    data += struct.pack('<II', 0x0600 | 0x8000, 22)               # 888 with mip levels; D3DFMT_X8R8G8B8
    data += struct.pack('<HHBBBB', size, size, 32, len(levels), 4, 0)
    for lv in levels:
        data += struct.pack('<I', len(lv)) + lv
    native = chunk(0x15, chunk(0x01, data) + chunk(0x03, b''))
    return chunk(0x16, chunk(0x01, struct.pack('<HH', 1, 0)) + native + chunk(0x03, b''))


# --- Model ---------------------------------------------------------------------------------------------------------------

def dff():
    n = 11                                                        # 11 x 11 vertices: 10 m quads
    verts, uvs = [], []
    for j in range(n):
        for i in range(n):
            x = -TILE / 2 + TILE * i / (n - 1)
            y = -TILE / 2 + TILE * j / (n - 1)
            verts.append((x, y, 0.0))
            uvs.append((x / TEX_M, y / TEX_M))                    # tiles start on a 50 m line: seamless
    tris = []
    for j in range(n - 1):
        for i in range(n - 1):
            a, b, c, d = j * n + i, j * n + i + 1, (j + 1) * n + i + 1, (j + 1) * n + i
            tris += [(a, b, c), (a, c, d)]                        # counter-clockwise seen from above
    nv, nt = len(verts), len(tris)
    flags = 0x02 | 0x04 | 0x08 | 0x10 | 0x20 | 0x40 | (1 << 16)  # positions, textured, prelit, normals, light, modulate; 1 uv set
    g = struct.pack('<Iiii', flags, nt, nv, 1)
    g += b''.join(struct.pack('<4B', 255, 255, 255, 255) for _ in verts)
    g += b''.join(struct.pack('<ff', u, v) for u, v in uvs)
    for a, b, c in tris:
        g += struct.pack('<HHHH', b, a, 0, c)                     # v1, v0, material, v2
    g += struct.pack('<ffff', 0.0, 0.0, 0.0, TILE * 0.71)        # bounding sphere
    g += struct.pack('<ii', 1, 1)
    g += b''.join(struct.pack('<fff', *v) for v in verts)
    g += b''.join(struct.pack('<fff', 0.0, 0.0, 1.0) for _ in verts)
    tex = chunk(0x06, chunk(0x01, struct.pack('<HH', 0x1106, 1)) + rw_string(NAME) + rw_string('') + chunk(0x03, b''))
    mat = chunk(0x07, chunk(0x01, struct.pack('<i4Biifff', 0, 255, 255, 255, 255, 0, 1, 1.0, 1.0, 1.0)) + tex + chunk(0x03, b''))
    matlist = chunk(0x08, chunk(0x01, struct.pack('<ii', 1, -1)) + mat)
    # Bin Mesh PLG: one triangle list
    idx = [i for t in tris for i in t]
    binmesh = chunk(0x50E, struct.pack('<III', 0, 1, len(idx)) + struct.pack('<II', len(idx), 0) +
                    struct.pack(f'<{len(idx)}I', *idx))
    geom = chunk(0x0F, chunk(0x01, g) + matlist + chunk(0x03, binmesh))
    geomlist = chunk(0x1A, chunk(0x01, struct.pack('<i', 1)) + geom)
    frame = struct.pack('<9f3fii', 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, -1, 0)
    framename = chunk(0x0253F2FE, NAME.encode())                  # the game finds the model by its frame's name
    framelist = chunk(0x0E, chunk(0x01, struct.pack('<i', 1) + frame) + chunk(0x03, framename))
    atomic = chunk(0x14, chunk(0x01, struct.pack('<iiii', 0, 0, 5, 0)) + chunk(0x03, b''))
    return chunk(0x10, chunk(0x01, struct.pack('<iii', 1, 0, 0)) + framelist + geomlist + atomic + chunk(0x03, b''))


def bump_mesh():
    w, length, h, n = 1.0, 3.0, 0.18, 12
    ys = [-length / 2 + length * i / n for i in range(n + 1)]
    zs = [h * math.sin(math.pi * i / n) for i in range(n + 1)]
    return box_profile(w, ys, zs)


def kicker_mesh():
    w, length, h = 4.0, 7.0, 0.45
    ys = [-length / 2 + length * i / 8 for i in range(9)]
    zs = [h * i / 8 for i in range(9)]                            # up all the way, then the drop at the back
    return box_profile(w, ys, zs)


def box_profile(w, ys, zs):
    """A solid of width w with the top following zs along ys (base at 0): top, both sides, the back; double-sided
    walls (seen and hit from either side)."""
    verts, tris = [], []

    def quad(a, b, c, d, both=False):
        i = len(verts)
        verts.extend([a, b, c, d])
        tris.extend([(i, i + 1, i + 2), (i, i + 2, i + 3)])
        if both:
            tris.extend([(i, i + 2, i + 1), (i, i + 3, i + 2)])
    x0, x1 = -w / 2, w / 2
    for k in range(len(ys) - 1):
        y0, y1, z0, z1 = ys[k], ys[k + 1], zs[k], zs[k + 1]
        quad((x0, y0, z0), (x1, y0, z0), (x1, y1, z1), (x0, y1, z1))           # top, counter-clockwise from above
        quad((x1, y0, 0.0), (x1, y1, 0.0), (x1, y1, z1), (x1, y0, z0), True)  # sides
        quad((x0, y1, 0.0), (x0, y0, 0.0), (x0, y0, z0), (x0, y1, z1), True)
    y, z = ys[-1], zs[-1]
    if z > 0.01:
        quad((x1, y, 0.0), (x0, y, 0.0), (x0, y, z), (x1, y, z), True)        # the back (the kicker's drop)
    return verts, tris


def mesh_dff(name, tex, verts, tris):
    nv, nt = len(verts), len(tris)
    flags = 0x02 | 0x04 | 0x08 | 0x10 | 0x20 | 0x40 | (1 << 16)
    g = struct.pack('<Iiii', flags, nt, nv, 1)
    g += b''.join(struct.pack('<4B', 255, 255, 255, 255) for _ in verts)
    g += b''.join(struct.pack('<ff', x * 0.5 + y * 0.5, y * 0.5) for x, y, z in verts)   # diagonal stripes
    for a, b, c in tris:
        g += struct.pack('<HHHH', b, a, 0, c)
    r = max(math.sqrt(x * x + y * y + z * z) for x, y, z in verts)
    g += struct.pack('<ffff', 0.0, 0.0, 0.0, r)
    g += struct.pack('<ii', 1, 1)
    g += b''.join(struct.pack('<fff', *v) for v in verts)
    g += b''.join(struct.pack('<fff', 0.0, 0.0, 1.0) for _ in verts)
    texc = chunk(0x06, chunk(0x01, struct.pack('<HH', 0x1106, 1)) + rw_string(tex) + rw_string('') + chunk(0x03, b''))
    mat = chunk(0x07, chunk(0x01, struct.pack('<i4Biifff', 0, 255, 255, 255, 255, 0, 1, 1.0, 1.0, 1.0)) + texc + chunk(0x03, b''))
    matlist = chunk(0x08, chunk(0x01, struct.pack('<ii', 1, -1)) + mat)
    idx = [i for t in tris for i in t]
    binmesh = chunk(0x50E, struct.pack('<III', 0, 1, len(idx)) + struct.pack('<II', len(idx), 0) + struct.pack(f'<{len(idx)}I', *idx))
    geom = chunk(0x0F, chunk(0x01, g) + matlist + chunk(0x03, binmesh))
    geomlist = chunk(0x1A, chunk(0x01, struct.pack('<i', 1)) + geom)
    frame = struct.pack('<9f3fii', 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, -1, 0)
    framelist = chunk(0x0E, chunk(0x01, struct.pack('<i', 1) + frame) + chunk(0x03, chunk(0x0253F2FE, name.encode())))
    atomic = chunk(0x14, chunk(0x01, struct.pack('<iiii', 0, 0, 5, 0)) + chunk(0x03, b''))
    return chunk(0x10, chunk(0x01, struct.pack('<iii', 1, 0, 0)) + framelist + geomlist + atomic + chunk(0x03, b''))


def mesh_col(name, model_id, verts, tris):
    xs, ys, zs = zip(*verts)
    lo, hi = (min(xs), min(ys), min(zs) - 0.1), (max(xs), max(ys), max(zs) + 0.1)
    r = max(math.sqrt(x * x + y * y + z * z) for x, y, z in verts)
    body = name.encode().ljust(22, b'\0') + struct.pack('<H', model_id)
    body += struct.pack('<f3f3f3f', r, 0, 0, 0, *lo, *hi)
    body += struct.pack('<I', 0) + struct.pack('<I', 0) + struct.pack('<I', 0)
    body += struct.pack('<I', len(verts)) + b''.join(struct.pack('<3f', *v) for v in verts)
    body += struct.pack('<I', len(tris))
    for a, b, c in tris:
        body += struct.pack('<3I4B', a, b, c, SURFACE_TARMAC, 0, 255, 0)
    return b'COLL' + struct.pack('<I', len(body)) + body


def stripes_texture():
    n = 64
    img = np.zeros((n, n, 3))
    for i in range(n):
        for j in range(n):
            img[i, j] = (240, 200, 20) if ((i + j) // 16) % 2 == 0 else (25, 25, 25)
    return img.astype(np.uint8)


def col():
    h = TILE / 2 + 0.5                                            # overlapping the neighbours: no seam for a wheel's ray
    body = NAME.encode().ljust(22, b'\0') + struct.pack('<H', MODEL_ID)
    body += struct.pack('<f3f3f3f', math.hypot(h, h), 0, 0, 0, -h, -h, -1.0, h, h, 1.0)
    body += struct.pack('<I', 0)                                  # spheres
    body += struct.pack('<I', 0)                                  # (unused)
    body += struct.pack('<I', 0)                                  # boxes
    vs = [(-h, -h, 0.0), (h, -h, 0.0), (h, h, 0.0), (-h, h, 0.0)]
    body += struct.pack('<I', len(vs)) + b''.join(struct.pack('<3f', *v) for v in vs)
    faces = [(0, 1, 2), (0, 2, 3)]
    body += struct.pack('<I', len(faces))
    for a, b, c in faces:
        body += struct.pack('<3I4B', a, b, c, SURFACE_TARMAC, 0, 255, 0)
    return b'COLL' + struct.pack('<I', len(body)) + body


def main(game_dir):
    out = os.path.join(game_dir, 'modloader', 'RealTrafficFix ProvingGrid')
    stream = os.path.join(out, 'Stream')
    os.makedirs(stream, exist_ok=True)
    open(os.path.join(stream, NAME + '.dff'), 'wb').write(dff())
    open(os.path.join(stream, NAME + '.txd'), 'wb').write(txd(grid_texture()))
    # (the obstacles' collision in the platform's file: loaded wherever the platform is, as the obstacles need)
    cols = col()
    for (mid, name), mesh in zip(OBSTACLES, (bump_mesh(), kicker_mesh())):
        verts, tris = mesh
        open(os.path.join(stream, name + '.dff'), 'wb').write(mesh_dff(name, OBST_TXD, verts, tris))
        cols += mesh_col(name, mid, verts, tris)
    open(os.path.join(stream, OBST_TXD + '.txd'), 'wb').write(txd(stripes_texture(), OBST_TXD))
    open(os.path.join(stream, NAME + '.col'), 'wb').write(cols)
    with open(os.path.join(out, 'ProvingGrid.ide'), 'w', newline='\r\n') as f:
        f.write(f'objs\n{MODEL_ID}, {NAME}, {NAME}, 299, 0\n')
        for mid, name in OBSTACLES:
            f.write(f'{mid}, {name}, {OBST_TXD}, 299, 0\n')
        f.write('end\n')
    # modloader adds new .ide/.ipl files to gta.dat only through gta.dat lines in a text file of the mod
    with open(os.path.join(out, 'Loader.txt'), 'w', newline='\r\n') as f:
        f.write('IDE DATA\\MAPS\\ProvingGrid.ide\nIPL DATA\\MAPS\\ProvingGrid.ipl\n')
    n = 0
    with open(os.path.join(out, 'ProvingGrid.ipl'), 'w', newline='\r\n') as f:
        f.write('inst\n')
        for z in DECKS:
            y = Y0 + TILE / 2
            while y < Y1:
                x = X0 + TILE / 2
                while x < X1:
                    f.write(f'{MODEL_ID}, {NAME}, 0, {x:.1f}, {y:.1f}, {z:.1f}, 0, 0, 0, 1, -1\n')
                    n += 1
                    x += TILE
                y += TILE
        f.write('end\n')
    print(f'{out}: {n} tiles of {TILE:.0f} m, platform x {X0:.0f}..{X1:.0f}, y {Y0:.0f}..{Y1:.0f}, z {DECKS}')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.environ.get('GTA_SA_DIR', r'C:\Users\death\Games\GTA\GTA San Andreas'))
