"""More road nodes: every vehicle road link split into shorter ones on a smooth curve (GTA SA NODES0..63.DAT).

The engine addresses a road link ("navi") with 10 bits per file: at most 1024 per 750 x 750 m area, and the densest
area already has 985. A navi can be stored in ANY area's file (the address has the area too): links that don't fit in
their own area go to the nearest one with room. That's only safe with every area's road data always loaded (the mod's
LoadAllPathNodes). Total room: 64 x ~1000 navis, about twice what the map has.

New node positions come from a centripetal Catmull-Rom spline through the neighbouring nodes: the curves become round
instead of a few straight pieces. Traffic lights stay on the piece touching their junction (its navi point is then
right before the junction: a better stop line).

usage: python densify_paths.py <src folder> <dst folder> [--target 60000] [--min-seg 10] [--cap 1000]
"""
import argparse
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all  # noqa: E402

LIGHT_BITS = (1 << 14) | (3 << 16)


def area_of(x, y):
    ax = min(max(int((x + 3000.0) // 750.0), 0), 7)
    ay = min(max(int((y + 3000.0) // 750.0), 0), 7)
    return ay * 8 + ax


def catmull(p0, p1, p2, p3, t):
    """Centripetal Catmull-Rom between p1 and p2 (t in 0..1)"""
    def tj(ti, a, b):
        d = math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))
        return ti + max(d, 1e-3) ** 0.5
    t0 = 0.0
    t1 = tj(t0, p0, p1)
    t2 = tj(t1, p1, p2)
    t3 = tj(t2, p2, p3)
    tt = t1 + (t2 - t1) * t

    def lerp(a, b, ta, tb):
        w = (tt - ta) / (tb - ta)
        return [a[i] + (b[i] - a[i]) * w for i in range(3)]
    a1 = lerp(p0, p1, t0, t1)
    a2 = lerp(p1, p2, t1, t2)
    a3 = lerp(p2, p3, t2, t3)
    b1 = lerp(a1, a2, t0, t2)
    b2 = lerp(a2, a3, t1, t3)
    return lerp(b1, b2, t1, t2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--target', type=int, default=60000)
    ap.add_argument('--min-seg', type=float, default=10.0)
    ap.add_argument('--cap', type=int, default=1000)
    ap.add_argument('--split-junctions', action='store_true',
                    help='also split links touching a junction (8 m pieces there swung cars wide in turns)')
    args = ap.parse_args()

    areas = load_all(args.src)
    pos = {}
    nbrs = {}       # vehicle node -> list of (neighbour, navi address)
    for a, ar in areas.items():
        for n in ar.nodes[:ar.n_veh]:
            pos[(a, n.id)] = (n.x, n.y, n.z)
    for a, ar in areas.items():
        for n in ar.nodes[:ar.n_veh]:
            lst = []
            for k in range(n.links):
                li = n.link + k
                t = tuple(ar.links[li])
                nl = ar.navi_links[li]
                lst.append((t, (nl >> 10, nl & 0x3FF)))
            nbrs[(a, n.id)] = lst

    # Undirected vehicle edges (both ends vehicle nodes)
    edges = {}
    for u, lst in nbrs.items():
        for v, nav in lst:
            if v in pos and (v, u) not in edges and (u, v) not in edges:
                edges[(u, v)] = nav

    def length(u, v):
        return math.dist(pos[u][:2], pos[v][:2])

    def splits(L, seg):
        if L < 2 * args.min_seg:
            return 0
        return max(0, min(round(L / seg) - 1, int(L // args.min_seg) - 1))

    lo, hi = args.min_seg, 200.0
    for _ in range(40):
        seg = (lo + hi) / 2
        total = sum(1 + (splits(length(u, v), seg) if args.split_junctions or (len(nbrs[u]) <= 2 and len(nbrs[v]) <= 2) else 0)
                    for (u, v) in edges)
        if total > args.target:
            lo = seg
        else:
            hi = seg
    seg = hi
    print('vehicle nodes %d, vehicle links %d; segment %.1f m' % (len(pos), len(edges), seg))

    def other_neighbour(u, v):
        vs = [w for w, _ in nbrs[u] if w in pos]
        if len(vs) == 2:
            w = vs[0] if vs[1] == v else vs[1]
            if w != v:
                return pos[w]
        return tuple(2 * pos[u][i] - pos[v][i] for i in range(3))

    # New nodes per area (appended after the original vehicle nodes) and new navis
    new_nodes = {a: [] for a in areas}      # area -> list of dict(x, y, z, src node, links=[(target key)])
    navis = []                              # dict(x, y, node key, dx, dy, flags)
    link_navi = {}                          # (from key, to key) -> navi index in navis
    replace = {}                            # (u, v) original link slot -> first node key towards v

    def navi_of(addr):
        a, i = addr
        return areas[a].navi[i]

    def add_navi(p, q, xkey, orig, lit):
        """Navi for the piece p-q; 'xkey' the end of the piece towards the original navi's node"""
        other = q if xkey == p else p
        P, X = pos_all[other], pos_all[xkey]
        dx, dy = X[0] - P[0], X[1] - P[1]
        d = math.hypot(dx, dy) or 1.0
        ox, oy = orig.dx / 100.0, orig.dy / 100.0
        sign = 1.0 if (ox * (pos_orig_x[0] - pos_orig_o[0]) + oy * (pos_orig_x[1] - pos_orig_o[1])) >= 0 else -1.0
        flags = orig.flags if lit else (orig.flags & ~LIGHT_BITS)
        navis.append({'x': (P[0] + X[0]) / 2, 'y': (P[1] + X[1]) / 2, 'node': xkey,
                      'dx': max(-100, min(100, int(round(sign * dx / d * 100)))),
                      'dy': max(-100, min(100, int(round(sign * dy / d * 100)))), 'flags': flags})
        idx = len(navis) - 1
        link_navi[(p, q)] = idx
        link_navi[(q, p)] = idx

    global pos_orig_x, pos_orig_o
    pos_all = dict(pos)
    new_count = 0
    for (u, v), nav in edges.items():
        orig = navi_of(nav)
        L = length(u, v)
        k = splits(L, seg) if L > 0.5 else 0
        if not args.split_junctions and (len(nbrs[u]) > 2 or len(nbrs[v]) > 2):
            k = 0   # turns keep their geometry
        X = (orig.tarea, orig.tnode)
        if X not in (u, v):
            X = v
        Y = u if X == v else v
        pos_orig_x, pos_orig_o = pos[X], pos[Y]
        # Which junction the light belongs to: X if bit 14 set, else Y
        light_at = X if (orig.flags >> 14) & 1 else Y
        chain = [u]
        if k > 0:
            p0, p3 = other_neighbour(u, v), other_neighbour(v, u)
            src = u if len(nbrs[u]) == 2 else v
            for i in range(1, k + 1):
                x, y, z = catmull(p0, pos[u], pos[v], p3, i / (k + 1))
                a = area_of(x, y)
                key = (a, ('new', len(new_nodes[a])))
                new_nodes[a].append({'x': x, 'y': y, 'z': z, 'src': src, 'links': []})
                pos_all[key] = (x, y, z)
                chain.append(key)
                new_count += 1
        chain.append(v)
        for i in range(len(chain) - 1):
            p, q = chain[i], chain[i + 1]
            # The end of this piece towards X along the chain
            xkey = q if X == v else p
            touches = light_at in (p, q)
            add_navi(p, q, xkey, orig, touches and bool(orig.light))
            if i > 0:
                new_nodes[p[0]][p[1][1]]['links'].append(q)
            if i + 1 < len(chain) - 1:
                new_nodes[q[0]][q[1][1]]['links'].append(p)
        replace[(u, v)] = chain[1]
        replace[(v, u)] = chain[-2]
    print('new nodes %d -> vehicle nodes %d, vehicle links %d' % (new_count, len(pos) + new_count, len(navis)))

    # Navi storage: own area, else the nearest with room
    centre = {a: (-3000 + 750 * (a % 8) + 375, -3000 + 750 * (a // 8) + 375) for a in range(64)}
    store = {a: [] for a in range(64)}
    spills = 0
    for i, nv in enumerate(navis):
        a = area_of(nv['x'], nv['y'])
        if len(store[a]) >= args.cap:
            spills += 1
            a = min((b for b in range(64) if len(store[b]) < args.cap),
                    key=lambda b: math.dist(centre[b], (nv['x'], nv['y'])))
        nv['area'] = a
        nv['id'] = len(store[a])
        store[a].append(i)
    print('navis per file: max %d, spilled to another area %d' % (max(len(s) for s in store.values()), spills))

    # Final node ids: original vehicle nodes, new ones, then ped nodes (shifted)
    final = {}
    for a, ar in areas.items():
        for n in ar.nodes[:ar.n_veh]:
            final[(a, n.id)] = n.id
        for j in range(len(new_nodes[a])):
            final[(a, ('new', j))] = ar.n_veh + j
        shift = len(new_nodes[a])
        for n in ar.nodes[ar.n_veh:]:
            final[(a, n.id)] = n.id + shift

    def addr(key):
        return key[0], final[key]

    os.makedirs(args.dst, exist_ok=True)
    for a, ar in areas.items():
        d = ar.data
        nodes_out, links_out, navil_out, len_out, inter_out = [], [], [], [], []

        def emit_links(node_key, targets, orig_slots):
            first = len(links_out)
            for j, t in enumerate(targets):
                ta, tid = addr(t)
                links_out.append(struct.pack('<HH', ta, tid))
                if t not in pos_all or (node_key, t) not in link_navi:
                    navil_out.append(struct.pack('<H', orig_slots[j][1] if orig_slots else 0))  # not a road link: original
                else:
                    nv = navis[link_navi[(node_key, t)]]
                    navil_out.append(struct.pack('<H', (nv['area'] << 10) | nv['id']))
                if t in pos_all and node_key in pos_all:
                    L = math.dist(pos_all[node_key][:2], pos_all[t][:2])
                    len_out.append(max(1, min(255, int(round(L)))))
                else:
                    len_out.append(orig_slots[j][2])
                inter_out.append(orig_slots[j][3] if orig_slots is not None else 0)
            return first

        def node_bytes(x, y, z, heur, first, nid, width, flood, flags):
            return struct.pack('<IIhhhhHHHBBI', 0, 0, int(round(x * 8)), int(round(y * 8)), int(round(z * 8)), heur,
                               first, a, nid, width, flood, flags)

        def orig_slot_info(n):
            out = []
            for k in range(n.links):
                li = n.link + k
                out.append((tuple(ar.links[li]), ar.navi_links[li], ar.lengths[li], ar.inter[li] if ar.inter else 0))
            return out

        raw = lambda n: struct.unpack_from('<IIhhhhHHHBBI', d, n.raw)  # noqa: E731
        # Original vehicle nodes
        for n in ar.nodes[:ar.n_veh]:
            key = (a, n.id)
            slots = orig_slot_info(n)
            targets = [replace.get((key, t), t) if t in pos else t for t, _, _, _ in slots]
            first = emit_links(key, targets, slots)
            r = raw(n)
            nodes_out.append(node_bytes(n.x, n.y, n.z, r[5], first, final[key], r[9], r[10], r[11]))
        # New vehicle nodes
        for j, nn in enumerate(new_nodes[a]):
            key = (a, ('new', j))
            first = emit_links(key, nn['links'], None)
            sa, sid = nn['src']
            sr = struct.unpack_from('<IIhhhhHHHBBI', areas[sa].data, areas[sa].nodes[sid].raw)
            flags = (sr[11] & ~0xF) | 2
            nodes_out.append(node_bytes(nn['x'], nn['y'], nn['z'], sr[5], first, final[key], sr[9], sr[10], flags))
        # Ped nodes (targets remapped, everything else original)
        for n in ar.nodes[ar.n_veh:]:
            key = (a, n.id)
            slots = orig_slot_info(n)
            first = len(links_out)
            for (t, nl, ln, it) in slots:
                ta, tid = t[0], final.get(t, t[1])
                links_out.append(struct.pack('<HH', ta, tid))
                navil_out.append(struct.pack('<H', nl))
                len_out.append(ln)
                inter_out.append(it)
            r = raw(n)
            nodes_out.append(node_bytes(n.x, n.y, n.z, r[5], first, final[key], r[9], r[10], r[11]))

        navi_out = []
        for i in store[a]:
            nv = navis[i]
            na, nid = addr(nv['node'])
            navi_out.append(struct.pack('<hhHHbbI', int(round(nv['x'] * 8)), int(round(nv['y'] * 8)), na, nid,
                                        nv['dx'], nv['dy'], nv['flags']))
        n_veh = ar.n_veh + len(new_nodes[a])
        n_links = len(links_out)
        filler_off = 20 + 28 * ar.n_nodes + 14 * ar.n_navi + 4 * ar.n_links
        filler = d[filler_off:filler_off + 768]
        tail = d[-384:]
        out = bytearray(struct.pack('<5I', n_veh + ar.n_ped, n_veh, ar.n_ped, len(navi_out), n_links))
        out += b''.join(nodes_out) + b''.join(navi_out) + b''.join(links_out) + filler + b''.join(navil_out)
        out += bytes(len_out) + bytes(inter_out) + tail
        open(os.path.join(args.dst, 'NODES%d.DAT' % a), 'wb').write(bytes(out))
    print('files written: %d -> %s' % (len(areas), args.dst))


if __name__ == '__main__':
    main()
