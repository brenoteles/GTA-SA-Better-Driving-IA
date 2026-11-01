"""Road classes and lanes from the network's own shape (GTA SA NODES0..63.DAT).

A road stretch = the nodes between two junctions (or dead ends). Each node has a speed class in flag bits 12-13 (0 slow,
1 normal, 2 freeway: the game and the mod drive, route and populate by it):
  - normal -> freeway: stretches 400 m or longer with 2+ lanes in each direction they're driven, no traffic light
  - freeway -> normal: stretches under 150 m inside the city with a traffic light at either end
  - slow -> normal: stretches with 2+ lanes in a direction
Links without any lane get the lanes of the link next to them on the same road.

usage: python reclassify_paths.py <src folder> <dst folder>
"""
import math
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402

SPEED_SHIFT = 12


def speed_of(n):
    return (n.flags >> SPEED_SHIFT) & 3


def set_speed(area, n, sp):
    import struct
    n.flags = (n.flags & ~(3 << SPEED_SHIFT)) | (sp << SPEED_SHIFT)
    b = bytearray(area.data)
    struct.pack_into('<I', b, n.raw + 24, n.flags)
    area.data = bytes(b)


def main(src, dst):
    areas = load_all(src)
    net = Network(areas)
    node = net.node
    stats = Counter()
    changed = set()

    def legs(k):
        return [(nb, nv) for nb, nv in net.adj[k] if nb in node and nv]

    # Links without lanes: from a neighbouring link through a 2-link node
    for k in node:
        for nb, nv in legs(k):
            if nv.left or nv.right:
                continue
            for k2 in (k, nb):
                ls = legs(k2)
                if len(ls) != 2:
                    continue
                other = [(b, v) for b, v in ls if v is not nv]
                if other and (other[0][1].left or other[0][1].right):
                    ov = other[0][1]
                    nv.flags = (nv.flags & ~(0x3F << 8)) | (ov.flags & (0x3F << 8))
                    changed.add(nv.area_index)
                    stats['links without lanes given their neighbour\'s'] += 1
                    break

    # Road stretches between junctions
    seen = set()
    for start in node:
        if len(legs(start)) == 2:
            continue
        for first, _ in legs(start):
            if (start, first) in seen:
                continue
            chain, prev, cur = [start], start, first
            length = 0.0
            while True:
                length += math.dist((node[prev].x, node[prev].y), (node[cur].x, node[cur].y))
                chain.append(cur)
                seen.add((prev, cur))
                ls = legs(cur)
                if len(ls) != 2 or len(chain) > 2000:
                    break
                nxt = ls[0][0] if ls[1][0] == prev else ls[1][0]
                prev, cur = cur, nxt
            seen.add((chain[-1], chain[-2]))
            inner = chain[1:-1]
            if not inner:
                continue
            links = []
            for a, b in zip(chain, chain[1:]):
                nv = next((v for x, v in legs(a) if x == b), None)
                if nv:
                    links.append((a, b, nv))
            if not links:
                continue
            lanes_ab = min(net.lanes_toward(a, b, nv) for a, b, nv in links)
            lanes_ba = min(net.lanes_toward(b, a, nv) for a, b, nv in links)
            driven = [l for l in (lanes_ab, lanes_ba) if l > 0]
            lit = any(nv.light for _, _, nv in links)
            speeds = Counter(speed_of(node[k]) for k in inner)
            cls = speeds.most_common(1)[0][0]
            new = cls
            if cls == 1 and length >= 400 and driven and min(driven) >= 2 and not lit:
                new = 2
            elif cls == 2 and length < 150 and lit:
                new = 1
            elif cls == 0 and driven and max(driven) >= 2:
                new = 1
            if new != cls:
                stats['stretches %d -> %d' % (cls, new)] += 1
                for k in inner:
                    if speed_of(node[k]) != new:
                        set_speed(areas[k[0]], node[k], new)
                        changed.add(k[0])
                        stats['nodes reclassified'] += 1

    os.makedirs(dst, exist_ok=True)
    for a in sorted(changed):
        areas[a].write(os.path.join(dst, 'NODES%d.DAT' % a))   # navi flags written over the (node-updated) data
    for name, c in sorted(stats.items()):
        print('  %-50s %d' % (name, c))
    print('files written: %d -> %s' % (len(changed), dst))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
