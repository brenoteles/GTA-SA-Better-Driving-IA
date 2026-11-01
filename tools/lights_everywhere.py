"""Traffic lights at every urban junction (GTA SA path files), coherent cycles.

Every junction (3+ roads) off the freeways with traffic arriving from 2+ roads gets a light on each road arriving at it:
the roads along the junction's main axis (its most opposite pair of roads) on one cycle, the crossing ones on the other.
Lights the map already has keep their cycle (the visible traffic light poles follow them). A light applies to the
traffic arriving at the junction (the navi's light direction bit); a road between two junctions has one light only:
if it's already the other junction's, it's kept.

usage: python lights_everywhere.py <data/paths folder> <output folder (written: changed NODESn.DAT only)>
"""
import math
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402

LIGHT_MASK = 3 << 16
DIR_BIT = 1 << 14


def leg_angle(n, m):
    return math.degrees(math.atan2(m.y - n.y, m.x - n.x)) % 360.0


def axis_diff(a, b):
    """Difference of two directions as axes (0..90): opposite directions are the same axis"""
    d = abs(a - b) % 180.0
    return min(d, 180.0 - d)


def main(src, dst):
    areas = load_all(src)
    net = Network(areas)
    node = net.node
    stats = Counter()

    # Which cycle the map gives a north-south road (to number new junctions like the existing ones)
    ns_cycle = Counter()
    for k, lst in net.adj.items():
        if k not in node or len(lst) < 3:
            continue
        for nb, nv in lst:
            if nv and nv.light and nb in node:
                a = leg_angle(node[k], node[nb])
                ns = axis_diff(a, 90.0) < 45.0
                ns_cycle[(ns, nv.light)] += 1
    ns_is_1 = ns_cycle[(True, 1)] + ns_cycle[(False, 2)] >= ns_cycle[(True, 2)] + ns_cycle[(False, 1)]
    print('existing lights: north-south roads on cycle %d (%s)' % (1 if ns_is_1 else 2, dict(ns_cycle)))

    changed_areas = set()
    for k, lst in net.adj.items():
        if k not in node or len(lst) < 3:
            continue
        n = node[k]
        legs = [(nb, nv) for nb, nv in lst if nv and nb in node]
        if len(legs) < 3:
            continue
        # Off the freeways: no lights on freeway merges/splits and ramps meeting them
        if n.highway or any(node[nb].highway for nb, _ in legs):
            stats['skipped: freeway'] += 1
            continue
        arriving = [(nb, nv) for nb, nv in legs if net.lanes_toward(nb, k, nv) > 0]
        if len(arriving) < 2:
            stats['skipped: traffic arrives from one road only'] += 1
            continue
        stats['junctions'] += 1
        angles = {nb: leg_angle(n, node[nb]) for nb, _ in legs}
        # Main axis: the most opposite pair of roads
        best, axis = -1.0, angles[legs[0][0]]
        for i in range(len(legs)):
            for j in range(i + 1, len(legs)):
                d = abs((angles[legs[i][0]] - angles[legs[j][0]] + 180.0) % 360.0 - 180.0)
                if d > best:
                    best, axis = d, angles[legs[i][0]]
        group = {nb: axis_diff(angles[nb], axis) < 45.0 for nb, _ in legs}  # True: along the main axis
        # Cycle of the main axis: from the lights it already has, else by compass like the rest of the map
        existing = Counter()
        for nb, nv in arriving:
            towards = (nv.tarea, nv.tnode) == k
            if nv.light and (nv.light_dir == 1) == towards:
                existing[(group[nb], nv.light)] += 1
        main_votes = Counter()
        for (g, c), v in existing.items():
            main_votes[c if g else 3 - c] += v
        if main_votes:
            main_cycle = main_votes.most_common(1)[0][0]
        else:
            ns = axis_diff(axis, 90.0) < 45.0
            main_cycle = (1 if ns_is_1 else 2) if ns else (2 if ns_is_1 else 1)
        had = sum(existing.values())
        for nb, nv in arriving:
            towards = (nv.tarea, nv.tnode) == k
            want = main_cycle if group[nb] else 3 - main_cycle
            if nv.light and (nv.light_dir == 1) != towards:
                stats['legs kept: the light of the junction at the other end'] += 1
                continue
            new_flags = (nv.flags & ~LIGHT_MASK & ~DIR_BIT) | (want << 16) | (DIR_BIT if towards else 0)
            if new_flags != nv.flags:
                if nv.light:
                    stats['legs: cycle corrected'] += 1
                else:
                    stats['legs: light added'] += 1
                nv.flags = new_flags
                changed_areas.add(nv.area_index)
        stats['junctions that had no light' if had == 0 else 'junctions that had lights'] += 1

    os.makedirs(dst, exist_ok=True)
    for a in sorted(changed_areas):
        areas[a].write(os.path.join(dst, 'NODES%d.DAT' % a))
    for name, c in sorted(stats.items()):
        print('  %-55s %d' % (name, c))
    print('files written: %d -> %s' % (len(changed_areas), dst))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
