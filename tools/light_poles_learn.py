"""How the map places its traffic light poles: for each pole, the lit road arriving at the nearest junction it serves,
where the pole stands relative to that road (metres before the junction centre, metres to the right of the road's
centre line) and which way it faces relative to the traffic. Printed as histograms per model.

usage: python light_poles_learn.py <game dir> <data/paths folder>
"""
import math
import os
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402
from sa_ipl import all_insts, TRAFFIC_LIGHT_MODELS  # noqa: E402


def lit_approaches(net):
    """junction key -> [(neighbour key, unit direction of travel into the junction, navi)]"""
    out = defaultdict(list)
    for k, n in net.node.items():
        if n.links < 3:
            continue
        for nb, nv in net.adj[k]:
            if not nv or not nv.light or nb not in net.node:
                continue
            m = net.node[nb]
            dx, dy = n.x - m.x, n.y - m.y
            ln = math.hypot(dx, dy)
            if ln < 1:
                continue
            # the light applies to traffic arriving at this junction: (light_dir == 1) == (navi points to the junction)
            if (nv.light_dir == 1) != ((nv.tarea, nv.tnode) == k):
                continue
            out[k].append((nb, (dx / ln, dy / ln), nv))
    return out


def yaw_of(q):
    _, _, _, _, _, qx, qy, qz, qw = q[:9]
    return math.degrees(2 * math.atan2(qz, qw))


def main(game_dir, paths_dir):
    net = Network(load_all(paths_dir))
    appr = lit_approaches(net)
    juncs = [(k, net.node[k]) for k in appr]
    poles = [i for i in all_insts(game_dir) if i[0] in TRAFFIC_LIGHT_MODELS]
    stats = defaultdict(Counter)
    lons, lats = defaultdict(list), defaultdict(list)
    unmatched = 0
    for p in poles:
        mid, _, x, y, z = p[:5]
        best = None
        for k, n in juncs:
            d = math.hypot(x - n.x, y - n.y)
            if d > 30 or abs(z - n.z) > 8:
                continue
            for nb, (ux, uy), nv in appr[k]:
                rx, ry = x - n.x, y - n.y
                lon = rx * ux + ry * uy          # < 0: before the junction centre
                lat = rx * uy - ry * ux          # > 0: right of the road's centre line
                if lon > 2 or lat < 0:
                    continue
                score = abs(lat - 8) + abs(lon + 10) * 0.5
                if best is None or score < best[0]:
                    best = (score, lon, lat, math.degrees(math.atan2(uy, ux)))
        if best is None:
            unmatched += 1
            continue
        _, lon, lat, travel = best
        rel = (yaw_of(p[2:]) - travel) % 360
        name = TRAFFIC_LIGHT_MODELS[mid]
        stats[name][int(round(rel / 15.0) * 15) % 360] += 1
        lons[name].append(lon)
        lats[name].append(lat)
    print('poles', len(poles), 'not next to a lit junction', unmatched)
    for name, c in stats.items():
        ls, ts = sorted(lons[name]), sorted(lats[name])
        print('%-18s n=%4d  facing (yaw - travel, deg): %s' % (name, sum(c.values()), c.most_common(4)))
        print('%18s  before centre: median %.1f m (p25 %.1f, p75 %.1f)  right of centre: median %.1f m (p25 %.1f, p75 %.1f)' % (
            '', -ls[len(ls) // 2], -ls[3 * len(ls) // 4], -ls[len(ls) // 4], ts[len(ts) // 2], ts[len(ts) // 4], ts[3 * len(ts) // 4]))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
