"""Physical traffic light poles for the lights the path data has but the map doesn't (lights_everywhere.py added them in
the road data only: cars stopped at lights nobody could see).

For each lit road arriving at a junction (the light for traffic arriving there) with no pole near its stop corner: a pole
on the right-hand corner, before the crossing road's edge (+3 m), 1 m outside the edge of the carriageway, axis-aligned with
the road (the game colours each face of a pole by its facing, CTrafficLights::FindTrafficLightType), upright (tilted poles
aren't lit). The model and its height above the road node are the ones the map's poles use nearby. Not on another road,
not within 8 m of an existing pole.

usage: python place_light_poles.py <game dir> <data/Paths> <override folder or -> <out .ipl> [yaw offset deg] [lift m]

lift: the poles raised that much over the road (temporary: placed on the road they were obstacles; 5.5 m puts
their base over any vehicle and leaves the lights floating over the junction)
"""
import math
import os
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402
from sa_ipl import all_insts, TRAFFIC_LIGHT_MODELS  # noqa: E402
from sa_to_sumo import merged_paths  # noqa: E402
from light_poles_learn import lit_approaches  # noqa: E402

LANE_W = 5.4


def one_way_offset(left, right):
    if left == 0:
        return 0.5 - 0.5 * right
    if right == 0:
        return 0.5 - 0.5 * left
    return 0.5


def seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 < 1e-6 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(ax + dx * t - px, ay + dy * t - py)


def main(game_dir, base, override, out_ipl, yaw_offset=180.0, lift=0.0):
    net = Network(load_all(merged_paths(base, override)))
    node = net.node
    appr = lit_approaches(net)
    poles = [i for i in all_insts(game_dir) if i[0] in TRAFFIC_LIGHT_MODELS]
    pgrid = defaultdict(list)
    for p in poles:
        pgrid[(int(p[2] // 50), int(p[3] // 50))].append(p)

    def poles_near(x, y, r):
        gx, gy = int(x // 50), int(y // 50)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for p in pgrid[(gx + dx, gy + dy)]:
                    if math.hypot(p[2] - x, p[3] - y) < r:
                        yield p

    # Height of a pole's origin over its junction node, per model (from the map's own poles)
    jgrid = defaultdict(list)
    for k in appr:
        n = node[k]
        jgrid[(int(n.x // 50), int(n.y // 50))].append(n)
    dz = defaultdict(list)
    for p in poles:
        best = None
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for n in jgrid[(int(p[2] // 50) + dx, int(p[3] // 50) + dy)]:
                    d = math.hypot(n.x - p[2], n.y - p[3])
                    if d < 25 and (best is None or d < best[0]):
                        best = (d, n)
        if best:
            dz[p[0]].append(p[4] - best[1].z)
    model_dz = {m: sorted(v)[len(v) // 2] for m, v in dz.items() if v}

    def road_half_width(nv):
        return max(nv.left + nv.right, 1) * LANE_W / 2.0

    out, stats = [], Counter()
    for k, roads in appr.items():
        J = node[k]
        # roads at the junction (for the crossing width and "not on a road")
        legs = []
        for nb, nv in net.adj[k]:
            if nv and nb in node:
                legs.append((node[nb], nv))
        for nb, (ux, uy), nv in roads:
            rx, ry = uy, -ux                                    # right of the direction of travel
            toward = net.lanes_toward(nb, k, nv)
            if toward <= 0:
                stats['no lanes towards the junction'] += 1
                continue
            ow = one_way_offset(nv.left, nv.right)
            o = (nv.x - J.x) * rx + (nv.y - J.y) * ry           # the navi line's offset from the node line
            lat = o + (ow + toward - 0.5) * LANE_W + 1.0
            # before the crossing roads' edge
            cross = 0.0
            for m, lv in legs:
                vx, vy = m.x - J.x, m.y - J.y
                ln = math.hypot(vx, vy)
                if ln < 1 or abs((vx * ux + vy * uy) / ln) > 0.7:
                    continue                                    # along our road, not crossing it
                cross = max(cross, road_half_width(lv))
            placed = None
            for back in (cross + 3.0, cross + 6.0, cross + 9.0):
                back = max(6.0, min(back, 18.0))
                x, y = J.x - ux * back + rx * lat, J.y - uy * back + ry * lat
                on_road = False
                for m, lv in legs:
                    if seg_dist(x, y, J.x, J.y, m.x, m.y) < road_half_width(lv) + 0.5:
                        on_road = True
                        break
                if not on_road:
                    placed = (x, y, back)
                    break
            if not placed:
                stats['skipped: every spot on a road'] += 1
                continue
            x, y, back = placed
            if any(True for _ in poles_near(x, y, 8.0)):
                stats['skipped: the map has a pole there'] += 1
                continue
            if any(math.hypot(q[2] - x, q[3] - y) < 8.0 for q in out):
                stats['skipped: one of ours already there'] += 1
                continue
            near = Counter(p[0] for p in poles_near(x, y, 600.0)) if False else Counter()
            # the city's model: the most common within 600 m (by grid), else MTraffic1
            gx, gy = int(x // 50), int(y // 50)
            for dx in range(-12, 13):
                for dy in range(-12, 13):
                    for p in pgrid.get((gx + dx, gy + dy), ()):
                        near[p[0]] += 1
            mid = near.most_common(1)[0][0] if near else 1283
            if mid in (1262, 1263):
                mid = 1283
            m = node[nb]
            seg = math.hypot(m.x - J.x, m.y - J.y)
            zroad = J.z + (m.z - J.z) * min(back / seg, 1.0) if seg > 1 else J.z
            z = zroad + model_dz.get(mid, 0.0) + lift
            travel = math.degrees(math.atan2(uy, ux))
            yaw = travel - 90.0 + yaw_offset                     # GTA heading: model +Y = yaw + 90 (math angle)
            h = math.radians(yaw) / 2.0
            out.append((mid, TRAFFIC_LIGHT_MODELS[mid], x, y, z, 0.0, 0.0, math.sin(h), math.cos(h)))
            stats['poles placed'] += 1
            stats['model ' + TRAFFIC_LIGHT_MODELS[mid]] += 1
    os.makedirs(os.path.dirname(out_ipl), exist_ok=True)
    with open(out_ipl, 'w') as f:
        f.write('# Real Traffic Fix: traffic light poles for the lights in the road data (tools/place_light_poles.py)\ninst\n')
        for mid, name, x, y, z, qx, qy, qz, qw in out:
            f.write('%d, %s, 0, %.3f, %.3f, %.3f, %.6f, %.6f, %.6f, %.6f, -1\n' % (mid, name, x, y, z, qx, qy, qz, qw))
        f.write('end\n')
    print('model height over the junction node:', {TRAFFIC_LIGHT_MODELS[m]: round(v, 2) for m, v in model_dz.items()})
    for kk, c in sorted(stats.items()):
        print('  %-40s %d' % (kk, c))
    print('written', out_ipl)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], float(sys.argv[5]) if len(sys.argv) > 5 else 180.0,
         float(sys.argv[6]) if len(sys.argv) > 6 else 0.0)
