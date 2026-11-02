"""Everything the road data says about the junctions around a point: nodes, their roads (lanes each way, light and cycle,
speed class), the measured asphalt (road_width.py csv) and the angle of each road.

usage: python junction_report.py <data/Paths> <override or -> <x> <y> [radius m] [widths.csv]
"""
import csv
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402
from sa_to_sumo import merged_paths  # noqa: E402


def main(base, override, x, y, radius=40.0, widths_csv=None):
    net = Network(load_all(merged_paths(base, override)))
    node = net.node
    widths = {}
    if widths_csv:
        for r in csv.DictReader(open(widths_csv)):
            widths[(int(r['area']), int(r['navi']))] = (float(r['width']), float(r['width_left']), float(r['width_right']))
    near = sorted((math.hypot(n.x - x, n.y - y), k) for k, n in node.items() if math.hypot(n.x - x, n.y - y) < radius and not n.boats)
    for d, k in near:
        n = node[k]
        print('node %d:%d at (%.1f, %.1f, %.1f) %.0f m away, %d roads, speed class %d%s' % (
            k[0], k[1], n.x, n.y, n.z, d, n.links, n.speed, ' JUNCTION' if n.links > 2 else ''))
        for nb, nv in net.adj[k]:
            if nb not in node or not nv:
                continue
            m = node[nb]
            ang = math.degrees(math.atan2(m.y - n.y, m.x - n.x))
            out_l, in_l = net.lanes_toward(k, nb, nv), net.lanes_toward(nb, k, nv)
            light = ''
            if nv.light:
                arriving_here = (nv.light_dir == 1) == ((nv.tarea, nv.tnode) == k)
                light = ' light cycle %d for traffic %s' % (nv.light, 'arriving here' if arriving_here else 'arriving at %d:%d' % nb)
            w = widths.get((nv.area_index, nv.id))
            ws = ' asphalt %.1f m' % w[0] if w else ''
            print('   -> %d:%d heading %4.0f deg, %4.1f m, lanes out %d / in %d%s%s' % (
                nb[0], nb[1], ang, math.hypot(m.x - n.x, m.y - n.y), out_l, in_l, light, ws))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4]),
         float(sys.argv[5]) if len(sys.argv) > 5 else 40.0, sys.argv[6] if len(sys.argv) > 6 else None)
