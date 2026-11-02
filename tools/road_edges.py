"""Asphalt edges for the mod (RealTrafficFix.roadedges.txt): the road links whose data lanes are wider than the measured
road (road_width.py), with the asphalt left and right of the link's line. The mod keeps the cars' bodies inside it.

Line format: area navi left_m right_m   (left/right of the link's own direction, from its position)

usage: python road_edges.py <widths.csv> <out.txt>
"""
import csv
import sys

LANE_W = 5.4


def main(path, out):
    n = 0
    with open(out, 'w') as f:
        f.write('# Real Traffic Fix: measured asphalt edges of narrow road links (tools/road_width.py + road_edges.py)\n')
        f.write('# area navi left_m right_m\n')
        for r in csv.DictReader(open(path)):
            w, wl, wr = float(r['width']), float(r['width_left']), float(r['width_right'])
            lanes = int(r['lanes'])
            if lanes == 0 or w < 3.0 or wl >= 24.9 or wr >= 24.9:
                continue
            if lanes * LANE_W <= w + 1.0:
                continue
            # The run's last road sample is where the asphalt ends (samples every 0.5 m: +0.25 to its middle)
            f.write('%s %s %.2f %.2f\n' % (r['area'], r['navi'], wl + 0.25, wr + 0.25))
            n += 1
    print('narrow links written:', n, '->', out)


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
