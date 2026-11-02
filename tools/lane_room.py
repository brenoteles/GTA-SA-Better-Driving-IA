"""From road_width.py's csv: stretches with room for more lanes (both edges found, at least one more 5.4 m lane of
asphalt in a direction) and stretches whose data lanes are wider than the asphalt (AI lanes running off the road).

usage: python lane_room.py <widths.csv> [out_prefix]
"""
import csv
import sys
from collections import Counter

LANE_W = 5.4


def main(path, out=None):
    rows = list(csv.DictReader(open(path)))
    room, over = [], []
    for r in rows:
        w, wl, wr = float(r['width']), float(r['width_left']), float(r['width_right'])
        lanes, left, right = int(r['lanes']), int(r['left']), int(r['right'])
        if w <= 1.0 or lanes == 0:
            continue
        bounded = wl < 24.9 and wr < 24.9
        if lanes * LANE_W > w + 1.0:
            over.append(r)
        if not bounded:
            continue
        if left == 0 or right == 0:
            fit = int(w // LANE_W)
            if fit > lanes:
                room.append((r, fit))
        else:
            fit = int((w / 2) // LANE_W)
            if fit > max(left, right):
                room.append((r, fit))
    measured = sum(1 for r in rows if float(r['width']) > 1.0)
    print('measured stretches: %d of %d' % (measured, len(rows)))
    print('room for more lanes: %d stretches (%s)' % (len(room), Counter('one-way' if (int(r['left']) == 0 or int(r['right']) == 0) else 'two-way' for r, _ in room)))
    print('data lanes wider than the asphalt: %d stretches; by lanes %s' % (len(over), Counter(int(r['lanes']) for r in over).most_common()))
    if out:
        with open(out + '_room.csv', 'w') as f:
            f.write('x,y,z,left,right,width,fits\n')
            for r, fit in room:
                f.write('%s,%s,%s,%s,%s,%s,%d\n' % (r['x'], r['y'], r['z'], r['left'], r['right'], r['width'], fit))
        with open(out + '_over.csv', 'w') as f:
            f.write('x,y,z,left,right,width\n')
            for r in over:
                f.write('%s,%s,%s,%s,%s,%s\n' % (r['x'], r['y'], r['z'], r['left'], r['right'], r['width']))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
