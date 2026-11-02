"""Extra lanes where the measured asphalt has room (road_width.py csv), whole road stretches only.

A stretch = the links between two junctions (or dead ends). It gets one more lane only when it's one-way (every link
carries traffic one way, the same number of lanes) and every link was measured with both edges found and room for that
lane (5.4 m each in the game; one-way lanes are centred on the link, so the extra lane fits if the asphalt is that wide).
Half a stretch would leave lane drops every few metres.

Output csv: area,navi,left,right  (the new lane counts of each link, as sa_to_sumo.py / an apply step read them)

usage: python lane_additions.py <data/Paths> <override or -> <widths.csv> <out.csv> [x0 y0 x1 y1]
"""
import csv
import math
import os
import sys
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402
from sa_to_sumo import merged_paths  # noqa: E402

LANE_W = 5.4


def main(base, override, widths_csv, out_csv, box=None):
    net = Network(load_all(merged_paths(base, override)))
    node = net.node
    meas = {}
    for r in csv.DictReader(open(widths_csv)):
        wl, wr, w = float(r['width_left']), float(r['width_right']), float(r['width'])
        if w > 1.0 and wl < 24.9 and wr < 24.9:
            meas[(int(r['area']), int(r['navi']))] = w

    def legs(k):
        return [(nb, nv) for nb, nv in net.adj[k] if nb in node and nv and not node[nb].boats]

    stats = Counter()
    seen, out = set(), []
    for start in node:
        if node[start].boats or len(legs(start)) == 2:
            continue
        for first, _ in legs(start):
            if (start, first) in seen:
                continue
            chain, prev, cur = [start], start, first
            while True:
                chain.append(cur)
                seen.add((prev, cur))
                ls = legs(cur)
                if len(ls) != 2 or len(chain) > 500:
                    break
                nxt = ls[0][0] if ls[1][0] == prev else ls[1][0]
                prev, cur = cur, nxt
            seen.add((chain[-1], chain[-2]))
            links = []
            for a, b in zip(chain, chain[1:]):
                nv = next((v for x, v in legs(a) if x == b), None)
                if nv:
                    links.append((a, b, nv))
            if not links:
                continue
            if box and not any(box[0] <= nv.x <= box[2] and box[1] <= nv.y <= box[3] for _, _, nv in links):
                continue
            stats['stretches'] += 1
            fwd = [net.lanes_toward(a, b, nv) for a, b, nv in links]
            back = [net.lanes_toward(b, a, nv) for a, b, nv in links]
            if any(fwd) and any(back):
                stats['two-way (left as they are)'] += 1
                continue
            lanes = fwd if any(fwd) else back
            if len(set(lanes)) != 1 or lanes[0] == 0:
                stats['one-way, lane count changes along it'] += 1
                continue
            n = lanes[0]
            ws = [meas.get((nv.area_index, nv.id)) for _, _, nv in links]
            if any(w is None for w in ws):
                stats['one-way, not every link measured'] += 1
                continue
            if min(ws) < (n + 1) * LANE_W:
                stats['one-way, no room'] += 1
                continue
            length = sum(math.hypot(node[a].x - node[b].x, node[a].y - node[b].y) for a, b, _ in links)
            if length < 30:
                stats['one-way, under 30 m'] += 1
                continue
            stats['one-way, gets a lane'] += 1
            for _, _, nv in links:
                left, right = nv.left, nv.right
                if left:
                    left += 1
                else:
                    right += 1
                out.append((nv.area_index, nv.id, left, right, nv.x, nv.y, n, min(ws)))
    with open(out_csv, 'w') as f:
        f.write('area,navi,left,right,x,y,lanes_before,min_width\n')
        seen_k = set()
        for row in out:
            if (row[0], row[1]) in seen_k:
                continue
            seen_k.add((row[0], row[1]))
            f.write('%d,%d,%d,%d,%.1f,%.1f,%d,%.1f\n' % row)
    for k, c in sorted(stats.items()):
        print('  %-42s %d' % (k, c))
    print('links changed: %d -> %s' % (len(seen_k), out_csv))


if __name__ == '__main__':
    box = tuple(float(v) for v in sys.argv[5:9]) if len(sys.argv) >= 9 else None
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], box)
