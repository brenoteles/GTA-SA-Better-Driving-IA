"""Real road width of every stretch of the GTA SA road network, from the map's collision (no driving).

For each car path link (the 'navi': road centre, direction, lanes) a line across the road every 0.5 m, out to 25 m each
side; at each point the highest map surface near the road's height and its material. The road is the run of road
surface (tarmac, DEFAULT, painted ground, concrete) around the centre, at the centre's height (a step over 0.5 m is
a barrier or the deck's edge), gaps under 1 m allowed. Compared with the lanes the data
gives it (5.4 m each in the game): roads with room for more lanes, and data lanes wider than the asphalt.

usage: python road_width.py <game dir> <data/Paths> <override or -> <out.csv> [x0 y0 x1 y1 (only that box)]
"""
import math
import os
import sys
import time
from collections import Counter

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all, Network  # noqa: E402
from sa_to_sumo import merged_paths  # noqa: E402
from sa_col import load_world_triangles, TriGrid  # noqa: E402

ROAD = {0, 1, 2, 3, 7, 8, 26, 27}   # SF builds its streets with DEFAULT (0), sidewalks as PAVEMENT; country roads DIRT/DIRTTRACK
LANE_W = 5.4


def main(game_dir, base, override, out_csv, box=None, conjugate=True):
    net = Network(load_all(merged_paths(base, override)))
    node = net.node
    links = {}
    for a, lst in net.adj.items():
        for b, nv in lst:
            if nv is None or b not in node or node[a].boats or node[b].boats:
                continue
            key = (nv.area_index, nv.id)
            if key in links:
                continue
            if box and not (box[0] <= nv.x <= box[2] and box[1] <= nv.y <= box[3]):
                continue
            na, nb = node[a], node[b]
            links[key] = (nv, (na.z + nb.z) / 2.0, a, b)
    road_cells = set()
    for nv, z, a, b in links.values():
        road_cells.add((int(nv.x // 50), int(nv.y // 50)))

    def keep(x, y, r):
        cx, cy = int(x // 50), int(y // 50)
        rr = int(r // 50) + 1
        return any((cx + dx, cy + dy) in road_cells for dx in range(-rr, rr + 1) for dy in range(-rr, rr + 1))

    t0 = time.time()
    tris, mats, used, ncols = load_world_triangles(game_dir, keep, conjugate)
    print('collision models %d, instances placed %d, triangles %d (%.0f s)' % (ncols, used, len(tris), time.time() - t0))
    grid = TriGrid(tris, mats)
    print('grid cells %d (%.0f s)' % (len(grid.cells), time.time() - t0))

    offs = np.arange(-25.0, 25.01, 0.5)
    rows, centre_mats = [], Counter()
    for (area, nid), (nv, z, a, b) in links.items():
        dx, dy = nv.dx / 100.0, nv.dy / 100.0
        ln = math.hypot(dx, dy)
        if ln < 0.3:
            continue
        dx, dy = dx / ln, dy / ln
        rx, ry = dy, -dx
        pts = np.stack([nv.x + rx * offs, nv.y + ry * offs], axis=1)
        mat, hz = grid.surface(pts, z)
        c = len(offs) // 2
        centre_mats[int(mat[c])] += 1
        hc = hz[c] if mat[c] >= 0 else z
        is_road = np.isin(mat, list(ROAD)) & (np.abs(hz - hc) < 0.5)

        def run(step):
            i, gap, last = c, 0, c
            while 0 <= i + step < len(offs):
                i += step
                if is_road[i]:
                    last, gap = i, 0
                else:
                    gap += 1
                    if gap > 2:
                        break
            return abs(offs[last])
        if not is_road[c] and not (is_road[c - 1] or is_road[c + 1]):
            width_l = width_r = 0.0
        else:
            width_l, width_r = run(-1), run(1)
        width = width_l + width_r + 0.5
        lanes = nv.left + nv.right
        edge_mat_r = Counter(int(m) for m in mat[c + int(width_r / 0.5) + 1:c + int(width_r / 0.5) + 5]).most_common(1)
        rows.append((area, nid, nv.x, nv.y, z, lanes, nv.left, nv.right, width, width_l, width_r, int(mat[c]),
                     edge_mat_r[0][0] if edge_mat_r else -1))
    with open(out_csv, 'w') as f:
        f.write('area,navi,x,y,z,lanes,left,right,width,width_left,width_right,centre_material,beyond_right_material\n')
        for r in rows:
            f.write('%d,%d,%.1f,%.1f,%.1f,%d,%d,%d,%.1f,%.1f,%.1f,%d,%d\n' % r)
    measured = [r for r in rows if r[8] > 1.0]
    print('links %d, measured (road surface under the centre) %d (%.0f%%)' % (len(rows), len(measured),
                                                                          100.0 * len(measured) / max(len(rows), 1)))
    print('material under the road centre:', centre_mats.most_common(8))
    for lanes in (1, 2, 3, 4):
        ws = sorted(r[8] for r in measured if r[5] == lanes)
        if ws:
            print('  %d lanes in the data: width median %.1f m (p10 %.1f, p90 %.1f), n=%d' % (
                lanes, ws[len(ws) // 2], ws[len(ws) // 10], ws[9 * len(ws) // 10], len(ws)))
    print('written', out_csv, '(%.0f s)' % (time.time() - t0))


if __name__ == '__main__':
    box = tuple(float(v) for v in sys.argv[5:9]) if len(sys.argv) >= 9 else None
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], box, os.environ.get("RW_CONJ", "1") == "1")
