"""Find flat, open tarmac in the map (a 'grid' proving ground for the vehicle physics tests): the map collision rasterised on a
2 m grid (top surface, material), cells that are road-like, level with their neighbours and with nothing above them; then
the largest circle of such cells in each candidate area.

python tools/flat_sites.py <game dir>
"""
import sys

import numpy as np
from scipy import ndimage

from sa_col import load_world_triangles

AREAS = {
    'LS airport': (1350, -2750, 2200, -2150),
    'LV airport': (1250, 1100, 1800, 1900),
    'SF airport': (-1750, -750, -1000, 100),
    'Verdant Meadows': (-200, 2350, 500, 2650),
    'LV desert strip / Bone County': (-100, 1700, 500, 2100),
}
ROAD = {0, 1, 2, 3, 4, 5, 7, 8}    # default (airport ground), tarmac x3, pavement x2, fucked concrete, painted ground (surfinfo.dat lines)
CELL = 2.0


def raster(tris, mats, box):
    x0, y0, x1, y1 = box
    nx, ny = int((x1 - x0) / CELL), int((y1 - y0) / CELL)
    top = np.full((ny, nx), -1e9, np.float32)
    mat = np.full((ny, nx), -1, np.int16)
    cnt = np.zeros((ny, nx), np.int16)       # how many surfaces stack in the cell (bridges, roofs)
    for t, m in zip(tris, mats):
        mn, mx = t[:, :2].min(0), t[:, :2].max(0)
        i0, i1 = int((mn[0] - x0) // CELL), int((mx[0] - x0) // CELL)
        j0, j1 = int((mn[1] - y0) // CELL), int((mx[1] - y0) // CELL)
        if i1 < 0 or j1 < 0 or i0 >= nx or j0 >= ny:
            continue
        i0, j0, i1, j1 = max(i0, 0), max(j0, 0), min(i1, nx - 1), min(j1, ny - 1)
        a, b, c = t
        v0, v1 = (b - a)[:2], (c - a)[:2]
        den = v0[0] * v1[1] - v1[0] * v0[1]
        if abs(den) < 1e-6:
            continue
        xs = x0 + (np.arange(i0, i1 + 1) + 0.5) * CELL
        ys = y0 + (np.arange(j0, j1 + 1) + 0.5) * CELL
        X, Y = np.meshgrid(xs, ys)
        dx, dy = X - a[0], Y - a[1]
        u = (dx * v1[1] - v1[0] * dy) / den
        v = (v0[0] * dy - dx * v0[1]) / den
        inside = (u >= 0) & (v >= 0) & (u + v <= 1)
        if not inside.any():
            continue
        z = a[2] + u * (b - a)[2] + v * (c - a)[2]
        sub_top = top[j0:j1 + 1, i0:i1 + 1]
        sub_mat = mat[j0:j1 + 1, i0:i1 + 1]
        cnt[j0:j1 + 1, i0:i1 + 1] += inside
        better = inside & (z > sub_top)
        sub_top[better] = z[better]
        sub_mat[better] = m
    return top, mat, cnt


def main(game_dir):
    for name, box in AREAS.items():
        x0, y0, x1, y1 = box
        keep = lambda x, y, r: x + r > x0 and x - r < x1 and y + r > y0 and y - r < y1
        tris, mats, used, _ = load_world_triangles(game_dir, keep=keep)
        top, mat, cnt = raster(tris, mats, box)
        valid = top > -1e8
        zmed = ndimage.median_filter(np.where(valid, top, -1000), size=5)
        gy, gx = np.gradient(np.where(valid, top, 0), CELL)
        slope = np.hypot(gx, gy)
        good = valid & np.isin(mat, list(ROAD)) & (np.abs(top - zmed) < 0.15) & (slope < 0.03) & (cnt == 1)
        # the level of the largest flat region: cells within 0.6 m of it
        lab, n = ndimage.label(good)
        if n == 0:
            print(f'{name}: nothing flat'); continue
        sizes = ndimage.sum(good, lab, range(1, n + 1))
        k = int(np.argmax(sizes)) + 1
        zref = float(np.median(top[lab == k]))
        good &= np.abs(top - zref) < 0.6
        dist = ndimage.distance_transform_edt(good) * CELL
        j, i = np.unravel_index(np.argmax(dist), dist.shape)
        cx, cy = x0 + (i + 0.5) * CELL, y0 + (j + 0.5) * CELL
        # the longest straight (x or y) through flat cells, for slaloms / acceleration runs
        print(f'{name}: {used} objects, flat area {good.sum() * CELL * CELL / 1e4:.1f} ha at z {zref:.1f}; '
              f'largest clear circle r={dist.max():.0f} m at ({cx:.0f}, {cy:.0f})')
        # the 5 best separated circles
        d2 = dist.copy()
        for _ in range(4):
            d2[max(j - int(dist[j, i] / CELL), 0):j + int(dist[j, i] / CELL) + 1,
               max(i - int(dist[j, i] / CELL), 0):i + int(dist[j, i] / CELL) + 1] = 0
            j, i = np.unravel_index(np.argmax(d2), d2.shape)
            print(f'   next r={d2[j, i]:.0f} m at ({x0 + (i + .5) * CELL:.0f}, {y0 + (j + .5) * CELL:.0f}) z {top[j, i]:.1f}')
        np.save(f'flat_{name.split()[0]}.npy', np.stack([top, mat.astype(np.float32), good.astype(np.float32)]))


if __name__ == '__main__':
    main(sys.argv[1])
