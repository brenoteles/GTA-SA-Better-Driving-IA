"""
Learns from a race against the AI (AutoTest Race = 1, race.csv): the sideways grip the player showed in curves, written
to the mod's learned limits for that car model (RealTrafficFix.limits.txt), so the AI plans its curves with what a human
did with the same car. Also prints the player's and the AI's curve speeds by radius.

usage: python race_learn.py <race.csv> <car model id> <game scripts folder> [--write]

The grip taken: the median sideways g of the player's steady curves (radius 30-400 m, over 40 km/h, no handbrake),
x 0.85 (a margin: the AI has no feel for the tyres). Never lowered: only raised over what the mod already learned.
"""
import csv
import os
import sys


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            if all(v not in (None, '') for v in r.values()):
                rows.append(r)
    return rows


def curves(rows, car):
    pts = []
    for r in rows:
        if r['car'] != car or r['handbrake'] == '1':
            continue
        v = float(r['speed_kmh']) / 3.6
        yaw = abs(float(r['yaw_rate']))
        if v > 11.0 and yaw > 0.05:
            pts.append((v / yaw, v * 3.6, float(r['lat_g'])))
    return pts


def main(csv_path, model, scripts, write=False):
    rows = load(csv_path)
    cars = sorted({r['car'] for r in rows})
    for car in cars:
        pts = curves(rows, car)
        print(f'{car}: {len(pts)} curve samples')
        for lo, hi in ((5, 15), (15, 30), (30, 60), (60, 120), (120, 250), (250, 600)):
            b = sorted(p for p in pts if lo <= p[0] < hi)
            if b:
                sp = sorted(p[1] for p in b)
                g = sorted(p[2] for p in b)
                print(f'  radius {lo:3d}-{hi:3d} m: {len(b):4d} samples, speed p50 {sp[len(sp) // 2]:.0f} km/h, sideways g p50 {g[len(g) // 2]:.2f}')
    steady = sorted(p[2] for p in curves(rows, 'PLAYER') if 30.0 <= p[0] < 400.0)
    if len(steady) < 50:
        print('not enough player curves to learn from')
        return
    shown = steady[len(steady) // 2] * 0.85
    path = os.path.join(scripts, 'RealTrafficFix.limits.txt')
    lines = open(path).read().splitlines() if os.path.exists(path) else []
    out, found = [], False
    for line in lines:
        c = line.split()
        if len(c) >= 3 and c[0] == str(model):
            found = True
            old = float(c[1])
            new = max(old, min(shown, 3.0))
            print(f'model {model}: learned grip {old:.2f} g -> {new:.2f} g (player median x 0.85 = {shown:.2f})')
            c[1] = f'{new:g}'
            line = ' '.join(c)
        out.append(line)
    if not found:
        print(f'model {model}: not in the learned limits yet -> {min(shown, 3.0):.2f} g')
        out.append(f'{model} {min(shown, 3.0):g} 9 1')
    if write:
        with open(path, 'w') as f:
            f.write('\n'.join(out) + '\n')
        print('written to', path)


if __name__ == '__main__':
    args = [a for a in sys.argv[1:] if a != '--write']
    main(args[0], int(args[1]), args[2], '--write' in sys.argv)
