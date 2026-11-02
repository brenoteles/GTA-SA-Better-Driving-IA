"""Write lane changes (lane_additions.py csv: area,navi,left,right) into the path files the game loads.

Reads data/Paths with the override folder over it, changes the lane bits (8-10 left, 11-13 right) of the listed links and
writes only the changed areas to the output folder (to copy over the mod's modloader path folder).

usage: python apply_lanes.py <data/Paths> <override or -> <lanes.csv> <out dir>
"""
import csv
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all  # noqa: E402
from sa_to_sumo import merged_paths  # noqa: E402


def main(base, override, lanes_csv, out):
    areas = load_all(merged_paths(base, override))
    changed = set()
    n = 0
    for r in csv.DictReader(open(lanes_csv)):
        a, i = int(r['area']), int(r['navi'])
        ar = areas.get(a)
        if not ar or i >= len(ar.navi):
            continue
        v = ar.navi[i]
        v.flags = (v.flags & ~(0x3F << 8)) | (int(r['left']) << 8) | (int(r['right']) << 11)
        changed.add(a)
        n += 1
    os.makedirs(out, exist_ok=True)
    for a in sorted(changed):
        areas[a].write(os.path.join(out, 'NODES%d.DAT' % a))
    print('links changed %d in %d area files -> %s' % (n, len(changed), out))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
