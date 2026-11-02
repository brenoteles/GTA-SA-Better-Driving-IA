"""
Compares SUMO runs of the driver variants (sumo_profiles.py): totals, and per driver profile.

usage: python sumo_compare.py <dir with out_<variant> folders> [variant ...]
"""
import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict


def stats(d):
    s = {}
    st = ET.parse(os.path.join(d, 'stats.xml')).getroot()
    v = st.find('vehicles')
    s['inserted'] = int(v.get('inserted'))
    s['running'] = int(v.get('running'))
    t = st.find('teleports')
    s['teleports'] = int(t.get('total'))
    s['tp_jam'] = int(t.get('jam'))
    s['tp_yield'] = int(t.get('yield'))
    s['collisions'] = int(st.find('safety').get('collisions'))
    s['emergency'] = int(st.find('safety').get('emergencyStops'))
    vs = st.find('vehicleTripStatistics')
    for k in ('count', 'speed', 'duration', 'timeLoss', 'waitingTime'):
        s[k] = float(vs.get(k))
    return s


def per_profile(d):
    agg = defaultdict(lambda: [0, 0.0, 0.0, 0.0])  # trips, timeLoss, speed, lane changes
    for _, e in ET.iterparse(os.path.join(d, 'tripinfo.xml')):
        if e.tag != 'tripinfo':
            continue
        p = e.get('vType').split('_')[0]
        a = agg[p]
        a[0] += 1
        a[1] += float(e.get('timeLoss'))
        dur = float(e.get('duration'))
        a[2] += float(e.get('routeLength')) / dur if dur > 0 else 0.0
        e.clear()
    hits = defaultdict(int)
    path = os.path.join(d, 'collisions.xml')
    if os.path.exists(path):
        for _, e in ET.iterparse(path):
            if e.tag == 'collision':
                hits[e.get('colliderType', '?').split('_')[0]] += 1
                e.clear()
    return {p: (a[0], a[1] / a[0], a[2] / a[0] * 3.6, hits.get(p, 0)) for p, a in agg.items()}


def main(root, variants):
    variants = variants or sorted(x[4:] for x in os.listdir(root) if x.startswith('out_'))
    rows = {}
    print(f"{'variant':9s} {'trips':>5s} {'speed':>6s} {'timeLoss':>8s} {'wait':>6s} {'teleports':>9s} {'jam/yield':>9s} "
          f"{'collisions':>10s} {'emerg':>5s}")
    for v in variants:
        d = os.path.join(root, 'out_' + v)
        try:
            s = stats(d)
        except (OSError, ET.ParseError, AttributeError):
            print(f'{v:9s} (not finished)')
            continue
        rows[v] = s
        print(f"{v:9s} {s['count']:5.0f} {s['speed'] * 3.6:5.1f}k {s['timeLoss']:8.0f} {s['waitingTime']:6.0f} {s['teleports']:9d} "
              f"{s['tp_jam']:4d}/{s['tp_yield']:<4d} {s['collisions']:10d} {s['emergency']:5d}")
    for v in rows:
        if v == 'default':
            continue
        print(f'\n{v}: per profile (trips, time loss s, avg speed km/h, collisions caused)')
        for p, (n, tl, sp, c) in sorted(per_profile(os.path.join(root, 'out_' + v)).items()):
            print(f'  {p:11s} {n:5d} {tl:7.0f} {sp:6.1f} {c:5d}')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2:])
