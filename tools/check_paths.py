"""Structural check of GTA SA path files (NODES0..63.DAT) - run on anything written before giving it to the game.

usage: python check_paths.py <folder> [<original folder to compare totals with>]
"""
import math
import sys
import os
from collections import Counter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import load_all  # noqa: E402


def check(folder):
    areas = load_all(folder)
    errors = Counter()
    ex = {}
    veh = {}
    ped = {}
    for a, ar in areas.items():
        if ar.n_nodes != ar.n_veh + ar.n_ped:
            errors['header node counts'] += 1
        if ar.n_navi > 1024:
            errors['more than 1024 navis in a file'] += 1
        for i, n in enumerate(ar.nodes):
            if n.id != i or n.area != a:
                errors['node id/area field'] += 1
            if n.link + n.links > ar.n_links:
                errors['links out of range'] += 1
            (veh if i < ar.n_veh else ped)[(a, i)] = n
    total_navi_refs = 0
    for a, ar in areas.items():
        for i, n in enumerate(ar.nodes):
            key = (a, i)
            is_veh = i < ar.n_veh
            for k in range(n.links):
                li = n.link + k
                t = tuple(ar.links[li])
                if t[0] not in areas or t[1] >= areas[t[0]].n_nodes:
                    errors['link to a missing node'] += 1
                    continue
                tn = areas[t[0]].nodes[t[1]]
                back = [tuple(areas[t[0]].links[tn.link + j]) for j in range(tn.links)]
                if key not in back:
                    errors['link not symmetric'] += 1
                    ex.setdefault('link not symmetric', (key, t))
                t_is_veh = t[1] < areas[t[0]].n_veh
                if is_veh != t_is_veh:
                    errors['vehicle node linked to a ped node'] += 1
                if is_veh and t_is_veh:
                    nl = ar.navi_links[li]
                    na, nid = nl >> 10, nl & 0x3FF
                    if na not in areas or nid >= areas[na].n_navi:
                        errors['navi link to a missing navi'] += 1
                        continue
                    nv = areas[na].navi[nid]
                    total_navi_refs += 1
                    if (nv.tarea, nv.tnode) not in (key, t):
                        errors["navi's node is not an end of its link"] += 1
                        ex.setdefault("navi's node is not an end of its link", (key, t, (nv.tarea, nv.tnode)))
                    if nv.left == 0 and nv.right == 0:
                        errors['navi without lanes'] += 1
                    d = math.dist((n.x, n.y), (tn.x, tn.y))
                    if abs(ar.lengths[li] - min(255, max(1, round(d)))) > 2 and d < 250:
                        errors['link length off'] += 1
    lights = sum(1 for ar in areas.values() for v in ar.navi if v.light)
    print('%s: vehicle nodes %d, ped nodes %d, navis %d (max per file %d), lights %d' % (
        folder, len(veh), len(ped), sum(ar.n_navi for ar in areas.values()), max(ar.n_navi for ar in areas.values()), lights))
    if errors:
        for k, v in errors.most_common():
            print('  ERROR %-45s %d  e.g. %s' % (k, v, ex.get(k, '')))
    else:
        print('  no errors')
    return not errors


if __name__ == '__main__':
    ok = all(check(f) for f in sys.argv[1:])
    sys.exit(0 if ok else 1)
