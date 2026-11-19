"""The test town: a road network for the traffic AI on the proving grid's top deck (after the proving ground: GTA's own
streets are poor test roads). One-way and two-way streets, 1 to 3 lanes each way, avenues with traffic lights,
junctions without lights, a one-way 3-lane ring freeway with exits and entries, a roundabout and a winding rural road.

The roads are GTA SA path nodes (NODES*.DAT): the game, its AI and the mod drive them like any street. They are added
to the installed road data (the modloader folder 'RealTrafficFix Paths'), every time from the copy made on the first
run (<game>/RealTrafficFix_paths_base), so running this again replaces the town, never stacks it.

Outputs:
  <modloader>/RealTrafficFix Paths/gta3.img/NODES*.DAT   the road data with the town (all 64 areas)
  <game>/scripts/RealTrafficFix.town.txt                 what the mod draws (asphalt, lane lines, stop lines, lights)
  docs/town.json, docs/town_map.png                      the network and its 2D map (world coordinates, 100 m grid)

python tools/make_town.py [game dir] [--no-install]
"""
import json
import math
import os
import shutil
import struct
import sys
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sa_paths import Area  # noqa: E402

GAME = r'C:\Users\death\Games\GTA\GTA San Andreas'
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DECK_Z = 300.0 + 40.0 * 49       # the grid's top deck (make_gridmap.py DECKS): open sky
LANE = 5.4                       # GTA's lane width (CCarCtrl, 0x858C50)
FLOOD = 60                       # its own road network (the map's flood fill ids are 1..33)
NAVI_CAP = 1000                  # the engine addresses 1024 road links per area file
SPACING = {'freeway': 30.0, 'avenue': 20.0, 'street': 20.0, 'ramp': 15.0, 'rural': 15.0, 'round': 8.0}
APPROACH = 12.0                  # a node this far before every junction (its light's link, the stop line)

# --- The network -------------------------------------------------------------------------------------------------
# A road: its points (x, y) or (x, y, radius: a rounded corner there), lanes along it (fwd) and against it (back),
# the kind (node spacing, speed class). Roads meet at points they share exactly: those are the junctions.
# GTA drives on the right; +y is north.
X_STREETS = (500, 800, 1100, 1400, 1700, 2000, 2300)
Y_A, Y_B, Y_C = -2150, -2350, -2550
ROADS = []


def road(name, pts, fwd, back, kind):
    ROADS.append({'name': name, 'pts': [tuple(p) for p in pts], 'fwd': fwd, 'back': back, 'kind': kind})


def grid_line(y):
    return [(x, y) for x in X_STREETS]


# Avenues (west -> east)
road('Av. A (2+2, semaforos)', grid_line(Y_A), 2, 2, 'avenue')
road('Av. B (3+3, semaforos)', grid_line(Y_B), 3, 3, 'avenue')
road('Av. C (1+1, sem semaforos)', grid_line(Y_C) + [], 1, 1, 'street')
# Streets (north -> south: from Av. A at y -2150 down to Av. C at y -2550); fwd = southbound
for x, fwd, back, label in ((500, 1, 1, '1+1'), (800, 0, 2, 'mao unica norte, 2'), (1100, 2, 0, 'mao unica sul, 2'),
                            (1400, 2, 2, '2+2'), (1700, 0, 1, 'mao unica norte, 1'), (2000, 3, 0, 'mao unica sul, 3'),
                            (2300, 1, 1, '1+1')):
    pts = [(x, Y_A), (x, Y_B), (x, Y_C)]
    if x == 2300:
        pts = [(x, Y_A), (x, Y_B), (x, -2450), (x, Y_C)]
    road('Rua x=%d (%s)' % (x, label), pts, fwd, back, 'street')

# The ring freeway: one way, clockwise (north side eastbound), 3 lanes; its exits and entries on the right (inside)
RING = [(350, -1850), (2450, -1850), (2700, -1850, 200), (2700, -2100), (2700, -2600), (2700, -2850, 200),
        (1700, -2850), (100, -2850, 200), (100, -2450), (100, -1850, 200), (350, -1850)]
road('Rodovia anel (3 faixas, mao unica)', RING, 3, 0, 'freeway')
# Exits and entries (one lane, one way)
road('Saida norte -> Av. A', [(350, -1850), (500, -1900, 60), (500, Y_A)], 1, 0, 'ramp')
road('Entrada norte (Rua x=2300)', [(2300, Y_A), (2300, -1910, 50), (2450, -1850)], 1, 0, 'ramp')
road('Saida leste -> rotatoria', [(2700, -2100), (2600, -2230, 60), (2550, -2410)], 1, 0, 'ramp')
road('Entrada leste (rotatoria)', [(2550, -2490), (2590, -2570, 40), (2700, -2600)], 1, 0, 'ramp')
road('Saida sul -> Rua x=1400', [(1700, -2850), (1420, -2790, 60), (1400, -2700, 40), (1400, Y_C)], 1, 0, 'ramp')
road('Entrada oeste (Av. C)', [(500, Y_C), (250, -2610, 60), (130, -2560, 50), (100, -2450)], 1, 0, 'ramp')
# The roundabout: one lane, counter-clockwise; arms north (the exit), south (the entry), west (to Rua x=2300)
RC, RR = (2550, -2450), 40.0
road('Rotatoria', [(2590, -2450), (2550, -2410), (2510, -2450), (2550, -2490), (2590, -2450)], 1, 0, 'round')
road('Ligacao rotatoria (1+1)', [(2300, -2450), (2510, -2450)], 1, 1, 'street')
# A winding rural road west of the town (1+1): Av. B's west end round to Av. A's
road('Estrada rural (1+1, curvas)', [(500, Y_B), (380, -2385, 40), (220, -2300, 60), (300, -2180, 50), (500, Y_A)], 1, 1, 'rural')

# Junctions with traffic lights: every junction of Av. A and Av. B
LIT = {(x, y) for x in X_STREETS for y in (Y_A, Y_B)}


# --- Geometry ----------------------------------------------------------------------------------------------------

def expand(road):
    """The road's centre line: (x, y, keep) points, rounded corners as arcs (keep: a point every road through it shares)"""
    pts = road['pts']
    out = []
    for i, p in enumerate(pts):
        r = p[2] if len(p) > 2 else 0.0
        if r <= 0 or i == 0 or i == len(pts) - 1:
            out.append((p[0], p[1], True))
            continue
        a, b, c = pts[i - 1], p, pts[i + 1]
        u = (b[0] - a[0], b[1] - a[1]); lu = math.hypot(*u); u = (u[0] / lu, u[1] / lu)
        v = (c[0] - b[0], c[1] - b[1]); lv = math.hypot(*v); v = (v[0] / lv, v[1] / lv)
        turn = math.atan2(u[0] * v[1] - u[1] * v[0], u[0] * v[0] + u[1] * v[1])
        if abs(turn) < 1e-3:
            out.append((p[0], p[1], True))
            continue
        t = r * math.tan(abs(turn) / 2)
        # (the corner's arc ends at most 30 m before a junction, halfway to another rounded corner)
        lim_in = 0.5 * lu - 5 if i > 1 and len(pts[i - 1]) > 2 else lu - 30
        lim_out = 0.5 * lv - 5 if i + 2 < len(pts) and len(pts[i + 1]) > 2 else lv - 30
        t = min(t, lim_in, lim_out)
        r = t / math.tan(abs(turn) / 2)
        p0 = (b[0] - u[0] * t, b[1] - u[1] * t)
        side = 1 if turn > 0 else -1
        centre = (p0[0] - u[1] * r * side, p0[1] + u[0] * r * side)
        a0 = math.atan2(p0[1] - centre[1], p0[0] - centre[0])
        steps = max(2, int(math.ceil(abs(turn) * r / 8.0)))
        for k in range(steps + 1):
            ang = a0 + turn * k / steps
            out.append((centre[0] + r * math.cos(ang), centre[1] + r * math.sin(ang), False))
    return out


def round_arc(pts):
    """The roundabout: its four points joined by arcs round its centre (counter-clockwise)"""
    out = []
    for i in range(len(pts) - 1):
        a = math.atan2(pts[i][1] - RC[1], pts[i][0] - RC[0])
        b = math.atan2(pts[i + 1][1] - RC[1], pts[i + 1][0] - RC[0])
        while b <= a:
            b += 2 * math.pi
        out.append((pts[i][0], pts[i][1], True))
        for k in range(1, 4):
            ang = a + (b - a) * k / 4
            out.append((RC[0] + RR * math.cos(ang), RC[1] + RR * math.sin(ang), False))
    out.append((pts[-1][0], pts[-1][1], True))
    return out


def sample(line, spacing):
    """Nodes along a centre line: every kept point, one APPROACH m from each, and at most 'spacing' apart between"""
    spans, cur = [], [line[0]]
    for p in line[1:]:
        cur.append(p)
        if p[2]:
            spans.append(cur)
            cur = [p]
    nodes = [line[0][:2]]
    for sp in spans:
        cum = [0.0]
        for i in range(1, len(sp)):
            cum.append(cum[-1] + math.dist(sp[i - 1][:2], sp[i][:2]))
        L = cum[-1]

        def at(s):
            for i in range(1, len(sp)):
                if cum[i] >= s - 1e-6:
                    w = (s - cum[i - 1]) / max(cum[i] - cum[i - 1], 1e-6)
                    return (sp[i - 1][0] + (sp[i][0] - sp[i - 1][0]) * w, sp[i - 1][1] + (sp[i][1] - sp[i - 1][1]) * w)
            return sp[-1][:2]
        marks = []
        if L > 3 * APPROACH:
            inner = L - 2 * APPROACH
            n = max(1, int(math.ceil(inner / spacing)))
            marks = [APPROACH + inner * k / n for k in range(n + 1)]
        elif L > spacing:
            marks = [L / 2]
        # the curve's own points as nodes too where they're far from a mark (a node every ~8 m on the arcs)
        for s in marks:
            nodes.append(at(s))
        nodes.append(sp[-1][:2])
    # (round curves: more nodes along them)
    out = [nodes[0]]
    for p in nodes[1:]:
        if math.dist(out[-1], p) > 0.5:
            out.append(p)
    return out


def curve_densify(line, nodes, kind):
    """On a rounded corner the nodes follow the arc: every arc point becomes a node"""
    arc = [p[:2] for p in line if not p[2]]
    if not arc:
        return nodes
    pts = nodes + arc
    # order along the line
    order = {p[:2]: i for i, p in enumerate(line)}

    def pos_on(p):
        best, bi = 1e18, 0
        for i in range(len(line) - 1):
            a, b = line[i], line[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            l2 = dx * dx + dy * dy or 1e-9
            t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2))
            d = math.hypot(a[0] + dx * t - p[0], a[1] + dy * t - p[1])
            if d < best:
                best, bi = d, i + t
        return bi
    pts = sorted(set(pts), key=pos_on)
    out = [pts[0]]
    for p in pts[1:]:
        if math.dist(out[-1], p) > 4.0:
            out.append(p)
    if out[-1] != nodes[-1]:
        out[-1] = nodes[-1]
    return out


def build():
    key = lambda p: (round(p[0], 2), round(p[1], 2))  # noqa: E731
    nodes = {}                  # key -> dict(x, y, idx)
    links = {}                  # (ka, kb) sorted pair -> dict(a, b, fwd(a->b), back, kind, road)
    order = []
    for r in ROADS:
        line = round_arc(r['pts']) if r['kind'] == 'round' else expand(r)
        pts = sample(line, SPACING[r['kind']])
        pts = curve_densify(line, pts, r['kind'])
        r['line'] = [p[:2] for p in line]
        r['nodes'] = pts
        for p in pts:
            k = key(p)
            if k not in nodes:
                nodes[k] = {'x': p[0], 'y': p[1], 'idx': len(order)}
                order.append(k)
        for p, q in zip(pts, pts[1:]):
            a, b = key(p), key(q)
            pair = (a, b) if a < b else (b, a)
            if pair in links:
                raise SystemExit('two roads on the same piece: %s %s (%s)' % (a, b, r['name']))
            fwd, back = (r['fwd'], r['back']) if pair == (a, b) else (r['back'], r['fwd'])
            links[pair] = {'a': pair[0], 'b': pair[1], 'fwd': fwd, 'back': back, 'kind': r['kind'], 'road': r['name']}
    adj = defaultdict(list)
    for pair in links:
        adj[pair[0]].append(pair[1])
        adj[pair[1]].append(pair[0])
    for k in order:
        if len(adj[k]) == 1:
            raise SystemExit('dead end at %s' % (k,))
        if len(adj[k]) > 15:
            raise SystemExit('too many roads at %s' % (k,))
    lit = {key(p) for p in LIT}
    # Each link's navi: its node (the end its 'left' lanes go to), its light
    for pair, l in links.items():
        a, b = pair
        tnode = b if l['fwd'] > 0 else a
        if l['fwd'] > 0 and l['back'] > 0:
            # two-way: the junction end with a light as its node (its light then belongs to it)
            if a in lit and len(adj[a]) > 2 and not (b in lit and len(adj[b]) > 2):
                tnode = a
        other = a if tnode == b else b
        left = l['fwd'] if tnode == b else l['back']
        right = l['back'] if tnode == b else l['fwd']
        light = 0
        if tnode in lit and len(adj[tnode]) > 2 and left > 0:
            P, X = nodes[other], nodes[tnode]
            ang = math.degrees(math.atan2(X['y'] - P['y'], X['x'] - P['x'])) % 360
            light = 1 if (45 <= ang <= 135 or 225 <= ang <= 315) else 2
        l.update({'tnode': tnode, 'other': other, 'left': left, 'right': right, 'light': light})
    return nodes, order, links, adj, lit


# --- Writing the road data -----------------------------------------------------------------------------------------

def area_of(x, y):
    return min(max(int((y + 3000.0) // 750.0), 0), 7) * 8 + min(max(int((x + 3000.0) // 750.0), 0), 7)


def install(nodes, order, links, adj, base, dst):
    areas = {}
    for i in range(64):
        p = os.path.join(base, 'NODES%d.DAT' % i)
        if os.path.exists(p):
            areas[i] = Area(p)
    # the town's nodes: their area, their id there (after the area's own vehicle nodes)
    new_by_area = defaultdict(list)
    addr = {}
    for k in order:
        n = nodes[k]
        a = area_of(n['x'], n['y'])
        addr[k] = (a, areas[a].n_veh + len(new_by_area[a]))
        new_by_area[a].append(k)
    shift = {a: len(new_by_area[a]) for a in areas}

    def remap(t):
        ta, tid = t
        if ta in areas and tid >= areas[ta].n_veh:
            return ta, tid + shift[ta]
        return ta, tid
    # the town's navis: the area of their middle, else the nearest with room
    centre = {a: (-3000 + 750 * (a % 8) + 375, -3000 + 750 * (a // 8) + 375) for a in range(64)}
    room = {a: NAVI_CAP - areas[a].n_navi for a in areas}
    navi_new = defaultdict(list)
    navi_addr = {}
    for pair, l in links.items():
        P, X = nodes[l['other']], nodes[l['tnode']]
        mx, my = (P['x'] + X['x']) / 2, (P['y'] + X['y']) / 2
        a = area_of(mx, my)
        if room.get(a, 0) <= 0:
            a = min((b for b in areas if room[b] > 0), key=lambda b: math.dist(centre[b], (mx, my)))
        room[a] -= 1
        navi_addr[pair] = (a, areas[a].n_navi + len(navi_new[a]))
        navi_new[a].append(pair)
        d = math.hypot(X['x'] - P['x'], X['y'] - P['y']) or 1.0
        flags = (l['right'] << 11) | (l['left'] << 8)
        if l['light']:
            flags |= (1 << 14) | (l['light'] << 16)  # bit 14: the light belongs to the navi's own node
        l['navi'] = struct.pack('<hhHHbbI', int(round(mx * 8)), int(round(my * 8)), *addr[l['tnode']],
                                int(round((X['x'] - P['x']) / d * 100)), int(round((X['y'] - P['y']) / d * 100)), flags)
    os.makedirs(dst, exist_ok=True)
    for a, ar in areas.items():
        d = ar.data
        nodes_out, links_out, navil_out, len_out, inter_out = [], [], [], [], []

        def node_rec(x, y, z, heur, first, nid, width, flood, flags):
            return struct.pack('<IIhhhhHHHBBI', 0, 0, int(round(x * 8)), int(round(y * 8)), int(round(z * 8)), heur,
                               first, a, nid, width, flood, flags)

        def own(n, final_id):
            r = struct.unpack_from('<IIhhhhHHHBBI', d, n.raw)
            first = len(links_out)
            for k in range(n.links):
                li = n.link + k
                links_out.append(struct.pack('<HH', *remap(tuple(ar.links[li]))))
                navil_out.append(struct.pack('<H', ar.navi_links[li]))
                len_out.append(ar.lengths[li])
                inter_out.append(ar.inter[li] if ar.inter else 0)
            nodes_out.append(node_rec(n.x, n.y, n.z, r[5], first, final_id, r[9], r[10], r[11]))
        for n in ar.nodes[:ar.n_veh]:
            own(n, n.id)
        for k in new_by_area[a]:
            n = nodes[k]
            first = len(links_out)
            for m in adj[k]:
                pair = (k, m) if k < m else (m, k)
                links_out.append(struct.pack('<HH', *addr[m]))
                na, ni = navi_addr[pair]
                navil_out.append(struct.pack('<H', (na << 10) | ni))
                len_out.append(max(1, min(255, int(round(math.dist((n['x'], n['y']), (nodes[m]['x'], nodes[m]['y'])))))))
                inter_out.append(0)
            kinds = {links[(k, m) if k < m else (m, k)]['kind'] for m in adj[k]}
            kind = 'freeway' if 'freeway' in kinds else 'street'
            flags = (0xF0000 | (0x2000 if kind == 'freeway' else 0x1000)) | len(adj[k])
            nodes_out.append(node_rec(n['x'], n['y'], DECK_Z, 32766, first, addr[k][1], 0, FLOOD, flags))
        for n in ar.nodes[ar.n_veh:]:
            own(n, n.id + shift[a])
        navi_out = [d[ar.navi_off + 14 * i: ar.navi_off + 14 * (i + 1)] for i in range(ar.n_navi)]
        navi_out += [links[p]['navi'] for p in navi_new[a]]
        n_veh = ar.n_veh + shift[a]
        filler_off = 20 + 28 * ar.n_nodes + 14 * ar.n_navi + 4 * ar.n_links
        out = bytearray(struct.pack('<5I', n_veh + ar.n_ped, n_veh, ar.n_ped, len(navi_out), len(links_out)))
        out += b''.join(nodes_out) + b''.join(navi_out) + b''.join(links_out) + d[filler_off:filler_off + 768]
        out += b''.join(navil_out) + bytes(len_out) + bytes(inter_out) + d[-384:]
        open(os.path.join(dst, 'NODES%d.DAT' % a), 'wb').write(bytes(out))
    print('road data: %d town nodes, %d links into %d areas -> %s' % (len(order), len(links), len(areas), dst))
    return addr


# --- What the mod draws, the JSON, the map -------------------------------------------------------------------------

def lane_edges(l, nodes):
    """The asphalt's left and right edge offsets from the navi's line (m, > 0 right of the way to its node), and the
    lane dividers"""
    left, right = l['left'], l['right']
    if left and right:
        return -right * LANE, left * LANE, [i * LANE for i in range(-right + 1, left)]
    n = left or right
    return -n * LANE / 2, n * LANE / 2, [(-n / 2 + i) * LANE for i in range(1, n)]


def outputs(nodes, order, links, adj, lit, addr):
    town_txt = []
    js = {'deck_z': DECK_Z, 'lane_width': LANE, 'roads': [], 'junctions': [], 'links': []}
    jid = {}
    for k in order:
        if len(adj[k]) > 2:
            jid[k] = 'J%d' % (len(jid) + 1)
            js['junctions'].append({'id': jid[k], 'x': round(nodes[k]['x'], 1), 'y': round(nodes[k]['y'], 1),
                                    'roads': len(adj[k]), 'lights': k in lit, 'node': list(addr[k])})
    for r in ROADS:
        js['roads'].append({'name': r['name'], 'kind': r['kind'], 'lanes_along': r['fwd'], 'lanes_against': r['back'],
                            'points': [[round(p[0], 1), round(p[1], 1)] for p in r['line']]})
    for pair, l in links.items():
        P, X = nodes[l['other']], nodes[l['tnode']]
        dx, dy = X['x'] - P['x'], X['y'] - P['y']
        L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        rx, ry = uy, -ux                         # right of the way to the navi's node
        lo, hi, divs = lane_edges(l, nodes)
        z = DECK_Z + 0.06
        # asphalt (a little longer: no gaps at the joints)
        e = 1.0
        q = [(P['x'] - ux * e + rx * lo, P['y'] - uy * e + ry * lo), (X['x'] + ux * e + rx * lo, X['y'] + uy * e + ry * lo),
             (X['x'] + ux * e + rx * hi, X['y'] + uy * e + ry * hi), (P['x'] - ux * e + rx * hi, P['y'] - uy * e + ry * hi)]
        town_txt.append('Q ' + ' '.join('%.2f %.2f' % p for p in q))
        # edges white, the middle of a two-way road yellow, lane dividers dashed white
        for off in (lo, hi):
            town_txt.append('L %.2f %.2f %.2f %.2f 255 255 255' % (P['x'] + rx * off, P['y'] + ry * off, X['x'] + rx * off, X['y'] + ry * off))
        if l['left'] and l['right']:
            town_txt.append('L %.2f %.2f %.2f %.2f 255 200 0' % (P['x'], P['y'], X['x'], X['y']))
        for off in divs:
            if off == 0 and l['left'] and l['right']:
                continue
            town_txt.append('D %.2f %.2f %.2f %.2f' % (P['x'] + rx * off, P['y'] + ry * off, X['x'] + rx * off, X['y'] + ry * off))
        # a light: the stop line and the signal head over its lanes (the mod colours it by the junction's signal)
        if l['light']:
            s = 8.0
            sx, sy = X['x'] - ux * s, X['y'] - uy * s
            ja, jn = addr[l['tnode']]
            s0 = 0.0 if l['right'] else lo
            town_txt.append('S %.2f %.2f %.2f %.2f %d %d %d' % (sx + rx * s0, sy + ry * s0, sx + rx * hi, sy + ry * hi, ja, jn, l['light']))
        js['links'].append({'from': [round(P['x'], 1), round(P['y'], 1)], 'to': [round(X['x'], 1), round(X['y'], 1)],
                            'lanes_to': l['left'], 'lanes_from': l['right'], 'light': l['light'], 'road': l['road']})
    os.makedirs(os.path.join(REPO, 'docs'), exist_ok=True)
    json.dump(js, open(os.path.join(REPO, 'docs', 'town.json'), 'w'), indent=1)
    return town_txt, js, jid


def draw_map(nodes, links, adj, lit, jid, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, FancyArrow
    fig, ax = plt.subplots(figsize=(30, 13), dpi=110)
    ax.set_facecolor('#d9d9d6')
    colours = {'freeway': '#3b3f46', 'avenue': '#4a4f57', 'street': '#5a5f66', 'ramp': '#3b3f46', 'rural': '#6a5f50', 'round': '#5a5f66'}
    for pair, l in links.items():
        P, X = nodes[l['other']], nodes[l['tnode']]
        dx, dy = X['x'] - P['x'], X['y'] - P['y']
        L = math.hypot(dx, dy) or 1.0
        ux, uy, rx, ry = dx / L, dy / L, dy / L, -dx / L
        lo, hi, divs = lane_edges(l, nodes)
        e = 1.0
        poly = [(P['x'] - ux * e + rx * lo, P['y'] - uy * e + ry * lo), (X['x'] + ux * e + rx * lo, X['y'] + uy * e + ry * lo),
                (X['x'] + ux * e + rx * hi, X['y'] + uy * e + ry * hi), (P['x'] - ux * e + rx * hi, P['y'] - uy * e + ry * hi)]
        ax.add_patch(Polygon(poly, closed=True, color=colours[l['kind']], lw=0, zorder=2))
        if l['left'] and l['right']:
            ax.plot([P['x'], X['x']], [P['y'], X['y']], color='#f2c200', lw=0.8, zorder=3)
        for off in divs:
            if off == 0 and l['left'] and l['right']:
                continue
            ax.plot([P['x'] + rx * off, X['x'] + rx * off], [P['y'] + ry * off, X['y'] + ry * off], color='white', lw=0.4, ls=(0, (3, 3)), zorder=3)
    # direction arrows, one per ~80 m of each road and direction
    for r in ROADS:
        pts = r['nodes']
        acc = 40.0
        for p, q in zip(pts, pts[1:]):
            seg = math.dist(p, q)
            acc += seg
            if acc < 80.0 or seg < 3:
                continue
            acc = 0.0
            ux, uy = (q[0] - p[0]) / seg, (q[1] - p[1]) / seg
            mx, my = (p[0] + q[0]) / 2, (p[1] + q[1]) / 2
            rx, ry = uy, -ux
            for lanes, sgn in ((r['fwd'], 1), (r['back'], -1)):
                if not lanes:
                    continue
                off = (LANE * lanes / 2) if (r['fwd'] and r['back']) else 0.0
                cx, cy = mx + rx * off * sgn, my + ry * off * sgn
                ax.add_patch(FancyArrow(cx - ux * sgn * 7, cy - uy * sgn * 7, ux * sgn * 12, uy * sgn * 12, width=1.2,
                                        head_width=4.5, head_length=5, color='white', zorder=4, lw=0))
    for k, name in jid.items():
        n = nodes[k]
        ax.plot(n['x'], n['y'], 'o', ms=9 if k in lit else 6, color='#e53935' if k in lit else '#1e88e5', zorder=5,
                markeredgecolor='white', markeredgewidth=0.8)
        ax.text(n['x'] + 9, n['y'] + 9, name, fontsize=9, color='#111', zorder=6, weight='bold')
    for r in ROADS:
        line = r['line']
        street = r['name'].startswith('Rua')
        mid = (line[0][0] + 16, -2450) if street else line[len(line) // 2]
        ax.text(mid[0], mid[1] + (0 if street else 14), r['name'], fontsize=9, color='#0b3d91', ha='center', va='center',
                rotation=90 if street else 0, zorder=7, bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='none', alpha=0.8))
    ax.set_xlim(-60, 2860)
    ax.set_ylim(-2920, -1780)
    ax.set_aspect('equal')
    ax.set_xticks(range(0, 2801, 100))
    ax.set_yticks(range(-2900, -1799, 100))
    ax.tick_params(labelsize=8)
    ax.grid(True, color='white', lw=0.5, alpha=0.7)
    ax.set_title('Cidade de testes do Real Traffic Fix (coordenadas do mundo, m; z %.0f). Vermelho: cruzamento com semaforo, '
                 'azul: sem semaforo. Setas: sentido. Linha amarela: mao dupla.' % DECK_Z, fontsize=12)
    fig.tight_layout()
    fig.savefig(path)
    print('map ->', path)


def main():
    game = next((a for a in sys.argv[1:] if not a.startswith('--')), GAME)
    nodes, order, links, adj, lit = build()
    mod_dir = os.path.join(game, 'modloader', 'RealTrafficFix Paths', 'gta3.img')
    base = os.path.join(game, 'RealTrafficFix_paths_base')
    if not os.path.isdir(base):
        # the road data before the town: the mod's files, the game's own for the areas it doesn't change
        os.makedirs(base)
        for i in range(64):
            src = os.path.join(mod_dir, 'NODES%d.DAT' % i)
            if not os.path.exists(src):
                src = os.path.join(game, 'data', 'paths', 'NODES%d.DAT' % i)
            shutil.copy2(src, base)
        print('the road data before the town kept in', base)
    addr = {k: (0, 0) for k in order}
    if '--no-install' not in sys.argv:
        addr = install(nodes, order, links, adj, base, mod_dir)
    town_txt, js, jid = outputs(nodes, order, links, adj, lit, addr)
    open(os.path.join(game, 'scripts', 'RealTrafficFix.town.txt'), 'w').write('Z %.2f\n' % DECK_Z + '\n'.join(town_txt) + '\n')
    draw_map(nodes, links, adj, lit, jid, os.path.join(REPO, 'docs', 'town_map.png'))
    lanes_km = sum(math.dist((nodes[l['a']]['x'], nodes[l['a']]['y']), (nodes[l['b']]['x'], nodes[l['b']]['y'])) *
                   (l['fwd'] + l['back']) for l in links.values()) / 1000
    print('%d roads, %d nodes, %d links, %d junctions (%d with lights), %.1f lane-km' %
          (len(ROADS), len(order), len(links), len(jid), sum(1 for k in jid if k in lit), lanes_km))


if __name__ == '__main__':
    main()
