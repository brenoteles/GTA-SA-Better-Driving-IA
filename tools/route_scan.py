"""Road scan analysis ("laser car"): where the game's lane data along the scanned route makes the AI change lanes for
nothing, and where its lanes don't sit on the real road.

usage: python route_scan.py <autotest folder with scan.txt> [--write <RealTrafficFix.routefixes.txt>]

scan.txt (AutoTest Scenario = scan), every 2 m along the route:
  S x y z dx dy curr next lanes opp linkX linkY oneWayLaneOffset width speedClass currAdj nextAdj carLat laneCur laneNext
    laneState lane0X lane0Y linkDirX linkDirY
      (x y z: the car; dx dy: its heading; carLat: the car across the link, m to the right of the link line;
       lane0X/Y: centre of the game's lane 0 there)
  R 121 ground heights across the road from the car, -30..+30 m every 0.5 m to the right ("x": nothing)

Findings:
  FORCED  the game changed the car's lane while the mod wasn't changing lanes (a lane change it had to make)
  JUMP    between one road link and the next, the lane the car is in moves sideways (the game steers it across)
  DIP     a link with fewer lanes than before, and they come back within ~120 m (the AI squeezes in and out)
  OFF     the game's rightmost/leftmost lane outside the real road (kerbs, barriers, drops measured by the scan)
Corrections written: lanes for DIP links ("A B lanes"), and a sideways shift of the link position for JUMP/OFF on
one-way links ("A B shift dx dy").
"""
import math
import os
import statistics
import sys

from PIL import Image, ImageDraw, ImageFont

W = 5.4          # the game's lane spacing
STEP_OK = 0.12   # m between readings 0.5 m apart on the same surface (kerbs are 0.15+)
MAX_DEV = 1.0    # m above/below the road at the car


def parse(path):
    samples, cur = [], None
    with open(path, encoding='utf-8', errors='replace') as f:
        for line in f:
            p = line.split()
            if not p:
                continue
            if p[0] == 'S' and len(p) >= 25 and 'nan' not in line:
                cur = {'x': float(p[1]), 'y': float(p[2]), 'z': float(p[3]), 'dx': float(p[4]), 'dy': float(p[5]),
                       'curr': p[6], 'next': p[7], 'lanes': int(p[8]), 'opp': int(p[9]), 'lx': float(p[10]), 'ly': float(p[11]),
                       'ow': float(p[12]), 'width': int(p[13]), 'cls': int(p[14]), 'currAdj': int(p[15]), 'nextAdj': int(p[16]),
                       'carLat': float(p[17]), 'laneCur': int(p[18]), 'laneNext': int(p[19]), 'state': int(p[20]),
                       'l0x': float(p[21]), 'l0y': float(p[22]), 'ldx': float(p[23]), 'ldy': float(p[24])}
            elif p[0] == 'R' and cur is not None:
                h = [None if t == 'x' or 'nan' in t else float(t.split('/')[0]) for t in p[1:122]]
                if len(h) == 121:
                    cur['h'] = h
                    samples.append(cur)
                cur = None
    total = 0.0
    for i, s in enumerate(samples):
        if i:
            total += min(math.hypot(s['x'] - samples[i - 1]['x'], s['y'] - samples[i - 1]['y']), 20.0)
        s['dist'] = total
    return samples


def measure(s):
    """Road edges across the car (m to the right of the car), walking out from under it"""
    h = s['h']
    if h[60] is None:
        return None
    h0 = h[60]

    def walk(step):
        i = 60
        while 0 <= i + step <= 120:
            a, b = h[i], h[i + step]
            if b is None or abs(b - a) > STEP_OK or abs(b - h0) > MAX_DEV:
                break
            i += step
        return -30.0 + i * 0.5 + 0.25 * step

    return walk(-1), walk(1)


def lane_center(s, lane):
    """World position of the centre of 'lane' on the sample's link (at the car)"""
    rx, ry = s['ldy'], -s['ldx']
    return s['l0x'] + rx * lane * W, s['l0y'] + ry * lane * W


def analyze(folder):
    samples = parse(os.path.join(folder, 'scan.txt'))
    for s in samples:
        # Usable for geometry: the car lined up with the link (not still turning onto it) and in its lane
        cosA = s['dx'] * s['ldx'] + s['dy'] * s['ldy']
        s['valid'] = cosA > math.cos(math.radians(12)) and abs(s['carLat'] - (s['ow'] + s['laneNext']) * W) < 3.0
        m = measure(s)
        if m is None:
            s['left'] = s['right'] = None
            continue
        # In link coordinates (m right of the link line): car's lateral + the measured offsets (car ~ parallel to link)
        s['left'], s['right'] = s['carLat'] + m[0], s['carLat'] + m[1]
        s['lo'] = (s['ow'] - 0.5) * W
        s['hi'] = (s['ow'] + s['lanes'] - 0.5) * W

    links = []
    for s in samples:
        key = (s['curr'], s['next'])
        if not links or links[-1]['key'] != key:
            links.append({'key': key, 'samples': [], 'lanes': s['lanes'], 'opp': s['opp'], 'cls': s['cls'],
                          'currAdj': s['currAdj'], 'nextAdj': s['nextAdj'], 'start': s['dist'], 'ow': s['ow']})
        links[-1]['samples'].append(s)
    for L in links:
        ss = [x for x in L['samples'] if x['left'] is not None and x['valid']]
        L['end'] = L['samples'][-1]['dist']
        L['ok'] = len(ss) >= 3
        if L['ok']:
            L['left'] = statistics.median(x['left'] for x in ss)
            L['right'] = statistics.median(x['right'] for x in ss)
            L['lo'], L['hi'] = ss[0]['lo'], ss[0]['hi']

    findings, fixes = [], []
    # FORCED: the game's lane changed while the mod kept its lane (state 0 keep, 2 overtaking = in a lane)
    for i in range(1, len(samples)):
        a, b = samples[i - 1], samples[i]
        if a['laneNext'] != b['laneNext'] and a['state'] in (0, 2) and b['state'] in (0, 2):
            newLink = (a['curr'], a['next']) != (b['curr'], b['next'])
            findings.append(('FORCED', b['dist'], f"lane {a['laneNext']}->{b['laneNext']} at ({b['x']:.0f}, {b['y']:.0f}) "
                             f"{'entering ' + b['curr'] + '->' + b['next'] if newLink else 'mid-link'}, lanes {a['lanes']}->{b['lanes']}"))
    # JUMP: the car's lane moves sideways from one link to the next
    for i in range(1, len(samples)):
        a, b = samples[i - 1], samples[i]
        if (a['curr'], a['next']) == (b['curr'], b['next']):
            continue
        # Links at an angle: the game joins them with a curve, the straight lines don't meet. Only (nearly) straight on
        if a['ldx'] * b['ldx'] + a['ldy'] * b['ldy'] < math.cos(math.radians(8)) or not (a['valid'] and b['valid']):
            continue
        lane = b['laneNext']
        if lane >= a['lanes'] or lane >= b['lanes']:
            continue
        ax, ay = lane_center(a, lane)
        bx, by = lane_center(b, lane)
        rx, ry = b['dy'], -b['dx']   # right of the car
        jump = (bx - ax) * rx + (by - ay) * ry
        L = next((x for x in links if x['key'] == (b['curr'], b['next'])), None)
        P = next((x for x in links if x['key'] == (a['curr'], a['next'])), None)
        if L is not None:
            L['entryJump'] = jump
        if P is not None:
            P['exitJump'] = jump
        if abs(jump) > 2.0:
            b['jump'] = jump
            findings.append(('JUMP', b['dist'], f"lane {lane} moves {jump:+.1f} m sideways entering {b['curr']}->{b['next']} "
                             f"at ({b['x']:.0f}, {b['y']:.0f}) (lanes {a['lanes']}->{b['lanes']}, offset {a['ow']:+.1f}->{b['ow']:+.1f})"))
    # A link whose lanes stick out sideways: jumps out entering it and back leaving it (opposite, about the same): moved
    # back in line with its neighbours (one-way links only: the position is shared by both directions)
    for L in links:
        j1, j2 = L.get('entryJump'), L.get('exitJump')
        if j1 is None or j2 is None or L['opp'] != 0:
            continue
        if abs(j1) > 1.5 and abs(j2) > 1.5 and (j1 > 0) != (j2 > 0) and abs(j1 + j2) < 1.2:
            shift = -(j1 - j2) / 2.0      # in the car's right direction
            s0 = L['samples'][0]
            rx, ry = s0['dy'], -s0['dx']
            fixes.append((L['key'][0], L['key'][1], f"shift {rx * shift:.2f} {ry * shift:.2f}",
                          f"lanes stick out {j1:+.1f} m and back {j2:+.1f} m: moved {shift:+.1f} m in line"))
    # DIP: lanes drop and come back within 120 m, road not narrower
    for k in range(1, len(links)):
        L, P = links[k], links[k - 1]
        if L['lanes'] >= P['lanes']:
            continue
        back = None
        for m in range(k + 1, len(links)):
            if links[m]['start'] - L['start'] > 120.0:
                break
            if links[m]['lanes'] >= P['lanes']:
                back = m
                break
        narrower = L['ok'] and P['ok'] and (L['right'] - L['left']) < (P['right'] - P['left']) - 1.5
        if back is not None and not narrower:
            for m in range(k, back):
                if links[m]['lanes'] < P['lanes']:
                    fixes.append((links[m]['key'][0], links[m]['key'][1], str(P['lanes']), f"dip {links[m]['lanes']} -> {P['lanes']}"))
            findings.append(('DIP', L['start'], f"{P['lanes']} -> {L['lanes']} lanes for {links[back]['start'] - L['start']:.0f} m at "
                             f"{L['key'][0]}->{L['key'][1]}"))
    # OFF: lanes outside the measured road (only where both edges were measured close: kerbs/barriers, not open ground)
    for L in links:
        if not L['ok']:
            continue
        outR = L['hi'] - L['right']
        outL = L['left'] - L['lo']
        width = L['right'] - L['left']
        if (outR > 1.5 or outL > 1.5) and width < 40.0:
            findings.append(('OFF', L['start'], f"{L['key'][0]}->{L['key'][1]}: game lanes {L['lo']:+.1f}..{L['hi']:+.1f} m, "
                             f"road {L['left']:+.1f}..{L['right']:+.1f} m ({L['lanes']} lanes, opp {L['opp']})"))
            # One-way and the road is wide enough: centre the lanes on it
            if L['opp'] == 0 and width >= (L['hi'] - L['lo']) - 0.5:
                shift = (L['left'] + L['right']) / 2 - (L['lo'] + L['hi']) / 2
                s0 = L['samples'][0]
                rx, ry = s0['ldy'], -s0['ldx']
                fixes.append((L['key'][0], L['key'][1], f"shift {rx * shift:.2f} {ry * shift:.2f}",
                              f"lanes {shift:+.1f} m to sit on the road"))
    findings.sort(key=lambda f: f[1])
    return samples, links, findings, fixes


def plot(samples, links, findings, folder):
    if not samples:
        return None
    total = samples[-1]['dist'] or 1.0
    Wd, H = 3200, 520
    img = Image.new('RGB', (Wd, H), (24, 24, 28))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype('consola.ttf', 14)
    except OSError:
        font = ImageFont.load_default()
    X = lambda dist: 50 + (Wd - 70) * dist / total
    Y = lambda lat: 260 + lat * 8   # m right of the link line -> down
    d.text((50, 6), 'across the road along the route (m right of the game\'s link line): measured road (grey), game lanes '
           '(blue), car (white); FORCED lane changes (red), JUMPs (yellow)', font=font, fill=(220, 220, 220))
    for s in samples:
        x = X(s['dist'])
        if s.get('left') is not None:
            d.line([(x, Y(max(s['left'], -30))), (x, Y(min(s['right'], 30)))], fill=(70, 70, 76))
        d.line([(x, Y(s['lo'])), (x, Y(s['hi']))], fill=(60, 110, 220))
        d.point((x, Y(s['carLat'])), fill=(255, 255, 255))
    for kind, dist, _ in findings:
        col = {'FORCED': (255, 70, 70), 'JUMP': (255, 220, 60), 'DIP': (60, 220, 90), 'OFF': (255, 150, 40)}[kind]
        x = X(dist)
        d.line([(x, 30), (x, 50)], fill=col, width=3)
    for km in range(0, int(total / 1000) + 1):
        d.text((X(km * 1000), H - 20), f'{km} km', font=font, fill=(160, 160, 160))
    for m in (-20, -10, 0, 10, 20):
        d.text((5, Y(m) - 8), f'{m:+d}', font=font, fill=(160, 160, 160))
    out = os.path.join(folder, 'route_scan.png')
    img.save(out)
    return out


def main():
    folder = sys.argv[1]
    samples, links, findings, fixes = analyze(folder)
    counts = {}
    for f in findings:
        counts[f[0]] = counts.get(f[0], 0) + 1
    print(f"{len(samples)} scan lines, {len(links)} road links, {samples[-1]['dist'] / 1000 if samples else 0:.1f} km; " +
          ', '.join(f'{k} {v}' for k, v in sorted(counts.items())))
    for kind, dist, text in findings:
        print(f'{kind:6} {dist:7.0f} m  {text}')
    print(f'{len(fixes)} corrections')
    for a, b, what, why in fixes:
        print(f'  {a} {b} {what}   # {why}')
    png = plot(samples, links, findings, folder)
    if png:
        print(png)
    if '--write' in sys.argv:
        out = sys.argv[sys.argv.index('--write') + 1]
        with open(out, 'w', encoding='ascii', newline='\r\n') as f:
            f.write('# Real Traffic Fix: lane data corrections from a road scan (tools\\route_scan.py)\n')
            f.write('# nodeA nodeB lanes         = lanes towards node B on the road link A->B (area:index)\n')
            f.write('# nodeA nodeB shift dx dy   = move the link position (m) so its lanes sit on the road\n')
            for a, b, what, why in fixes:
                f.write(f'{a} {b} {what}   # {why}\n')
        print('written', out)


if __name__ == '__main__':
    main()
