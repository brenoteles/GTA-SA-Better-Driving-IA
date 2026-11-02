"""
GTA SA drivers and cars for SUMO: the mod's driver profiles (RealTrafficFix.ini) x the cars the game spawns
(cargrp.dat, vehicles.ide, handling.cfg, and the braking the mod measured in the game) as one vTypeDistribution.

usage: python sumo_profiles.py <game dir> <mod ini> <out vtypes.add.xml> [variant]

Variants (what-ifs for the AI, the same trips on each):
  base        the profiles as in the ini
  nokeep      nobody keeps right (KeepRight 0)
  polite      lane changes cooperate more (MOBIL politeness up, less assertive)
  gap         city time gap x1.5 (CityGapFactor)
  junction    bigger gaps accepted at junctions (jmTimegapMinor +0.5 s)
  calm        everybody one profile calmer (a traffic mix without the fast ones)

Mapping (see docs/sensores.html):
  car following: IDM (the mod's IDM+), tau = TimeGap x CityGapFactor, accel = min(driver, car), decel = driver,
    emergencyDecel = what the mod measured for the model (limits.txt), minGap, speedFactor = SpeedMult (+-6% per driver),
    maxSpeed = car top speed x TopSpeedUse
  lane change (LC2013): lcKeepRight from KeepRight, lcSpeedGain from OvertakeSpeedRatio, lcAssertive /
    lcCooperative / lcImpatience from Aggression
  junctions: jmTimegapMinor and yellow-light behaviour from Aggression, jmDriveAfterRedTime for RunRedLights
  not modelled: corner grip (SUMO has no lateral dynamics; only turn speeds at junctions from netconvert),
    LaneChangeInterval (no per-driver minimum), LightApproachSpeed
"""
import configparser
import os
import re
import sys
from collections import Counter

PROFILES = ['Calm', 'Safe', 'Normal', 'Fast', 'Aggressive']
# handling.cfg fEngineAcceleration -> m/s2: x 0.00004 into game units (cHandlingDataMgr::ConvertDataToGameUnits,
# 0x6F5080), x 2500 (50 Hz ticks squared), x 2 driven wheels = 0.2 (Gemini; matches the mod's 6 m/s2 full throttle for
# the quickest cars, Infernus 30 -> 6)
ENGINE_TO_MS2 = 0.2


def read_ini(path):
    cp = configparser.ConfigParser(inline_comment_prefixes=(';',), strict=False)
    with open(path, encoding='latin-1') as f:
        cp.read_string(f.read())
    prof = {}
    for p in PROFILES:
        s = cp['Profile.' + p]
        prof[p] = {k: float(s[k]) for k in ('Chance', 'SpeedMult', 'Acceleration', 'Deceleration', 'TimeGap', 'MinGap',
                                            'TopSpeedUse', 'OvertakeSpeedRatio', 'Aggression', 'KeepRight', 'RunRedLights')}
    glob = {}
    for sec in cp.sections():
        for k in ('CityGapFactor', 'HighwayGapFactor'):
            if k in cp[sec]:
                glob[k] = float(cp[sec][k].split()[0])
    return prof, glob


def read_cars(game):
    data = os.path.join(game, 'data')
    # spawn weight: how many population groups list the model
    weight = Counter()
    with open(os.path.join(data, 'cargrp.dat'), encoding='latin-1') as f:
        for line in f:
            line = line.split('#')[0].strip()
            for name in re.split(r'[,\s]+', line):
                if name:
                    weight[name.lower()] += 1
    ide = {}
    section = None
    with open(os.path.join(data, 'vehicles.ide'), encoding='latin-1') as f:
        for line in f:
            t = line.split('#')[0].strip()
            if t in ('cars', 'end'):
                section = t
                continue
            if section != 'cars' or not t:
                continue
            c = [x.strip() for x in t.split(',')]
            if len(c) < 8 or c[3] != 'car':
                continue
            ide[c[1].lower()] = {'id': int(c[0]), 'handling': c[4].upper(), 'anims': c[6].lower(), 'class': c[7].lower()}
    hand = {}
    with open(os.path.join(data, 'handling.cfg'), encoding='latin-1') as f:
        for line in f:
            c = line.split()
            if len(c) > 20 and re.match(r'^[A-Z0-9_]+$', c[0]):
                try:
                    hand[c[0]] = {'mass': float(c[1]), 'maxVel': float(c[12]), 'engineAccel': float(c[13]),
                                  'brake': float(c[17])}
                except ValueError:
                    pass
    measured = {}
    lim = os.path.join(game, 'scripts', 'RealTrafficFix.limits.txt')
    if os.path.exists(lim):
        with open(lim) as f:
            for line in f:
                c = line.split()
                if len(c) >= 3:
                    measured[int(c[0])] = (float(c[1]), float(c[2]))  # lateral g, braking m/s2
    cars = []
    for name, w in weight.items():
        v = ide.get(name)
        if not v or v['handling'] not in hand:
            continue
        h = hand[v['handling']]
        heavy = v['anims'] in ('truck', 'bus', 'coach') or v['class'] == 'big' or h['mass'] >= 3500
        vclass = 'bus' if v['anims'] in ('bus', 'coach') else 'truck' if heavy else 'passenger'
        cars.append({'name': name, 'id': v['id'], 'weight': w, 'vclass': vclass, 'mass': h['mass'],
                     'top': h['maxVel'] / 3.6, 'accel': max(0.8, min(h['engineAccel'] * ENGINE_TO_MS2, 6.0)),
                     'brake': measured.get(v['id'], (None, None))[1] or min(h['brake'] * 1.2, 10.0)})
    return cars


def car_groups(cars):
    """A few car classes (one vType per profile x class keeps the distribution small)"""
    groups = {}
    for c in cars:
        if c['vclass'] != 'passenger':
            key = c['vclass']
        elif c['top'] * 3.6 >= 190:
            key = 'sports'
        elif c['mass'] >= 1900:
            key = 'suv'
        else:
            key = 'car'
        groups.setdefault(key, []).append(c)
    out = {}
    for k, cs in groups.items():
        w = sum(c['weight'] for c in cs)
        avg = lambda f: sum(c[f] * c['weight'] for c in cs) / w
        out[k] = {'weight': w, 'top': avg('top'), 'accel': avg('accel'), 'brake': avg('brake'),
                  'vclass': cs[0]['vclass'], 'models': len(cs)}
    return out


def apply_variant(prof, glob, variant):
    prof = {p: dict(v) for p, v in prof.items()}
    glob = dict(glob)
    if variant == 'nokeep':
        for v in prof.values():
            v['KeepRight'] = 0
    elif variant == 'gap':
        glob['CityGapFactor'] = glob.get('CityGapFactor', 0.7) * 1.5
    elif variant == 'calm':
        order = PROFILES
        chances = [prof[p]['Chance'] for p in order]
        # the chance of each profile goes to the one calmer than it
        new = [chances[0] + chances[1]] + chances[2:] + [0]
        for p, c in zip(order, new):
            prof[p]['Chance'] = c
    return prof, glob


def vtype_xml(pname, p, gname, g, glob, variant):
    aggr = p['Aggression']
    tau = max(p['TimeGap'] * glob.get('CityGapFactor', 0.7), 0.2)
    keep = {2: 3.0, 1: 1.0, 0: 0.0}.get(int(p['KeepRight']), 0.0)
    # OvertakeSpeedRatio 0.65 (overtakes only much slower cars) .. 0.97 (anything slower): SUMO speed gain 0.3 .. 2
    speedgain = 0.3 + (p['OvertakeSpeedRatio'] - 0.65) / 0.32 * 1.7
    if int(p['KeepRight']) == 2:
        speedgain = 0.0  # always on the right: the mod never overtakes with it
    coop = 1.0 - 0.6 * aggr
    assertive = 1.0 + 1.5 * aggr
    gap_minor = 1.5 - 0.8 * aggr
    if variant == 'polite':
        coop, assertive = min(coop + 0.3, 1.0), max(assertive - 0.5, 1.0)
    if variant == 'junction':
        gap_minor += 0.5
    heavy = g['vclass'] != 'passenger'
    accel = min(p['Acceleration'], g['accel'])
    decel = min(p['Deceleration'], g['brake'] * 0.6)
    length = {'passenger': 4.8, 'truck': 9.0, 'bus': 12.0}[g['vclass']]
    attrs = {
        'id': f'{pname}_{gname}', 'vClass': g['vclass'], 'length': length, 'width': 2.5 if heavy else 1.9,
        'carFollowModel': 'IDM', 'accel': accel, 'decel': decel, 'emergencyDecel': max(g['brake'], decel + 1.0),
        'apparentDecel': decel, 'tau': tau, 'minGap': p['MinGap'], 'delta': 4,
        'speedFactor': f"norm({p['SpeedMult']:.2f},{p['SpeedMult'] * 0.035:.3f})", 'maxSpeed': g['top'] * p['TopSpeedUse'],
        'laneChangeModel': 'LC2013', 'lcKeepRight': keep, 'lcSpeedGain': speedgain, 'lcCooperative': coop,
        'lcAssertive': assertive, 'lcImpatience': aggr, 'lcStrategic': 1.0,
        'jmTimegapMinor': gap_minor, 'jmDriveAfterYellowTime': 2.0 * aggr,
        'jmDriveAfterRedTime': 3.0 if p['RunRedLights'] else -1, 'impatience': aggr * 0.5,
    }
    fmt = lambda v: f'{v:.3f}'.rstrip('0').rstrip('.') if isinstance(v, float) else str(v)
    return '    <vType ' + ' '.join(f'{k}="{fmt(v)}"' for k, v in attrs.items()) + '/>'


def main(game, ini, out, variant='base'):
    prof, glob = read_ini(ini)
    prof, glob = apply_variant(prof, glob, variant)
    groups = car_groups(read_cars(game))
    total_car = sum(g['weight'] for g in groups.values())
    total_prof = sum(p['Chance'] for p in prof.values())
    lines = ['<additional>', '  <vTypeDistribution id="gta">']
    for pname, p in prof.items():
        if p['Chance'] <= 0:
            continue
        for gname, g in groups.items():
            prob = p['Chance'] / total_prof * g['weight'] / total_car
            lines.append(vtype_xml(pname, p, gname, g, glob, variant).replace('/>', f' probability="{prob:.4f}"/>'))
    lines += ['  </vTypeDistribution>', '</additional>']
    with open(out, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    for gname, g in sorted(groups.items()):
        print(f"{gname:10s} {g['models']:3d} models, weight {g['weight'] / total_car:.0%}, top {g['top'] * 3.6:.0f} km/h, "
              f"accel {g['accel']:.1f} m/s2, brake {g['brake']:.1f} m/s2")


if __name__ == '__main__':
    main(*sys.argv[1:5])
