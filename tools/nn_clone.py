"""The neural driver's first weights by imitation (behaviour cloning): a small network (ProvingGround.h, mode neural:
18 inputs -> 10 tanh -> wheel and pedal) fitted to laps already driven, so the GA polishes a driver instead of
starting from noise.

  python tools/nn_clone.py [--human-weight 3] [--top 5] [--apply]

Teachers: the human's laps (car A while Driver = human, mode "human" in the trace) and the best AI laps (the --top
finishers of every generation in ga_history*.csv), from the ga_trace/gen_*.csv files that log the network's inputs
(in0..in17) next to what the car did (steer, gas, brake). Targets: the wheel / the steering lock, gas - brake.
Writes scripts\\proving\\nn_seed.txt (the neural island's genomes start from it); --apply also puts it, a little varied,
into the neural genomes already in ga_state.txt (game closed).
"""
import argparse
import csv
import glob
import os
import random

import numpy as np

GAME_DIR = os.environ.get('GTA_SA_DIR', r'C:\Users\death\Games\GTA\GTA San Andreas')
PROVING = os.path.join(GAME_DIR, 'scripts', 'proving')
N_IN, N_HID = 18, 10
N_W = (N_IN + 1) * N_HID + (N_HID + 1) * 2
LOCK = 30.0 * np.pi / 180.0       # the steering lock (rad) of the Infernus, Solair and Super GT


def teachers(top):
    """(generation, car) of the best finishers of every generation."""
    best = set()
    for path in sorted(glob.glob(os.path.join(PROVING, 'ga_history*.csv'))):
        with open(path) as f:
            for r in csv.DictReader(f):
                if r.get('mode') != 'human' and r.get('result') == 'end' and int(r['rank']) <= top:
                    best.add((int(r['generation']), r['car']))
    return best


def trace_teachers(rows, top):
    """A generation not ranked yet (stopped before its end): the cars that got furthest with < 2% of their samples
    outside the lane."""
    far, outside, n = {}, {}, {}
    for r in rows:
        if r['mode'] == 'human':
            continue
        k = r['car']
        far[k] = max(far.get(k, 0.0), float(r['s']))
        n[k] = n.get(k, 0) + 1
        outside[k] = outside.get(k, 0) + (r.get('margin') not in (None, '') and float(r['margin']) < 0)
    ok = [k for k in far if outside[k] <= 0.02 * n[k]]
    return set(sorted(ok, key=lambda k: -far[k])[:top])


def samples(top, human_weight):
    best = teachers(top)
    X, Y, W, n_h, n_ai = [], [], [], 0, 0
    for path in sorted(glob.glob(os.path.join(PROVING, 'ga_trace', 'gen_*.csv'))):
        with open(path) as f:
            reader = csv.DictReader(f)
            if 'in17' not in (reader.fieldnames or []):
                continue
            rows = list(reader)
        gen = int(rows[0]['generation']) if rows else -1
        if not any(g == gen for g, _ in best):
            best |= {(gen, k) for k in trace_teachers(rows, top)}
        for r in rows:
            is_h = r['mode'] == 'human'
            if not is_h and (int(r['generation']), r['car']) not in best:
                continue
            if float(r['t']) < 3.5 or float(r['speed_kmh']) < 5.0:   # (the countdown, standing still)
                continue
            try:
                x = [float(r[f'in{i}']) for i in range(N_IN)]
                y = [np.clip(float(r['steer']) / LOCK, -1, 1), np.clip(float(r['gas']) - float(r['brake']), -1, 1)]
            except (ValueError, KeyError):
                continue
            X.append(x)
            Y.append(y)
            W.append(human_weight if is_h else 1.0)
            n_h += is_h
            n_ai += not is_h
    return np.array(X, np.float32), np.array(Y, np.float32), np.array(W, np.float32), n_h, n_ai


def train(X, Y, W, epochs=4000, lr=0.01, seed=1):
    rng = np.random.default_rng(seed)
    W1 = rng.normal(0, 0.3, (N_HID, N_IN)).astype(np.float32)
    b1 = np.zeros(N_HID, np.float32)
    W2 = rng.normal(0, 0.3, (2, N_HID)).astype(np.float32)
    b2 = np.zeros(2, np.float32)
    params = [W1, b1, W2, b2]
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    w = (W / W.sum())[:, None]
    for t in range(1, epochs + 1):
        H = np.tanh(X @ W1.T + b1)
        O = np.tanh(H @ W2.T + b2)
        dO = 2 * (O - Y) * w * (1 - O * O)
        gW2, gb2 = dO.T @ H, dO.sum(0)
        dH = (dO @ W2) * (1 - H * H)
        gW1, gb1 = dH.T @ X, dH.sum(0)
        for i, (p, g) in enumerate(zip(params, (gW1, gb1, gW2, gb2))):
            m[i] = 0.9 * m[i] + 0.1 * g
            v[i] = 0.999 * v[i] + 0.001 * g * g
            p -= lr * (m[i] / (1 - 0.9 ** t)) / (np.sqrt(v[i] / (1 - 0.999 ** t)) + 1e-8)
    O = np.tanh(np.tanh(X @ W1.T + b1) @ W2.T + b2)
    r2 = [1 - np.average((O[:, k] - Y[:, k]) ** 2, weights=W) / max(np.average((Y[:, k] - np.average(Y[:, k], weights=W)) ** 2, weights=W), 1e-9)
          for k in range(2)]
    return params, r2


def flatten(W1, b1, W2, b2):
    """In ProvingGround.h's order: each hidden unit's 18 weights then its bias, then each output's 10 weights and bias."""
    w = []
    for j in range(N_HID):
        w += list(W1[j]) + [b1[j]]
    for k in range(2):
        w += list(W2[k]) + [b2[k]]
    assert len(w) == N_W
    return w


def apply_to_state(w, sigma=0.03):
    path = os.path.join(PROVING, 'ga_state.txt')
    lines = open(path, encoding='utf-8').read().splitlines()
    out, last_neural, n = [], False, 0
    for line in lines:
        if line.startswith('nn ') and last_neural:
            line = 'nn ' + ' '.join(f'{x + random.gauss(0, sigma):.5g}' for x in w)
            n += 1
        out.append(line)
        if line.startswith('ind '):
            last_neural = line.split()[1] == 'neural'
    open(path, 'w', encoding='utf-8').write('\n'.join(out) + '\n')
    print(f'{path}: {n} neural genomes set from the clone')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--human-weight', type=float, default=3.0)
    ap.add_argument('--top', type=int, default=5)
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    X, Y, W, n_h, n_ai = samples(a.top, a.human_weight)
    if len(X) < 200:
        raise SystemExit(f'too few samples ({len(X)})')
    params, r2 = train(X, Y, W)
    print(f'{len(X)} samples ({n_h} human x{a.human_weight:g}, {n_ai} from the best AIs); fit R2: wheel {r2[0]:.3f}, pedal {r2[1]:.3f}')
    w = flatten(*params)
    seed = os.path.join(PROVING, 'nn_seed.txt')
    open(seed, 'w').write(' '.join(f'{x:.6g}' for x in w) + '\n')
    print(seed)
    if a.apply:
        apply_to_state(w)
