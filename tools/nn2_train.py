"""Neural2's central network by imitation (behaviour cloning + DAgger), for the proving ground's GA (ProvingGround.h,
mode neural2: 63 sensors -> 24 tanh -> wheel, pedal).

  python tools/nn2_train.py [--top 8] [--epochs 40]          train once on every generation logged so far
  python tools/nn2_train.py --loop 4                          retrain after each new generation, 4 times, then turn
                                                              the evolution strategies on (Nn2Es = 1, ini pushed)

Data (scripts\\proving\\ga_trace\\gen_*.csv, the columns n0..n62): the best laps of pursuit / rpp / blend (the --top
finishers of each generation, by ga_history*.csv; a generation not ranked yet: the furthest with < 2% outside), with
what they did (steer, gas - brake); and every neural2 lap with what pursuit would have done there (teacher_steer,
teacher_pedal: DAgger, so it learns from its own mistakes). Starts from the current network if there is one.
Writes scripts\\proving\\nn2_central.txt (the game reloads it at the next generation).
"""
import argparse
import csv
import glob
import os
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_INI = os.path.normpath(os.path.join(HERE, '..', 'RealTrafficFix2', 'RealTrafficFix.proving.ini'))
GAME_DIR = os.environ.get('GTA_SA_DIR', r'C:\Users\death\Games\GTA\GTA San Andreas')
PROVING = os.path.join(GAME_DIR, 'scripts', 'proving')
THETA = os.path.join(PROVING, 'nn2_central.txt')
N_IN, N_HID = 63, 24
N_W = (N_IN + 1) * N_HID + (N_HID + 1) * 2
LOCK = 30.0 * np.pi / 180.0            # the steering lock of the test cars (handling: 30 degrees)
TEACHERS = ('pursuit', 'rpp', 'blend')


def ranked(top):
    best = set()
    for path in sorted(glob.glob(os.path.join(PROVING, 'ga_history*.csv'))):
        with open(path) as f:
            for r in csv.DictReader(f):
                if r.get('mode') in TEACHERS and r.get('result') == 'end' and int(r['rank']) <= 60:
                    best.add((int(r['generation']), r['car'], int(r['rank'])))
    by_gen = {}
    for g, car, rank in best:
        by_gen.setdefault(g, []).append((rank, car))
    return {(g, car) for g, lst in by_gen.items() for _, car in sorted(lst)[:top]}


def furthest(rows, top):
    far, out, n = {}, {}, {}
    for r in rows:
        if r['mode'] not in TEACHERS:
            continue
        k = r['car']
        far[k] = max(far.get(k, 0.0), float(r['s']))
        n[k] = n.get(k, 0) + 1
        out[k] = out.get(k, 0) + (r.get('margin') not in (None, '') and float(r['margin']) < 0)
    ok = [k for k in far if out[k] <= 0.02 * n[k]]
    return set(sorted(ok, key=lambda k: -far[k])[:top])


def completed_files():
    files = sorted(glob.glob(os.path.join(PROVING, 'ga_trace', 'gen_*.csv')), key=os.path.getmtime)
    return files[:-1]    # (the newest one is still being written)


def load(top):
    teach = ranked(top)
    X, Y, n_t, n_d = [], [], 0, 0
    for path in completed_files():
        with open(path) as f:
            reader = csv.DictReader(f)
            if 'n62' not in (reader.fieldnames or []):
                continue
            rows = list(reader)
        if not rows:
            continue
        gen = int(rows[0]['generation'])
        best = {car for g, car in teach if g == gen} or furthest(rows, top)
        for r in rows:
            mode = r['mode']
            if mode == 'neural2':
                labels = (r.get('teacher_steer'), r.get('teacher_pedal'))
            elif mode in TEACHERS and r['car'] in best:
                labels = (r.get('teacher_steer'), r.get('teacher_pedal'))   # (their own actions, logged the same way)
            else:
                continue
            if float(r['t']) < 0.5 or labels[0] in (None, ''):
                continue
            try:
                x = [float(r[f'n{i}']) for i in range(N_IN)]
                y = [np.clip(float(labels[0]) / LOCK, -1, 1), np.clip(float(labels[1]), -1, 1)]
            except (ValueError, KeyError):
                continue
            X.append(x)
            Y.append(y)
            n_d += mode == 'neural2'
            n_t += mode != 'neural2'
    return np.array(X, np.float32), np.array(Y, np.float32), n_t, n_d


def unflatten(w):
    w = np.asarray(w, np.float32)
    k = (N_IN + 1) * N_HID
    hid = w[:k].reshape(N_HID, N_IN + 1)
    out = w[k:].reshape(2, N_HID + 1)
    return [hid[:, :N_IN].copy(), hid[:, N_IN].copy(), out[:, :N_HID].copy(), out[:, N_HID].copy()]


def flatten(W1, b1, W2, b2):
    """In ProvingGround.h's order: each hidden unit's 63 weights then its bias, then each output's 24 and its bias."""
    return np.concatenate([np.concatenate([W1, b1[:, None]], 1).ravel(), np.concatenate([W2, b2[:, None]], 1).ravel()])


def train(X, Y, params=None, epochs=40, batch=512, lr=0.003, seed=1):
    rng = np.random.default_rng(seed)
    if params is None:
        params = [rng.normal(0, 0.2, (N_HID, N_IN)).astype(np.float32), np.zeros(N_HID, np.float32),
                  rng.normal(0, 0.2, (2, N_HID)).astype(np.float32), np.zeros(2, np.float32)]
    m = [np.zeros_like(p) for p in params]
    v = [np.zeros_like(p) for p in params]
    t = 0
    for _ in range(epochs):
        idx = rng.permutation(len(X))
        for s in range(0, len(X), batch):
            b = idx[s:s + batch]
            x, y = X[b], Y[b]
            W1, b1, W2, b2 = params
            H = np.tanh(x @ W1.T + b1)
            O = np.tanh(H @ W2.T + b2)
            dO = 2 * (O - y) * (1 - O * O) / len(b)
            dO[:, 0] *= 2.0                                  # (the wheel matters most)
            dH = (dO @ W2) * (1 - H * H)
            grads = (dH.T @ x, dH.sum(0), dO.T @ H, dO.sum(0))
            t += 1
            for i, (p, g) in enumerate(zip(params, grads)):
                m[i] = 0.9 * m[i] + 0.1 * g
                v[i] = 0.999 * v[i] + 0.001 * g * g
                p -= lr * (m[i] / (1 - 0.9 ** t)) / (np.sqrt(v[i] / (1 - 0.999 ** t)) + 1e-8)
    W1, b1, W2, b2 = params
    O = np.tanh(np.tanh(X @ W1.T + b1) @ W2.T + b2)
    r2 = [1 - np.mean((O[:, k] - Y[:, k]) ** 2) / max(np.var(Y[:, k]), 1e-9) for k in range(2)]
    return params, r2


def once(top, epochs):
    X, Y, n_t, n_d = load(top)
    if len(X) < 500:
        print(f'too few samples ({len(X)}): a generation has to finish first')
        return False
    start = None
    if os.path.exists(THETA):
        w = open(THETA).read().split()
        if len(w) == N_W:
            start = unflatten([float(x) for x in w])
    params, r2 = train(X, Y, start, epochs)
    w = flatten(*params)
    tmp = THETA + '.tmp'
    open(tmp, 'w').write(' '.join(f'{x:.6g}' for x in w) + '\n')
    os.replace(tmp, THETA)
    print(f'{time.strftime("%H:%M:%S")} {len(X)} samples ({n_t} from the teachers, {n_d} neural2 laps labelled by pursuit); '
          f'R2 wheel {r2[0]:.3f}, pedal {r2[1]:.3f} -> {THETA}', flush=True)
    return True


def es_on():
    t = open(REPO_INI, encoding='utf-8').read()
    if '\nNn2Es = 0' in t:
        open(REPO_INI, 'w', encoding='utf-8').write(t.replace('\nNn2Es = 0', '\nNn2Es = 1'))
        subprocess.run([sys.executable, os.path.join(HERE, 'proving.py'), 'push'])
        print('evolution strategies on (Nn2Es = 1, ini pushed)', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--top', type=int, default=8)
    ap.add_argument('--epochs', type=int, default=40)
    ap.add_argument('--loop', type=int, default=0, help='retrain after each new generation, this many times, then ES on')
    a = ap.parse_args()
    if not a.loop:
        once(a.top, a.epochs)
        sys.exit(0)
    done, seen = 0, len(completed_files())
    once(a.top, a.epochs)
    while done < a.loop:
        time.sleep(20)
        now = len(completed_files())
        if now > seen:
            seen = now
            if once(a.top, a.epochs):
                done += 1
    es_on()
