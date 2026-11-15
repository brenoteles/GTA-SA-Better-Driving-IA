"""Neural2's central network by imitation, for the proving ground's GA (ProvingGround.h, mode neural2: 63 sensors ->
24 tanh -> two residuals on its teacher: the wheel over pursuit's, in quarters of the lock (kNn2SteerSpan), and the
target speed over the planner's, in +-30% (kNn2SpeedSpan)).

  python tools/nn2_train.py --residual --epochs 0            start over: zero output weights = the teacher itself
  python tools/nn2_train.py [--top 8] [--epochs 40]          train on every generation logged so far
  python tools/nn2_train.py --loop 4                          retrain after each new generation, 4 times, then turn
                                                              the evolution strategies on (Nn2Es = 1, ini pushed)

Data (scripts\\proving\\ga_trace\\gen_*.csv, the columns n0..n62, steer, teacher_steer): the best laps of pursuit / rpp
/ blend (the --top finishers of each generation, by ga_history*.csv; a generation not ranked yet: the furthest with
< 2% outside). Their label is the residual they drove over pursuit, (steer - teacher_steer) / (0.25 lock): what a
better finisher did where pursuit would have done otherwise (pursuit's own rows: ~0, the anchor). The speed residual
has no label in the trace (every car's target is the same planner's): 0. Starts from the current network if there is
one (--residual: its output layer back to zero first). Writes scripts\\proving\\nn2_central.txt (the game reloads it
at the next generation).

Traces written before the snapshot fix logged, in n49..n51, the frame's own wheel and pedals (= the labels of that
time, correlation 1.000: the net learned to copy them); such files are told apart (n49 == steer / lock on every row)
and n49..n51 are zeroed in them. n49 (the wheel it asked for last frame) is zeroed in every file unless
--keep-own-wheel: a residual label (steer - teacher) is a frame away from n49 - n50, and the net would copy it.
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
STEER_SPAN = 0.25                      # kNn2SteerSpan: o[0] = 1 adds a quarter of the lock
TEACHERS = ('pursuit', 'rpp', 'blend')
OWN = (49, 50, 51)                     # its own wheel, gas, brake in the old traces (the leak)
CLAMP3 = (19, 20, 23, 24)              # the offsets (in half-widths): +-3 (a car 12 half-widths off, as the trace had, is lost anyway)


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


def leaky(rows):
    """A trace from before the snapshot: n49 is the row's own steer / lock (and n50 - n51 its gas - brake)."""
    d = []
    for r in rows:
        if r['mode'] in TEACHERS and r.get('n62') not in (None, '') and float(r['t']) > 2.0:
            d.append(abs(float(r['n49']) - float(r['steer']) / LOCK))
            if len(d) >= 400:
                break
    return len(d) >= 50 and float(np.median(d)) < 1e-3


def completed_files():
    files = sorted(glob.glob(os.path.join(PROVING, 'ga_trace', 'gen_*.csv')), key=os.path.getmtime)
    return files[:-1]    # (the newest one is still being written)


def load(top, keep_own_wheel=False):
    teach = ranked(top)
    X, Y, n_t, n_anchor, n_leaky = [], [], 0, 0, 0
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
        leak = leaky(rows)
        n_leaky += leak
        for r in rows:
            mode = r['mode']
            # (the teachers only: neural2's own rows say what it did, not what it should have added to the teacher)
            if mode not in TEACHERS or r['car'] not in best:
                continue
            if float(r['t']) < 0.5 or r.get('teacher_steer') in (None, '') or r.get('n62') in (None, ''):
                continue
            try:
                x = np.array([float(r[f'n{i}']) for i in range(N_IN)], np.float32)
                res = (float(r['steer']) - float(r['teacher_steer'])) / (STEER_SPAN * LOCK)
            except (ValueError, KeyError):
                continue
            if leak:
                x[list(OWN)] = 0.0       # (the leak: the label itself)
                res = 0.0                # (teacher_steer was the car's own wheel there: no residual to learn)
            if not keep_own_wheel:
                x[49] = 0.0
            x[list(CLAMP3)] = np.clip(x[list(CLAMP3)], -3.0, 3.0)
            X.append(x)
            Y.append([np.clip(res, -1, 1), 0.0])
            n_t += 1
            n_anchor += mode == 'pursuit'
    return np.array(X, np.float32), np.array(Y, np.float32), n_t, n_anchor, n_leaky


def unflatten(w):
    w = np.asarray(w, np.float32)
    k = (N_IN + 1) * N_HID
    hid = w[:k].reshape(N_HID, N_IN + 1)
    out = w[k:].reshape(2, N_HID + 1)
    return [hid[:, :N_IN].copy(), hid[:, N_IN].copy(), out[:, :N_HID].copy(), out[:, N_HID].copy()]


def flatten(W1, b1, W2, b2):
    """In ProvingGround.h's order: each hidden unit's 63 weights then its bias, then each output's 24 and its bias."""
    return np.concatenate([np.concatenate([W1, b1[:, None]], 1).ravel(), np.concatenate([W2, b2[:, None]], 1).ravel()])


def fresh(seed=1):
    rng = np.random.default_rng(seed)
    # (the output layer zero: a residual of nothing, the teacher itself, until the data or the ES say otherwise)
    return [rng.normal(0, 1.0 / np.sqrt(N_IN), (N_HID, N_IN)).astype(np.float32), np.zeros(N_HID, np.float32),
            np.zeros((2, N_HID), np.float32), np.zeros(2, np.float32)]


def train(X, Y, params=None, epochs=40, batch=512, lr=0.003, seed=1):
    rng = np.random.default_rng(seed)
    if params is None:
        params = fresh(seed)
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


def once(top, epochs, residual, keep_own_wheel):
    X, Y, n_t, n_anchor, n_leaky = load(top, keep_own_wheel)
    if len(X) < 500 and epochs > 0:
        print(f'too few samples ({len(X)}): a generation has to finish first')
        return False
    start = None
    if os.path.exists(THETA):
        w = open(THETA).read().split()
        if len(w) == N_W:
            start = unflatten([float(x) for x in w])
    if residual:
        # back to the teacher: the hidden layer kept (or new), the output layer zero
        start = start or fresh()
        start[2][:] = 0.0
        start[3][:] = 0.0
    if epochs > 0:
        params, r2 = train(X, Y, start, epochs)
    else:
        params, r2 = start or fresh(), [float('nan'), float('nan')]
    w = flatten(*params)
    tmp = THETA + '.tmp'
    open(tmp, 'w').write(' '.join(f'{x:.6g}' for x in w) + '\n')
    os.replace(tmp, THETA)
    print(f'{time.strftime("%H:%M:%S")} {len(X)} samples ({n_t} from the teachers, {n_anchor} of them pursuit, {n_leaky} files from before the '
          f'snapshot: no residual in them); residual std {Y[:, 0].std() if len(Y) else 0.0:.3f} ({0.25 * 30 * (Y[:, 0].std() if len(Y) else 0.0):.2f} deg), '
          f'R2 wheel residual {r2[0]:.3f} -> {THETA}' + (' (output layer reset: the teacher itself)' if residual else ''), flush=True)
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
    ap.add_argument('--epochs', type=int, default=40, help='0: write the network as it is (with --residual: the teacher itself)')
    ap.add_argument('--residual', action='store_true', help='reset the output layer to zero first (neural2 drives as pursuit until trained or evolved)')
    ap.add_argument('--keep-own-wheel', action='store_true', help='leave n49 (the wheel it asked for last frame) in: the net may copy it')
    ap.add_argument('--loop', type=int, default=0, help='retrain after each new generation, this many times, then ES on')
    a = ap.parse_args()
    if not a.loop:
        once(a.top, a.epochs, a.residual, a.keep_own_wheel)
        sys.exit(0)
    done, seen = 0, len(completed_files())
    once(a.top, a.epochs, a.residual, a.keep_own_wheel)
    while done < a.loop:
        time.sleep(20)
        now = len(completed_files())
        if now > seen:
            seen = now
            if once(a.top, a.epochs, False, a.keep_own_wheel):
                done += 1
    es_on()
