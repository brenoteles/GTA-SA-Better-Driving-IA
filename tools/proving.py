"""Proving ground (ProvingGround.h): start the game on it, change its settings while it runs, analyse the runs.

  python tools/proving.py start [ini]     copy the ini (default RealTrafficFix2/RealTrafficFix.proving.ini) and start the game
  python tools/proving.py push [ini]      copy the ini into the running game: the program starts over (no rebuild/restart)
  python tools/proving.py stop            close the game and remove the proving ini (the game is normal again)
  python tools/proving.py report [dir]    analyse a session (default: the latest): physics numbers per run + plots
  python tools/proving.py wait            wait for the session to print 'Done.' (or 20 min)
"""
import glob
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_INI = os.path.normpath(os.path.join(HERE, '..', 'RealTrafficFix2', 'RealTrafficFix.proving.ini'))
GAME_DIR = os.environ.get('GTA_SA_DIR', r'C:\Users\death\Games\GTA\GTA San Andreas')
SCRIPTS = os.path.join(GAME_DIR, 'scripts')
GAME_INI = os.path.join(SCRIPTS, 'RealTrafficFix.proving.ini')
AUTOTEST_INI = os.path.join(SCRIPTS, 'RealTrafficFix.autotest.ini')
MIXSETS_INI = os.path.join(GAME_DIR, 'modloader', 'MixSets', 'MixSets.ini')
G = 9.81


def game_running():
    out = subprocess.run(['tasklist', '/FI', 'IMAGENAME eq gta_sa.exe'], capture_output=True, text=True).stdout
    return 'gta_sa.exe' in out


def push(ini):
    shutil.copy2(ini, GAME_INI)
    os.utime(GAME_INI, None)
    print(f'ini -> {GAME_INI}')


def start(ini):
    if game_running():
        print('the game is already running: pushing the ini instead')
        push(ini)
        return
    if os.path.exists(AUTOTEST_INI):
        sys.exit(f'{AUTOTEST_INI} exists (an AutoTest would run too): remove it first')
    push(ini)
    if os.path.exists(MIXSETS_INI):
        c = open(MIXSETS_INI, encoding='latin-1').read()
        c = re.sub(r'(?m)^(NoPauseWhenUnfocus\s*=\s*)\d', r'\g<1>1', c)
        open(MIXSETS_INI, 'w', encoding='latin-1').write(c)
    subprocess.Popen([os.path.join(GAME_DIR, 'gta_sa.exe')], cwd=GAME_DIR)
    sys.path.insert(0, HERE)
    import ctypes
    import fla_window
    import focus_game
    fla_ini = os.path.join(GAME_DIR, 'fastman92limitAdjuster_GTASA.ini')
    # the limit adjuster's start window, and the focus until the game is in front once (it only loads with the focus)
    t0 = time.time()
    fla_done = not os.path.exists(fla_ini)
    while time.time() - t0 < 120:
        time.sleep(1)
        if not fla_done and 'clicking' in fla_window.run(True, fla_ini):
            fla_done = True
        hwnd = focus_game.game_window()
        if hwnd and ctypes.windll.user32.GetForegroundWindow() == hwnd and fla_done:
            break
        if hwnd:
            focus_game.focus()
    print('game started')


def stop():
    subprocess.run(['taskkill', '/IM', 'gta_sa.exe', '/F'], capture_output=True)
    time.sleep(1)
    if os.path.exists(GAME_INI):
        os.remove(GAME_INI)
    print('game closed, proving ini removed')


def latest_dir():
    p = os.path.join(SCRIPTS, 'proving', 'latest.txt')
    return os.path.join(SCRIPTS, 'proving', open(p).read().strip())


def wait(timeout=1200):
    d0 = latest_dir()
    t0 = time.time()
    seen = 0
    while time.time() - t0 < timeout:
        d = latest_dir()
        s = os.path.join(d, 'summary.txt')
        if os.path.exists(s):
            lines = open(s, encoding='utf-8', errors='replace').read().splitlines()
            for l in lines[seen:]:
                print(l, flush=True)
            seen = len(lines)
            if lines and lines[-1].startswith('Done'):
                return d
        if not game_running():
            print('game not running')
            return d
        if d != d0:
            d0, seen = d, 0
        time.sleep(3)
    return latest_dir()


# --- Analysis ---------------------------------------------------------------------------------------------------------

def load(path):
    with open(path) as f:
        head = f.readline().strip().split(',')
    a = np.genfromtxt(path, delimiter=',', skip_header=1)
    if a.ndim == 1:
        a = a[None, :]
    return {h: a[:, i] for i, h in enumerate(head)}


def smooth(x, n=5):
    if len(x) < n:
        return x
    k = np.ones(n) / n
    return np.convolve(x, k, mode='same')


def analyse(path, model_info):
    d = load(path)
    name = os.path.basename(path)
    t, v, r, st = d['t'], d['v_fwd'], d['yaw_rate'], d['steer']
    ay = v * r                                         # m/s2 centripetal (steady)
    skid = (d['ws_fl'] + d['ws_fr'] + d['ws_rl'] + d['ws_rr']) > 0
    front = (d['ws_fl'] + d['ws_fr']) > 0
    rear = (d['ws_rl'] + d['ws_rr']) > 0
    slip = np.arctan2(d['v_lat'], np.maximum(np.abs(v), 0.5))
    out = {'run': name}
    m = re.match(r'\d+_(\w+?)_(\d+)_', name)
    test = m.group(1) if m else ''
    L = model_info.get('L', 2.7)
    ok = (np.abs(st) > 0.004) & (v > 5) & ~skid & (t > 1.5)
    if ok.sum() > 10:
        out['L_eff'] = float(np.median(v[ok] * np.tan(st[ok]) / np.where(np.abs(r[ok]) > 1e-4, r[ok], np.nan)))
    out['ay_max_g'] = float(np.nanmax(np.abs(smooth(ay, 9))) / G)
    out['slip_max_deg'] = float(np.degrees(np.nanmax(np.abs(slip))))
    out['skid_front_%'] = float(100 * front.mean())
    out['skid_rear_%'] = float(100 * rear.mean())
    if test.startswith('ramp'):
        # yaw rate vs the kinematic one (v tan d / L): 1 while the tyres hold; where it departs = the limit
        kin = v * np.tan(st) / L
        ratio = np.where(np.abs(kin) > 0.02, r / np.where(np.abs(kin) > 1e-6, kin, np.nan), np.nan)
        lim = np.where((np.abs(kin) > 0.02) & (ratio < 0.9))[0]
        if len(lim):
            i = lim[0]
            out['limit_at_steer'] = float(st[i])
            out['limit_ay_g'] = float(abs(v[i] * r[i]) / G)
            out['first_skid'] = 'front' if front[i:i + 10].any() and not rear[i:i + 10].any() else (
                'rear' if rear[i:i + 10].any() and not front[i:i + 10].any() else 'both/none')
            after = slice(i, len(t))
            out['after_limit'] = ('oversteer/spin' if np.nanmax(np.abs(slip[after])) > 0.25 else
                                  'understeer (yaw plateau)' if np.nanmax(np.abs(ratio[after])) < 0.95 else 'mixed')
        first = np.where(skid & (t > 1.0))[0]
        if len(first):
            out['first_skid_steer'] = float(st[first[0]])
            out['first_skid_ay_g'] = float(abs(v[first[0]] * r[first[0]]) / G)
            out['first_skid_wheels'] = ''.join(n for n, k in (('FL', 'ws_fl'), ('RL', 'ws_rl'), ('FR', 'ws_fr'), ('RR', 'ws_rr'))
                                               if d[k][first[0]])
    if test.startswith('step'):
        i0 = np.argmax(np.abs(st) > 1e-4)
        rf = np.median(r[-15:])
        if abs(rf) > 1e-3:
            k63 = np.where(np.abs(r[i0:]) >= 0.63 * abs(rf))[0]
            k90 = np.where(np.abs(r[i0:]) >= 0.9 * abs(rf))[0]
            out['yaw_t63_s'] = float(t[i0 + k63[0]] - t[i0]) if len(k63) else None
            out['yaw_t90_s'] = float(t[i0 + k90[0]] - t[i0]) if len(k90) else None
            out['yaw_overshoot_%'] = float(100 * (np.max(np.abs(r[i0:])) / abs(rf) - 1))
            out['yaw_final_vs_kin'] = float(rf / (np.median(v[-15:]) * np.tan(st[-1]) / L))
    if test.startswith('sweep'):
        # gain and lag of the yaw rate vs the wheel, per window of ~1.5 cycles
        res = []
        for f0 in (0.3, 0.5, 0.8, 1.2, 1.8, 2.5):
            phase_t = None
            sel = (t > 1.2)
            if sel.sum() < 30:
                break
            # instantaneous frequency of the sweep: f(t) = F + (FE-F) (t-1)/(T-1)
            T = t[-1]
            fi = 0.3 + (3.0 - 0.3) * (t - 1.0) / max(T - 1.0, 1)
            w = sel & (np.abs(fi - f0) < 0.15)
            if w.sum() < 20:
                continue
            x, y = st[w] - st[w].mean(), r[w] - r[w].mean()
            vv = np.mean(v[w])
            kin = vv * x / L
            gain = np.std(y) / max(np.std(kin), 1e-6)
            # lag from the cross-correlation
            dt = np.median(np.diff(t[w]))
            c = np.correlate(y, kin, 'full')
            lag = (np.argmax(c) - (len(kin) - 1)) * dt
            res.append(f'{f0}Hz gain {gain:.2f} lag {lag * 1000:.0f} ms')
        out['sweep'] = '; '.join(res)
    if test.startswith('accel'):
        k = np.where(d['speed_kmh'] >= 100)[0]
        out['t_0_100'] = float(t[k[0]] - 0.5) if len(k) else None
        out['v_max_kmh'] = float(d['speed_kmh'].max())
        out['a_long_max_g'] = float(np.nanmax(smooth(d['a_long'], 9)) / G)
    if test.startswith('brake'):
        b = t >= 1.0
        out['decel_mean_g'] = float(-np.nanmean(d['a_long'][b & (d['speed_kmh'] > 10)]) / G)
        out['locked_%'] = float(100 * np.mean((d['ws_fl'][b] == 3) | (d['ws_rl'][b] == 3)))
        out['stop_m'] = float(np.sum(d['speed_kmh'][b] / 3.6 * d['dt'][b]))
    if 'offset' in d and not test.startswith(('ramp', 'step', 'sweep', 'accel', 'brake')):
        sel = t > 2
        out['offset_rms'] = float(np.sqrt(np.mean(d['offset'][sel] ** 2))) if sel.any() else None
        out['offset_max'] = float(np.max(np.abs(d['offset'][sel]))) if sel.any() else None
    out['surfaces'] = sorted(set(int(s) for s in d.get('surface', [])))
    return out, d


def plot(path, d, png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(15, 5.5))
    ax[0].plot(d['x'], d['y'], 'c-', lw=1.5, label='car')
    ax[0].set_aspect('equal')
    ax[0].set_title(os.path.basename(path))
    ax[0].grid(alpha=.3)
    t = d['t']
    a = ax[1]
    a.plot(t, d['steer'] * 10, label='steer x10 (rad)')
    a.plot(t, d['yaw_rate'], label='yaw rate (rad/s)')
    a.plot(t, d['v_fwd'] * d['yaw_rate'] / G, label='v*r (g)')
    a.plot(t, d['offset'], label='offset (m)')
    a.plot(t, np.degrees(np.arctan2(d['v_lat'], np.maximum(np.abs(d['v_fwd']), .5))) / 10, label='slip /10 (deg)')
    sk = (d['ws_fl'] + d['ws_fr']) > 0
    a.fill_between(t, -0.05, 0.05, where=sk, color='r', alpha=.3, label='front skid')
    sk = (d['ws_rl'] + d['ws_rr']) > 0
    a.fill_between(t, -0.1, -0.05, where=sk, color='m', alpha=.3, label='rear skid')
    a.grid(alpha=.3)
    a.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(png, dpi=80)
    plt.close(fig)


def report(d=None, plots=True):
    d = d or latest_dir()
    summ = open(os.path.join(d, 'summary.txt'), encoding='utf-8', errors='replace').read()
    models = {}
    for m in re.finditer(r'Model (\d+): (.*?wheelbase ([\d.]+) m)', summ):
        models[int(m.group(1))] = {'L': float(m.group(3)), 'text': m.group(2)}
    print(f'Session {d}')
    cur = None
    for path in sorted(glob.glob(os.path.join(d, '*.csv'))):
        mm = re.match(r'\d+_\w+?_(\d+)_', os.path.basename(path))
        model = int(mm.group(1)) if mm else 0
        if model != cur:
            cur = model
            print(f'\n== model {model}: {models.get(model, {}).get("text", "")}')
        try:
            out, data = analyse(path, models.get(model, {}))
        except Exception as e:  # a run cut short
            print(f'  {os.path.basename(path)}: {e!r}')
            continue
        name = out.pop('run')
        print('  ' + name + ': ' + ', '.join(f'{k} {v:.3f}' if isinstance(v, float) else f'{k} {v}' for k, v in out.items()))
        if plots:
            plot(path, data, path[:-4] + '.png')


if __name__ == '__main__':
    cmd = sys.argv[1] if len(sys.argv) > 1 else 'report'
    arg = sys.argv[2] if len(sys.argv) > 2 else None
    if cmd == 'start':
        start(arg or REPO_INI)
    elif cmd == 'push':
        push(arg or REPO_INI)
    elif cmd == 'stop':
        stop()
    elif cmd == 'wait':
        wait()
    else:
        report(arg)
