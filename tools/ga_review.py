"""The proving ground's genetic algorithm (ProvingGround.h, [GA]): its results in SQLite, Gemini's review of them, our
validation of each review, and an HTML viewer of the course and every car's line.

  python tools/ga_review.py ingest                 ga_history.csv + ga_trace/*.csv -> the database
  python tools/ga_review.py wait [--every 5]       wait for the next generation to review (the first one, then every
                                                   5th), ask Gemini (agy), store its review as 'pending', write the
                                                   HTML, exit (run it in the background: it ends when a review is due)
  python tools/ga_review.py show [id]              a review (default: the latest) and its section report
  python tools/ga_review.py validate <id> --verdict valid|partial|rejected --notes "..." [--claim "n|ok|evidence"]...
                                                   our check of Gemini's review, claim by claim
  python tools/ga_review.py html                   docs/ga_viewer.html (the data inside: open it in a browser)
  python tools/ga_review.py pilots [--seed]        pilot profiles from the runs so far (the best balanced, the most
                                                   precise, the fastest, the most aggressive, the best of each other
                                                   algorithm) -> RealTrafficFix2/RealTrafficFix.pilots.ini and the
                                                   database; --seed adds them to ga_state.txt (game closed)

Database: scripts/proving/ga.sqlite (tables generations, individuals, trace, sections, reviews, claims).
"""
import argparse
import csv
import datetime
import glob
import json
import os
import re
import sqlite3
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, '..'))
GAME_DIR = os.environ.get('GTA_SA_DIR', r'C:\Users\death\Games\GTA\GTA San Andreas')
PROVING = os.path.join(GAME_DIR, 'scripts', 'proving')
TRACE = os.path.join(PROVING, 'ga_trace')
HISTORY = os.path.join(PROVING, 'ga_history.csv')
DB = os.path.join(PROVING, 'ga.sqlite')
HTML = os.path.join(REPO, 'docs', 'ga_viewer.html')
REVIEWS = os.path.join(PROVING, 'ga_reviews')
AGY = r'C:/Users/death/AppData/Local/agy/bin/agy'
GEMINI_MODEL = 'gemini-4-argon-high'
# the genes and their ranges (ProvingGround.h, GA::kGenes)
RANGES = {'Bandwidth': (0.5, 4.0), 'Damping': (0.4, 2.0), 'Preview': (0.0, 0.3), 'LookaheadTime': (0.05, 1.5),
          'LookaheadMin': (1.5, 15.0), 'StanleyK': (0.3, 5.0), 'Gain': (0.5, 1.3), 'Smoothing': (0.0, 0.12),
          'GripFactor': (0.3, 0.75), 'BrakeG': (0.5, 1.6), 'Kp': (0.1, 1.0), 'BrakeKp': (0.2, 1.5),
          'GripTight': (0.6, 1.6), 'GripFast': (0.6, 1.6), 'LearnRate': (0.0, 0.4), 'CruiseGas': (0.0, 0.4),
          'AbsMargin': (0.5, 1.2), 'AbsDrop': (0.02, 0.4), 'AbsRecover': (0.01, 0.2), 'AbsCircle': (0.0, 1.0),
          'TcSlip': (0.03, 1.0), 'TcMinGas': (0.0, 1.0), 'TcDrop': (0.0, 0.3), 'TcRecover': (0.01, 0.1),
          'TcCircle': (0.0, 1.0)}
# (genes a generation did not have yet: the value they had before they were genes)
DEFAULTS = {'GripTight': 1.0, 'GripFast': 1.0, 'LearnRate': 0.0, 'CruiseGas': 0.15, 'AbsMargin': 0.9, 'AbsDrop': 0.15,
            'AbsRecover': 0.05, 'AbsCircle': 1.0, 'TcSlip': 0.08, 'TcMinGas': 0.15, 'TcDrop': 0.1, 'TcRecover': 0.03,
            'TcCircle': 1.0}
GENES = list(RANGES)

SCHEMA = """
CREATE TABLE IF NOT EXISTS generations (gen INTEGER PRIMARY KEY, stamp TEXT, sigma REAL, best_score REAL, best_car TEXT,
    best_mode TEXT, mean_score REAL, finished INTEGER, population INTEGER);
CREATE TABLE IF NOT EXISTS individuals (gen INTEGER, rank INTEGER, car TEXT, mode TEXT, score REAL, result TEXT,
    time_s REAL, mean_m REAL, rms_m REAL, max_m REAL, out_s REAL, reached_m REAL, genes TEXT, PRIMARY KEY (gen, car));
CREATE TABLE IF NOT EXISTS trace (gen INTEGER, car TEXT, t REAL, x REAL, y REAL, speed REAL, target REAL, offset REAL,
    steer REAL, gas REAL, brake REAL, s REAL, lat_g REAL, slip REAL, abs INTEGER, tc INTEGER);
CREATE INDEX IF NOT EXISTS trace_gen ON trace (gen, car);
CREATE TABLE IF NOT EXISTS sections (gen INTEGER, car TEXT, section TEXT, idx INTEGER, time_s REAL, avg_kmh REAL,
    mean_offset REAL, max_offset REAL, min_kmh REAL, PRIMARY KEY (gen, car, idx));
CREATE TABLE IF NOT EXISTS reviews (id INTEGER PRIMARY KEY AUTOINCREMENT, gen_from INTEGER, gen_to INTEGER,
    created TEXT, model TEXT, prompt TEXT, gemini TEXT, verdict TEXT DEFAULT 'pending', notes TEXT, validated TEXT);
CREATE TABLE IF NOT EXISTS claims (review_id INTEGER, n INTEGER, claim TEXT, verdict TEXT, evidence TEXT,
    PRIMARY KEY (review_id, n));
CREATE TABLE IF NOT EXISTS pilots (name TEXT PRIMARY KEY, mode TEXT, genes TEXT, why TEXT, gen INTEGER, car TEXT,
    score REAL, time_s REAL, mean_m REAL, max_m REAL, created TEXT);
"""
PILOTS_INI = os.path.join(REPO, 'RealTrafficFix2', 'RealTrafficFix.pilots.ini')
STATE = os.path.join(PROVING, 'ga_state.txt')


def db():
    c = sqlite3.connect(DB)
    c.executescript(SCHEMA)
    cols = [r[1] for r in c.execute('PRAGMA table_info(individuals)')]
    for col, kind in (('label', 'TEXT'), ('deficit_ms', 'REAL'), ('min_margin_m', 'REAL')):
        if col not in cols:
            c.execute(f'ALTER TABLE individuals ADD COLUMN {col} {kind}')
    if 'min_margin' not in [r[1] for r in c.execute('PRAGMA table_info(sections)')]:
        c.execute('ALTER TABLE sections ADD COLUMN min_margin REAL')
    if 'margin' not in [r[1] for r in c.execute('PRAGMA table_info(trace)')]:
        c.execute('ALTER TABLE trace ADD COLUMN margin REAL')
    return c


def course():
    pts, secs, cones = [], [], []
    p = os.path.join(TRACE, 'course.csv')
    if os.path.exists(p):
        with open(p) as f:
            pts = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    p = os.path.join(TRACE, 'sections.csv')
    if os.path.exists(p):
        with open(p, encoding='utf-8', errors='replace') as f:
            secs = [(float(r['s']), r['name']) for r in csv.DictReader(f)]
    p = os.path.join(TRACE, 'cones.csv')
    if os.path.exists(p):
        with open(p) as f:
            cones = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
    return pts, secs, cones


def section_stats(rows, secs, length):
    """Per section: time, average speed, the most it was off the line, the slowest point (from the 0.25 s trace)."""
    bounds = [s for s, _ in secs] + [length]
    out = []
    for i, (s0, name) in enumerate(secs):
        s1 = bounds[i + 1]
        inside = [r for r in rows if s0 <= r['s'] < s1]
        before = [r for r in rows if r['s'] < s0]
        after = [r for r in rows if r['s'] >= s1]
        if not inside:
            continue
        t0 = before[-1]['t'] if before else inside[0]['t']
        # (the last section: the trace ends where the run ends)
        t1 = after[0]['t'] if after else inside[-1]['t'] if inside[-1]['s'] >= s1 - 30 else None
        tt = (t1 - t0) if t1 is not None else None
        out.append({'idx': i, 'section': name, 'time_s': tt, 'avg_kmh': (s1 - s0) / tt * 3.6 if tt else None,
                    'mean_offset': sum(abs(r['offset']) for r in inside) / len(inside),
                    'min_margin': min((r['margin'] for r in inside if r.get('margin') is not None), default=None),
                    'max_offset': max(abs(r['offset']) for r in inside), 'min_kmh': min(r['speed'] for r in inside)})
    return out


def ingest(quiet=False):
    c = db()
    pts, secs, _ = course()
    length = pts[-1]['s'] if pts else 0.0
    gens = set()
    paths = sorted(glob.glob(os.path.join(PROVING, 'ga_history_*.csv'))) + ([HISTORY] if os.path.exists(HISTORY) else [])
    if paths:
        rows = []
        for path in paths:
            with open(path) as f:
                rows += list(csv.DictReader(f))
        # (a generation that ran again after a restart: its last rows)
        by = {}
        for r in rows:
            by[(int(r['generation']), r['car'])] = r
        for (g, car), r in by.items():
            genes = {k: float(r[k]) if r.get(k) not in (None, '') else DEFAULTS.get(k, 0.0) for k in GENES}
            c.execute('INSERT OR REPLACE INTO individuals VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                      (g, int(r['rank']), car, r['mode'], float(r['score']), r['result'], float(r['time_s']),
                       float(r.get('mean_m') or 0), float(r['rms_m']), float(r['max_m']), float(r['out_s']), float(r['reached_m']),
                       json.dumps(genes), r.get('label') or '',
                       float(r['deficit_ms']) if r.get('deficit_ms') else None, float(r['min_margin_m']) if r.get('min_margin_m') else None))
            gens.add(g)
        for g in gens:
            ind = c.execute('SELECT car, mode, score, result FROM individuals WHERE gen=? ORDER BY score', (g,)).fetchall()
            stamp = next((r['stamp'] for (gg, _), r in by.items() if gg == g), '')
            sigma = next((float(r['sigma']) for (gg, _), r in by.items() if gg == g), 0.0)
            c.execute('INSERT OR REPLACE INTO generations VALUES (?,?,?,?,?,?,?,?,?)',
                      (g, stamp, sigma, ind[0][2], ind[0][0], ind[0][1], sum(i[2] for i in ind) / len(ind),
                       sum(1 for i in ind if i[3] == 'end'), len(ind)))
    # traces of finished generations (the running one's file is still being written)
    done = {g for (g,) in c.execute('SELECT gen FROM generations')}
    have = {g for (g,) in c.execute('SELECT DISTINCT gen FROM trace')}
    for path in sorted(glob.glob(os.path.join(TRACE, 'gen_*.csv'))):
        g = int(re.search(r'gen_(\d+)', path).group(1))
        if g not in done or g in have:
            continue
        with open(path) as f:
            rows = [r for r in csv.DictReader(f)]
        recs = []
        for r in rows:
            try:
                recs.append((g, r['car'], float(r['t']), float(r['x']), float(r['y']), float(r['speed_kmh']),
                             float(r['target_kmh']), float(r['offset']), float(r['steer']), float(r['gas']),
                             float(r['brake']), float(r['s']), float(r['lat_g']), float(r['slip']), int(r['abs']), int(r['tc']),
                             float(r['margin']) if r.get('margin') not in (None, '') else None))
            except (ValueError, KeyError):
                pass  # a line cut short
        c.executemany('INSERT INTO trace VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)', recs)
        for car in sorted({r[1] for r in recs}):
            rr = [{'t': r[2], 's': r[11], 'offset': r[7], 'speed': r[5], 'margin': r[16]} for r in recs if r[1] == car]
            for st in section_stats(rr, secs, length):
                c.execute('INSERT OR REPLACE INTO sections VALUES (?,?,?,?,?,?,?,?,?,?)',
                          (g, car, st['section'], st['idx'], st['time_s'], st['avg_kmh'], st['mean_offset'], st['max_offset'],
                           st['min_kmh'], st['min_margin']))
    c.commit()
    if not quiet:
        n = c.execute('SELECT COUNT(*), MAX(gen) FROM generations').fetchone()
        print(f'{DB}: {n[0]} generations (last {n[1]}), {c.execute("SELECT COUNT(*) FROM trace").fetchone()[0]} trace rows')
    return c


def evolution(c):
    """Every generation so far in one table: is the score making the cars faster AND closer to the line, and is the
    population still varied enough to find better (or stuck on one peak)?"""
    L = ['== Evolution, every generation (best = lowest score; median of the cars that finished; diversity = mean over '
         'the genes of their spread / their range, 0 = all equal; distinct = genomes that differ; stale = generations '
         'since the best ever improved)',
         '  gen  sigma   best   time  mean_m | median  time  mean_m | finished | modes                          | '
         'diversity distinct | stale']
    best, stale = 1e18, 0
    for (g, sigma) in c.execute('SELECT gen, sigma FROM generations ORDER BY gen').fetchall():
        ind = c.execute('SELECT score, result, time_s, mean_m, mode, genes FROM individuals WHERE gen=? ORDER BY score',
                        (g,)).fetchall()
        fin = [i for i in ind if i[1] == 'end']
        med = fin[len(fin) // 2] if fin else None
        modes = {}
        for i in ind:
            modes[i[4]] = modes.get(i[4], 0) + 1
        genes = [json.loads(i[5]) for i in ind]
        spread = []
        for k, (lo, hi) in RANGES.items():
            v = [x[k] for x in genes if k in x]
            if len(v) > 1:
                m = sum(v) / len(v)
                spread.append((sum((a - m) ** 2 for a in v) / len(v)) ** 0.5 / (hi - lo))
        distinct = len({(i[4],) + tuple(round(json.loads(i[5]).get(k, 0), 3) for k in GENES) for i in ind})
        if ind[0][0] < best - 0.05:
            best, stale = ind[0][0], 0
        else:
            stale += 1
        b = ind[0]
        L.append(f'  {g:3d}  {sigma:.3f} {b[0]:7.1f} {b[2]:6.1f} {b[3]:6.3f} | ' +
                 (f'{med[0]:6.1f} {med[2]:5.1f} {med[3]:6.3f}' if med else '     -     -      -') +
                 f' | {len(fin):3d}/{len(ind):<3d}  | {" ".join(f"{k} {v}" for k, v in sorted(modes.items())):<30} | '
                 f'{(sum(spread) / len(spread) if spread else 0):9.3f} {distinct:8d} | {stale:5d}')
    return '\n'.join(L)


def report(c, g0, g1):
    """A plain-text report of generations g0..g1 for Gemini (and for checking what it says)."""
    L = [evolution(c)]
    for (g, sigma, best, car, mode, mean, fin, pop) in c.execute(
            'SELECT gen, sigma, best_score, best_car, best_mode, mean_score, finished, population FROM generations '
            'WHERE gen BETWEEN ? AND ? ORDER BY gen', (g0, g1)):
        L.append(f'\n== Generation {g}: sigma {sigma:.3f}, best {best:.2f} (car {car}, {mode}), mean {mean:.1f}, '
                 f'{fin}/{pop} finished')
        for r in c.execute('SELECT rank, car, mode, score, result, time_s, mean_m, rms_m, max_m, out_s, reached_m, genes, label, '
                           'deficit_ms, min_margin_m FROM individuals WHERE gen=? ORDER BY rank', (g,)):
            genes = json.loads(r[11])
            room = f' room min {r[14]:.2f} m, near edge {r[13]:.1f} m.s' if r[13] is not None else ''
            L.append(f'  {r[0]:2d}. {r[1]} {r[2]:<9}' + (f' [{r[12]}]' if r[12] else '') + f' score {r[3]:8.2f} {r[4]:<7} {r[5]:6.1f} s offset mean {r[6]:.2f} '
                     f'max {r[8]:5.2f}{room} out {r[9]:5.1f} s reached {r[10]:.0f} m | ' + ' '.join(f'{k} {v:.3g}' for k, v in genes.items()))
    g = g1
    rows = c.execute('SELECT car, section, idx, time_s, avg_kmh, mean_offset, max_offset, min_kmh FROM sections WHERE gen=? '
                     'ORDER BY idx, time_s', (g,)).fetchall()
    if rows:
        L.append(f'\n== Sections, generation {g} (per car: time s, average km/h, mean / most off the line m, slowest km/h)')
        cur = None
        for car, sec, idx, t, v, mean, off, vmin in rows:
            if sec != cur:
                cur = sec
                L.append(f'  {sec}:')
            L.append(f'    {car} ' + (f'{t:6.2f} s {v:6.1f} km/h' if t else '   (not through)') + f'  off mean {mean:4.2f} max {off:5.2f} m  min {vmin:5.1f} km/h')
    return '\n'.join(L)


def due(c, every):
    """The generation to review next: the first one, then every 'every'-th after the last review."""
    last = c.execute('SELECT MAX(gen_to) FROM reviews').fetchone()[0]
    top = c.execute('SELECT MAX(gen) FROM generations').fetchone()[0]
    if top is None:
        return None
    if last is None:
        return (0, 0) if top >= 0 else None
    if top >= last + every:
        return (last + 1, last + every)
    return None


def ask_gemini(c, g0, g1):
    os.makedirs(REVIEWS, exist_ok=True)
    rep = os.path.join(REVIEWS, f'report_gen{g0:04d}-{g1:04d}.txt')
    open(rep, 'w', encoding='utf-8').write(report(c, g0, g1))
    out = os.path.join(REVIEWS, f'gemini_gen{g0:04d}-{g1:04d}.md')
    prompt = (
        'Voce revisa um algoritmo genetico que ajusta o controle de direcao/velocidade de carros da IA do GTA San Andreas '
        'numa pista de testes (40 carros por geracao no mesmo circuito de 5,1 km). A pista e uma faixa de mao unica de 4 m (transito): '
        'o carro deve manter a carroceria inteira entre as duas bordas, a linha central so guia; o objetivo nao e o mais '
        'rapido, e o que fica com folga dentro da pista e ainda assim e rapido. Pontuacao (menor e melhor): tempo + 5 x '
        'm.s com a carroceria a menos de 0,3 m da borda + 10 x segundos com a carroceria fora da pista (o desvio do centro '
        'nao conta). NAO altere nenhum arquivo: so leia e '
        'responda no terminal, em portugues.\n'
        f'Leia: {rep} (geracoes {g0}-{g1}: ranking, genes, e tempos por trecho da ultima), {HISTORY} (todas as geracoes), '
        f'{os.path.join(REPO, "RealTrafficFix2", "source", "ProvingGround.h")} (namespace GA: genes, faixas, pontuacao, '
        f'selecao; ClosedLoopSteer e Control: os algoritmos kinematic/stanley/pursuit/mod), '
        f'{os.path.join(REPO, "RealTrafficFix2", "RealTrafficFix.proving.ini")} ([GA] e [Test.circuit]).\n'
        'Voce tem duas responsabilidades fixas, alem do resumo: (A) verificar se a pontuacao realmente esta deixando '
        'os carros mais rapidos E melhores (mais folga ate a borda, nunca fora da pista) ao longo das geracoes, ou se esta trocando um pelo outro '
        '(ver a tabela "Evolution": tempo e desvio do melhor e da mediana); (B) verificar se a geracao de novos cerebros '
        'funciona bem: diversidade da populacao, algoritmos que sumiram, genes presos no limite da faixa, quantas '
        'geracoes sem melhorar (stale), e se os carros estao presos num pico local ruim.\n'
        'Responda com:\n'
        '1. Resumo do progresso (melhor/mediana por geracao, qual algoritmo domina e por que, segundo os dados).\n'
        '2a. Veredito (A) sobre a pontuacao e (B) sobre a evolucao, cada um com os numeros da tabela Evolution.\n'
        '2. Afirmacoes numeradas no formato "C1: <afirmacao> [dado: <numero e de onde veio>]", cada uma verificavel nos '
        'arquivos (onde os carros perdem tempo, saem da linha, ficam presos; genes que convergiram).\n'
        '3. Sugestoes numeradas "S1: <mudanca concreta no ini ou no codigo> -- motivo", no maximo 5, so as que os dados '
        'sustentam.\n'
        'Nao invente numeros: cite so o que esta nos arquivos.')
    t0 = time.time()
    r = subprocess.run([AGY, '-p', prompt, '--model', GEMINI_MODEL, '--print-timeout', '400s'], capture_output=True,
                       text=True, encoding='utf-8', errors='replace', cwd=REPO, timeout=600)
    text = (r.stdout or '') + (('\n[stderr]\n' + r.stderr) if r.returncode else '')
    open(out, 'w', encoding='utf-8').write(text)
    c.execute('INSERT INTO reviews (gen_from, gen_to, created, model, prompt, gemini) VALUES (?,?,?,?,?,?)',
              (g0, g1, datetime.datetime.now().isoformat(timespec='seconds'), GEMINI_MODEL, prompt, text))
    c.commit()
    rid = c.execute('SELECT MAX(id) FROM reviews').fetchone()[0]
    print(f'review {rid}: generations {g0}-{g1}, Gemini {time.time() - t0:.0f} s -> {out}\nreport: {rep}')
    return rid


def wait(every, timeout):
    t0 = time.time()
    while time.time() - t0 < timeout:
        c = ingest(quiet=True)
        d = due(c, every)
        if d:
            rid = ask_gemini(c, *d)
            write_html(c)
            return rid
        out = subprocess.run(['tasklist', '/FI', 'IMAGENAME eq gta_sa.exe'], capture_output=True, text=True).stdout
        if 'gta_sa.exe' not in out:
            print('the game is not running')
            write_html(c)
            return None
        time.sleep(20)
    print('timeout')


def show(rid=None):
    c = db()
    r = c.execute('SELECT id, gen_from, gen_to, created, gemini, verdict, notes FROM reviews WHERE id=COALESCE(?, '
                  '(SELECT MAX(id) FROM reviews))', (rid,)).fetchone()
    if not r:
        print('no review')
        return
    print(f'review {r[0]}: generations {r[1]}-{r[2]}, {r[3]}, verdict {r[5]}\n')
    print(r[4])
    if r[6]:
        print('\nnotes:', r[6])
    for n, claim, v, ev in c.execute('SELECT n, claim, verdict, evidence FROM claims WHERE review_id=? ORDER BY n', (r[0],)):
        print(f'  C{n} [{v}] {claim} -- {ev}')


def validate(rid, verdict, notes, claims):
    c = db()
    c.execute('UPDATE reviews SET verdict=?, notes=?, validated=? WHERE id=?',
              (verdict, notes, datetime.datetime.now().isoformat(timespec='seconds'), rid))
    gem = c.execute('SELECT gemini FROM reviews WHERE id=?', (rid,)).fetchone()[0] or ''
    for cl in claims or []:
        n, v, ev = (cl.split('|', 2) + ['', ''])[:3]
        n = int(n.strip().lstrip('CcSs'))
        m = re.search(rf'(?m)^\W*[CS]{n}\s*:\s*(.+)$', gem)
        c.execute('INSERT OR REPLACE INTO claims VALUES (?,?,?,?,?)', (rid, n, m.group(1).strip() if m else '', v.strip(), ev.strip()))
    c.commit()
    write_html(c)
    print(f'review {rid}: {verdict}')


# --- HTML viewer ---------------------------------------------------------------------------------------------------

def write_html(c=None):
    c = c or db()
    pts, secs, cones = course()
    gens = [dict(zip(('gen', 'stamp', 'sigma', 'best', 'car', 'mode', 'mean', 'finished', 'population'), r))
            for r in c.execute('SELECT * FROM generations ORDER BY gen')]
    inds = {}
    for r in c.execute('SELECT gen, rank, car, mode, score, result, time_s, mean_m, rms_m, max_m, out_s, reached_m, genes, label, '
                       'deficit_ms, min_margin_m FROM individuals ORDER BY gen, rank'):
        inds.setdefault(r[0], []).append({'rank': r[1], 'car': r[2], 'mode': r[3], 'score': r[4], 'result': r[5],
                                         'time': r[6], 'mean': r[7], 'rms': r[8], 'max': r[9], 'out': r[10], 'reached': r[11],
                                         'genes': json.loads(r[12]), 'label': r[13] or '', 'deficit': r[14], 'margin': r[15]})
    traces = {}
    for g, car, t, x, y, v, off, s, br in c.execute(
            'SELECT gen, car, t, x, y, speed, offset, s, brake FROM trace WHERE CAST(ROUND(t * 4) AS INTEGER) % 2 = 0 '
            'ORDER BY gen, car, t'):
        d = traces.setdefault(str(g), {}).setdefault(car, [])
        d.append([round(t, 2), round(x, 1), round(y, 2), round(v), round(off, 2), round(s), round(br, 2)])
    sects = {}
    for g, car, sec, idx, t, v, mean, off in c.execute('SELECT gen, car, section, idx, time_s, avg_kmh, mean_offset, max_offset '
                                                       'FROM sections'):
        sects.setdefault(str(g), {}).setdefault(car, {})[idx] = [t, v, off, mean]
    reviews = []
    for rid, g0, g1, created, gem, verdict, notes, validated in c.execute(
            'SELECT id, gen_from, gen_to, created, gemini, verdict, notes, validated FROM reviews ORDER BY id'):
        cl = [dict(zip(('n', 'claim', 'verdict', 'evidence'), r)) for r in
              c.execute('SELECT n, claim, verdict, evidence FROM claims WHERE review_id=? ORDER BY n', (rid,))]
        reviews.append({'id': rid, 'from': g0, 'to': g1, 'created': created, 'gemini': gem, 'verdict': verdict,
                        'notes': notes, 'validated': validated, 'claims': cl})
    data = {'course': [[round(p['x'], 1), round(p['y'], 2), round(p['s']), p['vcap_kmh'], p['vsud_kmh'], p.get('wind', 0.0),
                        p.get('half_width', 0.0)]
                       for p in pts],
            'sections': secs, 'cones': cones, 'generations': gens, 'individuals': inds, 'traces': traces,
            'sectionStats': sects, 'reviews': reviews, 'written': datetime.datetime.now().isoformat(timespec='seconds')}
    tpl = open(os.path.join(HERE, 'ga_viewer_template.html'), encoding='utf-8').read()
    os.makedirs(os.path.dirname(HTML), exist_ok=True)
    open(HTML, 'w', encoding='utf-8').write(tpl.replace('/*DATA*/null', json.dumps(data, separators=(',', ':'))))
    print(f'{HTML}: {len(gens)} generations, {sum(len(v) for g in traces.values() for v in g.values())} trace points, '
          f'{len(reviews)} reviews')


PROFILES = [
    # name, why, pick from the cars that finished (rows: dicts)
    ('Equilibrado', 'a melhor pontuacao (tempo + desvio)', lambda rs: min(rs, key=lambda r: r['score'])),
    ('Preciso', 'o menor desvio medio entre os que nao foram mais lentos que a mediana',
     lambda rs: min([r for r in rs if r['time'] <= sorted(x['time'] for x in rs)[len(rs) // 2]] or rs, key=lambda r: r['mean'])),
    ('Veloz', 'o menor tempo com desvio medio ate 0,25 m e nunca mais de 2 m fora da linha',
     lambda rs: min([r for r in rs if r['mean'] <= 0.25 and r['out'] == 0] or rs, key=lambda r: r['time'])),
    ('Agressivo', 'a maior aderencia de curva (GripFactor) que chegou sem sair da linha',
     lambda rs: max([r for r in rs if r['out'] == 0] or rs, key=lambda r: (r['genes']['GripFactor'], -r['score']))),
]


def pilots(seed):
    c = ingest(quiet=True)
    rows = [dict(zip(('gen', 'car', 'mode', 'score', 'time', 'mean', 'max', 'out', 'genes'), r)) for r in c.execute(
        "SELECT gen, car, mode, score, time_s, mean_m, max_m, out_s, genes FROM individuals WHERE result='end'")]
    for r in rows:
        r['genes'] = json.loads(r['genes'])
    if not rows:
        sys.exit('no finished runs yet')
    # each profile a car the ones before did not pick (the most aggressive was also the best balanced)
    chosen = []
    for name, why, pick in PROFILES:
        free = [r for r in rows if all((r['gen'], r['car']) != (p[2]['gen'], p[2]['car']) for p in chosen)]
        chosen.append((name, why, pick(free or rows)))
    # the best of each algorithm that is not already there (the others went extinct: a way back for them)
    for mode in sorted({r['mode'] for r in rows}):
        if any(p[2]['mode'] == mode for p in chosen):
            continue
        chosen.append((f'Melhor {mode}', f'a melhor pontuacao do algoritmo {mode}',
                       min([r for r in rows if r['mode'] == mode], key=lambda r: r['score'])))
    now = datetime.datetime.now().isoformat(timespec='seconds')
    with open(PILOTS_INI, 'w', encoding='utf-8') as f:
        f.write('; Pilot profiles from the proving ground GA (tools/ga_review.py pilots, ' + now + '): the controller of a\n'
                '; car that stood out, in the [Steer] / [Speed] keys of RealTrafficFix.proving.ini (copy the keys of a\n'
                '; section there to drive with it). Score = time + 100 x mean offset (m) + 5 x s beyond 2 m, 5.1 km circuit.\n')
        for name, why, r in chosen:
            f.write(f'\n; {why}: generation {r["gen"]} car {r["car"]}, score {r["score"]:.1f}, {r["time"]:.1f} s, '
                    f'mean offset {r["mean"]:.3f} m, max {r["max"]:.2f} m\n[Pilot.{name}]\nMode = {r["mode"]}\n')
            for k in GENES:
                f.write(f'{k} = {r["genes"][k]:.4g}\n')
            c.execute('INSERT OR REPLACE INTO pilots VALUES (?,?,?,?,?,?,?,?,?,?,?)',
                      (name, r['mode'], json.dumps(r['genes']), why, r['gen'], r['car'], r['score'], r['time'], r['mean'],
                       r['max'], now))
            print(f'{name:<18} {r["mode"]:<9} gen {r["gen"]:2d} {r["car"]}  score {r["score"]:6.1f}  {r["time"]:6.1f} s  '
                  f'mean {r["mean"]:.3f}  max {r["max"]:.2f}')
    c.commit()
    print(PILOTS_INI)
    if seed:
        lines = open(STATE, encoding='utf-8').read().splitlines()
        names = {name for name, _, _ in chosen}
        lines = [l for l in lines if not (l.startswith('ind ') and l.split('| ')[-1] in names)]  # (an earlier seed of them)
        last = max(i for i, l in enumerate(lines) if l.startswith('ind ') or l.startswith('genes'))
        new = [f'ind {r["mode"]} ' + ' '.join(f'{r["genes"][k]:.4g}' for k in GENES) + f' | {name}' for name, _, r in chosen]
        open(STATE, 'w', encoding='utf-8').write('\n'.join(lines[:last + 1] + new + lines[last + 1:]) + '\n')
        print(f'{STATE}: + {len(new)} pilots')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['ingest', 'wait', 'show', 'validate', 'html', 'report', 'pilots'])
    ap.add_argument('--seed', action='store_true')
    ap.add_argument('id', nargs='?', type=int)
    ap.add_argument('--every', type=int, default=5)
    ap.add_argument('--timeout', type=float, default=3 * 3600)
    ap.add_argument('--verdict')
    ap.add_argument('--notes', default='')
    ap.add_argument('--claim', action='append')
    ap.add_argument('--gens')
    a = ap.parse_args()
    if a.cmd == 'ingest':
        ingest()
    elif a.cmd == 'wait':
        wait(a.every, a.timeout)
    elif a.cmd == 'show':
        show(a.id)
    elif a.cmd == 'validate':
        validate(a.id, a.verdict, a.notes, a.claim)
    elif a.cmd == 'pilots':
        pilots(a.seed)
    elif a.cmd == 'report':
        g0, g1 = (int(x) for x in (a.gens or '0-0').split('-'))
        print(report(ingest(quiet=True), g0, g1))
    else:
        write_html()
