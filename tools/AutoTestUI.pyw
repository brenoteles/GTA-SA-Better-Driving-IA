"""
Real Traffic Fix - AutoTest UI

Runs the mod's AutoTest in GTA SA (build, deploy, launch), follows it live and saves every test, with the settings it
used, to an .xlsx workbook:
  Testes         one row per test: result and metrics (one column per collision cause)
  Configuracoes  one row per test: every key of the RealTrafficFix.ini used + the test options
  Batidas        one row per collision: cause, place, what both drivers were doing
"""
import configparser
import datetime
import hashlib
import os
import queue
import re
import shutil
import sys
import subprocess
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import openpyxl
from openpyxl.styles import Font

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.normpath(os.path.join(HERE, '..', 'RealTrafficFix2'))
PROJECT = os.path.join(PROJECT_DIR, 'RealTrafficFix.vcxproj')
PROJECT_INI = os.path.join(PROJECT_DIR, 'RealTrafficFix.ini')
BUILT_ASI = os.path.join(PROJECT_DIR, 'bin', 'GTA-SA', 'Release', 'RealTrafficFix.SA.asi')
MSBUILD = r'C:\Program Files\Microsoft Visual Studio\18\Community\MSBuild\Current\Bin\MSBuild.exe'
# The game folder: the GTA_SA_DIR environment variable (the same one the build copies the .asi with)
GAME_DIR = os.environ.get('GTA_SA_DIR', '')
if not GAME_DIR or not os.path.isdir(GAME_DIR):
    sys.exit('Set the GTA_SA_DIR environment variable to the GTA San Andreas folder')
SCRIPTS = os.path.join(GAME_DIR, 'scripts')
GAME_INI = os.path.join(SCRIPTS, 'RealTrafficFix.ini')
DEPLOYED_ASI = os.path.join(SCRIPTS, 'RealTrafficFix.SA.asi')
AUTOTEST_INI = os.path.join(SCRIPTS, 'RealTrafficFix.autotest.ini')
MIXSETS_INI = os.path.join(GAME_DIR, 'modloader', 'MixSets', 'MixSets.ini')
# Driver profile of the test car (AutoTest TripProfile)
DRIVERS = {'calmo': 'calm', 'seguro': 'safe', 'normal': 'normal', 'rapido': 'fast', 'agressivo': 'aggressive', 'fuga': 'fleeing',
           'destracionador': 'wheelspin'}
DEFAULT_XLSX = os.path.join(os.path.expanduser('~'), 'Desktop', 'RealTrafficFix_Testes.xlsx')

SCENARIOS = {
    'Viagem Grove Street -> Bayside': {'Scenario': 'route', 'HighwayOnly': '0', 'Route': 'Grove Street'},
    'Viagem Bayside -> LS Centro (estacionar)': {'Scenario': 'route', 'HighwayOnly': '0', 'Route': 'Bayside -> LS'},
    'Rodovias (7 trechos)': {'Scenario': '', 'HighwayOnly': '1'},
    'Cidade (11 locais)': {'Scenario': '', 'HighwayOnly': '0'},
    'Varredura da rota (laser, sem trafego)': {'Scenario': 'scan', 'HighwayOnly': '0', 'Route': 'Grove Street'},
}

LIVE_RE = re.compile(r'\[live ([\d.]+) min\] driving collisions (\d+) \(([\d.]+)/min\), spawn (\d+), oscillations (\d+) \| '
                     r'lane changes ([\d.]+)/veh-km, overtakes (\d+) started (\d+) done (\d+) aborted, board (\d+)/(\d+) denied.*'
                     r'freeway mean (\d+) km/h')
TRIP_RE = re.compile(r'\[trip\] >?(\w+): (\d+) m left, (\d+) km/h \(([^)]*)\), ([\d.]+) km, (\d+) coll')
RESULT_RE = re.compile(r'\[(\w+)\] (ARRIVED|FAIL[^\n]*?) after ([\d.]+) min, ([\d.]+) km driven, (\d+) collisions')
COLL_RE = re.compile(r'=== COLLISION #(\d+) \[([^\]]*)\] t=([\d.]+) min .*? \((-?\d+), (-?\d+), (-?\d+)\) closing ([\d.]+) km/h')


def file_hash(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def read_ini_flat(path):
    """Every key of an ini as 'Section.Key' -> value (keeps the file's order)"""
    cp = configparser.ConfigParser(inline_comment_prefixes=(';', '#'), strict=False, interpolation=None)
    cp.optionxform = str
    with open(path, encoding='utf-8', errors='replace') as f:
        cp.read_string(f.read())
    flat = {}
    for sec in cp.sections():
        for k, v in cp.items(sec):
            flat[f'{sec}.{k}'] = v.strip()
    return flat


def game_running():
    out = subprocess.run(['tasklist', '/FI', 'IMAGENAME eq gta_sa.exe'], capture_output=True, text=True).stdout
    return 'gta_sa.exe' in out


def kill_game():
    subprocess.run(['taskkill', '/IM', 'gta_sa.exe', '/F'], capture_output=True)


def parse_log(text):
    """Metrics of a test from its autotest.log (complete or stopped half-way)"""
    m = {}
    lives = LIVE_RE.findall(text)
    if lives:
        l = lives[-1]
        m.update({'Duracao (min)': float(l[0]), 'Batidas de pilotagem': int(l[1]), 'Batidas/min': float(l[2]),
                  'Batidas spawn/estacionado': int(l[3]), 'Oscilacoes': int(l[4]), 'Trocas de faixa/veic-km': float(l[5]),
                  'Ultrapassagens iniciadas': int(l[6]), 'Ultrapassagens concluidas': int(l[7]),
                  'Ultrapassagens abortadas': int(l[8]),
                  'Reservas negadas %': round(100.0 * int(l[9]) / max(int(l[10]), 1), 1),
                  'Media rodovia (km/h)': int(l[11])})
    trips = TRIP_RE.findall(text)
    if trips:
        t = trips[-1]
        m['Distancia restante (m)'] = int(t[1])
        m['Km rodados'] = float(t[4])
    results = RESULT_RE.findall(text)
    if results:
        r = results[0]
        m['Resultado'] = r[1]
        m['Km rodados'] = float(r[3])
        m['Tempo viagem (min)'] = float(r[2])
        m['Media viagem (km/h)'] = round(float(r[3]) / max(float(r[2]) / 60.0, 1e-6), 1)
    elif 'FAIL' in text:
        m['Resultado'] = re.findall(r'FAIL[^\n]*', text)[-1].strip()
    else:
        m['Resultado'] = 'interrompido' if 'SUMMARY' not in text else 'concluido'
    for name, key in [('street', 'Media rua (km/h)'), ('avenue', 'Media avenida (km/h)'), ('highway', 'Media rodovia final (km/h)')]:
        mm = re.search(r'^\s+' + name + r' \([^)]*\)\s+([\d.]+) km/h', text, re.M)
        if mm:
            m[key] = float(mm.group(1))
    for prefix, key in [('what limits the speed on freeways', 'Limitadores rodovia'), ('what limits the speed elsewhere', 'Limitadores cidade')]:
        mm = re.search(re.escape(prefix) + r'[^:]*:(.*)', text)
        if mm:
            m[key] = mm.group(1).strip().rstrip(',')
    block = re.search(r'collisions by probable cause:\n((?:\s+\d+\s+.*\n)+)', text)
    causes = {}
    if block:
        for line in block.group(1).splitlines():
            mm = re.match(r'\s+(\d+)\s+(.*)', line)
            if mm:
                causes[mm.group(2).strip()] = int(mm.group(1))
    else:  # stopped before the summary: count the CAUSE lines
        for c in re.findall(r'^\s+CAUSE: (.*)$', text, re.M):
            causes[c.strip()] = causes.get(c.strip(), 0) + 1
    return m, causes


def parse_collisions(text):
    rows = []
    parts = re.split(r'\n(?==== )', text)
    for part in parts:
        mm = COLL_RE.search(part)
        if not mm:
            continue
        cause = re.search(r'CAUSE: (.*)', part)
        a = re.search(r'^\s+A: (.*)$', part, re.M)
        b = re.search(r'^\s+B: (.*)$', part, re.M)
        rows.append({'#': int(mm.group(1)), 'Tipo': mm.group(2), 't (min)': float(mm.group(3)),
                     'Causa': cause.group(1).strip() if cause else '', 'X': int(mm.group(4)), 'Y': int(mm.group(5)),
                     'Z': int(mm.group(6)), 'Velocidade de impacto (km/h)': float(mm.group(7)),
                     'Carro A': a.group(1).strip() if a else '', 'Carro B': b.group(1).strip() if b else ''})
    return rows


def append_rows(path, sheet, rows):
    """Appends rows (dicts) to a sheet; new keys become new columns"""
    if os.path.exists(path):
        wb = openpyxl.load_workbook(path)
    else:
        wb = openpyxl.Workbook()
        wb.remove(wb.active)
    ws = wb[sheet] if sheet in wb.sheetnames else wb.create_sheet(sheet)
    headers = [c.value for c in ws[1]] if ws.max_row >= 1 and ws.cell(1, 1).value else []
    for row in rows:
        for k in row:
            if k not in headers:
                headers.append(k)
                ws.cell(1, len(headers), k).font = Font(bold=True)
    for row in rows:
        r = ws.max_row + 1
        for k, v in row.items():
            ws.cell(r, headers.index(k) + 1, v)
    ws.freeze_panes = 'B2'
    for i, h in enumerate(headers, 1):
        ws.column_dimensions[openpyxl.utils.get_column_letter(i)].width = max(10, min(45, len(str(h)) + 2))
    wb.save(path)


class App:
    def __init__(self, root):
        self.root = root
        root.title('Real Traffic Fix - Testes')
        root.geometry('900x700')
        self.q = queue.Queue()
        self.proc = None
        self.running = False
        self.stop_requested = False
        self.log_path = None
        self.mixsets_backup = None

        frm = ttk.LabelFrame(root, text='Teste')
        frm.pack(fill='x', padx=8, pady=6)
        self.scenario = tk.StringVar(value=list(SCENARIOS)[0])
        self.car = tk.StringVar(value='495')
        self.seconds = tk.StringVar(value='90')
        self.maxmin = tk.StringVar(value='35')
        self.nopeds = tk.BooleanVar(value=True)
        self.stoposc = tk.BooleanVar(value=False)
        self.twocars = tk.BooleanVar(value=False)
        self.driver = tk.StringVar(value='rapido')
        self.pending_drivers = []  # tests still to run one after the other (--drivers normal,fast)
        self.build = tk.BooleanVar(value=True)
        self.useini = tk.BooleanVar(value=True)
        self.notes = tk.StringVar()
        self.xlsx = tk.StringVar(value=DEFAULT_XLSX)

        row = 0
        ttk.Label(frm, text='Cenario').grid(row=row, column=0, sticky='w', padx=4, pady=2)
        ttk.Combobox(frm, textvariable=self.scenario, values=list(SCENARIOS), state='readonly', width=34).grid(row=row, column=1, sticky='w')
        ttk.Label(frm, text='Carro (modelo, 495 = Sandking)').grid(row=row, column=2, sticky='w', padx=8)
        ttk.Entry(frm, textvariable=self.car, width=8).grid(row=row, column=3, sticky='w')
        row += 1
        ttk.Label(frm, text='Segundos por local').grid(row=row, column=0, sticky='w', padx=4, pady=2)
        ttk.Entry(frm, textvariable=self.seconds, width=8).grid(row=row, column=1, sticky='w')
        ttk.Label(frm, text='Tempo maximo da viagem (min)').grid(row=row, column=2, sticky='w', padx=8)
        ttk.Entry(frm, textvariable=self.maxmin, width=8).grid(row=row, column=3, sticky='w')
        row += 1
        ttk.Label(frm, text='Motorista do carro de teste').grid(row=row, column=0, sticky='w', padx=4, pady=2)
        ttk.Combobox(frm, textvariable=self.driver, values=list(DRIVERS), state='readonly', width=16).grid(row=row, column=1, sticky='w')
        row += 1
        opts = ttk.Frame(frm)
        opts.grid(row=row, column=0, columnspan=4, sticky='w', pady=2)
        for text, var in [('Sem pedestres', self.nopeds), ('Parar em oscilacao', self.stoposc), ('Dois carros (normal + agressivo)', self.twocars),
                          ('Compilar antes', self.build), ('Usar o .ini do projeto', self.useini)]:
            ttk.Checkbutton(opts, text=text, variable=var).pack(side='left', padx=4)
        row += 1
        ttk.Label(frm, text='Observacoes').grid(row=row, column=0, sticky='w', padx=4, pady=2)
        ttk.Entry(frm, textvariable=self.notes, width=90).grid(row=row, column=1, columnspan=3, sticky='we')
        row += 1
        ttk.Label(frm, text='Planilha').grid(row=row, column=0, sticky='w', padx=4, pady=2)
        ttk.Entry(frm, textvariable=self.xlsx, width=90).grid(row=row, column=1, columnspan=3, sticky='we')

        btns = ttk.Frame(root)
        btns.pack(fill='x', padx=8)
        self.start_btn = ttk.Button(btns, text='Iniciar teste', command=self.start)
        self.start_btn.pack(side='left', padx=4)
        self.stop_btn = ttk.Button(btns, text='Parar', command=self.stop, state='disabled')
        self.stop_btn.pack(side='left', padx=4)
        ttk.Button(btns, text='Abrir .ini do projeto', command=lambda: os.startfile(PROJECT_INI)).pack(side='left', padx=4)
        ttk.Button(btns, text='Abrir planilha', command=self.open_xlsx).pack(side='left', padx=4)

        live = ttk.LabelFrame(root, text='Ao vivo')
        live.pack(fill='x', padx=8, pady=6)
        self.status = tk.StringVar(value='Pronto')
        self.metrics = tk.StringVar(value='')
        self.trip = tk.StringVar(value='')
        self.last = tk.StringVar(value='')
        ttk.Label(live, textvariable=self.status, font=('Segoe UI', 10, 'bold')).pack(anchor='w', padx=4)
        ttk.Label(live, textvariable=self.trip).pack(anchor='w', padx=4)
        ttk.Label(live, textvariable=self.metrics, wraplength=860).pack(anchor='w', padx=4)
        ttk.Label(live, textvariable=self.last, foreground='#b00').pack(anchor='w', padx=4)

        self.text = tk.Text(root, height=20, font=('Consolas', 9))
        self.text.pack(fill='both', expand=True, padx=8, pady=6)
        root.protocol('WM_DELETE_WINDOW', self.on_close)
        root.after(200, self.pump)

    # ---------------------------------------------------------------- UI helpers
    def log(self, line):
        self.q.put(('log', line))

    def pump(self):
        try:
            while True:
                kind, data = self.q.get_nowait()
                if kind == 'log':
                    self.text.insert('end', data.rstrip('\n') + '\n')
                    self.text.see('end')
                elif kind == 'status':
                    self.status.set(data)
                elif kind == 'live':
                    self.metrics.set(data)
                elif kind == 'trip':
                    self.trip.set(data)
                elif kind == 'last':
                    self.last.set(data)
                elif kind == 'done':
                    self.running = False
                    self.start_btn['state'] = 'normal'
                    self.stop_btn['state'] = 'disabled'
                    # Next one of a sequence (not after the user stopped it)
                    if self.pending_drivers and not self.stop_requested:
                        self.driver.set(self.pending_drivers.pop(0))
                        self.root.after(3000, self.start)
        except queue.Empty:
            pass
        self.root.after(200, self.pump)

    def open_xlsx(self):
        if os.path.exists(self.xlsx.get()):
            os.startfile(self.xlsx.get())
        else:
            messagebox.showinfo('Planilha', 'A planilha ainda nao existe: ela e criada no fim do primeiro teste.')

    def on_close(self):
        if self.running and not messagebox.askyesno('Teste rodando', 'Ha um teste rodando. Parar e fechar?'):
            return
        if self.running:
            self.stop_requested = True
            kill_game()
        self.root.destroy()

    # ---------------------------------------------------------------- test run
    def keep_game_focused(self):
        # In this process (a console window every 3 s flashed and took the focus), and only until the game has the
        # focus once: then it's left alone (the user may be using another window)
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import ctypes
        import focus_game
        for _ in range(80):
            time.sleep(3)
            try:
                if self.log_path and os.path.exists(self.log_path) and 'starting at' in open(self.log_path, encoding='utf-8', errors='replace').read():
                    return
            except OSError:
                pass
            hwnd = focus_game.game_window()
            if hwnd and ctypes.windll.user32.GetForegroundWindow() == hwnd:
                return
            if hwnd:
                focus_game.focus()

    def handle_fla_window(self, fla_ini):
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import fla_window
        for _ in range(90):
            time.sleep(1)
            out = fla_window.run(True, fla_ini)
            if 'clicking' in out:
                try:
                    with open(os.path.join(SCRIPTS, 'fla_window_last.txt'), 'w', encoding='utf-8') as f:
                        f.write(out)
                except OSError:
                    pass
                code = re.search(r'disable code: (\S+)', out)
                self.log('fastman92: janela fechada' + (f', codigo {code.group(1)} salvo no ini' if code else
                                                        ' (codigo nao encontrado no texto da janela)'))
                return

    def start(self):
        if self.running:
            return
        self.running = True
        self.stop_requested = False
        self.start_btn['state'] = 'disabled'
        self.stop_btn['state'] = 'normal'
        self.text.delete('1.0', 'end')
        for v in (self.metrics, self.trip, self.last):
            v.set('')
        threading.Thread(target=self.run_test, daemon=True).start()

    def stop(self):
        self.stop_requested = True
        self.q.put(('status', 'Parando...'))
        kill_game()

    def run_test(self):
        tag = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        options = {'Cenario': self.scenario.get(), 'Carro': self.car.get(), 'Segundos por local': self.seconds.get(),
                   'Tempo maximo (min)': self.maxmin.get(), 'Sem pedestres': self.nopeds.get(),
                   'Parar em oscilacao': self.stoposc.get(), 'Dois carros': self.twocars.get(), 'Motorista': self.driver.get(),
                   'Usar ini do projeto': self.useini.get(), 'Observacoes': self.notes.get()}
        try:
            if self.build.get():
                self.q.put(('status', 'Compilando...'))
                res = subprocess.run([MSBUILD, PROJECT, '/p:Configuration=Release GTA-SA', '/p:Platform=Win32', '/m', '/v:m', '/nologo'],
                                     capture_output=True, text=True, errors='replace')
                errors = [l for l in res.stdout.splitlines() if re.search(r'error C\d|error LNK|fatal error', l)]
                if errors or not os.path.exists(BUILT_ASI):
                    for l in errors[:15]:
                        self.log(l)
                    self.q.put(('status', 'ERRO de compilacao - jogo nao iniciado'))
                    return
                self.log('compilado')
            self.q.put(('status', 'Instalando...'))
            for _ in range(30):
                if not game_running():
                    break
                time.sleep(0.5)
            for _ in range(10):
                try:
                    if not os.path.exists(DEPLOYED_ASI) or file_hash(BUILT_ASI) != file_hash(DEPLOYED_ASI):
                        shutil.copy2(BUILT_ASI, DEPLOYED_ASI)
                    break
                except OSError:
                    time.sleep(1)
            if file_hash(BUILT_ASI) != file_hash(DEPLOYED_ASI):
                self.q.put(('status', 'ERRO: nao consegui instalar o .asi'))
                return
            if self.useini.get():
                shutil.copy2(PROJECT_INI, GAME_INI)
            config = read_ini_flat(GAME_INI)
            asi_info = datetime.datetime.fromtimestamp(os.path.getmtime(DEPLOYED_ASI)).strftime('%Y-%m-%d %H:%M')

            sc = SCENARIOS[self.scenario.get()]
            autotest = ['[AutoTest]', 'Mode = mod', f'Tag = {tag}', f'SecondsPerLocation = {self.seconds.get()}',
                        f'StopOnOscillation = {int(self.stoposc.get())}', f'CarModel = {self.car.get()}', 'MaxLocations = 20',
                        f'HighwayOnly = {sc["HighwayOnly"]}', f'Scenario = {sc["Scenario"]}', f'Route = {sc.get("Route", "")}', f'RouteMaxMinutes = {self.maxmin.get()}',
                        f'NoPeds = {int(self.nopeds.get())}', f'TripCars = {2 if self.twocars.get() else 1}', f'TripProfile = {DRIVERS[self.driver.get()]}', 'ExitWhenDone = 1']
            with open(AUTOTEST_INI, 'w', encoding='ascii', newline='') as f:
                f.write('\r\n'.join(autotest) + '\r\n')
            self.set_nopause(True)

            self.log_path = os.path.join(SCRIPTS, f'autotest_{tag}', 'autotest.log')
            self.q.put(('status', f'Rodando ({self.scenario.get()}) - teste {tag}'))
            start = time.time()
            self.proc = subprocess.Popen([os.path.join(GAME_DIR, 'gta_sa.exe')], cwd=GAME_DIR)
            # The fastman92 limit adjuster's start window holds the game until its button is pressed: read its disable code
            # into the FLA ini (it won't come back) and press it (tools/fla_window.py), for up to 90 s
            fla_ini = os.path.join(GAME_DIR, 'fastman92limitAdjuster_GTASA.ini')
            if os.path.exists(fla_ini):
                threading.Thread(target=self.handle_fla_window, args=(fla_ini,), daemon=True).start()
            # The game only loads while its window has the focus (with the terminal in front it sat on a flickering
            # black screen): brought to the front until the test starts (tools/focus_game.py)
            threading.Thread(target=self.keep_game_focused, daemon=True).start()
            self.follow_log()
            elapsed = (time.time() - start) / 60.0
            self.save_results(tag, options, config, asi_info, elapsed)
        except Exception as e:  # keep the UI alive and say what happened
            self.log(f'ERRO: {e!r}')
            self.q.put(('status', f'ERRO: {e}'))
        finally:
            try:
                if os.path.exists(AUTOTEST_INI):
                    os.remove(AUTOTEST_INI)
            except OSError:
                pass
            self.set_nopause(False)
            self.q.put(('done', None))

    def follow_log(self):
        pos = 0
        while True:
            alive = self.proc.poll() is None
            if os.path.exists(self.log_path):
                with open(self.log_path, encoding='utf-8', errors='replace') as f:
                    f.seek(pos)
                    chunk = f.read()
                    pos = f.tell()
                for line in chunk.splitlines():
                    if '[live' in line and 'last collision' not in line:
                        mm = LIVE_RE.search(line)
                        if mm:
                            g = mm.groups()
                            self.q.put(('live', f't {g[0]} min | batidas {g[1]} ({g[2]}/min), spawn {g[3]}, oscilacoes {g[4]} | '
                                                f'trocas de faixa {g[5]}/veic-km | ultrapassagens {g[6]} iniciadas, {g[7]} concluidas, '
                                                f'{g[8]} abortadas | media rodovia {g[11]} km/h'))
                    elif 'last collision' in line:
                        self.q.put(('last', 'Ultima batida: ' + line.split('last collision', 1)[1].strip()))
                    elif '[trip]' in line:
                        mm = TRIP_RE.search(line)
                        if mm:
                            g = mm.groups()
                            self.q.put(('trip', f'{g[0]}: faltam {int(g[1]) / 1000:.2f} km | {g[2]} km/h ({g[3]}) | rodou {g[4]} km | batidas {g[5]}'))
                    elif re.search(r'ARRIVED|FAIL|NO PROGRESS|=== (OSCILLATION|JAM|STUCK)|^--- Location|CAUSE:', line):
                        self.log(line.strip())
            if not alive:
                break
            time.sleep(1.0)

    def save_results(self, tag, options, config, asi_info, elapsed):
        text = ''
        if self.log_path and os.path.exists(self.log_path):
            with open(self.log_path, encoding='utf-8', errors='replace') as f:
                text = f.read()
        if not text:
            self.q.put(('status', 'Sem log: nada salvo'))
            return
        # Collision diagrams (collision_NNN.trk -> .png, seen from above)
        try:
            import collision_diagram
            folder = os.path.dirname(self.log_path)
            pngs = [collision_diagram.render(os.path.join(folder, f)) for f in sorted(os.listdir(folder)) if f.endswith('.trk')]
            self.q.put(('status', f'{len([p for p in pngs if p])} diagramas de batida em {folder}'))
        except Exception as e:
            self.q.put(('status', f'Diagramas de batida: erro {e}'))
        metrics, causes = parse_log(text)
        now = datetime.datetime.now()
        row = {'Teste': tag, 'Data': now.strftime('%Y-%m-%d'), 'Hora': now.strftime('%H:%M'), 'Cenario': options['Cenario'],
               'Carro': options['Carro'], 'Interrompido pelo usuario': 'sim' if self.stop_requested else 'nao',
               'Tempo real (min)': round(elapsed, 1)}
        row.update(metrics)
        for c, n in causes.items():
            row['Causa: ' + c] = n
        row['Observacoes'] = options['Observacoes']
        row['ASI compilado em'] = asi_info
        conf = {'Teste': tag}
        conf.update({'AutoTest.' + k: str(v) for k, v in options.items()})
        conf.update(config)
        coll = parse_collisions(text)
        for c in coll:
            c['Teste'] = tag
        path = self.xlsx.get()
        for attempt in range(2):
            try:
                append_rows(path, 'Testes', [row])
                append_rows(path, 'Configuracoes', [conf])
                if coll:
                    append_rows(path, 'Batidas', [{'Teste': c.pop('Teste'), **c} for c in coll])
                self.q.put(('status', f'Teste {tag} salvo em {path} ({metrics.get("Resultado", "")})'))
                return
            except PermissionError:
                path = os.path.splitext(self.xlsx.get())[0] + f'_{tag}.xlsx'  # open in Excel: a new file
        self.q.put(('status', 'ERRO: nao consegui salvar a planilha'))

    def set_nopause(self, on):
        """MixSets NoPauseWhenUnfocus = 1 during the test (the game keeps running without focus), back afterwards"""
        if not os.path.exists(MIXSETS_INI):
            return
        with open(MIXSETS_INI, encoding='latin-1') as f:
            content = f.read()
        if on:
            self.mixsets_backup = re.search(r'(?m)^NoPauseWhenUnfocus\s*=\s*(\d)', content)
            content = re.sub(r'(?m)^(NoPauseWhenUnfocus\s*=\s*)\d', r'\g<1>1', content)
        elif self.mixsets_backup:
            content = re.sub(r'(?m)^(NoPauseWhenUnfocus\s*=\s*)\d', r'\g<1>' + self.mixsets_backup.group(1), content)
        else:
            return
        with open(MIXSETS_INI, 'w', encoding='latin-1') as f:
            f.write(content)


if __name__ == '__main__':
    import sys
    root = tk.Tk()
    app = App(root)
    # --drivers normal,rapido: one test per driver, one after the other
    if '--drivers' in sys.argv:
        names = [d for d in sys.argv[sys.argv.index('--drivers') + 1].split(',') if d in DRIVERS]
        if names:
            app.driver.set(names[0])
            app.pending_drivers = names[1:]
    # --car 587: test car model
    if '--car' in sys.argv:
        app.car.set(sys.argv[sys.argv.index('--car') + 1])
    # --scenario scan: the road scan instead of the trip
    if '--scenario' in sys.argv:
        want = sys.argv[sys.argv.index('--scenario') + 1]
        for name, sc in SCENARIOS.items():
            if sc['Scenario'] == want:
                app.scenario.set(name)
    if '--start' in sys.argv:  # start a test right away (default options)
        root.after(500, app.start)
    root.mainloop()
