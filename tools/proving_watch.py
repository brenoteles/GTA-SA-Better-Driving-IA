"""Proving ground, unattended: keeps the GA running by generations with no one at the keyboard (no Claude, no Fable).

  python tools/proving_watch.py            start the game if it is not running, then watch it

The game runs the GA by itself and saves its state every generation (scripts/proving/ga_state.txt, ga_history.csv,
the trace per generation), so closing the game at any time loses at most the generation running; the next start
resumes from the state. This script only:
  - starts the game again after a crash (a new crash log in the game's logs folder); closing the game yourself
    (no new crash log) ends the watch
  - runs the Gemini reviews every 5 generations in the background (tools/ga_review.py wait --every 5), as before
  - writes scripts/proving/watch.txt: when it started, each restart, the last generation seen
"""
import glob
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GAME_DIR = os.environ.get('GTA_SA_DIR', r'C:\Users\death\Games\GTA\GTA San Andreas')
PROVING = os.path.join(GAME_DIR, 'scripts', 'proving')
LOGS = os.path.join(GAME_DIR, 'logs')
STATE = os.path.join(PROVING, 'ga_state.txt')
WATCH = os.path.join(PROVING, 'watch.txt')


def game_running():
    out = subprocess.run(['tasklist', '/FI', 'IMAGENAME eq gta_sa.exe'], capture_output=True, text=True).stdout
    return 'gta_sa.exe' in out


def crash_logs():
    return set(glob.glob(os.path.join(LOGS, 'gta_sa.exe_*.log')))


def generation():
    try:
        for line in open(STATE, encoding='utf-8', errors='replace'):
            if line.startswith('generation '):
                return int(line.split()[1])
    except (OSError, ValueError):
        pass
    return -1


def note(msg):
    line = time.strftime('%Y-%m-%d %H:%M:%S ') + msg
    print(line, flush=True)
    with open(WATCH, 'a', encoding='utf-8') as f:
        f.write(line + '\n')


def start_game():
    subprocess.run([sys.executable, os.path.join(HERE, 'proving.py'), 'start'], timeout=600)


def main():
    note('watch started, generation %d' % generation())
    review = subprocess.Popen([sys.executable, os.path.join(HERE, 'ga_review.py'), 'wait', '--every', '5'])
    if not game_running():
        start_game()
    seen = crash_logs()
    last_gen = generation()
    restarts = 0
    try:
        while True:
            time.sleep(20)
            g = generation()
            if g != last_gen:
                note('generation %d' % g)
                last_gen = g
            if game_running():
                continue
            time.sleep(5)  # (the crash log is written as the game goes)
            new = crash_logs() - seen
            if not new:
                note('game closed (no crash log): watch ends at generation %d' % g)
                break
            seen |= new
            restarts += 1
            note('crash (%s): restart %d' % (', '.join(os.path.basename(x) for x in sorted(new)), restarts))
            if restarts > 20:
                note('more than 20 crashes: giving up')
                break
            start_game()
    finally:
        if review.poll() is None:
            review.terminate()


if __name__ == '__main__':
    main()
