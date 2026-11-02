"""fastman92 limit adjuster's start window: lists the windows of gta_sa.exe with their controls' text, finds the
'disable code' shown in it, writes it to the FLA ini ('FLA main window disable code = ...') so the window doesn't come
back, and with --click presses its button so the game goes on.

usage: python fla_window.py [--click] [--ini path]
"""
import ctypes
import ctypes.wintypes as wt
import re
import sys

user32 = ctypes.windll.user32
kernel32 = ctypes.windll.kernel32
EnumWindowsProc = ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
BM_CLICK = 0x00F5
WM_GETTEXT, WM_GETTEXTLENGTH = 0x000D, 0x000E


def process_name(pid):
    h = kernel32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
    if not h:
        return ''
    buf = ctypes.create_unicode_buffer(512)
    size = wt.DWORD(512)
    ok = kernel32.QueryFullProcessImageNameW(h, 0, buf, ctypes.byref(size))
    kernel32.CloseHandle(h)
    return buf.value.split('\\')[-1].lower() if ok else ''


def text_of(hwnd):
    n = user32.SendMessageW(hwnd, WM_GETTEXTLENGTH, 0, 0)
    buf = ctypes.create_unicode_buffer(n + 1)
    user32.SendMessageW(hwnd, WM_GETTEXT, n + 1, buf)
    return buf.value


def class_of(hwnd):
    buf = ctypes.create_unicode_buffer(256)
    user32.GetClassNameW(hwnd, buf, 256)
    return buf.value


def windows_of(exe):
    found = []

    def cb(hwnd, _):
        pid = wt.DWORD()
        user32.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        if user32.IsWindowVisible(hwnd) and process_name(pid.value) == exe:
            found.append(hwnd)
        return True
    user32.EnumWindows(EnumWindowsProc(cb), 0)
    return found


def children(hwnd):
    out = []

    def cb(h, _):
        out.append(h)
        return True
    user32.EnumChildWindows(hwnd, EnumWindowsProc(cb), 0)
    return out


def main():
    click = '--click' in sys.argv
    ini = None
    if '--ini' in sys.argv:
        ini = sys.argv[sys.argv.index('--ini') + 1]
    code = None
    for w in windows_of('gta_sa.exe'):
        title = text_of(w)
        kids = children(w)
        print('window %r class %s, %d controls' % (title, class_of(w), len(kids)))
        buttons = []
        for k in kids:
            t, c = text_of(k), class_of(k)
            print('   %-12s %r' % (c, t[:200]))
            if c.lower() == 'button':
                buttons.append((k, t))
            m = re.search(r'disable code[^\n\r]*?[:=]\s*([0-9A-Za-z\-]{4,})', t, re.I)
            if m:
                code = m.group(1)
            elif c.lower() == 'edit' and re.fullmatch(r'[0-9A-Za-z\-]{6,}', t.strip() or '-'):
                code = code or t.strip()
        if 'fastman92' in title.lower() and click and buttons:
            print('clicking', buttons[0][1])
            user32.SendMessageW(buttons[0][0], BM_CLICK, 0, 0)
    if code:
        print('disable code:', code)
        if ini:
            raw = open(ini, 'rb').read().decode('latin-1')
            raw2 = re.sub(r'FLA main window disable code = [^\r\n]*', 'FLA main window disable code = ' + code, raw)
            if raw2 != raw:
                open(ini, 'wb').write(raw2.encode('latin-1'))
                print('written to', ini)


if __name__ == '__main__':
    main()
