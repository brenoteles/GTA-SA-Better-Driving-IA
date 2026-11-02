"""Brings the game's window to the front. GTA SA only loads while its window has the focus: launched from the test UI
with the terminal in front, it sat on a flickering black loading screen until someone clicked it.

Windows only lets the foreground process hand the focus over, so: a synthetic Alt key press (counts as user input)
and attaching to the foreground window's thread before SetForegroundWindow.

usage: python focus_game.py
"""
import ctypes
import ctypes.wintypes as wt

user32 = ctypes.windll.user32
kernel32 = ctypes.windll.kernel32
EnumWindowsProc = ctypes.WINFUNCTYPE(wt.BOOL, wt.HWND, wt.LPARAM)
SW_RESTORE, VK_MENU, KEYEVENTF_KEYUP = 9, 0x12, 0x0002


def game_window():
    found = []

    def cb(hwnd, _):
        buf = ctypes.create_unicode_buffer(256)
        user32.GetClassNameW(hwnd, buf, 256)
        if buf.value == 'Grand theft auto San Andreas' and user32.IsWindowVisible(hwnd):
            found.append(hwnd)
        return True
    user32.EnumWindows(EnumWindowsProc(cb), 0)
    return found[0] if found else None


def focus():
    hwnd = game_window()
    if not hwnd:
        print('game window not found')
        return False
    if user32.GetForegroundWindow() == hwnd:
        print('already in front')
        return True
    fg = user32.GetForegroundWindow()
    fg_thread = user32.GetWindowThreadProcessId(fg, None)
    me = kernel32.GetCurrentThreadId()
    user32.AttachThreadInput(me, fg_thread, True)
    user32.keybd_event(VK_MENU, 0, 0, 0)
    user32.keybd_event(VK_MENU, 0, KEYEVENTF_KEYUP, 0)
    if user32.IsIconic(hwnd):
        user32.ShowWindow(hwnd, SW_RESTORE)
    user32.BringWindowToTop(hwnd)
    ok = user32.SetForegroundWindow(hwnd)
    user32.AttachThreadInput(me, fg_thread, False)
    if user32.GetForegroundWindow() != hwnd:
        # Refused (someone typing in another window): the shell's own switch, then minimise and restore it
        user32.SwitchToThisWindow(hwnd, True)
    # (No minimise/restore: GTA opens its pause menu on it, and the test UI calling this every 3 s kept it there)
    print('focused' if user32.GetForegroundWindow() == hwnd else 'focus refused (%d)' % ok)
    return user32.GetForegroundWindow() == hwnd


if __name__ == '__main__':
    focus()
