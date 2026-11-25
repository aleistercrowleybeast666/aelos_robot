"""Register the R3 command for the current Windows user."""

import base64
import ctypes
from ctypes import wintypes
import os
from pathlib import Path
import shutil
import winreg


_LAUNCHER_SIGNATURE = 'rem Aelos R3 Deploy command'


def _launch_source(project):
    # Keep the batch file ASCII even when the project is under a Chinese path.
    encoded = base64.b64encode(str(project).encode('utf-8')).decode('ascii')
    code = (
        'import base64,sys; '
        f"sys.path.insert(0,base64.b64decode('{encoded}').decode('utf-8')); "
        'from r3_deployer.__main__ import main; raise SystemExit(main())'
    )
    return (
        '@echo off\n'
        f'{_LAUNCHER_SIGNATURE}\n'
        'setlocal\n'
        'set PYTHONUTF8=1\n'
        'where.exe python >nul 2>&1\n'
        'if errorlevel 1 (\n'
        f'    py -3 -B -c "{code}" %*\n'
        ') else (\n'
        f'    python -B -c "{code}" %*\n'
        ')\n'
        'exit /b %ERRORLEVEL%\n'
    )


def _path_key(value):
    return os.path.normcase(os.path.normpath(os.path.expandvars(value.strip().strip('"'))))


def _register_user_path(directory):
    with winreg.CreateKeyEx(winreg.HKEY_CURRENT_USER, 'Environment',
                            access=winreg.KEY_QUERY_VALUE | winreg.KEY_SET_VALUE) as key:
        try:
            current, kind = winreg.QueryValueEx(key, 'Path')
        except FileNotFoundError:
            current, kind = '', winreg.REG_EXPAND_SZ
        if kind not in (winreg.REG_SZ, winreg.REG_EXPAND_SZ):
            raise RuntimeError('Windows user PATH is not a string.')
        if _path_key(str(directory)) not in {_path_key(part) for part in current.split(';') if part}:
            separator = '' if not current or current.endswith(';') else ';'
            winreg.SetValueEx(key, 'Path', 0, kind, current + separator + str(directory))

    send = ctypes.WinDLL('user32', use_last_error=True).SendMessageTimeoutW
    send.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, ctypes.c_wchar_p,
                     wintypes.UINT, wintypes.UINT, ctypes.POINTER(ctypes.c_size_t)]
    send.restype = ctypes.c_ssize_t
    result = ctypes.c_size_t()
    send(0xFFFF, 0x001A, 0, 'Environment', 0x0002, 1000, ctypes.byref(result))


def main():
    project = Path(__file__).resolve().parents[1]
    directory = Path(os.environ['LOCALAPPDATA']) / 'AelosR3Deploy' / 'bin'
    launcher = directory / 'robot.cmd'
    existing = shutil.which('robot')
    if existing and Path(existing).resolve() != launcher.resolve():
        raise RuntimeError('An existing robot command would take precedence: '+existing)
    if launcher.exists():
        lines = launcher.read_text(encoding='ascii').splitlines()
        if _LAUNCHER_SIGNATURE not in lines[:2]:
            raise RuntimeError('A different robot.cmd already exists: '+str(launcher))
    directory.mkdir(parents=True, exist_ok=True)
    launcher.write_text(_launch_source(project), encoding='ascii', newline='\r\n')
    _register_user_path(directory)
    print('Registered robot for the current Windows user.')


if __name__ == '__main__':
    main()
