"""Launch the AELOS R3 control window or the packaged console commands."""

import sys


if __name__ == '__main__':
    if len(sys.argv) > 1:
        from r3_deployer.__main__ import main

        raise SystemExit(main())
    try:
        from r3_deployer.control_window import run_control_window

        run_control_window()
    except Exception as exc:
        from ctypes import windll

        windll.user32.MessageBoxW(None, str(exc), 'AELOS R3 Control', 0x10)
        raise SystemExit(1) from exc
