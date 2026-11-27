"""Optional Linux boot timing collected once by the warm owner."""

import subprocess


def BootTiming_ReadSshReady():
    """Return ssh.service's last activation in CLOCK_MONOTONIC seconds since boot."""
    try:
        result = subprocess.run(
            ['systemctl', 'show', 'ssh.service', '--property=ActiveEnterTimestampMonotonic'],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=2, check=True)
        key, sep, value = result.stdout.strip().partition(b'=')
        if key == b'ActiveEnterTimestampMonotonic' and sep and value.isdigit():
            microseconds = int(value)
            if microseconds > 0:
                return microseconds / 1_000_000
    except (OSError, subprocess.SubprocessError, ValueError):
        pass
    return None
