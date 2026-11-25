"""Copy a checksummed R3 ABI 42 Lua overlay to a physically mounted Windows U-disk."""

import ctypes
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import uuid


_MCU_FILES = (
    'lua/craic_actions.lua',
    'lua/craic_auto_bridge.lua',
    'lua/craic_button_gaits.lua',
    'lua/craic_config.lua',
    'lua/craic_future_actions.lua',
    'lua/craic_runtime.lua',
    'lua/craic_single_micro.lua',
    'dat/craic_grab_safety.lua',
    'lua/main.lua',
)
_PROFILE = 'dat/craic_robot_profile.lua'
_BACKUP_SCHEMA = 'r3-usb-backup/1'


@dataclass(frozen=True)
class _Volume:
    root: Path
    serial: str
    label: str
    filesystem: str
    profile_hash: str

    def identity(self):
        return dict(serial=self.serial, label=self.label,
                    filesystem=self.filesystem, profile_sha256=self.profile_hash)


def _digest(data):
    return hashlib.sha256(data).hexdigest() if data is not None else None


def _regular_path(root, relative):
    """Resolve only known paths and reject on-volume junctions and symlinks."""
    path = root
    parts = relative.split('/')
    for index, name in enumerate(parts):
        path = path / name
        try:
            entry = path.lstat()
        except FileNotFoundError:
            return path.joinpath(*parts[index + 1:])
        if (stat.S_ISLNK(entry.st_mode)
                or getattr(entry, 'st_file_attributes', 0) & stat.FILE_ATTRIBUTE_REPARSE_POINT):
            raise ValueError(f'Unexpected link in USB path: {path}')
        expected = stat.S_ISREG if index == len(parts) - 1 else stat.S_ISDIR
        if not expected(entry.st_mode):
            raise ValueError(f'Unexpected file type: {path}')
    return path


def _fixed_usb(letter):
    # Some USB mass-storage devices advertise fixed disks. Resolve the actual partition.
    command = f"(Get-Partition -DriveLetter {letter} -ErrorAction Stop | Get-Disk -ErrorAction Stop).BusType"
    try:
        result = subprocess.run(
            ['powershell.exe', '-NoProfile', '-NonInteractive', '-Command', command],
            capture_output=True, text=True, timeout=8, check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return result.returncode == 0 and result.stdout.strip().upper() == 'USB'


def _mounted_volume(drive):
    if os.name != 'nt':
        raise RuntimeError('Physical USB mode requires Windows.')
    from ctypes import wintypes

    value = str(drive).strip()
    if not re.fullmatch(r'[A-Za-z]:[\\/]?', value):
        raise ValueError('Specify the robot U-disk drive letter, for example E:.')
    root = Path(value[0].upper() + ':\\')
    if root.drive.upper() == os.environ.get('SystemDrive', 'C:').upper():
        raise ValueError('The system drive cannot be the robot U-disk.')
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetDriveTypeW.argtypes = [wintypes.LPCWSTR]
    kernel.GetDriveTypeW.restype = wintypes.UINT
    drive_type = kernel.GetDriveTypeW(str(root))
    if not (drive_type == 2 or drive_type == 3 and _fixed_usb(value[0].upper())):
        raise ValueError(f'{root} is not a locally mounted removable or USB volume.')
    label = ctypes.create_unicode_buffer(256)
    filesystem = ctypes.create_unicode_buffer(64)
    serial = wintypes.DWORD()
    kernel.GetVolumeInformationW.argtypes = [
        wintypes.LPCWSTR, wintypes.LPWSTR, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD),
        ctypes.POINTER(wintypes.DWORD), ctypes.POINTER(wintypes.DWORD), wintypes.LPWSTR,
        wintypes.DWORD,
    ]
    kernel.GetVolumeInformationW.restype = wintypes.BOOL
    if not kernel.GetVolumeInformationW(str(root), label, len(label), ctypes.byref(serial),
                                        None, None, filesystem, len(filesystem)):
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        profile = _regular_path(root, _PROFILE).read_bytes()
    except FileNotFoundError as error:
        raise ValueError(f'{root} has no R3 MCU profile.') from error
    profile_text = profile.decode('utf-8-sig')
    robot_ids = re.findall(r'(?m)^\s*robot_id\s*=\s*(\d+)\s*[,}]', profile_text)
    if robot_ids != ['3'] or not _regular_path(root, 'lua/main.lua').is_file():
        raise ValueError(f'{root} does not contain the R3 robot profile and MCU layout.')
    return _Volume(root, f'{serial.value:08X}', label.value, filesystem.value, _digest(profile))


@contextmanager
def _lock_volume(volume):
    from ctypes import wintypes

    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.CreateMutexW.argtypes = [wintypes.LPVOID, wintypes.BOOL, wintypes.LPCWSTR]
    kernel.CreateMutexW.restype = wintypes.HANDLE
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    kernel.ReleaseMutex.argtypes = [wintypes.HANDLE]
    ctypes.set_last_error(0)
    handle = kernel.CreateMutexW(None, True, f'Local\\AelosR3Usb-{volume.serial}')
    error = ctypes.get_last_error()
    if not handle:
        raise ctypes.WinError(error)
    if error == 183:  # ERROR_ALREADY_EXISTS
        kernel.CloseHandle(handle)
        raise RuntimeError('Another deployment or restore is using this R3 USB volume.')
    try:
        yield
    finally:
        kernel.ReleaseMutex(handle)
        kernel.CloseHandle(handle)


def _verify_volume(volume):
    if _mounted_volume(volume.root) != volume:
        raise RuntimeError('The mounted R3 USB volume changed during the operation.')


def _unique_keys(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise ValueError(f'Duplicate manifest key: {key}')
        obj[key] = value
    return obj


def _load_json(path):
    with path.open(encoding='utf-8-sig') as stream:
        return json.load(stream, object_pairs_hook=_unique_keys)


def _validate_hashes(manifest, allow_missing=False):
    if not isinstance(manifest, dict) or set(manifest) != set(_MCU_FILES):
        raise ValueError('USB manifest must enumerate the nine R3 ABI 42 overlay files.')
    for relative, digest in manifest.items():
        if digest is None and allow_missing:
            continue
        if not isinstance(digest, str) or not re.fullmatch('[0-9A-Fa-f]{64}', digest):
            raise ValueError(f'Invalid SHA-256 for {relative}')
    return {path: value.lower() if value is not None else None for path, value in manifest.items()}


def _overlay(overlay, manifest):
    root = Path(overlay).expanduser().resolve(strict=True)
    expected = _validate_hashes(_load_json(Path(manifest).expanduser()))
    payload = {path: _regular_path(root, path).read_bytes() for path in _MCU_FILES}
    if any(_digest(data) != expected[path] for path, data in payload.items()):
        raise ValueError('The selected USB overlay does not match its SHA-256 manifest.')
    bridge = payload['lua/craic_auto_bridge.lua']
    if not re.search(rb'\bR3_VISUAL_PICKUP_MISSION_ABI\s*=\s*1\b', bridge) or b'0x2A00' not in bridge:
        raise ValueError('The overlay does not declare the R3 V5 automatic bridge ABI.')
    return root, expected, payload


def _current(root, relative):
    try:
        return _regular_path(root, relative).read_bytes()
    except FileNotFoundError:
        return None


def _atomic_write(path, data):
    temp = path.with_name(f'.{path.name}.{uuid.uuid4().hex}.part')
    try:
        with temp.open('xb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp, path)
    finally:
        temp.unlink(missing_ok=True)


def _status(volume, expected, current):
    return {
        'robot_id': 3, 'drive': str(volume.root), 'volume': volume.identity(),
        'files': expected, 'different_files': [path for path in _MCU_FILES
                                               if current[path] != expected[path]],
    }


def inspect_usb(overlay, manifest, drive):
    """Verify the selected overlay and report how its files compare to the mounted R3."""
    root, expected, _ = _overlay(overlay, manifest)
    volume = _mounted_volume(drive)
    return _inspect_files(root, expected, volume)


def _inspect_files(root, expected, volume):
    current = {path: _digest(_current(volume.root, path)) for path in _MCU_FILES}
    _verify_volume(volume)
    return dict(_status(volume, expected, current), overlay=str(root))


def McuReadback_VerifyAndSave(drive, overlay, receipt_dir):
    """Read an already copied R3 volume and save the result on the PC."""
    root = Path(overlay)
    report = inspect_usb(root, root / 'MANIFEST.json', drive)
    return _save_readback(report, receipt_dir)


def _connected_r3_volumes():
    if os.name != 'nt':
        return []
    from ctypes import wintypes

    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    kernel.GetLogicalDrives.restype = wintypes.DWORD
    kernel.GetDriveTypeW.argtypes = [wintypes.LPCWSTR]
    kernel.GetDriveTypeW.restype = wintypes.UINT
    mask = kernel.GetLogicalDrives()
    if not mask:
        raise ctypes.WinError(ctypes.get_last_error())
    volumes = []
    for index in range(26):
        if not mask & (1 << index):
            continue
        root = Path(chr(ord('A') + index) + ':\\')
        if (root.drive == os.environ.get('SystemDrive', 'C:').upper()
                or kernel.GetDriveTypeW(str(root)) not in (2, 3)):
            continue
        try:
            if (root / _PROFILE).is_file():
                volumes.append(_mounted_volume(root.drive))
        except (OSError, ValueError):
            continue
    return volumes


def McuReadback_VerifyConnected(overlay, receipt_dir, log):
    """Find one mounted R3 and read back its files. No volume is an ordinary Pi-only deployment."""
    volumes = _connected_r3_volumes()
    if not volumes:
        return None
    if len(volumes) != 1:
        drives = '、'.join(str(volume.root) for volume in volumes)
        raise ValueError(f'发现多个 R3 U 盘（{drives}）。请只保留要部署的机器人。')
    volume = volumes[0]
    log(f'已识别 R3 U 盘 {volume.root}，正在核验 MCU 文件。')
    overlay = Path(overlay)
    root, expected, _ = _overlay(overlay, overlay / 'MANIFEST.json')
    with _lock_volume(volume):
        return _save_readback(_inspect_files(root, expected, volume), receipt_dir)


def _save_readback(report, receipt_dir):
    if report['different_files']:
        raise ValueError('MCU U 盘与本机覆盖包不一致。差异文件为 ' + '、'.join(report['different_files']))
    directory = Path(receipt_dir).resolve()
    volume = Path(report['drive']).resolve()
    if directory == volume or volume in directory.parents:
        raise ValueError('请把核验回执保存在电脑上。')
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
    path = directory / f'r3-mcu-{stamp}-{uuid.uuid4().hex[:8]}.json'
    report.update(readback_all_match=True, reload_confirmed=False)
    _atomic_write(path, (json.dumps(report, ensure_ascii=False, indent=2) + '\n').encode('utf-8'))
    return dict(report, receipt_path=str(path))


def _write_backup(backup_root, volume, original, expected):
    root = Path(backup_root).expanduser().resolve()
    if root == volume.root or volume.root in root.parents:
        raise ValueError('Save USB backups on the computer, outside the selected U-disk.')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S')
    backup = root / f'r3-usb-{stamp}-{uuid.uuid4().hex[:8]}'
    backup.mkdir(parents=True)
    for relative, data in original.items():
        if data is not None:
            dest = backup / 'files' / relative
            dest.parent.mkdir(parents=True, exist_ok=True)
            _atomic_write(dest, data)
            if _digest(dest.read_bytes()) != _digest(data):
                raise RuntimeError(f'Could not verify the local backup of {relative}')
    receipt = {'schema': _BACKUP_SCHEMA, 'volume': volume.identity(),
               'original_files': {path: _digest(data) for path, data in original.items()},
               'overlay_files': expected}
    _atomic_write(backup / 'backup.json', json.dumps(receipt, ensure_ascii=False, indent=2).encode('utf-8'))
    return backup


def _replace_group(volume, original, replacement, backup):
    attempted = []
    try:
        for path in _MCU_FILES:
            _verify_volume(volume)
            if _current(volume.root, path) != original[path]:
                raise RuntimeError(f'USB file changed before replacement: {path}')
            new = replacement[path]
            if new == original[path]:
                continue
            dest = _regular_path(volume.root, path)
            attempted.append(path)
            if new is None:
                dest.unlink()
            else:
                _atomic_write(dest, new)
            if _current(volume.root, path) != new:
                raise RuntimeError(f'USB readback mismatch: {path}')
        _verify_volume(volume)
        if any(_current(volume.root, path) != replacement[path] for path in _MCU_FILES):
            raise RuntimeError('Final USB readback did not match the complete overlay.')
    except (Exception, KeyboardInterrupt) as error:
        failed = []
        for path in reversed(attempted):
            try:
                _verify_volume(volume)
                dest = _regular_path(volume.root, path)
                if _current(volume.root, path) != original[path]:
                    if original[path] is None:
                        dest.unlink(missing_ok=True)
                    else:
                        _atomic_write(dest, original[path])
                if _current(volume.root, path) != original[path]:
                    failed.append(path)
            except Exception:
                failed.append(path)
        rollback = 'rollback incomplete for ' + ', '.join(failed) if failed else 'attempted changes rolled back'
        raise RuntimeError(f'USB replacement failed: {error}. {rollback}. PC backup: {backup}') from error


def deploy_usb(overlay, manifest, drive, backup_root):
    """Back up and replace the nine files, then read them back."""
    root, expected, payload = _overlay(overlay, manifest)
    volume = _mounted_volume(drive)
    with _lock_volume(volume):
        _verify_volume(volume)
        original = {path: _current(volume.root, path) for path in _MCU_FILES}
        backup = _write_backup(backup_root, volume, original, expected)
        _replace_group(volume, original, payload, backup)
    current = {path: _digest(data) for path, data in payload.items()}
    return dict(_status(volume, expected, current), overlay=str(root), backup=str(backup),
                readback_all_match=True, reload_confirmed=False)


def restore_usb(drive, backup):
    """Restore an earlier snapshot if the mounted volume and files still match it."""
    backup = Path(backup).expanduser().resolve(strict=True)
    record = _load_json(backup / 'backup.json')
    if not isinstance(record, dict) or record.get('schema') != _BACKUP_SCHEMA:
        raise ValueError('Select an R3 USB deployment backup directory.')
    expected = _validate_hashes(record.get('original_files'), allow_missing=True)
    updated = _validate_hashes(record.get('overlay_files'))
    volume = _mounted_volume(drive)
    if record.get('volume') != volume.identity():
        raise ValueError('This backup is for a different R3 USB volume.')
    payload = {path: _current(backup, 'files/' + path) for path in _MCU_FILES}
    if any(_digest(data) != expected[path] for path, data in payload.items()):
        raise ValueError('The selected USB backup files do not match their SHA-256 receipt.')
    with _lock_volume(volume):
        _verify_volume(volume)
        original = {path: _current(volume.root, path) for path in _MCU_FILES}
        if any(_digest(data) not in (expected[path], updated[path]) for path, data in original.items()):
            raise ValueError('USB files have changed since this backup. Choose the matching backup.')
        _replace_group(volume, original, payload, backup)
    return dict(_status(volume, expected, expected), backup=str(backup),
                readback_all_match=True, reload_confirmed=False)
