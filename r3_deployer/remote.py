"""Standard-library transaction helper executed on the verified R3 Pi."""

import ctypes
import errno
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import uuid


DEFAULT_PATH = Path('/home/lemon/aelos_auto_r3_python')
OWNER_DIR = Path('/home/lemon/catkin_ws/src/aelos_smart_ros/scripts/CMDcontrol_node')
OWNER_NAME = 'r3_auto_warm.py'
REQUIRED_PROGRAMS = frozenset(('app/warm_client.py', 'app/run_auto.py', 'app/task_runner.py', 'owner/'+OWNER_NAME))
_SERIAL_ORIGINAL_SHA = 'c2d0a64de4e28a3b8e914d623f4a8ba56f30533963cf50f6c5c394ee4f4981e2'
_SERIAL_TIMED_SHA = '8f396415968782628aaff3276af2a604e0ae9e34c7fc73556101be6ed1e6b09d'
OWNER_SUPPORT = ('r3_micro_gate.py', 'r3_auto_gate.py')
OWNER_VERSION = 'r3-visual-pickup-20261109-v5'
NODE = '/CMDcontrol_server_node'
MARKER = '.r3-deployment.json'
DEVICE_IDENTITY_SHA256 = '7e4b8c1ab846f4d8e883e8376f657d098d8cadc62e691de3c18e9ad41509f0d7'
MCU_MANIFEST_PATH = 'app/reference/cmd0_mcu_manifest.json'
_MCU_MANIFEST_SHA256 = 'ff778dc5494639832ebc50ee2bbc3e12324c6997c634c18293d25a9e1b9d6f2f'
_MCU_TUNED_MICRO_SHA256 = '48931c9beea5ceb9122bfaa705457279c8a1591ebb4212d0c665b28b86244004'
_MCU_MEDIUM_SHA256 = {
    'lua/craic_auto_bridge.lua': '12e51ad9684904e88b8d36e4ee701cf37c14bb71c5c5adae9cff41f168c3710f',
    'lua/craic_single_micro.lua': 'e8d3c542a0c4e7ba9f737952a09d5e761b4e0a1a41711c954a5c7ec50dee6121',
}
_MCU_RIGHT5_SHA256 = {
    **_MCU_MEDIUM_SHA256,
    'lua/craic_auto_bridge.lua': 'b921f01f9937ff22eb2f5f00184fcbcee2ba30c242f65540aaf45c29beb98426',
}
_MCU_PAIR_SHA256 = {
    **_MCU_RIGHT5_SHA256,
    'lua/craic_actions.lua': '7fa0daf3caae45c32379a668c6e0f2f8954607d119de9bd5ecba9a3fe424995d',
    'lua/craic_auto_bridge.lua': '4cfeeb48b3c2a11d5f1bcd8c561cc280a7fb4595e64ccf9bd36ad8e17f0b1cc8',
    'lua/craic_runtime.lua': '96328475f14389ba82c6136aa5b16ce74d3f35cbff0388b0aaa8360e12e7893f',
    'lua/craic_single_micro.lua': 'b73f1a9d704612a5f4f7c99ad0853c1757318364c4da36c543ad2be09bc1cf56',
}
_MCU_PICKUP_BACKWARD4_SHA256 = {
    **_MCU_PAIR_SHA256,
    'lua/craic_auto_bridge.lua': '7d55f4a959f91dc3d35924dc2f35ad5a143867f709be2e325ab14359148193a4',
}
_MCU_PICKUP_BACKWARD6_SHA256 = {
    **_MCU_PICKUP_BACKWARD4_SHA256,
    'lua/craic_auto_bridge.lua': '0ea67743b8f03e55d40ee5c8e73cc8ca396e0eadbaeb7a99569e5cb0b6212594',
}
_MCU_PICKUP_BACKWARD8_SHA256 = {
    **_MCU_PICKUP_BACKWARD6_SHA256,
    'lua/craic_auto_bridge.lua': '14a4687b14c94f8160f056f6c0950ebc0dcca561a15d24685d5523c1e0c8c94d',
}
_MCU_PICKUP_BACKWARD8_TUNED_LEFT_SHA256 = {
    **_MCU_PICKUP_BACKWARD8_SHA256,
    'lua/craic_single_micro.lua': '390935bd8ef1c073c0609d8bac7c171a572539d270330d3db8d630b8de0a751b',
}
_MCU_PICKUP_BACKWARD8_LOW_RELEASE_SHA256 = {
    **_MCU_PICKUP_BACKWARD8_TUNED_LEFT_SHA256,
    'lua/craic_actions.lua': '94e6eb25149bc6e00181997618c3bea54ba28a13af622fc38fa6c15ffddc3989',
}
_MCU_PICKUP_LEVEL_BACKWARD6_SHA256 = {
    **_MCU_PICKUP_BACKWARD8_LOW_RELEASE_SHA256,
    'lua/craic_button_gaits.lua': 'b950f0adb939ced0c38172290c781e5f171882394aec5b8d6c7058ce31017846',
}
_MCU_EMPTY_RECOVERY_SHA256 = {
    **_MCU_PICKUP_LEVEL_BACKWARD6_SHA256,
    'lua/craic_auto_bridge.lua': '8d96ece61e0d817a03c112983ed573755fa309ff739c64334faeef10b482fb53',
}
_MCU_INTERMEDIATE_STRAFE_SHA256 = {
    **_MCU_EMPTY_RECOVERY_SHA256,
    'lua/craic_auto_bridge.lua': 'c696f17ca3be1da5a11c21046db433c2849ba54451ddf6c8cd739a5cf2382d13',
}
_MCU_TRANSFER_STRAFE_SHA256 = {
    **_MCU_INTERMEDIATE_STRAFE_SHA256,
    'lua/craic_auto_bridge.lua': 'cca85e5f51fcdb5f25f3e41705aad57ad846885a362e8390537ba388e5b6be61',
    'lua/craic_single_micro.lua': '4043c09d9a77a603df28346c27f0b11680eb279dcdd059b1da074f5e0c1222b4',
}
_MCU_LEVEL_BACKWARD8_SHA256 = {
    **_MCU_TRANSFER_STRAFE_SHA256,
    'lua/craic_auto_bridge.lua': '1cf0bf7bcfe7b0511a171373ccda1e8a77233546b5acc4cc9eb68fa9b26b1da5',
    'lua/craic_button_gaits.lua': '0ced4cce313f9c961a1d2f8dfbb52779fcec59a342de8063f41431e00de6e7f9',
}
_MCU_RELAXED_CARRY_HIP_SHA256 = {
    **_MCU_LEVEL_BACKWARD8_SHA256,
    'lua/craic_button_gaits.lua': '19ea709c83ac4305109382e37af6d01f8d8b9c8d00e2f1d6d2afa4b10d18b454',
    'lua/craic_single_micro.lua': '053c6c9413104e02f0fabce52cf69fb916e704c7295a6217dd0151b792dbd69e',
    'lua/craic_runtime.lua': '32ef7f457609175edf2b5f53629422dfd5aef09a07a7ea1131b3a2cb5b800352',
}
_MCU_PLACEMENT_STRAFE_SHA256 = {
    **_MCU_RELAXED_CARRY_HIP_SHA256,
    'lua/craic_auto_bridge.lua': 'f45520484bf762612ac30422d43e07dc345319a674c0409684489544c00558cf',
    'lua/craic_button_gaits.lua': '8e88a582008615204d14b28230ae071dcc597f4a60e6f47a3789ff12261d22ea',
}
_MCU_STANDING_START_SHA256 = {
    **_MCU_PLACEMENT_STRAFE_SHA256,
    'lua/craic_actions.lua': '045a4241b23a8e2fcd2663400f6e77fe58e5ec9c048880dfe5114b9e3ac326b6',
    'lua/craic_auto_bridge.lua': '778427b5b57299e28f8ceb9afa56514c4f3862f45b3226cd8a5361deb92d99f7',
}
_MCU_PRIOR_TUNED_MICRO_SHA256 = (
    'a8156fefd6da3709882508abc7cc3e3d1a5dc499ee17eefcb3ed846abafffd14',
    'f0e52e82024132ec0c7be7d5aa556e756a7642fd2c489a1172920ea11be78bf1',
)


def _json(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise ValueError(f'Invalid JSON in {path}: {exc}') from exc


def _digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _regular(path, optional=False):
    if path.is_symlink() or (path.exists() and not path.is_file()):
        raise RuntimeError('Expected an ordinary file: '+str(path))
    if not optional and not path.is_file():
        raise RuntimeError('Missing file: '+str(path))
    return path


def _relative(name):
    entry = PurePosixPath(name)
    if (entry.is_absolute() or '..' in entry.parts or not entry.parts
            or str(entry) != name or '\\' in name or any(ord(ch) < 32 for ch in name)):
        raise ValueError('Invalid relative path in bundle')
    return entry


def _verified_local_identity():
    serial = re.search(r'^Serial\s*:\s*(\S+)', Path('/proc/cpuinfo').read_text(), re.M)
    machine = Path('/etc/machine-id').read_text().strip()
    if not serial or hashlib.sha256((serial[1] + '\0' + machine).encode('utf-8')).hexdigest() != DEVICE_IDENTITY_SHA256:
        raise RuntimeError('R3 CPU / machine / deployment identity mismatch.')
    return serial[1], machine


def _target(value):
    path = Path(value)
    home = Path.home().resolve()
    if (not path.is_absolute() or path != path.resolve() or home not in path.parents
            or not path.is_dir() or path.stat().st_uid != os.getuid()):
        raise RuntimeError('Target must be an existing R3 application directory owned by the SSH account.')
    serial, machine = _verified_local_identity()
    record = _json(_regular(path/'deployment.json'))
    if (record.get('robot_id') != 3 or record.get('cpu_serial') != serial
            or record.get('machine_id') != machine):
        raise RuntimeError('R3 CPU / machine / deployment identity mismatch.')
    return path


def _transaction(path, ticket):
    if not re.fullmatch('[0-9a-f]{32}', ticket):
        raise ValueError('Invalid deployment receipt')
    return path.with_name('.'+path.name+'.r3-'+ticket)


def _status(require_idle=False):
    address = Path.home()/'.local/state/r3_simple/control.sock'
    with socket.socket(socket.AF_UNIX) as conn:
        conn.settimeout(3)
        try:
            conn.connect(str(address))
            conn.sendall(b'{"protocol":"r3.warm.v1","op":"status"}\n')
            with conn.makefile('rb') as reply:
                row = json.loads(reply.readline(131072))
        except FileNotFoundError as exc:
            raise FileNotFoundError(exc.errno, '树莓派 ROS 控制节点的状态套接字不存在', str(address)) from exc
        except OSError as exc:
            raise OSError(exc.errno, f'读取 ROS 控制节点状态失败（{exc}）', str(address)) from exc
    owner = row.get('owner', {})
    if (row.get('protocol') != 'r3.warm.v1' or owner.get('robot_id') != 3
            or owner.get('owner_version') != OWNER_VERSION or owner.get('expected_wire_abi') != 42):
        raise RuntimeError('R3 warm owner / ABI mismatch.')
    if require_idle and not (row.get('prepared') is True and row.get('task_active') is False
                             and owner.get('mode') == 'manual' and owner.get('in_flight') is False):
        raise RuntimeError(f"ROS 控制节点尚未手动就绪（prepared={row.get('prepared')}, "
                           f"task_active={row.get('task_active')}, mode={owner.get('mode')}, "
                           f"in_flight={owner.get('in_flight')}）")
    return row


def _Ros_Run(*args):
    script = ('for setup in /opt/ros/noetic/setup.bash "$HOME/ros_catkin_ws/install_isolated/setup.bash" '
              '"$HOME/catkin_ws/devel/setup.bash"; do '
              'if test -r "$setup"; then . "$setup" >/dev/null; fi; done; exec "$@"')
    return subprocess.run(['bash', '-c', script, 'r3-deploy', *args],
                          text=True, capture_output=True, timeout=10, check=True).stdout


def _ros(*args):
    return _Ros_Run('rosnode', *args)


def _Gate_CheckWarmLock(pid):
    # The ROS gate lacks warm-task activity. A vanished socket may still have an
    # owner with an accepted task. The unprivileged SSH user can read /proc/locks.
    lock = _regular(Path.home()/'.local/state/r3_simple/warm.lock', optional=True)
    info = lock.stat() if lock.is_file() else None
    mount = next(parent for parent in lock.parents if parent.exists()).stat()
    device = (os.major(mount.st_dev), os.minor(mount.st_dev))
    for line in Path('/proc/locks').read_text().splitlines():
        fields = line.split()
        if len(fields) < 6 or fields[1:4] != ['FLOCK', 'ADVISORY', 'WRITE']:
            continue
        major, minor, inode = fields[5].split(':')
        if (int(major, 16), int(minor, 16)) != device:
            continue
        # A matching owner with a different inode may hold an unlinked warm.lock.
        if int(fields[4]) == pid or (info is not None and int(inode) == info.st_ino):
            raise RuntimeError('R3 控制进程仍持有任务服务锁，无法通过备用状态确认空闲。')


def _Gate_ReadIdleOwner():
    pid = _node_pid()
    _Gate_CheckWarmLock(pid)
    probe = """import json, rospy
from std_msgs.msg import String
rospy.init_node('r3_deploy_status', anonymous=True, disable_signals=True)
message = rospy.wait_for_message('/r3_auto/status', String, timeout=3)
print(json.dumps([message._connection_header.get('callerid'), json.loads(message.data)]))
"""
    try:
        raw = _Ros_Run('python3', '-c', probe)
        publisher, owner = json.loads(raw.strip().splitlines()[-1])
    except (subprocess.SubprocessError, ValueError, IndexError, TypeError) as exc:
        raise RuntimeError('读取 ROS 独立状态 /r3_auto/status 失败，无法确认可恢复部署。') from exc
    if (publisher != NODE or not isinstance(owner, dict) or owner.get('schema') != 'r3.simple.v1'
            or owner.get('robot_id') != 3 or owner.get('expected_wire_abi') != 42
            or owner.get('owner_version') != OWNER_VERSION
            or not re.fullmatch(r'[0-9a-f]{32}', str(owner.get('session', '')))):
        raise RuntimeError('ROS 独立状态未确认 R3 的发布节点、身份、ABI 或会话。')
    published = owner.get('published_at_s')
    if type(published) not in (int, float) or not 0 <= time.time() - published <= 2:
        raise RuntimeError('ROS 独立状态的采样时间异常或已过期。')
    if (owner.get('mode') != 'manual' or owner.get('in_flight') is not False
            or 'current_operation' not in owner or owner['current_operation'] is not None
            or owner.get('task_active', False) is not False):
        raise RuntimeError('ROS 独立状态未确认手动空闲，不能进行恢复部署。')
    _Gate_CheckWarmLock(pid)
    return owner


def _Deployment_Status(root, require_idle):
    try:
        return _status(require_idle=require_idle)
    except OSError as exc:
        if exc.errno not in (errno.ENOENT, errno.ECONNREFUSED) or root != DEFAULT_PATH:
            raise
        marker = _regular(root/MARKER, optional=True)
        if not marker.is_file():
            raise
        record = _json(marker)
        if (not isinstance(record, dict) or record.get('path') != str(root)
                or not re.fullmatch(r'[0-9a-f]{32}', str(record.get('id', '')))
                or not re.fullmatch(r'[0-9a-f]{64}', str(record.get('bundle_sha256', '')))):
            raise RuntimeError('当前 R3 部署记录未确认，不能使用 ROS 独立状态恢复。') from exc
        for name in sorted(REQUIRED_PROGRAMS):
            folder, filename = name.split('/')
            path = _regular((root if folder == 'app' else OWNER_DIR)/filename, optional=True)
            if not path.exists() or path.stat().st_size == 0:
                break
        else:
            raise
    owner = _Gate_ReadIdleOwner()
    return {'owner': owner, 'task_active': False, 'recovery_session': owner['session']}


def _node_pid():
    info = _ros('info', NODE)
    match = re.search(r'^\s*Pid:\s*(\d+)\s*$', info, re.M | re.I)
    if not match:
        raise RuntimeError('Cannot identify the running R3 ROS node.')
    cmdline = (Path('/proc')/match[1]/'cmdline').read_bytes().split(b'\0')
    if str(OWNER_DIR/'CMDcontrol_service.py').encode() not in cmdline:
        raise RuntimeError('ROS node is not the expected local R3 owner script.')
    return int(match[1])


def _reload(session, require_new_helper):
    _node_pid()
    _ros('kill', NODE)
    return _wait_owner(session, require_new_helper)


def _wait_owner(session, require_new_helper):
    deadline = time.monotonic()+35
    detail = '尚未收到状态'
    while time.monotonic() < deadline:
        try:
            row = _status(require_idle=True)
            current = row['owner'].get('session')
            if not current:
                detail = '控制节点未返回会话编号'
            elif current == session:
                detail = '仍是重载前的会话'
            elif require_new_helper and 'mission_progress' not in row:
                detail = '新会话尚未返回任务进度接口'
            else:
                return row
        except (OSError, ValueError, RuntimeError) as exc:
            detail = str(exc)
        time.sleep(1)
    raise RuntimeError(f'等待 ROS 控制节点 {NODE} 重载超时，最后一次检查为「{detail}」')


def _reload_restored(session, require_new_helper):
    try:
        current = _status(require_idle=True)['owner']['session']
    except (OSError, ValueError):
        # The supervisor may already be respawning a node against the restored files.
        return _wait_owner(session, require_new_helper)
    return _reload(current, require_new_helper)


def _exchange(left, right):
    # Both directories must remain on the Pi filesystem beside the deployment target.
    library = ctypes.CDLL(None, use_errno=True)
    rename = getattr(library, 'renameat2', None)
    if rename is None:
        raise RuntimeError('This Pi does not expose atomic directory exchange.')
    rename.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p, ctypes.c_uint]
    rename.restype = ctypes.c_int
    if rename(-100, os.fsencode(left), -100, os.fsencode(right), 2):
        code = ctypes.get_errno()
        raise OSError(code, os.strerror(code), str(right))


def _Path_Sync(path):
    fd = os.open(path, os.O_RDONLY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def _Stage_Sync(slot):
    # SFTP close and checksum readback can both finish while data is still in the Pi's page cache.
    for directory, _, files in os.walk(slot, topdown=False):
        parent = Path(directory)
        for name in files:
            _Path_Sync(_regular(parent/name))
        _Path_Sync(parent)
    _Path_Sync(slot.parent)


def _Exchange_SyncParents(left, right):
    _Path_Sync(left.parent)
    _Path_Sync(right.parent)


def _atomic_bytes(path, data, *, owner_stat=None):
    mode = path.stat().st_mode & 0o777 if path.exists() else 0o600
    fd, name = tempfile.mkstemp(prefix='.r3-deploy-', dir=path.parent)
    temp = Path(name)
    try:
        with os.fdopen(fd, 'wb') as out:
            out.write(data)
            out.flush()
            if owner_stat is not None and os.fstat(out.fileno()).st_gid != owner_stat.st_gid:
                os.fchown(out.fileno(), -1, owner_stat.st_gid)
            temp.chmod(mode)
            os.fsync(out.fileno())
        os.replace(temp, path)
        _Path_Sync(path.parent)
    finally:
        temp.unlink(missing_ok=True)


def _write_json(path, value):
    _atomic_bytes(path, (json.dumps(value, indent=2)+'\n').encode('utf-8'))


def validate_mcu_readback(receipt, expected):
    manifest_digest = hashlib.sha256(json.dumps(expected, sort_keys=True, separators=(',', ':')).encode('ascii')).hexdigest()
    if manifest_digest != _MCU_MANIFEST_SHA256:
        raise ValueError('The staged application lacks the approved continuous-route MCU manifest.')
    # The medium-turn export requires the paired bridge and turn modules.
    tuned = ({**expected, **_MCU_MEDIUM_SHA256}, {**expected, **_MCU_RIGHT5_SHA256}, {**expected, **_MCU_PAIR_SHA256},
             {**expected, **_MCU_PICKUP_BACKWARD4_SHA256}, {**expected, **_MCU_PICKUP_BACKWARD6_SHA256},
             {**expected, **_MCU_PICKUP_BACKWARD8_SHA256},
             {**expected, **_MCU_PICKUP_BACKWARD8_TUNED_LEFT_SHA256},
             {**expected, **_MCU_PICKUP_BACKWARD8_LOW_RELEASE_SHA256},
             {**expected, **_MCU_PICKUP_LEVEL_BACKWARD6_SHA256}, {**expected, **_MCU_EMPTY_RECOVERY_SHA256},
             {**expected, **_MCU_INTERMEDIATE_STRAFE_SHA256}, {**expected, **_MCU_TRANSFER_STRAFE_SHA256},
             {**expected, **_MCU_LEVEL_BACKWARD8_SHA256}, {**expected, **_MCU_RELAXED_CARRY_HIP_SHA256}, {**expected, **_MCU_PLACEMENT_STRAFE_SHA256},
             {**expected, **_MCU_STANDING_START_SHA256},
             *({**expected, 'lua/craic_single_micro.lua': digest}
               for digest in (_MCU_TUNED_MICRO_SHA256, *_MCU_PRIOR_TUNED_MICRO_SHA256)))
    if (not isinstance(receipt, dict) or receipt.get('robot_id') != 3
            or receipt.get('readback_all_match') is not True or receipt.get('reload_confirmed') is not False
            or receipt.get('different_files') != [] or receipt.get('files') not in (expected, *tuned)):
        raise ValueError('MCU 回执与受支持的动作包不符，请选择本次 U 盘核验或覆盖生成的回执。')
    volume = receipt.get('volume')
    drive = receipt.get('drive')
    if (not isinstance(volume, dict) or not isinstance(drive, str) or not re.fullmatch(r'[A-Za-z]:[\\/]', drive)
            or not isinstance(volume.get('serial'), str) or not re.fullmatch(r'[0-9A-F]{8}', volume['serial'])
            or not isinstance(volume.get('profile_sha256'), str)
            or not re.fullmatch(r'[0-9a-f]{64}', volume['profile_sha256'])
            or not isinstance(volume.get('filesystem'), str) or not volume['filesystem']):
        raise ValueError('USB receipt is missing verified R3 volume profile evidence.')
    return {key: receipt[key] for key in ('robot_id', 'drive', 'files', 'different_files',
                                          'readback_all_match', 'reload_confirmed')} | {
        'volume': {key: volume[key] for key in ('serial', 'profile_sha256', 'filesystem')},
    }


def _boot_id():
    value = Path('/proc/sys/kernel/random/boot_id').read_text(encoding='ascii').strip()
    try:
        if str(uuid.UUID(value)) == value:
            return value
    except ValueError:
        pass
    raise RuntimeError('The current R3 Pi boot identifier could not be verified.')


def _write_mcu_readback(app, owner, readback):
    boot = _boot_id()
    if owner.get('mcu_ready') is not True or owner.get('micro_trial', {}).get('boot_id') != boot:
        raise RuntimeError('The current R3 owner must report a live MCU handshake and matching Pi boot.')
    serial, machine = _verified_local_identity()
    common = {'installed': True, 'robot_id': 3, 'cpu_serial': serial, 'machine_id': machine,
              'policy_single_enabled': True, 'reload_confirmed': True, 'verified_boot_id': boot,
              'installation_evidence_scope': 'persistent_mcu_files_until_next_deployment',
              'reload_confirmation_source': 'operator_confirmed_after_usb_readback', 'usb_volume': readback['volume']}
    micro_path = _regular(app/'config/micro_mcu_deployment.json')
    guard_path = _regular(app/'config/mcu_guard_deployment.json')
    micro = dict(_json(micro_path), **common, contract='r3.single_micro_turn.v1', mission_abi=1, wire_abi=42,
                 single_grab_policy='repeat_after_success_empty_only', verified_files_sha256=readback['files'])
    guard_files = {name: digest for name, digest in readback['files'].items() if name != 'lua/craic_single_micro.lua'}
    guard = dict(_json(guard_path), **common, guard_version='r3-grab-guard-20261106-v1', verified_files_sha256=guard_files)
    _write_json(micro_path, micro)
    _write_json(guard_path, guard)
    return boot


def _bundle(slot):
    manifest = _json(_regular(slot/'manifest.json'))
    if manifest.get('schema') != 'r3.pi-deploy.v1' or manifest.get('owner_directory') != str(OWNER_DIR):
        raise RuntimeError('Unexpected R3 bundle schema or owner directory.')
    files = manifest['files']
    missing = REQUIRED_PROGRAMS - files.keys()
    if missing:
        raise RuntimeError('部署包缺少必需程序：'+', '.join(sorted(missing)))
    found = {path.relative_to(slot).as_posix() for part in ('app', 'owner')
             for path in (slot/part).rglob('*') if path.is_file() or path.is_symlink()}
    if found != files.keys():
        raise RuntimeError('Uploaded files do not match the bundle manifest.')
    for name, sha in files.items():
        path = _regular(slot/_relative(name))
        if name in REQUIRED_PROGRAMS and not path.stat().st_size:
            raise RuntimeError('部署包中的必需程序为空：'+name)
        if _digest(path) != sha:
            raise RuntimeError('Upload checksum mismatch: '+name)
    return manifest


def _merge(defaults, live):
    output = dict(defaults)
    for k, value in live.items():
        old = output.get(k)
        if isinstance(old, dict) and isinstance(value, dict):
            output[k] = _merge(old, value)
        elif value is not None or old is None:
            output[k] = value
    return output


def _copy_live(source, dest, name, *, unless_sha256=None):
    rel = _relative(name)
    path = _regular(source/rel, optional=True)
    if path.exists():
        if path.resolve() != path:
            raise RuntimeError('Live configuration contains a symlink: '+name)
        if unless_sha256 is not None and _digest(path) == unless_sha256:
            return
        copy = dest/rel
        copy.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, copy)


def _apply_settings(root, app, manifest):
    if (root/'config').is_symlink():
        raise RuntimeError('Live configuration directory is a symlink.')
    for name in manifest['required_live']:
        _regular(root/_relative(name))
    for name in manifest['preserve_exact']:
        stock_routes = (manifest.get('replace_stock_routes_sha256')
                        if name == 'routes.py' and (app/'routes.py').is_file() else None)
        _copy_live(root, app, name, unless_sha256=stock_routes)
    for name in manifest['merge_live_over_defaults']:
        live = _regular(root/_relative(name), optional=True)
        if live.exists():
            defaults = _json(app/name)
            if (name == 'config/mission_v5.json' and live.stat().st_size == 0
                    and defaults.get('schema') == 'r3.mission-v5.v1' and defaults.get('enabled') is False):
                # An empty file has no field settings to preserve. Keep the
                # inactive staged default and retain the old file in the backup.
                continue
            merged = _merge(defaults, _json(live))
            if name == 'config/runtime.json':
                key = 'max_body_macros_per_journey'
                merged[key] = max(defaults[key], merged[key])
            _write_json(app/name, merged)
    for path in (root/'config').glob('*.json'):
        name = path.relative_to(root).as_posix()
        if not (app/name).exists():
            _copy_live(root, app, name)
    for config in ('entry_alignment', 'tag_reporting'):
        path = app/'config'/(config+'.json')
        if path.is_file():
            for key in ('reference_path', 'effects_path'):
                value = _json(path).get(key)
                if not value:
                    continue
                absolute = (root/value).resolve()
                if root in absolute.parents:
                    _copy_live(root, app, absolute.relative_to(root).as_posix())


def _inspect(root, for_deploy=False):
    report = {'robot_id': 3, 'path': str(root), 'activation': 'ros_owner_reload' if root == DEFAULT_PATH else 'files_only'}
    if root == DEFAULT_PATH:
        row = _Deployment_Status(root, require_idle=False) if for_deploy else _status()
        if 'recovery_session' in row:
            report['recovery_session'] = row['recovery_session']
        report.update(task_active=row.get('task_active'), owner_mode=row['owner'].get('mode'),
                      mcu_ready=row['owner'].get('mcu_ready'))
    if (root/MARKER).is_file():
        report['deployment'] = _json(_regular(root/MARKER))
    return report


def _owner_backup(slot, name):
    return slot/('owner.backup' if name == OWNER_NAME else name+'.backup')


def SerialReceiver_PrepareShutdown(root, slot, session):
    """Upgrade only the known blocking UART source before an application exchange."""
    service = _regular(OWNER_DIR/'CMDcontrol_service.py')
    if service.resolve() != service:
        raise RuntimeError(f'ROS 串口程序路径异常 {service}')
    source = service.read_bytes()
    digest = hashlib.sha256(source).hexdigest()
    note = root.with_name('.'+root.name+'.uart-maintenance.json')
    restart = '请在机器人静止时重启树莓派一次，然后重新连接并部署。'
    if digest == _SERIAL_TIMED_SHA and _regular(note, optional=True).is_file():
        record = _json(note)
        if record.get('session') == session:
            raise RuntimeError(f'串口修复已写入磁盘，旧控制进程仍在运行。{restart} 原件备份 {record.get("backup")}')
    if digest != _SERIAL_ORIGINAL_SHA:
        return
    original = service.stat()
    if original.st_uid != os.getuid():
        raise RuntimeError(f'当前 SSH 账户不拥有 ROS 串口程序 {service}')
    updated = source.replace(b'def main():\n', b'''def main():
    def read_exact(port, size):
        # Preserve a partial UART frame across timed reads. Only ROS shutdown abandons it.
        received = bytearray()
        while len(received) < size:
            if rospy.is_shutdown():
                return None
            received.extend(port.read(size - len(received)))
        return None if rospy.is_shutdown() else received

''')
    updated = updated.replace(b"serial.Serial('/dev/serial0', 9600)", b"serial.Serial('/dev/serial0', 9600, timeout=0.2)")
    updated = updated.replace(b'            cur_byte = ser.read()', b'            cur_byte = read_exact(ser, 1)\n            if cur_byte is None:\n                break')
    updated = updated.replace(b'            tmp_head.append(ord(cur_byte))', b'            tmp_head.append(cur_byte[0])')
    updated = updated.replace(b'                    length_high = ord(ser.read())\n                    length_low = ord(ser.read())',
                              b'                    length = read_exact(ser, 2)\n                    if length is None:\n                        break\n                    length_high, length_low = length')
    updated = updated.replace(b'                    package_data_byte = ser.read(package_data_length)',
                              b'                    package_data_byte = read_exact(ser, package_data_length)\n                    if package_data_byte is None:\n                        break')
    if hashlib.sha256(updated).hexdigest() != _SERIAL_TIMED_SHA:
        raise RuntimeError('串口修补结果与已验证版本不符，原程序仍在使用。')
    backup = _regular(slot/('pre-serial-shutdown-'+service.name), optional=True)
    if not backup.exists():
        _atomic_bytes(backup, source)
    if _digest(backup) != digest:
        raise RuntimeError(f'串口原件备份校验失败 {backup}')
    if _Deployment_Status(root, require_idle=True)['owner']['session'] != session or _digest(service) != digest:
        raise RuntimeError('部署期间控制会话或串口程序发生变化，请重新连接后部署。')
    _regular(note, optional=True)
    _write_json(note, {'session':session, 'backup':str(backup)})
    _atomic_bytes(service, updated, owner_stat=original)
    actual = service.stat()
    if (_digest(service) != _SERIAL_TIMED_SHA or actual.st_uid != original.st_uid
            or actual.st_gid != original.st_gid or actual.st_mode != original.st_mode):
        raise RuntimeError(f'串口修复写入后的内容或权限校验失败，原件备份 {backup}')
    raise RuntimeError(f'已修复并核验 ROS 串口退出机制，旧控制进程仍在运行。{restart} 原件备份 {backup}')


def _Client_CheckStatus(root, session):
    reply = subprocess.run([sys.executable, '-B', 'warm_client.py', 'status'], cwd=root,
                           capture_output=True, timeout=10)
    if reply.returncode:
        detail = reply.stderr.decode('utf-8', errors='replace').strip()[-600:]
        raise RuntimeError(f'warm_client.py status 退出码 {reply.returncode}：{detail}')
    try:
        row = json.loads(reply.stdout)
    except (ValueError, UnicodeDecodeError) as exc:
        raise RuntimeError('warm_client.py status 没有返回有效的 JSON 状态。') from exc
    owner = row.get('owner') if isinstance(row, dict) else None
    if (not isinstance(owner, dict) or owner.get('robot_id') != 3
            or row.get('protocol') != 'r3.warm.v1' or row.get('event') != 'status'
            or not session or owner.get('session') != session):
        raise RuntimeError('warm_client.py status 未确认当前 R3 控制会话。')


def _commit(root, ticket, mcu_readback=None, recovery_session=None):
    slot = _transaction(root, ticket)
    if slot.resolve() != slot or (slot/'receipt.json').exists():
        raise RuntimeError('Invalid or previously used deployment staging directory.')
    manifest = _bundle(slot)
    if mcu_readback is not None:
        if (root != DEFAULT_PATH or MCU_MANIFEST_PATH not in manifest['files']
                or not all('owner/'+name in manifest['files'] for name in OWNER_SUPPORT)):
            raise RuntimeError('USB readback registration requires the default R3 target and paired owner bundle.')
        mcu_readback = validate_mcu_readback(mcu_readback, _json(_regular(slot/MCU_MANIFEST_PATH)))
    app = slot/'app'
    _apply_settings(root, app, manifest)
    if _digest(_regular(app/'warm_client.py')) != manifest['files']['app/warm_client.py']:
        raise RuntimeError('合并现场配置后 warm_client.py 与部署清单不符。')
    subprocess.run([sys.executable, '-B', 'run_auto.py', '0', '--check'],
                   cwd=app, capture_output=True, timeout=15, check=True)
    owners = []
    session = None
    verified_boot = None
    if root == DEFAULT_PATH:
        support = [name for name in OWNER_SUPPORT if 'owner/'+name in manifest['files']]
        if support and len(support) != len(OWNER_SUPPORT):
            raise RuntimeError('The ROS owner gate modules must be updated together.')
        owners = [_regular(OWNER_DIR/name) for name in (*support, OWNER_NAME)]
        if any(owner.resolve() != owner or owner.stat().st_uid != os.getuid() for owner in owners):
            raise RuntimeError('Unexpected local ROS owner file.')
        _node_pid()
        owner_status = _Deployment_Status(root, require_idle=True)['owner']
        session = owner_status['session']
        if recovery_session is not None and session != recovery_session:
            raise RuntimeError('从恢复检查到提交期间 R3 控制会话已更换，请重新部署。')
        SerialReceiver_PrepareShutdown(root, slot, session)
        if mcu_readback is not None:
            verified_boot = _write_mcu_readback(app, owner_status, mcu_readback)
        for owner in owners:
            _atomic_bytes(_owner_backup(slot, owner.name), owner.read_bytes())
    receipt = {'id': ticket, 'path': str(root), 'bundle_sha256': _digest(slot/'manifest.json')}
    _write_json(app/MARKER, receipt)
    installed = {file.relative_to(app): _digest(file) for file in app.rglob('*') if file.is_file()}
    _write_json(slot/'receipt.json', receipt)
    _Stage_Sync(slot)
    swapped = False
    owners_changed = []
    try:
        if owners:
            if _Deployment_Status(root, require_idle=True)['owner']['session'] != session:
                raise RuntimeError('Owner changed during staging.')
        _exchange(root, app)
        swapped = True
        _Exchange_SyncParents(root, app)
        for relative, digest in installed.items():
            if _digest(_regular(root/relative)) != digest:
                raise RuntimeError('Application readback mismatch: '+str(relative))
        for owner in owners:
            replacement = slot/'owner'/owner.name
            owners_changed.append(owner)
            _atomic_bytes(owner, replacement.read_bytes())
            if _digest(owner) != _digest(replacement):
                raise RuntimeError('Owner file readback mismatch: '+owner.name)
        if owners:
            activated = _reload(session, require_new_helper=True)
            _Client_CheckStatus(root, activated.get('owner', {}).get('session'))
            if verified_boot:
                micro = activated.get('owner', {}).get('micro_trial', {})
                if micro.get('cmd0_route_deployment_readback_ready') is not True:
                    raise RuntimeError('The reloaded owner did not accept the registered continuous-route MCU record.')
                bridge_hash = mcu_readback['files']['lua/craic_auto_bridge.lua']
                if (bridge_hash in (_MCU_MEDIUM_SHA256['lua/craic_auto_bridge.lua'], _MCU_RIGHT5_SHA256['lua/craic_auto_bridge.lua'])
                        and micro.get('medium_turn_deployment_readback_ready') is not True):
                    raise RuntimeError('The reloaded owner did not accept the registered medium-turn MCU record.')
                if (bridge_hash == _MCU_RIGHT5_SHA256['lua/craic_auto_bridge.lua']
                        and micro.get('cmd0_right5_deployment_readback_ready') is not True):
                    raise RuntimeError('The reloaded owner did not accept the registered five-stride MCU record.')
                if (bridge_hash in (_MCU_PAIR_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_PICKUP_BACKWARD4_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_PICKUP_BACKWARD6_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_PICKUP_BACKWARD8_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_EMPTY_RECOVERY_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_INTERMEDIATE_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_TRANSFER_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_LEVEL_BACKWARD8_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_PLACEMENT_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_STANDING_START_SHA256['lua/craic_auto_bridge.lua'])
                        and not all(micro.get(key) is True for key in (
                            'pair_deployment_readback_ready', 'cmd0_right5_deployment_readback_ready',
                            'medium_turn_deployment_readback_ready'))):
                    raise RuntimeError('The reloaded owner did not accept the registered horizontal-pair MCU record.')
                if (bridge_hash == _MCU_PICKUP_BACKWARD4_SHA256['lua/craic_auto_bridge.lua']
                        and micro.get('pickup_backward4_deployment_readback_ready') is not True):
                    raise RuntimeError('The reloaded owner did not accept the registered four-step backward MCU record.')
                if (bridge_hash == _MCU_PICKUP_BACKWARD6_SHA256['lua/craic_auto_bridge.lua']
                        and micro.get('pickup_backward6_deployment_readback_ready') is not True):
                    raise RuntimeError('The reloaded owner did not accept the registered six-step backward MCU record.')
                if (bridge_hash in (_MCU_EMPTY_RECOVERY_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_INTERMEDIATE_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_TRANSFER_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_LEVEL_BACKWARD8_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_PLACEMENT_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_STANDING_START_SHA256['lua/craic_auto_bridge.lua'])
                        and micro.get('empty_recovery_deployment_readback_ready') is not True):
                    raise RuntimeError('The reloaded owner did not accept the registered empty-return MCU record.')
                if (bridge_hash in (_MCU_PICKUP_BACKWARD8_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_EMPTY_RECOVERY_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_INTERMEDIATE_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_TRANSFER_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_LEVEL_BACKWARD8_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_PLACEMENT_STRAFE_SHA256['lua/craic_auto_bridge.lua'],
                                    _MCU_STANDING_START_SHA256['lua/craic_auto_bridge.lua'])
                        and micro.get('pickup_backward8_deployment_readback_ready') is not True):
                    raise RuntimeError('The reloaded owner did not accept the registered eight-step backward MCU record.')
                if (bridge_hash == _MCU_STANDING_START_SHA256['lua/craic_auto_bridge.lua']
                        and micro.get('standing_start_deployment_readback_ready') is not True):
                    raise RuntimeError('The reloaded owner did not accept the registered standing-start MCU record.')
        if _json(_regular(root/MARKER)) != receipt:
            raise RuntimeError('Deployment marker readback mismatch.')
    except Exception as exc:
        if owners_changed:
            try:
                live = _status()
            except (OSError, ValueError, RuntimeError):
                live = None
            if live is not None and (live['task_active'] or live['owner']['in_flight']
                                     or live['owner']['mode'] != 'manual'):
                raise RuntimeError('New files are active but the owner is busy; backup kept at '+str(slot)) from exc
        for owner in reversed(owners_changed):
            _atomic_bytes(owner, _owner_backup(slot, owner.name).read_bytes())
        if swapped:
            _exchange(root, app)
            _Exchange_SyncParents(root, app)
        if owners_changed:
            try:
                _reload_restored(session, require_new_helper=False)
            except Exception as restore_exc:
                raise RuntimeError(f'新程序激活失败（{exc}）。已恢复原文件，恢复后的控制服务核验也失败'
                                   f'（{restore_exc}）。事务目录 {slot}') from restore_exc
        raise RuntimeError('Deployment did not activate; original files restored. Details: '+str(exc)) from exc
    result = {'status': 'deployed', 'id': ticket, 'path': str(root), 'backup': str(app),
              'owner_reloaded': bool(owners)}
    if verified_boot:
        result.update(mcu_readback_registered=True, mcu_record_boot_id=verified_boot, mcu_runtime_probe_sent=False)
    return result


def _rollback(root):
    receipt = _json(_regular(root/MARKER))
    slot = _transaction(root, receipt['id'])
    if slot.resolve() != slot or _json(_regular(slot/'receipt.json')) != receipt or receipt['path'] != str(root):
        raise RuntimeError('Active deployment does not match its backup record.')
    backup = slot/'app'
    _target(str(backup))
    owners = {}
    if root == DEFAULT_PATH:
        for name in (*OWNER_SUPPORT, OWNER_NAME):
            saved = _regular(_owner_backup(slot, name), optional=name != OWNER_NAME)
            if saved.is_file():
                owner = _regular(OWNER_DIR/name)
                owners[owner] = (owner.read_bytes(), saved.read_bytes())
    session = _status(require_idle=True)['owner']['session'] if owners else None
    if owners:
        _node_pid()
        if _status(require_idle=True)['owner']['session'] != session:
            raise RuntimeError('Owner changed before rollback.')
    _exchange(root, backup)
    try:
        _Exchange_SyncParents(root, backup)
        for owner, (_, old) in owners.items():
            _atomic_bytes(owner, old)
        if owners:
            restored = _reload(session, require_new_helper=False)
            # A factory installation may predate the SSH client. Managed generations must have one.
            if (root/MARKER).is_file() or (root/'warm_client.py').exists():
                _Client_CheckStatus(root, restored.get('owner', {}).get('session'))
    except Exception as exc:
        _exchange(root, backup)
        _Exchange_SyncParents(root, backup)
        for owner, (current, _) in owners.items():
            _atomic_bytes(owner, current)
        try:
            _reload_restored(session, require_new_helper=True)
        except Exception as restore_exc:
            raise RuntimeError(f'回滚版本激活失败（{exc}）。已换回较新文件，控制服务核验也失败'
                               f'（{restore_exc}）。事务目录 {slot}') from restore_exc
        raise
    return {'status': 'rolled_back', 'path': str(root), 'saved_newer_version': str(backup),
            'owner_reloaded': bool(owners)}


def execute(request):
    root = _target(request['path'])
    action = request['action']
    if action == 'inspect':
        return _inspect(root, for_deploy=request.get('for_deploy') is True)
    if action == 'stage':
        ticket = uuid.uuid4().hex
        slot = _transaction(root, ticket)
        slot.mkdir(mode=0o700)
        return {'id': ticket, 'stage': str(slot)}
    if action not in ('commit', 'rollback'):
        raise ValueError('Unknown deploy operation')
    import fcntl

    lock = Path.home()/'.r3-deploy.lock'
    with os.fdopen(os.open(lock, os.O_CREAT | os.O_NOFOLLOW | os.O_RDWR, 0o600), 'wb') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if action == 'commit':
            return _commit(root, request['id'], request.get('mcu_readback'), request.get('recovery_session'))
        return _rollback(root)


if __name__ == '__main__':
    try:
        print(json.dumps(execute(json.loads(sys.argv[1]))))
    except Exception as exc:
        print(json.dumps({'error': str(exc)}))
        sys.exit(1)
