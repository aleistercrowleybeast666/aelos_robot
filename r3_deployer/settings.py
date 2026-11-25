"""Local deployment settings and Windows current-user password protection."""

import base64
import ctypes
from ctypes import wintypes
import json
import os
from pathlib import Path, PurePosixPath
import re
import sys
import tempfile
from urllib.parse import unquote, urlsplit, urlunsplit


PROJECT = Path(sys.executable).resolve().parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent.parent
_SOURCE_FILE = PROJECT / 'robot_source.json'
_CONFIGURED_WORKSPACE = ((PROJECT / json.loads(_SOURCE_FILE.read_text(encoding='utf-8'))['workspace']).resolve()
                         if _SOURCE_FILE.is_file() else None)
ROBOT_WORKSPACE = (_CONFIGURED_WORKSPACE if _CONFIGURED_WORKSPACE and _CONFIGURED_WORKSPACE.is_dir()
                   else next((path for path in (PROJECT, PROJECT.parent)
                              if (path / 'pi_deploy_bundle' / 'manifest.json').is_file()), PROJECT))
DEFAULT_BUNDLE = ROBOT_WORKSPACE / 'pi_deploy_bundle'
DEFAULT_MCU_OVERLAY = (ROBOT_WORKSPACE.parents[1] / '覆盖包' if ROBOT_WORKSPACE == _CONFIGURED_WORKSPACE
                       else PROJECT / 'mcu_overlay')
DEFAULT_PI_PATH = '/home/lemon/aelos_auto_r3_python'
DEFAULT_REMOTE_URL = 'https://raw.githubusercontent.com/aleistercrowleybeast666/aelos_robot/main/pi_deploy_bundle'
DEFAULTS = {
    'host': '192.168.1.100',
    'port': 22,
    'username': 'lemon',
    'local_path': str(DEFAULT_BUNDLE),
    'remote_path': DEFAULT_PI_PATH,
    'deployment_source': 'remote',
    'remote_url': DEFAULT_REMOTE_URL,
}
MANUAL = (Path.home() / 'Desktop' / 'aelos' / 'CRAIC2026-NF' / 'CRAIC2026-NF'
          / '机器人通用资料' / 'AELOS_SMART用户手册v1.3.pdf')


def settings_path():
    local = Path(os.environ.get('LOCALAPPDATA') or Path.home())
    return local / 'AelosR3Deploy' / 'settings.json'


def RemoteBundle_NormalizeUrl(value):
    """Accept a manifest directory, a manifest URL or the equivalent GitHub page."""
    error = '请输入 HTTPS 部署包网址，例如 '+DEFAULT_REMOTE_URL
    if not isinstance(value, str):
        raise ValueError(error)
    value = value.strip()
    if not value or any(ch.isspace() or ord(ch) < 32 or ch == '\\' for ch in value):
        raise ValueError(error)
    try:
        url = urlsplit(value)
        host = url.hostname
        port = url.port
    except ValueError as exc:
        raise ValueError(error) from exc
    if (url.scheme != 'https' or not host or port == 0 or url.username is not None or url.password is not None
            or url.query or url.fragment):
        raise ValueError(error)
    path = url.path.rstrip('/')
    decoded = unquote(path)
    if '\\' in decoded or any(ord(ch) < 32 for ch in decoded) or any(
            part in ('.', '..') for part in decoded.split('/')):
        raise ValueError(error)
    if host == 'github.com' and port in (None, 443):
        segments = path.strip('/').split('/')
        if len(segments) == 2 and all(segments):
            owner, repo = segments
            path = f'/{owner}/{repo.removesuffix(".git")}/main/pi_deploy_bundle'
        elif len(segments) >= 5 and segments[2] in ('tree', 'blob'):
            path = '/' + '/'.join(segments[:2] + segments[3:])
        else:
            raise ValueError(error)
        url = url._replace(netloc='raw.githubusercontent.com')
    return urlunsplit((url.scheme, url.netloc, path.removesuffix('/manifest.json'), '', ''))


def validate(values):
    result = {key: values.get(key, default) for key, default in DEFAULTS.items()}
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9.:-]{0,252}', result['host']):
        raise ValueError('请输入有效的 SSH 地址。')
    if type(result['port']) is not int or not 1 <= result['port'] <= 65535:
        raise ValueError('端口须在 1 到 65535 之间。')
    if not re.fullmatch(r'[a-zA-Z_][a-zA-Z0-9_.-]{0,63}', result['username']):
        raise ValueError('请输入有效的 SSH 用户名。')
    if result['deployment_source'] not in ('remote', 'local'):
        raise ValueError('请选择远程或本地部署来源。')
    if result['deployment_source'] == 'remote':
        result['remote_url'] = RemoteBundle_NormalizeUrl(result['remote_url'])
    elif not isinstance(result['local_path'], str) or not result['local_path'].strip():
        raise ValueError('请选择电脑上的部署包目录。')
    remote = result['remote_path']
    path = PurePosixPath(remote)
    if (not path.is_absolute() or len(path.parts) < 4 or str(path) != remote
            or '..' in path.parts or any(ord(ch) < 32 for ch in remote)):
        raise ValueError('树莓派路径须为规范的绝对应用目录，例如 '+DEFAULT_PI_PATH)
    return result


def _endpoint(values):
    return [values['host'].lower(), values['port'], values['username']]


class _DataBlob(ctypes.Structure):
    _fields_ = [('size', wintypes.DWORD), ('data', ctypes.POINTER(ctypes.c_ubyte))]


def _dpapi(data, protect):
    if os.name != 'nt':
        raise OSError('保存密码需要 Windows 用户加密。')
    crypt = ctypes.WinDLL('crypt32', use_last_error=True)
    kernel = ctypes.WinDLL('kernel32', use_last_error=True)
    operation = crypt.CryptProtectData if protect else crypt.CryptUnprotectData
    operation.argtypes = [
        ctypes.POINTER(_DataBlob), ctypes.c_void_p, ctypes.c_void_p,
        ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(_DataBlob),
    ]
    operation.restype = wintypes.BOOL
    kernel.LocalFree.argtypes = [ctypes.c_void_p]
    kernel.LocalFree.restype = ctypes.c_void_p
    buf = (ctypes.c_ubyte * len(data)).from_buffer_copy(data)
    source = _DataBlob(len(data), buf)
    dest = _DataBlob()
    try:
        if not operation(ctypes.byref(source), None, None, None, None, 1, ctypes.byref(dest)):
            raise ctypes.WinError(ctypes.get_last_error())
        return ctypes.string_at(dest.data, dest.size)
    finally:
        ctypes.memset(buf, 0, len(buf))
        if dest.data:
            ctypes.memset(dest.data, 0, dest.size)
            kernel.LocalFree(dest.data)


def _read():
    path = settings_path()
    return json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}


def load():
    return validate({**DEFAULTS, **_read()})


def password_for(values):
    protected = _read().get('credential', {})
    if protected.get('endpoint') != _endpoint(values):
        return None
    try:
        return _dpapi(base64.b64decode(protected['dpapi'], validate=True), False).decode('utf-8')
    except (OSError, ValueError, KeyError) as exc:
        raise ValueError('无法读取保存的密码，请重新输入。') from exc


def save(values, password=None):
    values = validate(values)
    if password:
        values['credential'] = {
            'endpoint': _endpoint(values),
            'dpapi': base64.b64encode(_dpapi(password.encode('utf-8'), True)).decode('ascii'),
        }
    else:
        current = _read().get('credential')
        if current:
            values['credential'] = current
    path = settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile('w', dir=path.parent, encoding='utf-8', delete=False) as f:
            name = Path(f.name)
            json.dump(values, f, indent=2, ensure_ascii=False)
            f.write('\n')
        os.replace(name, path)
    finally:
        if name is not None:
            name.unlink(missing_ok=True)


def manual_password(path):
    from pypdf import PdfReader

    pattern = re.compile(r'输入\s*密码[：:]\s*([A-Za-z0-9_!@#.$%+\-]{3,32})\s*[。．]')
    matches = {match.group(1) for page in PdfReader(path).pages
               for match in pattern.finditer(page.extract_text())}
    if len(matches) != 1:
        raise ValueError('手册中的 SSH 密码字段无法唯一确认。')
    return matches.pop()
