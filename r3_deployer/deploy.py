"""Verified local and HTTPS bundle sources for R3 SSH deployment."""

from concurrent.futures import ThreadPoolExecutor
import errno
import hashlib
from http.client import HTTPException, IncompleteRead, RemoteDisconnected
from io import BytesIO
import json
import logging
from pathlib import Path, PurePosixPath
import re
import shlex
import socket
import stat
import subprocess
import sys
from time import monotonic, sleep
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlsplit
from urllib.request import HTTPRedirectHandler, Request, build_opener
from uuid import uuid4
from zipfile import BadZipFile, ZipFile

from . import remote, settings


_MAX_BUNDLE_FILES = 256
_MAX_REMOTE_MANIFEST = 256 * 1024
_MAX_REMOTE_FILE = 4 * 1024 * 1024
_MAX_REMOTE_BUNDLE = 32 * 1024 * 1024
_MAX_REMOTE_ARCHIVE = 8 * 1024 * 1024
_MAX_ARCHIVE_MEMBERS = 2048
_REMOTE_DOWNLOAD_SECONDS = 90
_REMOTE_REQUEST_SECONDS = 12
_REMOTE_DOWNLOAD_WORKERS = 3
_REMOTE_RETRY_DELAYS = (0.2,0.5)
_TRANSIENT_HTTP_ERRORS = frozenset((408,429,500,502,503,504))
_TRANSIENT_NETWORK_ERRORS = frozenset((errno.ECONNRESET,errno.ECONNABORTED,errno.EPIPE,
                                      errno.ETIMEDOUT,10053,10054,10060,socket.EAI_AGAIN))
_GITHUB_ARCHIVE_URL = 'https://codeload.github.com/aleistercrowleybeast666/aelos_robot/zip/refs/heads/main'
_GITHUB_ARCHIVE_BUNDLE = 'aelos_robot-main/pi_deploy_bundle/'


class _RemoteBundleUnavailable(RuntimeError):
    """The HTTP source did not respond with a downloadable artifact."""


class SshLinkError(RuntimeError):
    """A network interruption or an endpoint that has not answered SSH yet."""

    def __init__(self, detail):
        super().__init__('SSH 连接中断或机器人未响应。请确认供电和网络后重新连接。')
        self.detail = str(detail)


def _enable_tcp_keepalive(client):
    transport = client.get_transport()
    connection = getattr(transport, 'sock', None)
    if not isinstance(connection, socket.socket):
        return
    try:
        connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, 'SIO_KEEPALIVE_VALS'):
            connection.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 8000, 2000))
        for name, value in (('TCP_KEEPIDLE', 8), ('TCP_KEEPINTVL', 2), ('TCP_KEEPCNT', 3)):
            option = getattr(socket, name, None)
            if option is not None:
                connection.setsockopt(socket.IPPROTO_TCP, option, value)
    except OSError:
        pass  # Unsupported socket options must not prevent an established session.


def _manifest_files(manifest):
    try:
        document = json.loads(manifest)
    except (TypeError, ValueError) as exc:
        raise ValueError('部署包清单不是有效的 JSON。') from exc
    if (not isinstance(document, dict) or document.get('schema') != 'r3.pi-deploy.v1'
            or document.get('owner_directory') != remote.OWNER_DIR.as_posix()):
        raise ValueError('部署来源缺少适用的 R3 树莓派部署包清单。')
    files = document.get('files')
    if not isinstance(files, dict) or len(files) > _MAX_BUNDLE_FILES:
        raise ValueError('部署清单的文件数量无效。')
    missing = remote.REQUIRED_PROGRAMS - files.keys()
    if missing:
        raise ValueError('部署清单缺少必需程序：'+', '.join(sorted(missing)))
    parents = set()
    for name, digest in files.items():
        path = PurePosixPath(name)
        if (str(path) != name or path.is_absolute() or len(path.parts) < 2
                or '..' in path.parts or path.parts[0] not in ('app', 'owner')
                or '\\' in name or ':' in name or any(ord(ch) < 32 for ch in name)):
            raise ValueError('部署清单含有无效文件路径。')
        if not isinstance(digest, str) or not re.fullmatch(r'[0-9a-f]{64}', digest):
            raise ValueError('部署清单含有无效 SHA-256 值：'+name)
        parents.update(parent.as_posix() for parent in path.parents)
    if parents.intersection(files):
        raise ValueError('部署清单中有文件和目录重名。')
    return files


def _verify_bundle_file(data, name, digest):
    if name in remote.REQUIRED_PROGRAMS and not data:
        raise ValueError('部署包中的必需程序为空：'+name)
    if hashlib.sha256(data).hexdigest() != digest:
        raise ValueError('部署包文件哈希不匹配：'+name)
    return data


def _local_package(values, log):
    folder = Path(values['local_path']).expanduser().resolve()
    if folder == settings.DEFAULT_BUNDLE.resolve() and not getattr(sys, 'frozen', False):
        builder = settings.ROBOT_WORKSPACE/'tools/build_pi_deploy_bundle.py'
        if builder.is_file():
            log('检查本地应用包是否与工作区一致…')
            command = [sys.executable, '-B', str(builder)]
            checked = subprocess.run([*command, '--check'], capture_output=True, timeout=30)
            if checked.returncode:
                rebuilt = subprocess.run(command, capture_output=True, timeout=30)
                if rebuilt.returncode:
                    error = rebuilt.stderr.decode('utf-8', 'replace')[-1500:]
                    raise RuntimeError('本地部署包生成失败：'+error)
    manifest = (folder/'manifest.json').read_bytes()
    files = {}
    for name, digest in _manifest_files(manifest).items():
        source = folder/PurePosixPath(name)
        if source.is_symlink() or source.resolve() != source or not source.is_file():
            raise ValueError('部署包文件不完整或含链接：'+name)
        files[name] = _verify_bundle_file(source.read_bytes(),name,digest)
    log('本地包已核对，共 {} 个文件。'.format(len(files)))
    return manifest, files


class _HttpsRedirect(HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, msg, headers, newurl):
        if urlsplit(newurl).scheme.lower() != 'https':
            raise URLError('远程部署包的重定向必须使用 HTTPS。')
        return super().redirect_request(request,fp,code,msg,headers,newurl)


def _remote_request(opener, url, name, limit, deadline):
    request = Request(url, headers={
        'User-Agent':'AELOS-R3-Control','Cache-Control':'no-cache','Pragma':'no-cache',
    })
    for attempt in range(len(_REMOTE_RETRY_DELAYS)+1):
        try:
            remaining = deadline-monotonic()
            if remaining <= 0:
                raise TimeoutError('下载超过时限')
            with opener.open(request,timeout=min(_REMOTE_REQUEST_SECONDS,remaining)) as response:
                declared_length = response.headers.get('Content-Length','')
                length = int(declared_length) if declared_length.isdigit() else None
                if length is not None and length > limit:
                    raise ValueError('远程部署包文件超出大小限制：'+name)
                data = bytearray()
                while monotonic() < deadline:
                    chunk = response.read1(min(64*1024,limit+1-len(data)))
                    if not chunk:
                        if length is not None and len(data) != length:
                            raise IncompleteRead(bytes(),length-len(data))
                        return bytes(data)
                    data.extend(chunk)
                    if len(data) > limit:
                        raise ValueError('远程部署包文件超出大小限制：'+name)
                raise TimeoutError('下载超过时限')
        except HTTPError as exc:
            retryable = exc.code in _TRANSIENT_HTTP_ERRORS
            error = _RemoteBundleUnavailable(f'无法读取远程部署包文件 {name}（HTTP {exc.code}）。')
            cause = exc
            exc.close()
        except (URLError,OSError,HTTPException) as exc:
            cause = exc
            detail = getattr(exc,'reason',exc)
            retryable = (isinstance(detail,(TimeoutError,ConnectionError,IncompleteRead,RemoteDisconnected))
                         or getattr(detail,'errno',None) in _TRANSIENT_NETWORK_ERRORS
                         or getattr(detail,'winerror',None) in _TRANSIENT_NETWORK_ERRORS)
            error = _RemoteBundleUnavailable(f'下载远程部署包文件 {name} 失败。{detail}')
        if not retryable or attempt == len(_REMOTE_RETRY_DELAYS) or monotonic() >= deadline:
            raise error from cause
        sleep(min(_REMOTE_RETRY_DELAYS[attempt],max(0.,deadline-monotonic())))


def _remote_bundle_read(opener, source, name, limit, deadline, cache_suffix):
    url = f'{source}/{quote(name,safe="/")}{cache_suffix}'
    return _remote_request(opener,url,name,limit,deadline)


def _raw_remote_package(opener, source, deadline):
    cache_suffix = '?r3_bundle='+uuid4().hex if urlsplit(source).hostname == 'raw.githubusercontent.com' else ''
    manifest = _remote_bundle_read(opener,source,'manifest.json',_MAX_REMOTE_MANIFEST,deadline,cache_suffix)
    expected = list(_manifest_files(manifest).items())
    files = {}
    total = 0

    def read_file(entry):
        name, digest = entry
        data = _remote_bundle_read(opener,source,name,_MAX_REMOTE_FILE,deadline,cache_suffix)
        return name,_verify_bundle_file(data,name,digest)

    with ThreadPoolExecutor(max_workers=_REMOTE_DOWNLOAD_WORKERS) as executor:
        for offset in range(0,len(expected),_REMOTE_DOWNLOAD_WORKERS):
            batch = expected[offset:offset+_REMOTE_DOWNLOAD_WORKERS]
            for name,data in executor.map(read_file,batch):
                total += len(data)
                if total > _MAX_REMOTE_BUNDLE:
                    raise ValueError('远程部署包超出总大小限制。')
                files[name] = data
    return manifest, files


def _archive_member(archive, entry, limit):
    kind = stat.S_IFMT(entry.external_attr>>16)
    if kind not in (0,stat.S_IFREG) or entry.file_size > limit:
        raise ValueError('GitHub 部署包含有链接、特殊文件或超限文件：'+entry.filename)
    with archive.open(entry) as stream:
        data = stream.read(limit+1)
    if len(data) != entry.file_size or len(data) > limit:
        raise ValueError('GitHub 部署包文件大小不符：'+entry.filename)
    return data


def _github_remote_package(opener, deadline):
    url = _GITHUB_ARCHIVE_URL+'?r3_bundle='+uuid4().hex
    artifact = _remote_request(opener,url,'GitHub ZIP',_MAX_REMOTE_ARCHIVE,deadline)
    try:
        with ZipFile(BytesIO(artifact)) as archive:
            members = archive.infolist()
            if len(members) > _MAX_ARCHIVE_MEMBERS:
                raise ValueError('GitHub ZIP 文件数量超出读取限制。')
            bundled = {}
            for member in members:
                if member.is_dir() or not member.filename.startswith(_GITHUB_ARCHIVE_BUNDLE):
                    continue
                name = member.filename.removeprefix(_GITHUB_ARCHIVE_BUNDLE)
                if name in bundled:
                    raise ValueError('GitHub 部署包包含重复文件：'+name)
                bundled[name] = member
            if 'manifest.json' not in bundled:
                raise ValueError('GitHub ZIP 中缺少树莓派部署清单。')
            manifest = _archive_member(archive,bundled.pop('manifest.json'),_MAX_REMOTE_MANIFEST)
            expected = _manifest_files(manifest)
            if bundled.keys() != expected.keys():
                raise ValueError('GitHub ZIP 中的部署文件与清单不符。')
            if sum(member.file_size for member in bundled.values()) > _MAX_REMOTE_BUNDLE:
                raise ValueError('GitHub 部署包超出总大小限制。')
            files = {name:_verify_bundle_file(_archive_member(archive,bundled[name],_MAX_REMOTE_FILE),name,digest)
                     for name,digest in expected.items()}
    except (BadZipFile,NotImplementedError,RuntimeError) as exc:
        raise ValueError('GitHub 返回的部署 ZIP 无法读取。') from exc
    return manifest, files


def _remote_package(values, log):
    source = settings.RemoteBundle_NormalizeUrl(values['remote_url'])
    log('下载远程树莓派部署包…')
    deadline = monotonic()+_REMOTE_DOWNLOAD_SECONDS
    opener = build_opener(_HttpsRedirect())
    if source == settings.DEFAULT_REMOTE_URL:
        try:
            manifest,files = _github_remote_package(opener,deadline)
        except _RemoteBundleUnavailable:
            log('GitHub 压缩包暂不可用，改用文件下载。')
            manifest,files = _raw_remote_package(opener,source,deadline)
    else:
        manifest,files = _raw_remote_package(opener,source,deadline)
    log('远程包下载并校验完成，共 {} 个文件。'.format(len(files)))
    return manifest, files


def _package(values, log):
    if values['deployment_source'] == 'remote':
        return _remote_package(values,log)
    return _local_package(values,log)


def _connect(client, values, password):
    import paramiko

    known = Path.home()/'.ssh'/'known_hosts'
    lookup = values['host'] if values['port'] == 22 else '[{}]:{}'.format(values['host'], values['port'])
    if not known.is_file() or paramiko.HostKeys(str(known)).lookup(lookup) is None:
        raise RuntimeError('未找到此地址的 SSH 主机记录。请核对真实设备的主机密钥，并用系统 ssh 建立已知主机记录。')
    client.load_system_host_keys(str(known))
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    logging.getLogger('paramiko.transport').setLevel(logging.CRITICAL)
    try:
        client.connect(values['host'], port=values['port'], username=values['username'],
                       password=password, look_for_keys=False, allow_agent=False,
                       timeout=7, banner_timeout=7, auth_timeout=7)
    except paramiko.BadHostKeyException as exc:
        raise RuntimeError('SSH 主机密钥已变化。请核对地址和树莓派身份。') from exc
    except paramiko.AuthenticationException as exc:
        raise RuntimeError('SSH 登录被拒绝，请核对用户名和密码。') from exc
    except (OSError, EOFError, paramiko.SSHException) as exc:
        detail = str(exc).replace(password, '******') if password else str(exc)
        if isinstance(exc, (OSError, EOFError, paramiko.ssh_exception.NoValidConnectionsError)) or detail.startswith(
            ('Error reading SSH protocol banner', 'No existing session')
        ):
            raise SshLinkError(detail) from exc
        raise RuntimeError('SSH 握手失败：'+detail) from exc
    _enable_tcp_keepalive(client)


def _remote(client, values, action, **extra):
    source = Path(__file__).with_name('remote.py').read_text(encoding='utf-8')
    request = json.dumps({'action': action, 'path': values['remote_path'], **extra})
    command = 'python3 -B -c '+shlex.quote(source)+' '+shlex.quote(request)
    _, output, errors = client.exec_command(command, timeout=155)
    raw = output.read()
    detail = errors.read().decode('utf-8', 'replace')
    code = output.channel.recv_exit_status()
    try:
        data = json.loads(raw)
    except ValueError as exc:
        raise RuntimeError('远程部署程序未返回有效结果。'+detail[-1500:]) from exc
    if code:
        raise RuntimeError(data.get('error', '远程部署操作失败。'))
    return data


def _upload(ssh, path, manifest, files):
    with ssh.open_sftp() as sftp:
        sftp.get_channel().settimeout(20)
        created = set()
        for name, data in {'manifest.json': manifest, **files}.items():
            parent = PurePosixPath(name).parent
            for directory in reversed((parent, *parent.parents)):
                if directory != PurePosixPath('.') and directory not in created:
                    sftp.mkdir(path+'/'+str(directory), mode=0o755)
                    created.add(directory)
            dest = path+'/'+name
            with sftp.open(dest, 'wb') as fd:
                fd.write(data)
            sftp.chmod(dest, 0o644)


def _mcu_readback(path, files):
    if remote.MCU_MANIFEST_PATH not in files:
        raise ValueError('部署包缺少连续路线 MCU 校验清单。')
    content = Path(path).expanduser().read_bytes()
    encoding = 'utf-16' if content.startswith((b'\xff\xfe', b'\xfe\xff')) else 'utf-8-sig'
    try:
        receipt = json.loads(content.decode(encoding))
    except (ValueError, UnicodeError) as exc:
        raise ValueError('请选择 MCU U 盘核验生成的 JSON 读回回执。') from exc
    return remote.validate_mcu_readback(receipt, json.loads(files[remote.MCU_MANIFEST_PATH]))


def operate(action, values, entered_password, log, *, mcu_readback=None):
    import paramiko

    values = settings.validate(values)
    if mcu_readback is not None and (action != 'deploy' or values['remote_path'] != settings.DEFAULT_PI_PATH):
        raise ValueError('MCU 读回登记只用于默认路径的 R3 部署。')
    secret = entered_password or settings.password_for(values)
    if not secret:
        raise ValueError('请输入 SSH 密码，或先从手册导入。')
    payload = _package(values, log) if action == 'deploy' else None
    receipt = _mcu_readback(mcu_readback, payload[1]) if mcu_readback is not None else None
    try:
        with paramiko.SSHClient() as ssh:
            log('连接 {}:{}…'.format(values['host'], values['port']))
            _connect(ssh, values, secret)
            report = (_remote(ssh, values, 'inspect', for_deploy=True)
                      if action == 'deploy' else _remote(ssh, values, 'inspect'))
            log('已校验 R3 机体身份与目标目录。')
            if report.get('recovery_session'):
                log('控制入口损坏，已通过 ROS 独立状态确认空闲，准备恢复部署。')
            if action == 'check':
                log(json.dumps(report, ensure_ascii=False, indent=2))
                return report
            if action == 'rollback':
                log('交换当前目录和上一份备份…')
                result = _remote(ssh, values, 'rollback')
            elif action == 'deploy':
                stage = _remote(ssh, values, 'stage')
                log('上传至临时目录，随后核对校验和…')
                _upload(ssh, stage['stage'], *payload)
                log('校验并交换整目录。默认路径会重载 ROS 运动控制节点…')
                try:
                    commit = {'id': stage['id']}
                    if 'recovery_session' in report:
                        commit['recovery_session'] = report['recovery_session']
                    if receipt is not None:
                        commit['mcu_readback'] = receipt
                    result = _remote(ssh, values, 'commit', **commit)
                except (OSError, socket.timeout, paramiko.SSHException) as exc:
                    raise RuntimeError('提交后连接中断。先测试连接核对当前部署编号 '+stage['id']+'，再决定是否回退。') from exc
            else:
                raise ValueError('未知操作。')
            log(json.dumps(result, ensure_ascii=False, indent=2))
            if not result.get('owner_reloaded'):
                log('自定义目录已替换。正式 ROS 启动仍指向原配置路径。')
            return result
    except SshLinkError:
        raise
    except Exception as exc:
        message = str(exc).replace(secret, '******')
        raise RuntimeError(message) from None
