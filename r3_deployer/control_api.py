"""Blocking Python API for the installed R3 warm owner over verified SSH.

Result dictionaries retain the Pi's response fields and add `ok`. Use
independent caller threads for long-running routes and Stop.
"""

from contextlib import contextmanager
import json
from pathlib import PurePosixPath
import re
import shlex
import time

from . import deploy, settings


_ROUTES = frozenset({'0', '1', '2'})
DROP_YAW_LIMIT_DEG = 90
_ACCEPTED_PREFIX = 'R3 command accepted;'
_MAX_RESPONSE_BYTES = 16 * 1024 * 1024
_STATUS_TIMEOUT_S = 8
_RESPONSE_TIMEOUT_S = 30
_SCAN_TIMEOUT_S = 90
_CHANNEL_DRAIN_TIMEOUT_S = 8
EMPTY_RESET_REQUIRED_MESSAGE = '上次任务可能仍在持物。固定或移走方块后，点击「恢复空载站姿」，再把机器人摆回起点。'


class RobotControlUncertainError(RuntimeError):
    """A dispatched command lost its confirmation, so repeating it is unsafe."""

    def __init__(self, message, *, accepted=None, request=None):
        super().__init__(message)
        self.accepted = accepted
        self.request = request

    def as_result(self):
        result = {'ok': False, 'status': 'start_unconfirmed', 'exit_code': None,
                  'accepted': self.accepted, 'exception_type': type(self).__name__, 'reason': str(self)}
        if self.request is not None:
            result['request'] = self.request
        return result


def _cancelled(event):
    return event is not None and event.is_set()


def DropYaw_Validate(degrees):
    if type(degrees) is not int or abs(degrees) > DROP_YAW_LIMIT_DEG:
        raise ValueError('放物转角须为 -90 到 +90 之间的整数度。')
    return degrees


def _skipped():
    return {'ok': False, 'status': 'cancelled_before_dispatch', 'accepted': False, 'exit_code': None}


def _preflight_rejection_reason(ready):
    observed = ready.get('observed_status')
    if isinstance(observed, dict):
        owner = observed.get('owner')
        if not isinstance(owner, dict):
            owner = {}
        if observed.get('prepared') is False:
            return '树莓派控制服务尚未就绪，任务未发送。'
        if observed.get('task_active') is True or owner.get('in_flight') is True or owner.get('mode') == 'auto':
            return '机器人正在执行任务，新的任务未发送。'
        if owner.get('mcu_ready') is False:
            return 'MCU 握手未就绪，任务未发送。'
    return ready.get('reason') or 'R3 空闲与机体检查未通过，任务未发送。'


def RobotStart_EmptyResetStatus(result):
    """Recover the owner state carried by a rejected route or its preflight."""
    if not isinstance(result, dict) or result.get('accepted') is not False:
        return None
    preflight = result.get('preflight')
    for status in (result.get('observed_status'), preflight,
                   preflight.get('observed_status') if isinstance(preflight, dict) else None):
        if not isinstance(status, dict):
            continue
        owner = status.get('owner')
        if not isinstance(owner, dict):
            owner = status.get('diagnostic')
        if (isinstance(owner, dict) and owner.get('robot_id') == 3
                and owner.get('restart_requires_empty_reset') is True):
            return dict(status, owner=owner,
                        task_active=status.get('task_active', owner.get('mode') == 'auto'))
    return None


class _Reply:
    def __init__(self, log):
        self.log = log
        self.pending = [bytearray(), bytearray()]
        self.stdout_result = None
        self.stderr_result = None
        self.accepted = False
        self.accepted_monotonic = None
        self.bytes_read = 0

    def feed(self, data, stderr):
        self.bytes_read += len(data)
        if self.bytes_read > _MAX_RESPONSE_BYTES:
            raise RuntimeError('R3 返回数据超出读取上限。')
        index = int(stderr)
        first, newline, remaining = data.partition(b'\n')
        self.pending[index].extend(first)
        if not newline:
            return
        self._line(self.pending[index], stderr)
        *lines, remainder = remaining.split(b'\n')
        self.pending[index] = bytearray(remainder)
        for line in lines:
            self._line(line, stderr)

    def _line(self, raw, stderr):
        line = raw.decode('utf-8', 'replace').strip()
        if not line:
            return
        if not stderr and line.startswith(_ACCEPTED_PREFIX):
            self.accepted = True
            if self.accepted_monotonic is None:
                self.accepted_monotonic = time.monotonic()
            self.log('树莓派已接受自动命令，正在等待任务结果。')
            return
        try:
            data = json.loads(line)
        except ValueError:
            data = None
        if isinstance(data, dict) and ('event' in data or 'status' in data):
            if stderr:
                self.stderr_result = data
            else:
                self.stdout_result = data
        else:
            self.log(line)

    def finish(self, exit_code):
        for stderr, remaining in enumerate(self.pending):
            self._line(remaining, bool(stderr))
        preferred = self.stderr_result if exit_code else self.stdout_result
        result = preferred or self.stdout_result or self.stderr_result
        if result is None or exit_code < 0:
            raise RuntimeError('树莓派未返回可确认的 R3 结果。')
        return dict(result, ok=exit_code == 0, exit_code=exit_code,
                    accepted=self.accepted or result.get('accepted') is True)


class RobotApi:
    """Use saved SSH settings, optionally overridden for this API instance."""

    def __init__(self, values=None, password=None, log=None):
        self._values = settings.validate({**settings.load(), **(values or {})})
        self._password = password
        self._log = log if log is not None else lambda _line: None

    def _credential(self):
        secret = self._password or settings.password_for(self._values)
        if not secret:
            raise ValueError('请输入 SSH 密码，或先在部署设置中保存密码。')
        return secret

    @contextmanager
    def _connection(self):
        import paramiko

        secret = self._credential()
        with paramiko.SSHClient() as client:
            deploy._connect(client, self._values, secret)
            transport = client.get_transport()
            if transport is None or not transport.is_active():
                raise deploy.SshLinkError('SSH transport is inactive.')
            transport.set_keepalive(10)
            yield transport, lambda line: self._log(str(line).replace(secret, '******'))

    def _exchange(self, transport, log, arguments, *, motion=False, stopping=False, cancel_event=None):
        if _cancelled(cancel_event):
            return _skipped()
        # An alternate upload path does not move the already-installed ROS owner.
        entry = PurePosixPath(settings.DEFAULT_PI_PATH) / 'warm_client.py'
        command = shlex.join(['python3', '-B', '-u', str(entry), *arguments])
        reply = _Reply(log)
        timeout = {'scan': _SCAN_TIMEOUT_S, 'status': _STATUS_TIMEOUT_S}.get(arguments[0], _RESPONSE_TIMEOUT_S)
        try:
            channel = transport.open_session(timeout=10)
        except Exception as exc:
            if isinstance(exc, (OSError, EOFError)) or not transport.is_active():
                raise deploy.SshLinkError(exc) from exc
            raise
        with channel:
            channel.settimeout(8)
            if _cancelled(cancel_event):
                return _skipped()
            try:
                dispatch_started = time.monotonic()
                channel.exec_command(command)
                deadline = time.monotonic() + timeout
                exit_code = None
                drain_deadline = None
                while True:
                    if channel.recv_ready():
                        reply.feed(channel.recv(32768), False)
                    if channel.recv_stderr_ready():
                        reply.feed(channel.recv_stderr(32768), True)
                    if exit_code is None and channel.exit_status_ready():
                        exit_code = channel.recv_exit_status()
                        drain_deadline = time.monotonic() + _CHANNEL_DRAIN_TIMEOUT_S
                    # SSH can deliver exit-status before the last stdout/stderr packets.
                    if (exit_code is not None
                            and (channel.eof_received or channel.closed or time.monotonic() >= drain_deadline)
                            and not channel.recv_ready() and not channel.recv_stderr_ready()):
                        if exit_code < 0:
                            raise deploy.SshLinkError('SSH 通道已关闭，未收到结束回执。')
                        result = reply.finish(exit_code)
                        if motion and result.get('event') != 'done':
                            observed = result.get('observed_status')
                            if (result.get('status') != 'not_started_or_stopped'
                                    or not isinstance(observed, dict)
                                    or observed.get('event') not in ('error', 'cancelled')):
                                raise RuntimeError('自动任务没有返回结束回执。')
                        if stopping and (result.get('event') != 'stopped'
                                         or result.get('status') not in ('stop_requested', 'already_idle')):
                            raise RuntimeError('停止请求没有返回确认回执。')
                        if motion and reply.accepted_monotonic is not None and arguments[0] in _ROUTES:
                            result['client_dispatch_to_accept_s'] = reply.accepted_monotonic - dispatch_started
                        return result
                    if exit_code is None:
                        if not transport.is_active():
                            raise deploy.SshLinkError('SSH 会话在回执前结束。')
                        if not (motion and reply.accepted) and time.monotonic() >= deadline:
                            raise TimeoutError('等待 R3 回执超时。')
                    time.sleep(.025)
            except (Exception, KeyboardInterrupt) as exc:
                cause = deploy.SshLinkError(exc) if isinstance(exc, (OSError, EOFError)) else exc
                if motion or stopping:
                    accepted = True if reply.accepted else None
                    message = ('树莓派已接收任务，但未收到结束回执。请重新连接查看状态，必要时单独发送停止。'
                               if accepted else '无法确认机器人当前状态。请读取状态或单独发送停止。')
                    raise RobotControlUncertainError(
                        message, accepted=accepted, request=arguments[0]
                    ) from cause
                if cause is not exc:
                    raise cause from exc
                raise

    def status(self):
        """Read the motion owner's latest reported state, including mission progress."""
        with self._connection() as (transport, log):
            return self._exchange(transport, log, ['status'])

    def start(self, command, *, cancel_event=None, drop_yaw_deg=0):
        """Explicitly run route 0, 1 or 2 and wait for its final R3 response."""
        route = self._route(command)
        angle = DropYaw_Validate(drop_yaw_deg)
        if _cancelled(cancel_event):
            return _skipped()
        self._require_default_path()
        started = time.monotonic()
        with self._connection() as (transport, log):
            connected = time.monotonic()
            ready = self._exchange(transport, log, ['status', '--device-check'], cancel_event=cancel_event)
            checked = time.monotonic()
            if ready.get('status') == 'cancelled_before_dispatch':
                return ready
            if not ready['ok'] or ready.get('event') != 'status' or ready.get('protocol') != 'r3.warm.v1':
                return {
                    'ok': False, 'status': 'preflight_rejected', 'accepted': False,
                    'exit_code': ready['exit_code'],
                    'reason': _preflight_rejection_reason(ready),
                    'preflight': ready,
                }
            empty_reset = {'ok': False, 'status': 'empty_reset_required', 'accepted': False,
                           'exit_code': None, 'reason': EMPTY_RESET_REQUIRED_MESSAGE, 'preflight': ready}
            if RobotStart_EmptyResetStatus(empty_reset):
                return empty_reset
            arguments = [route]
            if angle:
                arguments.extend(['--drop-yaw-deg', str(angle)])
            result = self._exchange(transport, log, arguments, motion=True, cancel_event=cancel_event)
            result['client_startup_timing'] = {'ssh_connect_s': connected - started,
                                               'device_check_s': checked - connected}
            if 'client_dispatch_to_accept_s' in result:
                result['client_startup_timing']['dispatch_to_accept_s'] = result.pop('client_dispatch_to_accept_s')
            if RobotStart_EmptyResetStatus(result):
                result['reason_zh'] = EMPTY_RESET_REQUIRED_MESSAGE
            return result

    def stop(self):
        """Cancel remaining automatic actions through an independent SSH connection."""
        with self._connection() as (transport, log):
            return self._exchange(transport, log, ['stop'], stopping=True)

    def reset_empty(self, *, cancel_event=None):
        """Request the explicit arms-down stand once the motion owner is idle."""
        self._require_default_path()
        with self._connection() as (transport, log):
            return self._exchange(transport, log, ['stand', '--idle-only'], motion=True,
                                  cancel_event=cancel_event)

    def scan(self):
        """Read chest-camera AR detections through the Pi's read-only scanner."""
        with self._connection() as (transport, log):
            return self._exchange(transport, log, ['scan', '--json'])

    def deploy(self, *, mcu_readback=None):
        """Run the existing deployment transaction and return its verified receipt."""
        options = {'mcu_readback': mcu_readback} if mcu_readback is not None else {}
        result = deploy.operate('deploy', self._values, self._credential(), self._log, **options)
        if not isinstance(result, dict):
            raise RuntimeError('部署没有返回可确认的回执。')
        return dict(result, ok=result.get('status') == 'deployed')

    def update_then_start(self, command, *, cancel_event=None, mcu_readback=None, drop_yaw_deg=0):
        """Deploy, verify activation and only then dispatch the explicitly chosen route."""
        route = self._route(command)
        angle = DropYaw_Validate(drop_yaw_deg)
        self._require_default_path()
        if _cancelled(cancel_event):
            return {'ok': False, 'deployment': None, 'start': _skipped()}
        deployment = self.deploy(mcu_readback=mcu_readback) if mcu_readback is not None else self.deploy()
        if _cancelled(cancel_event):
            return {'ok': False, 'deployment': deployment, 'start': _skipped()}
        activated = (deployment['ok'] and deployment.get('owner_reloaded') is True
                     and deployment.get('path') == settings.DEFAULT_PI_PATH
                     and (mcu_readback is None or deployment.get('mcu_readback_registered') is True)
                     and re.fullmatch(r'[0-9a-f]{32}', deployment.get('id') or '') is not None)
        if not activated:
            return {'ok': False, 'deployment': deployment, 'start': {
                'ok': False, 'status': 'deployment_activation_unconfirmed', 'accepted': False,
                'exit_code': None, 'reason': '部署回执尚未确认正式路径和控制节点已激活。',
            }}
        try:
            started = self.start(route, cancel_event=cancel_event, drop_yaw_deg=angle)
        except RobotControlUncertainError as exc:
            started = exc.as_result()
        except Exception as exc:
            started = {'ok': False, 'status': 'startup_error', 'accepted': False,
                       'exit_code': None, 'reason': str(exc)}
        return {'ok': started['ok'], 'deployment': deployment, 'start': started}

    @staticmethod
    def _route(command):
        route = str(command)
        if route not in _ROUTES:
            raise ValueError('自动命令只接受 0、1 或 2。')
        return route

    def _require_default_path(self):
        if self._values['remote_path'] != settings.DEFAULT_PI_PATH:
            raise ValueError('启动要求部署路径为运行中 ROS owner 使用的 '+settings.DEFAULT_PI_PATH)
