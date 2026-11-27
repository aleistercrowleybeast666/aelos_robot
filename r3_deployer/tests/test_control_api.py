import contextlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import threading
import unittest
from unittest.mock import Mock, patch

import paramiko

from r3_deployer import control_api, deploy, settings


def _line(**fields):
    return json.dumps(fields).encode('utf-8') + b'\n'


class FakeChannel:
    def __init__(self, output=(), errors=(), exit_code=0, on_exec=None, eof=True):
        self.output = list(output)
        self.errors = list(errors)
        self.exit_code = exit_code
        self.on_exec = on_exec
        self.commands = []
        self.closed = False
        self.eof_received = eof
        self.exit_status_read = False

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass

    def settimeout(self, timeout):
        pass

    def exec_command(self, command):
        self.commands.append(command)
        if self.on_exec:
            self.on_exec()

    def recv_ready(self):
        return bool(self.output)

    def recv_stderr_ready(self):
        return bool(self.errors)

    def recv(self, _size):
        return self.output.pop(0)

    def recv_stderr(self, _size):
        return self.errors.pop(0)

    def exit_status_ready(self):
        return not (self.output or self.errors)

    def recv_exit_status(self):
        self.exit_status_read = True
        return self.exit_code


class FakeTransport:
    def __init__(self, *channels):
        self.channels = list(channels)

    def open_session(self, timeout):
        return self.channels.pop(0)

    def is_active(self):
        return True


class RobotApiTests(unittest.TestCase):
    def setUp(self):
        self.logs = []
        with patch.object(settings, 'load', return_value=settings.DEFAULTS):
            self.api = control_api.RobotApi(password='test-password', log=self.logs.append)

    @staticmethod
    def ready(on_exec=None):
        return FakeChannel([_line(event='status', protocol='r3.warm.v1', body_commands=0)],
                           on_exec=on_exec)

    @contextlib.contextmanager
    def connection(self, *channels):
        yield FakeTransport(*channels), self.logs.append

    def test_start_has_read_only_preflight_then_one_motion_request(self):
        check = self.ready()
        motion = FakeChannel([b'R3 command acce', b'pted; queue acknowledged\n',
                              _line(event='done', status='task_complete')])
        with patch.object(self.api, '_connection', return_value=self.connection(check, motion)):
            result = self.api.start('1')
        self.assertTrue(result['ok'])
        self.assertTrue(result['accepted'])
        self.assertEqual(set(result['client_startup_timing']),
                         {'ssh_connect_s', 'device_check_s', 'dispatch_to_accept_s'})
        self.assertTrue(all(seconds >= 0 for seconds in result['client_startup_timing'].values()))
        self.assertTrue(check.commands[0].endswith('status --device-check'))
        self.assertEqual(len(motion.commands), 1)
        self.assertTrue(motion.commands[0].endswith('warm_client.py 1'))
        self.assertEqual(len(self.logs), 1)

    def test_mcu_not_ready_returns_specific_preflight_reason_and_never_dispatches(self):
        check = FakeChannel(errors=[_line(
            status='not_started_or_stopped', reason='warm owner/MCU is not ready and idle',
            observed_status={'prepared': True, 'task_active': False,
                             'owner': {'mcu_ready': False, 'mode': 'manual'}}
        )], exit_code=1)
        unused_motion = FakeChannel()
        with patch.object(self.api, '_connection', return_value=self.connection(check, unused_motion)):
            outcome = self.api.start('0')
        self.assertEqual(outcome['status'], 'preflight_rejected')
        self.assertEqual(outcome['reason'], 'MCU 握手未就绪，任务未发送。')
        self.assertTrue(check.commands[0].endswith('status --device-check'))
        self.assertFalse(unused_motion.commands)

    def test_release_angle_reaches_warm_client_only_on_dispatch(self):
        check = self.ready()
        motion = FakeChannel([_line(event='done', status='task_complete')])
        with patch.object(self.api, '_connection', return_value=self.connection(check, motion)):
            self.api.start(2, drop_yaw_deg=-20)
        self.assertTrue(check.commands[0].endswith('status --device-check'))
        self.assertTrue(motion.commands[0].endswith('2 --drop-yaw-deg -20'))

    def test_invalid_angle_rejected_before_connecting_or_deploying(self):
        with patch.object(self.api, 'deploy') as deploy, patch.object(self.api, '_connection') as connect:
            for angle in (91, -91, 1.5, '0', True):
                with self.subTest(angle=angle), self.assertRaises(ValueError):
                    self.api.update_then_start(1, drop_yaw_deg=angle)
                with self.assertRaises(ValueError):
                    self.api.start(2, drop_yaw_deg=angle)
            deploy.assert_not_called()
            connect.assert_not_called()

    def test_loaded_owner_preflight_rejects_start_without_sending_motion(self):
        owner = {'robot_id': 3, 'mode': 'manual', 'in_flight': False, 'restart_requires_empty_reset': True}
        check = FakeChannel([_line(event='status', protocol='r3.warm.v1',
                                   task_active=False, owner=owner)])
        unused_motion = FakeChannel()
        with patch.object(self.api, '_connection', return_value=self.connection(check, unused_motion)):
            outcome = self.api.start('1')
        self.assertEqual(outcome['status'], 'empty_reset_required')
        self.assertFalse(outcome['accepted'])
        self.assertIn('恢复空载站姿', outcome['reason'])
        self.assertEqual(control_api.RobotStart_EmptyResetStatus(outcome)['owner'], owner)
        self.assertFalse(unused_motion.commands)

    def test_loaded_owner_rejection_after_preflight_preserves_and_translates_receipt(self):
        owner = {'robot_id': 3, 'mode': 'manual', 'in_flight': False, 'restart_requires_empty_reset': True}
        no_start = FakeChannel(errors=[_line(
            status='not_started_or_stopped', accepted=False, reason='owner rejected start',
            observed_status={'event': 'error', 'diagnostic': owner}
        )], exit_code=1)
        with patch.object(self.api, '_connection', return_value=self.connection(self.ready(), no_start)):
            outcome = self.api.start('1')
        self.assertFalse(outcome['accepted'])
        self.assertEqual(outcome['observed_status']['diagnostic'], owner)
        self.assertIn('恢复空载站姿', outcome['reason_zh'])

    def test_stop_intent_during_preflight_blocks_pending_dispatch(self):
        cancelled = threading.Event()
        unused_motion = FakeChannel()
        with patch.object(self.api, '_connection', return_value=self.connection(self.ready(cancelled.set),
                                                                                 unused_motion)):
            result = self.api.start(0, cancel_event=cancelled)
        self.assertEqual(result['status'], 'cancelled_before_dispatch')
        self.assertFalse(unused_motion.commands)

    def test_network_failure_after_submit_is_uncertain_without_retry(self):
        def fail():
            raise OSError('transport reset')

        motion = FakeChannel(on_exec=fail)
        with patch.object(self.api, '_connection', return_value=self.connection(self.ready(), motion)):
            with self.assertRaises(control_api.RobotControlUncertainError) as error:
                self.api.start('0')
        self.assertEqual(len(motion.commands), 1)
        self.assertIsInstance(error.exception.__cause__, deploy.SshLinkError)
        self.assertIsNone(error.exception.as_result()['accepted'])

    def test_network_failure_after_acceptance_retains_acknowledgement_without_retry(self):
        motion = FakeChannel([b'R3 command accepted; queue acknowledged\n'], exit_code=-1)
        with patch.object(self.api, '_connection', return_value=self.connection(self.ready(), motion)):
            with self.assertRaises(control_api.RobotControlUncertainError) as error:
                self.api.start('1')
        result = error.exception.as_result()
        self.assertTrue(result['accepted'])
        self.assertEqual(result['status'], 'start_unconfirmed')
        self.assertEqual(result['request'], '1')
        self.assertIn('已接收任务', result['reason'])
        self.assertEqual(len(motion.commands), 1)

    def test_exit_status_before_stdout_eof_does_not_truncate_terminal_json(self):
        line = _line(event='done', status='task_complete', report='x' * (20 * 16384))
        split = 19 * 16384
        stdout = [line[offset:offset + 16384] for offset in range(0, split, 16384)]
        motion = FakeChannel([b'R3 command accepted; queue acknowledged\n', *stdout], eof=False)

        def deliver_stdout_tail(_delay):
            if motion.exit_status_read and not motion.eof_received:
                motion.output.append(line[split:])
                motion.eof_received = True

        with patch.object(self.api, '_connection', return_value=self.connection(self.ready(), motion)), \
                patch.object(control_api.time, 'sleep', side_effect=deliver_stdout_tail):
            result = self.api.start('0')
        self.assertTrue(result['ok'])
        self.assertEqual(len(result['report']), 20 * 16384)
        self.assertEqual(self.logs, ['树莓派已接受自动命令，正在等待任务结果。'])

    def test_lost_read_only_ssh_channel_is_a_link_failure(self):
        with patch.object(self.api, '_connection', return_value=self.connection(FakeChannel(exit_code=-1))):
            with self.assertRaises(deploy.SshLinkError):
                self.api.status()

    def test_banner_reset_is_friendly_and_auth_rejection_stays_specific(self):
        with TemporaryDirectory() as directory:
            home = Path(directory)
            (home/'.ssh').mkdir()
            (home/'.ssh/known_hosts').touch()
            client = Mock()
            with patch.object(deploy.Path, 'home', return_value=home), patch.object(paramiko, 'HostKeys'):
                client.connect.side_effect = paramiko.SSHException(
                    'Error reading SSH protocol banner[WinError 10054]'
                )
                with self.assertRaises(deploy.SshLinkError) as error:
                    deploy._connect(client, settings.DEFAULTS, 'test-password')
                self.assertNotIn('WinError', str(error.exception))
                self.assertIn('10054', error.exception.detail)

                client.connect.side_effect = paramiko.AuthenticationException('Authentication failed.')
                with self.assertRaisesRegex(RuntimeError, '登录被拒绝') as error:
                    deploy._connect(client, settings.DEFAULTS, 'test-password')
                self.assertNotIsInstance(error.exception, deploy.SshLinkError)

    def test_cancel_intent_after_dispatch_keeps_accepted_result(self):
        cancelled = threading.Event()
        motion = FakeChannel([_line(event='done', status='stopped_no_further_auto_macros')],
                             exit_code=1, on_exec=cancelled.set)
        with patch.object(self.api, '_connection', return_value=self.connection(self.ready(), motion)):
            result = self.api.start('2', cancel_event=cancelled)
        self.assertEqual(result['status'], 'stopped_no_further_auto_macros')
        self.assertFalse(result['ok'])

    def test_update_does_not_start_if_owner_reactivation_unconfirmed(self):
        report = {'ok': True, 'status': 'deployed', 'path': settings.DEFAULT_PI_PATH,
                  'owner_reloaded': False, 'id': '1' * 32}
        with patch.object(self.api, 'deploy', return_value=report), patch.object(self.api, 'start') as start:
            result = self.api.update_then_start('0')
        start.assert_not_called()
        self.assertEqual(result['start']['status'], 'deployment_activation_unconfirmed')

    def test_update_preserves_verified_receipt_if_start_is_unconfirmed(self):
        report = {'ok': True, 'status': 'deployed', 'path': settings.DEFAULT_PI_PATH,
                  'owner_reloaded': True, 'id': 'a' * 32}
        with patch.object(self.api, 'deploy', return_value=report), patch.object(
                self.api, 'start', side_effect=control_api.RobotControlUncertainError('lost',accepted=True)
        ) as start:
            result = self.api.update_then_start(1)
        start.assert_called_once_with('1', cancel_event=None, drop_yaw_deg=0)
        self.assertEqual(result['deployment'], report)
        self.assertEqual(result['start']['status'], 'start_unconfirmed')
        self.assertTrue(result['start']['accepted'])

    def test_stop_preserves_non_immediate_semantics(self):
        stop = FakeChannel([_line(event='stopped', status='stop_requested', native_immediate_stop=False)])
        with patch.object(self.api, '_connection', return_value=self.connection(stop)):
            result = self.api.stop()
        self.assertTrue(result['ok'])
        self.assertIs(result['native_immediate_stop'], False)
        self.assertTrue(stop.commands[0].endswith('warm_client.py stop'))

    def test_empty_reset_is_standalone_and_idle_only(self):
        stand = FakeChannel([_line(event='done', status='stand_request_completed')])
        with patch.object(self.api, '_connection', return_value=self.connection(stand)):
            result = self.api.reset_empty()
        self.assertTrue(result['ok'])
        self.assertTrue(stand.commands[0].endswith('warm_client.py stand --idle-only'))
