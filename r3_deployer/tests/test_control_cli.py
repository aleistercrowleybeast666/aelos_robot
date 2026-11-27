from contextlib import redirect_stderr, redirect_stdout
import io
import json
from pathlib import Path
import unittest
from unittest.mock import patch

from r3_deployer import __main__, settings


class ControlCliTests(unittest.TestCase):
    def run_command(self, args):
        output = io.StringIO()
        with patch.object(settings, 'load', return_value=settings.DEFAULTS), \
                patch.object(settings, 'password_for', return_value='fixture'), \
                patch('r3_deployer.control_api.RobotApi') as api, redirect_stdout(output):
            instance = api.return_value
            instance.start.return_value = {'ok': True, 'accepted': True}
            instance.stop.return_value = {'ok': True, 'status': 'stop_requested'}
            instance.update_then_start.return_value = {'ok': False, 'deployment': {}}
            status = __main__.main(args)
        return status, json.loads(output.getvalue()), instance

    def test_run_dispatch(self):
        status, result, api = self.run_command(['run', '0'])
        self.assertEqual(status, 0)
        self.assertTrue(result['accepted'])
        api.start.assert_called_once_with('0')

    def test_update_then_start_propagates_failure(self):
        status, result, api = self.run_command(['update', '--run', '2'])
        self.assertEqual(status, 1)
        self.assertFalse(result['ok'])
        api.update_then_start.assert_called_once_with('2')
        api.start.assert_not_called()

    def test_release_angle_is_forwarded_for_direct_and_combined_start(self):
        _, _, api = self.run_command(['run', '0', '--drop-yaw-deg', '-15'])
        api.start.assert_called_once_with('0', drop_yaw_deg=-15)
        _, _, api = self.run_command(['update', '--run', '2', '--drop-yaw-deg', '20'])
        api.update_then_start.assert_called_once_with('2', drop_yaw_deg=20)

    def test_release_angle_rejected_without_route(self):
        with self.assertRaises(SystemExit), redirect_stderr(io.StringIO()):
            __main__.main(['update', '--drop-yaw-deg', '10'])

    def test_update_source_override_does_not_change_saved_values(self):
        saved = dict(settings.DEFAULTS)
        with patch.object(settings, 'load', return_value=saved), \
                patch.object(settings, 'password_for', return_value='fixture'), \
                patch.object(settings, 'save') as save, \
                patch('r3_deployer.deploy.operate') as operate:
            status = __main__.main(['update', '--source', 'local', '--local-path', r'D:\release'])
        action, values, _, _ = operate.call_args.args
        self.assertEqual(status, 0)
        self.assertEqual(action, 'deploy')
        self.assertEqual(values['deployment_source'], 'local')
        self.assertEqual(values['local_path'], r'D:\release')
        self.assertEqual(saved['deployment_source'], 'remote')
        save.assert_not_called()

    def test_update_then_run_uses_one_time_remote_url_override(self):
        url = 'https://example.org/release'
        with patch.object(settings, 'load', return_value=dict(settings.DEFAULTS, deployment_source='local')), \
                patch.object(settings, 'password_for', return_value='fixture'), \
                patch('r3_deployer.control_api.RobotApi') as api, redirect_stdout(io.StringIO()):
            api.return_value.update_then_start.return_value = {'ok': True}
            status = __main__.main(['deploy', '--source', 'remote', '--remote-url', url, '--run', '1'])
        self.assertEqual(status, 0)
        self.assertEqual(api.call_args.args[0]['deployment_source'], 'remote')
        self.assertEqual(api.call_args.args[0]['remote_url'], url)
        api.return_value.update_then_start.assert_called_once_with('1')

    def test_config_saves_source_and_remote_url(self):
        url = 'https://example.org/packages/current'
        output = io.StringIO()
        with patch.object(settings, 'load', return_value=settings.DEFAULTS), \
                patch.object(settings, 'password_for', return_value='fixture'), \
                patch.object(settings, 'save') as save, redirect_stdout(output):
            status = __main__.main(['config', '--source', 'local', '--remote-url', url])
        self.assertEqual(status, 0)
        self.assertEqual(save.call_args.args[0]['deployment_source'], 'local')
        self.assertEqual(save.call_args.args[0]['remote_url'], url)
        self.assertTrue(json.loads(output.getvalue())['password_saved'])

    def test_stop_uses_the_stop_operation(self):
        status, result, api = self.run_command(['stop'])
        self.assertEqual(status, 0)
        self.assertEqual(result['status'], 'stop_requested')
        api.stop.assert_called_once_with()

    def test_readback_registration_has_no_implicit_start(self):
        with patch.object(settings, 'load', return_value=settings.DEFAULTS), \
                patch.object(settings, 'password_for', return_value='fixture'), \
                patch('r3_deployer.deploy.operate') as operate, \
                patch('r3_deployer.control_api.RobotApi') as api:
            self.assertEqual(__main__.main(['update', '--mcu-readback', 'usb.json']), 0)
        self.assertEqual(operate.call_args.kwargs['mcu_readback'], Path('usb.json'))
        api.assert_not_called()

    def test_readback_update_then_run_requires_confirmed_registration_and_owner(self):
        result = {'status': 'deployed', 'owner_reloaded': True, 'path': settings.DEFAULT_PI_PATH,
                  'id': 'a'*32, 'mcu_readback_registered': False}
        with patch.object(settings, 'load', return_value=settings.DEFAULTS), \
                patch.object(settings, 'password_for', return_value='fixture'), \
                patch('r3_deployer.deploy.operate', return_value=result), \
                patch('r3_deployer.control_api.RobotApi') as api, \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(__main__.main(['update', '--mcu-readback', 'usb.json', '--run', '0']), 1)
            api.assert_not_called()
            result['mcu_readback_registered'] = True
            api.return_value.start.return_value = {'ok': True}
            self.assertEqual(__main__.main(['update', '--mcu-readback', 'usb.json', '--run', '0']), 0)
            api.return_value.start.assert_called_once_with('0')
