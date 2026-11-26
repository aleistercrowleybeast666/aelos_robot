import hashlib
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

from r3_deployer import deploy, remote, settings, usb_mode


def _exchange(left, right):
    """Windows test substitute for the Pi's one-step Linux exchange."""
    temporary = left.with_name('swap-test')
    left.rename(temporary)
    right.rename(left)
    temporary.rename(right)


class DeploymentTransactionTests(unittest.TestCase):
    def setUp(self):
        if not (settings.DEFAULT_BUNDLE/'manifest.json').is_file():
            self.skipTest('Pi deployment bundle is not present beside the control source.')
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        directory = Path(temporary.name).resolve()
        self.target = directory/'app'
        self.owner_dir = directory/'owner'
        self.owner_dir.mkdir()
        self.owner = self.owner_dir/remote.OWNER_NAME
        self.owner.write_text('old owner', encoding='utf-8')
        for name in remote.OWNER_SUPPORT:
            (self.owner_dir/name).write_text('old '+name, encoding='utf-8')
        self.ticket = 'a'*32
        self.slot = remote._transaction(self.target, self.ticket)
        shutil.copytree(settings.DEFAULT_BUNDLE, self.slot)
        manifest = self.slot/'manifest.json'
        doc = json.loads(manifest.read_text(encoding='utf-8'))
        manifest.write_text(json.dumps(dict(doc, owner_directory=str(self.owner_dir))), encoding='utf-8')
        self.target.mkdir()
        shutil.copytree(self.slot/'app/config', self.target/'config')
        (self.target/'deployment.json').write_text('{"robot_id":3}', encoding='utf-8')
        (self.target/'old.py').write_text('old app', encoding='utf-8')
        self.calibration = self.target/'config/navigation.json'
        self.calibration.write_bytes(self.calibration.read_bytes()+b'\n')
        self.calibration_bytes = self.calibration.read_bytes()
        self.identity = ('0000000000000001','00000000000000000000000000000002')
        uid = patch.object(remote.os, 'getuid', return_value=self.owner.stat().st_uid, create=True)
        uid.start()
        self.addCleanup(uid.stop)
        patches = {
            'DEFAULT_PATH': patch.object(remote, 'DEFAULT_PATH', self.target),
            'OWNER_DIR': patch.object(remote, 'OWNER_DIR', self.owner_dir),
            '_target': patch.object(remote, '_target', side_effect=Path),
            '_verified_local_identity': patch.object(remote, '_verified_local_identity', return_value=self.identity),
            '_status': patch.object(remote, '_status', return_value={
                'task_active': False, 'owner': {'session': 'old', 'in_flight': False, 'mode': 'manual'},
            }),
            '_node_pid': patch.object(remote, '_node_pid', return_value=1),
            '_exchange': patch.object(remote, '_exchange', side_effect=_exchange),
            '_reload': patch.object(remote, '_reload', return_value={}),
        }
        self.mocks = {}
        for key, change in patches.items():
            self.mocks[key] = change.start()
            self.addCleanup(change.stop)

    def test_replacement_and_explicit_rollback_keep_both_generations(self):
        outcome = remote._commit(self.target, self.ticket)
        self.assertTrue(outcome['owner_reloaded'])
        self.assertFalse((self.target/'old.py').exists())
        self.assertTrue((self.target/'run_auto.py').exists())
        self.assertEqual(self.calibration.read_bytes(), self.calibration_bytes)
        self.assertEqual((Path(outcome['backup'])/'old.py').read_text(), 'old app')
        for name in remote.OWNER_SUPPORT:
            self.assertEqual((self.owner_dir/name).read_bytes(), (self.slot/'owner'/name).read_bytes())
        remote._rollback(self.target)
        self.assertEqual((self.target/'old.py').read_text(), 'old app')
        self.assertEqual(self.owner.read_text(), 'old owner')
        for name in remote.OWNER_SUPPORT:
            self.assertEqual((self.owner_dir/name).read_text(), 'old '+name)
        self.assertTrue((self.slot/'app/run_auto.py').exists())

    def test_failed_owner_reload_restores_both_filesystems(self):
        self.mocks['_reload'].side_effect = [RuntimeError('failed activation'), {}]
        with self.assertRaisesRegex(RuntimeError, 'original files restored'):
            remote._commit(self.target, self.ticket)
        self.assertEqual((self.target/'old.py').read_text(), 'old app')
        self.assertEqual(self.owner.read_text(), 'old owner')
        for name in remote.OWNER_SUPPORT:
            self.assertEqual((self.owner_dir/name).read_text(), 'old '+name)
        self.assertTrue((self.slot/'app/run_auto.py').exists())

    def _stage_route_upgrade(self, *, customized=False):
        updated = (self.slot/'app/routes.py').read_bytes()
        previous = updated + b'\n# previous stock revision\n'
        live = previous + b'\n# field customization\n' if customized else previous
        (self.target/'routes.py').write_bytes(live)
        manifest = self.slot/'manifest.json'
        document = json.loads(manifest.read_text(encoding='utf-8'))
        document['replace_stock_routes_sha256'] = hashlib.sha256(previous).hexdigest()
        manifest.write_text(json.dumps(document), encoding='utf-8')
        return live, updated

    def test_shipped_route_updates_and_remains_in_backup(self):
        previous, updated = self._stage_route_upgrade()
        outcome = remote._commit(self.target, self.ticket)
        self.assertEqual((self.target/'routes.py').read_bytes(), updated)
        self.assertEqual((Path(outcome['backup'])/'routes.py').read_bytes(), previous)
        remote._rollback(self.target)
        self.assertEqual((self.target/'routes.py').read_bytes(), previous)

    def test_custom_route_is_preserved_during_stock_upgrade(self):
        customized, _ = self._stage_route_upgrade(customized=True)
        remote._commit(self.target, self.ticket)
        self.assertEqual((self.target/'routes.py').read_bytes(), customized)

    def test_deploys_route_filter_with_the_pc_route_and_preserves_navigation_calibration(self):
        name = 'config/route_steps.json'
        live = self.target/name
        previous = json.loads(live.read_text(encoding='utf-8'))
        previous['micro_turn_filter']['alpha'] = 0.25
        live.write_text(json.dumps(previous), encoding='utf-8')
        staged = self.slot/'app'/name
        steps = json.loads(staged.read_text(encoding='utf-8'))
        steps['tag1_to_tag2_forward_steps'] = 10
        payload = json.dumps(steps).encode('utf-8')
        (self.slot/'app'/name).write_bytes(payload)
        manifest = self.slot/'manifest.json'
        document = json.loads(manifest.read_text(encoding='utf-8'))
        document['files']['app/'+name] = hashlib.sha256(payload).hexdigest()
        manifest.write_text(json.dumps(document), encoding='utf-8')

        outcome = remote._commit(self.target, self.ticket)

        deployed = json.loads(live.read_text(encoding='utf-8'))
        self.assertEqual(deployed, steps)
        self.assertEqual(deployed['micro_turn_filter']['alpha'], 0.15)
        self.assertEqual(self.calibration.read_bytes(), self.calibration_bytes)
        self.assertEqual(json.loads((Path(outcome['backup'])/name).read_text(encoding='utf-8')), previous)

    def test_bad_upload_is_refused_before_exchange(self):
        (self.slot/'app/run_auto.py').write_text('damaged', encoding='utf-8')
        with self.assertRaisesRegex(RuntimeError, 'checksum mismatch'):
            remote._commit(self.target, self.ticket)
        self.mocks['_exchange'].assert_not_called()

    def test_empty_live_mission_config_keeps_inactive_default_and_original_backup(self):
        config = self.target/'config/mission_v5.json'
        config.write_bytes(b'')
        outcome = remote._commit(self.target, self.ticket)
        self.assertIs(json.loads(config.read_text(encoding='utf-8'))['enabled'], False)
        self.assertEqual((Path(outcome['backup'])/'config/mission_v5.json').read_bytes(), b'')
        self.assertEqual(self.calibration.read_bytes(), self.calibration_bytes)

    def test_invalid_nonempty_live_mission_config_identifies_file(self):
        config = self.target/'config/mission_v5.json'
        config.write_bytes(b'{')
        with self.assertRaisesRegex(ValueError, 'mission_v5.json'):
            remote._commit(self.target, self.ticket)
        self.mocks['_exchange'].assert_not_called()

    def _mcu_receipt(self):
        data = (self.slot/remote.MCU_MANIFEST_PATH).read_bytes()
        return {'robot_id': 3, 'drive': 'G:\\', 'readback_all_match': True, 'reload_confirmed': False,
                'different_files': [], 'files': json.loads(data),
                'volume': {'serial': 'A12BC234', 'profile_sha256': 'a'*64, 'filesystem': 'FAT32'}}

    def _private_mcu_manifest(self, root):
        manifest = root/'MANIFEST.json'
        if not manifest.is_file():
            self.skipTest('This comparison needs the original MCU action fixture.')
        return json.loads(manifest.read_text(encoding='utf-8'))

    def test_existing_mcu_copy_can_produce_a_receipt_without_deploying_to_usb(self):
        expected = self._mcu_receipt()
        comparison = {k:v for k,v in expected.items() if k not in ('readback_all_match', 'reload_confirmed')}
        report_dir = self.target/'receipts'
        with patch.object(usb_mode, 'inspect_usb', return_value=comparison.copy()) as inspect:
            saved = usb_mode.McuReadback_VerifyAndSave('G:', settings.DEFAULT_MCU_OVERLAY, report_dir)
        inspect.assert_called_once_with(settings.DEFAULT_MCU_OVERLAY, settings.DEFAULT_MCU_OVERLAY/'MANIFEST.json', 'G:')
        actual = json.loads(Path(saved['receipt_path']).read_text(encoding='utf-8'))
        self.assertEqual(remote.validate_mcu_readback(actual, expected['files']), expected)
        with patch.object(usb_mode, 'inspect_usb',
                          return_value=dict(comparison, different_files=['lua/craic_single_micro.lua'])):
            with self.assertRaisesRegex(ValueError, 'craic_single_micro.lua'):
                usb_mode.McuReadback_VerifyAndSave('G:',settings.DEFAULT_MCU_OVERLAY,report_dir)
        self.assertEqual(list(report_dir.iterdir()), [Path(saved['receipt_path'])])

    def test_readback_registers_both_records_and_restores_them_on_rollback(self):
        readback = self._mcu_receipt()
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        before = {name: (self.target/'config'/name).read_bytes() for name in (
            'mcu_guard_deployment.json', 'micro_mcu_deployment.json',
        )}
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': {'cmd0_route_deployment_readback_ready': True}}}
        with patch.object(remote, '_boot_id', return_value=boot):
            result = remote._commit(self.target, self.ticket, readback)
        self.assertTrue(result['mcu_readback_registered'])
        self.assertFalse(result['mcu_runtime_probe_sent'])
        records = {name: json.loads((self.target/'config'/name).read_text(encoding='utf-8')) for name in before}
        for record in records.values():
            self.assertEqual(record['verified_boot_id'], boot)
            self.assertTrue(record['reload_confirmed'])
            self.assertEqual((record['cpu_serial'],record['machine_id']),self.identity)
        self.assertEqual(records['micro_mcu_deployment.json']['verified_files_sha256'], readback['files'])
        self.assertEqual(set(records['mcu_guard_deployment.json']['verified_files_sha256']),
                         set(readback['files']) - {'lua/craic_single_micro.lua'})
        remote._rollback(self.target)
        for name, original in before.items():
            self.assertEqual((self.target/'config'/name).read_bytes(), original)

    def test_readback_requires_matching_usb_evidence_before_exchange(self):
        readback = self._mcu_receipt()
        readback['files']['lua/craic_auto_bridge.lua'] = '0'*64
        with self.assertRaisesRegex(ValueError, 'MCU 回执与受支持的动作包不符'):
            remote._commit(self.target, self.ticket, readback)
        self.mocks['_exchange'].assert_not_called()

    def test_five_stride_exact_readback_registers_turn_and_bridge_in_both_records(self):
        baseline = self._mcu_receipt()
        files = self._private_mcu_manifest(settings.ROBOT_WORKSPACE/'mcu_micro_turn_tuned_candidate')
        readback = dict(baseline, files=files)
        path = self.target.parent/'readback.json'
        path.write_text(json.dumps(readback), encoding='utf-8')
        payload = {remote.MCU_MANIFEST_PATH: (self.slot/remote.MCU_MANIFEST_PATH).read_bytes()}
        self.assertEqual(deploy._mcu_readback(path, payload), readback)
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': {
            'cmd0_route_deployment_readback_ready': True, 'medium_turn_deployment_readback_ready': True,
            'cmd0_right5_deployment_readback_ready': True}}}
        with patch.object(remote, '_boot_id', return_value=boot):
            result = remote._commit(self.target, self.ticket, readback)
        self.assertTrue(result['mcu_readback_registered'])
        self.assertFalse(result['mcu_runtime_probe_sent'])
        micro = json.loads((self.target/'config/micro_mcu_deployment.json').read_text(encoding='utf-8'))
        guard = json.loads((self.target/'config/mcu_guard_deployment.json').read_text(encoding='utf-8'))
        self.assertEqual(micro['verified_files_sha256'], readback['files'])
        self.assertEqual(guard['verified_files_sha256'],
                         {name: digest for name, digest in readback['files'].items()
                          if name != 'lua/craic_single_micro.lua'})

    def test_medium_receipt_rejects_a_bridge_or_turn_from_the_previous_variant(self):
        readback = self._mcu_receipt()
        expected = readback['files']
        medium = {**expected, **remote._MCU_MEDIUM_SHA256}
        previous = {**expected, 'lua/craic_single_micro.lua': remote._MCU_TUNED_MICRO_SHA256}
        for name in remote._MCU_MEDIUM_SHA256:
            with self.subTest(file=name):
                with self.assertRaisesRegex(ValueError, 'MCU 回执与受支持的动作包不符'):
                    remote.validate_mcu_readback(dict(readback, files={**medium, name: previous[name]}), expected)

    def test_pair_receipt_registers_exact_files_and_requires_pair_capability(self):
        baseline = self._mcu_receipt()
        files = self._private_mcu_manifest(settings.ROBOT_WORKSPACE/'mcu_horizontal_pair_candidate')
        readback = dict(baseline, files=files)
        self.assertEqual(remote.validate_mcu_readback(readback,baseline['files']),readback)
        mixed = dict(readback, files={**readback['files'],
                     'lua/craic_actions.lua': baseline['files']['lua/craic_actions.lua']})
        with self.assertRaisesRegex(ValueError, 'MCU 回执与受支持的动作包不符'):
            remote.validate_mcu_readback(mixed,baseline['files'])
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        status = {'cmd0_route_deployment_readback_ready': True, 'medium_turn_deployment_readback_ready': True,
                  'cmd0_right5_deployment_readback_ready': True, 'pair_deployment_readback_ready': True}
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': status}}
        with patch.object(remote,'_boot_id',return_value=boot):
            result = remote._commit(self.target,self.ticket,readback)
        self.assertTrue(result['mcu_readback_registered'])

    def test_medium_registration_rejects_owner_without_medium_readback(self):
        baseline = self._mcu_receipt()
        readback = dict(baseline, files={**baseline['files'], **remote._MCU_MEDIUM_SHA256})
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': {'cmd0_route_deployment_readback_ready': True}}}
        with patch.object(remote, '_boot_id', return_value=boot):
            with self.assertRaisesRegex(RuntimeError, 'registered medium-turn MCU record'):
                remote._commit(self.target, self.ticket, readback)
        self.assertFalse((self.target/remote.MARKER).exists())

    def test_backward4_receipt_registers_exact_bundle_and_requires_owner_readback(self):
        baseline = self._mcu_receipt()
        files = self._private_mcu_manifest(settings.ROBOT_WORKSPACE/'mcu_command1_backward4_candidate')
        readback = dict(baseline, files=files)
        self.assertEqual(remote.validate_mcu_readback(readback,baseline['files']),readback)
        mixed = dict(readback, files={**readback['files'],
                     'lua/craic_runtime.lua': baseline['files']['lua/craic_runtime.lua']})
        with self.assertRaisesRegex(ValueError, 'MCU 回执与受支持的动作包不符'):
            remote.validate_mcu_readback(mixed,baseline['files'])
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        status = {'cmd0_route_deployment_readback_ready': True, 'medium_turn_deployment_readback_ready': True,
                  'cmd0_right5_deployment_readback_ready': True, 'pair_deployment_readback_ready': True,
                  'pickup_backward4_deployment_readback_ready': True}
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': status}}
        with patch.object(remote,'_boot_id',return_value=boot):
            result = remote._commit(self.target,self.ticket,readback)
        self.assertTrue(result['mcu_readback_registered'])

    def test_backward4_registration_rolls_back_if_owner_accepts_only_old_pair_readback(self):
        baseline = self._mcu_receipt()
        readback = dict(baseline, files={**baseline['files'], **remote._MCU_PICKUP_BACKWARD4_SHA256})
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': {
            'cmd0_route_deployment_readback_ready': True, 'medium_turn_deployment_readback_ready': True,
            'cmd0_right5_deployment_readback_ready': True, 'pair_deployment_readback_ready': True}}}
        with patch.object(remote,'_boot_id',return_value=boot):
            with self.assertRaisesRegex(RuntimeError,'registered four-step backward MCU record'):
                remote._commit(self.target,self.ticket,readback)
        self.assertFalse((self.target/remote.MARKER).exists())

    def test_five_stride_registration_requires_specific_owner_readback(self):
        baseline = self._mcu_receipt()
        readback = dict(baseline, files={**baseline['files'], **remote._MCU_RIGHT5_SHA256})
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': {
            'cmd0_route_deployment_readback_ready': True, 'medium_turn_deployment_readback_ready': True}}}
        with patch.object(remote, '_boot_id', return_value=boot):
            with self.assertRaisesRegex(RuntimeError, 'registered five-stride MCU record'):
                remote._commit(self.target, self.ticket, readback)
        self.assertFalse((self.target/remote.MARKER).exists())

    def test_current_empty_return_overlay_registers_the_exact_manifest_and_owner_gate(self):
        baseline = self._mcu_receipt()
        files = self._private_mcu_manifest(settings.DEFAULT_MCU_OVERLAY)
        self.assertEqual(files, {**baseline['files'], **remote._MCU_INTERMEDIATE_STRAFE_SHA256})
        readback = dict(baseline, files=files)
        self.assertEqual(remote.validate_mcu_readback(readback,baseline['files']),readback)
        boot = 'ae8ca915-435e-4481-a85d-b097ed2a2f14'
        self.mocks['_status'].return_value['owner'].update(mcu_ready=True, micro_trial={'boot_id': boot})
        self.mocks['_reload'].return_value = {'owner': {'micro_trial': {
            'cmd0_route_deployment_readback_ready': True, 'medium_turn_deployment_readback_ready': True,
            'cmd0_right5_deployment_readback_ready': True, 'pair_deployment_readback_ready': True,
            'pickup_backward8_deployment_readback_ready': True, 'empty_recovery_deployment_readback_ready': True}}}
        with patch.object(remote,'_boot_id',return_value=boot):
            result = remote._commit(self.target,self.ticket,readback)
        self.assertTrue(result['mcu_readback_registered'])

    def test_previous_eight_step_and_low_release_receipts_require_exact_files(self):
        baseline = self._mcu_receipt()
        expected = baseline['files']
        for variant in (remote._MCU_PICKUP_BACKWARD8_TUNED_LEFT_SHA256,
                        remote._MCU_PICKUP_BACKWARD8_LOW_RELEASE_SHA256, remote._MCU_PICKUP_LEVEL_BACKWARD6_SHA256,
                        remote._MCU_EMPTY_RECOVERY_SHA256, remote._MCU_INTERMEDIATE_STRAFE_SHA256):
            with self.subTest(actions=variant['lua/craic_actions.lua']):
                receipt = dict(baseline, files={**expected, **variant})
                self.assertEqual(remote.validate_mcu_readback(receipt, expected), receipt)
        mixed = dict(baseline, files={**expected, **remote._MCU_PICKUP_BACKWARD8_LOW_RELEASE_SHA256,
                     'lua/craic_single_micro.lua': remote._MCU_PICKUP_BACKWARD8_SHA256['lua/craic_single_micro.lua']})
        with self.assertRaisesRegex(ValueError, 'MCU 回执与受支持的动作包不符'):
            remote.validate_mcu_readback(mixed, expected)

    def test_tuned_receipt_rejects_missing_extra_or_changed_files(self):
        readback = self._mcu_receipt()
        expected = readback['files']
        tuned = {**expected, 'lua/craic_single_micro.lua': remote._MCU_TUNED_MICRO_SHA256}
        invalid = (
            {name: digest for name, digest in tuned.items() if name != 'lua/main.lua'},
            {**tuned, 'lua/unapproved.lua': 'a'*64},
            {**tuned, 'lua/craic_auto_bridge.lua': 'b'*64},
            {**tuned, 'lua/craic_single_micro.lua': 'c'*64},
        )
        for files in invalid:
            with self.subTest(files=files):
                with self.assertRaisesRegex(ValueError, 'MCU 回执与受支持的动作包不符'):
                    remote.validate_mcu_readback(dict(readback, files=files), expected)

    def test_previous_tuned_receipt_still_matches_the_exact_bundle(self):
        readback = self._mcu_receipt()
        for digest in (
                'a8156fefd6da3709882508abc7cc3e3d1a5dc499ee17eefcb3ed846abafffd14',
                'f0e52e82024132ec0c7be7d5aa556e756a7642fd2c489a1172920ea11be78bf1'):
            with self.subTest(digest=digest):
                tuned = {**readback['files'], 'lua/craic_single_micro.lua': digest}
                receipt = dict(readback, files=tuned)
                self.assertEqual(remote.validate_mcu_readback(receipt, readback['files']), receipt)

    def test_readback_requires_default_target_and_current_reported_boot(self):
        readback = self._mcu_receipt()
        with patch.object(remote, 'DEFAULT_PATH', self.target.parent/'another'):
            with self.assertRaisesRegex(RuntimeError, 'default R3 target'):
                remote._commit(self.target, self.ticket, readback)
        with patch.object(remote, '_boot_id', return_value='ae8ca915-435e-4481-a85d-b097ed2a2f14'):
            with self.assertRaisesRegex(RuntimeError, 'matching Pi boot'):
                remote._commit(self.target, self.ticket, readback)
        self.mocks['_exchange'].assert_not_called()

    def test_utf16_windows_usb_receipt_is_accepted_and_volume_proof_is_checked(self):
        receipt = self._mcu_receipt()
        payload = {remote.MCU_MANIFEST_PATH: (self.slot/remote.MCU_MANIFEST_PATH).read_bytes()}
        path = self.target.parent/'readback.json'
        path.write_text(json.dumps(receipt), encoding='utf-16')
        self.assertEqual(deploy._mcu_readback(path, payload), receipt)
        del receipt['volume']['profile_sha256']
        path.write_text(json.dumps(receipt), encoding='utf-16')
        with self.assertRaisesRegex(ValueError, 'profile evidence'):
            deploy._mcu_readback(path, payload)
