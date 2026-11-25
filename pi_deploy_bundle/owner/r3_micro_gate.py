"""Dedicated micro capability and human trial permit. Never persist live permits."""
import hashlib
from pathlib import Path
import re

VERSION = 'r3-visual-pickup-20261109-v5'
CONTRACT = 'r3.single_micro_turn.v1'
_DEVICE_IDENTITY_SHA256 = '7e4b8c1ab846f4d8e883e8376f657d098d8cadc62e691de3c18e9ad41509f0d7'
NAMES = {op: 'r3_micro_v1_'+op for op in
         ('capability', 'single_micro_left', 'single_micro_right')}
NAMES['capability']='r3_mission_v5_capability'
SMALL_STRAFE_PROBE = 'small_strafe_capability'
SMALL_STRAFE_OPS = frozenset(('single_small_strafe_left', 'single_small_strafe_right'))
CMD0_ROUTE_PROBE = 'cmd0_route_capability'
MEDIUM_TURN_PROBE = 'medium_turn_capability'
RIGHT5_ROUTE_PROBE = 'cmd0_right5_capability'
RIGHT5_ROUTE_OP = 'cmd0_strafe_right5'
PAIR_PROBE = 'horizontal_pair_capability'
PICKUP_BACKWARD4_PROBE = 'pickup_backward4_capability'
PICKUP_BACKWARD4_OP = 'cmd1_carry_backward4'
PICKUP_BACKWARD4_PAIR_OP = 'double_cmd1_carry_backward4'
PICKUP_BACKWARD4_OPS = frozenset((PICKUP_BACKWARD4_OP, PICKUP_BACKWARD4_PAIR_OP))
PICKUP_BACKWARD6_OPS = frozenset(('cmd1_carry_backward6', 'double_cmd1_carry_backward6'))
PICKUP_BACKWARD8_OPS = frozenset(('cmd1_carry_backward8', 'double_cmd1_carry_backward8'))
_PAIR_PROBES = (PAIR_PROBE, PICKUP_BACKWARD4_PROBE)
MEDIUM_TURN_OPS = frozenset(('single_medium_left', 'single_medium_right'))
_CMD0_ROUTE_NAMES = {f'cmd0_{op}': f'r3_cmd0_v1_{op}' for op in (
    'carry_forward5', 'carry_forward10', 'carry_forward15', 'carry_backward2', 'place_single',
    'strafe_left2', 'strafe_left3', 'strafe_left4', 'strafe_left6',
    'strafe_right2', 'strafe_right3', 'strafe_right4',
)}
CMD0_ROUTE_OPS = frozenset(_CMD0_ROUTE_NAMES)
PAIR_BODY_OPS = frozenset(('grab_double', 'place_double')) | frozenset(
    'double_'+op for op in (CMD0_ROUTE_OPS - {'cmd0_place_single'}) | SMALL_STRAFE_OPS | MEDIUM_TURN_OPS |
    {RIGHT5_ROUTE_OP, 'single_micro_left', 'single_micro_right', 'left_turn', 'right_turn',
     'strafe_left_once', 'strafe_right_once', 'carry_forward', 'carry_forward2', 'carry_forward4',
     'carry_forward7', 'carry_forward9'}
)
PROBE_OPS = frozenset(('capability', SMALL_STRAFE_PROBE, CMD0_ROUTE_PROBE, MEDIUM_TURN_PROBE, RIGHT5_ROUTE_PROBE, *_PAIR_PROBES))
NAMES[SMALL_STRAFE_PROBE] = 'r3_micro_strafe_v1_capability'
NAMES[CMD0_ROUTE_PROBE] = 'r3_cmd0_v1_capability'
NAMES[MEDIUM_TURN_PROBE] = 'r3_medium_v1_capability'
NAMES[RIGHT5_ROUTE_PROBE] = 'r3_cmd0_v1_strafe_right5_capability'
NAMES[RIGHT5_ROUTE_OP] = 'r3_cmd0_v1_strafe_right5'
NAMES[PAIR_PROBE] = 'r3_pair_v1_capability'
NAMES[PICKUP_BACKWARD4_PROBE] = 'r3_pickup_v1_backward4_capability'
NAMES[PICKUP_BACKWARD4_OP] = 'r3_pickup_v1_backward4'
NAMES[PICKUP_BACKWARD4_PAIR_OP] = 'r3_pickup_v1_double_backward4'
NAMES['cmd1_carry_backward6'] = 'r3_pickup_v1_backward6'
NAMES['double_cmd1_carry_backward6'] = 'r3_pickup_v1_double_backward6'
NAMES['cmd1_carry_backward8'] = 'r3_pickup_v1_backward8'
NAMES['double_cmd1_carry_backward8'] = 'r3_pickup_v1_double_backward8'
NAMES.update({op: 'r3_pair_v1_' + op for op in PAIR_BODY_OPS})
NAMES.update({op: 'r3_micro_strafe_v1_' + op for op in SMALL_STRAFE_OPS})
NAMES.update(_CMD0_ROUTE_NAMES)
NAMES.update({op: 'r3_medium_v1_' + op for op in MEDIUM_TURN_OPS})
MOTION_OPS = (frozenset(('single_micro_left', 'single_micro_right', RIGHT5_ROUTE_OP))
              | SMALL_STRAFE_OPS | CMD0_ROUTE_OPS | MEDIUM_TURN_OPS | PAIR_BODY_OPS
              | PICKUP_BACKWARD4_OPS | PICKUP_BACKWARD6_OPS | PICKUP_BACKWARD8_OPS)
REQUIRED_CODE_SHA256 = {'lua/craic_actions.lua': 'e1f76bf441711e9fa6ab791f24a1bb3844b63e191f2d6bba0bc27936e63abf8f', 'lua/craic_auto_bridge.lua': '573ef6182a6b78ac4deaf0fa04d1fd12153a477d35a3ab723532cdd48d41ee8d', 'lua/craic_button_gaits.lua': 'd116e043026273e0d577dbe4af396cbf8fdab2c3299b66482a6f0edfa07b3a86', 'lua/craic_config.lua': '1e443a23e3c577c4c232dfc302276d99383b96123072d2373d2c0bf9114b9436', 'lua/craic_future_actions.lua': '612267f8e10c7135ca76861785f6b63a60cfa1a55402a6bca122b4848bc91da6', 'lua/craic_runtime.lua': '453560bf02f6f107025231ae4656e1c0372a50c4509d583f506a09a4d48efe70', 'lua/craic_single_micro.lua': 'c7827f10bb33d3b51ce3ac4bae2d4e023301dfcee909d586d8f82ffb44a5d5df', 'lua/main.lua': '075dae0de6a3170bdac4a419161b757222d234b848d9c3e7bf59b856e79ecb58'}
POLICY_SHA256 = {False: '70200084130ba458e8b32f3485e5efe1bc314a08d05e3c08065ce13968dc7051', True: '046bff97199d2c0b1338cd0b7782fde4548f30a030001e6c1a74c17383bda181'}
SMALL_STRAFE_CODE_SHA256 = dict(REQUIRED_CODE_SHA256, **{
    'lua/craic_auto_bridge.lua': '37741945afada05a07f7a3c03c058e4da4eb0b74e03e56766b4a394de4b78147',
})
CMD0_ROUTE_CODE_SHA256 = dict(SMALL_STRAFE_CODE_SHA256, **{
    'lua/craic_auto_bridge.lua': 'fd499a6529f2d6d544f9bb6def0b22ebca049ca95c10f01f2934ab69d59dfbb9',
})
CMD0_TUNED_ROUTE_CODE_SHA256 = dict(CMD0_ROUTE_CODE_SHA256, **{
    'lua/craic_single_micro.lua': '48931c9beea5ceb9122bfaa705457279c8a1591ebb4212d0c665b28b86244004',
})
MEDIUM_TURN_CODE_SHA256 = dict(CMD0_ROUTE_CODE_SHA256, **{
    'lua/craic_auto_bridge.lua': '12e51ad9684904e88b8d36e4ee701cf37c14bb71c5c5adae9cff41f168c3710f',
    'lua/craic_single_micro.lua': 'e8d3c542a0c4e7ba9f737952a09d5e761b4e0a1a41711c954a5c7ec50dee6121',
})
RIGHT5_ROUTE_CODE_SHA256 = dict(MEDIUM_TURN_CODE_SHA256, **{
    'lua/craic_auto_bridge.lua': 'b921f01f9937ff22eb2f5f00184fcbcee2ba30c242f65540aaf45c29beb98426',
})
PAIR_ROUTE_CODE_SHA256 = dict(RIGHT5_ROUTE_CODE_SHA256, **{
    'lua/craic_actions.lua': '7fa0daf3caae45c32379a668c6e0f2f8954607d119de9bd5ecba9a3fe424995d',
    'lua/craic_auto_bridge.lua': '4cfeeb48b3c2a11d5f1bcd8c561cc280a7fb4595e64ccf9bd36ad8e17f0b1cc8',
    'lua/craic_runtime.lua': '96328475f14389ba82c6136aa5b16ce74d3f35cbff0388b0aaa8360e12e7893f',
    'lua/craic_single_micro.lua': 'b73f1a9d704612a5f4f7c99ad0853c1757318364c4da36c543ad2be09bc1cf56',
})
PICKUP_BACKWARD4_CODE_SHA256 = dict(PAIR_ROUTE_CODE_SHA256, **{
    'lua/craic_auto_bridge.lua': '7d55f4a959f91dc3d35924dc2f35ad5a143867f709be2e325ab14359148193a4',
})
PICKUP_BACKWARD6_CODE_SHA256 = dict(PICKUP_BACKWARD4_CODE_SHA256, **{
    'lua/craic_auto_bridge.lua': '0ea67743b8f03e55d40ee5c8e73cc8ca396e0eadbaeb7a99569e5cb0b6212594',
})
PICKUP_BACKWARD8_CODE_SHA256 = dict(PICKUP_BACKWARD6_CODE_SHA256, **{
    'lua/craic_auto_bridge.lua': '14a4687b14c94f8160f056f6c0950ebc0dcca561a15d24685d5523c1e0c8c94d',
})
PICKUP_BACKWARD8_TUNED_LEFT_CODE_SHA256 = dict(PICKUP_BACKWARD8_CODE_SHA256, **{
    'lua/craic_single_micro.lua': '390935bd8ef1c073c0609d8bac7c171a572539d270330d3db8d630b8de0a751b',
})
PICKUP_BACKWARD8_LOW_RELEASE_CODE_SHA256 = dict(PICKUP_BACKWARD8_TUNED_LEFT_CODE_SHA256, **{
    'lua/craic_actions.lua': '94e6eb25149bc6e00181997618c3bea54ba28a13af622fc38fa6c15ffddc3989',
})
_PICKUP_BACKWARD_CODE_VARIANTS = (PICKUP_BACKWARD4_CODE_SHA256, PICKUP_BACKWARD6_CODE_SHA256,
                                  PICKUP_BACKWARD8_CODE_SHA256, PICKUP_BACKWARD8_TUNED_LEFT_CODE_SHA256,
                                  PICKUP_BACKWARD8_LOW_RELEASE_CODE_SHA256)
_PAIR_ROUTE_CODE_VARIANTS = (PAIR_ROUTE_CODE_SHA256, *_PICKUP_BACKWARD_CODE_VARIANTS)
_CMD0_PRIOR_TUNED_MICRO_SHA256 = (
    'a8156fefd6da3709882508abc7cc3e3d1a5dc499ee17eefcb3ed846abafffd14',
    'f0e52e82024132ec0c7be7d5aa556e756a7642fd2c489a1172920ea11be78bf1',
)
_CMD0_ROUTE_CODE_VARIANTS = (
    CMD0_ROUTE_CODE_SHA256, CMD0_TUNED_ROUTE_CODE_SHA256, MEDIUM_TURN_CODE_SHA256, RIGHT5_ROUTE_CODE_SHA256,
    *_PAIR_ROUTE_CODE_VARIANTS,
    *({**CMD0_ROUTE_CODE_SHA256, 'lua/craic_single_micro.lua': digest}
      for digest in _CMD0_PRIOR_TUNED_MICRO_SHA256),
)


def _matches_readback(files, code):
    return bool(code and isinstance(files, dict)
                and set(files) == set(code) | {'dat/craic_grab_safety.lua'}
                and all(files.get(path) == digest for path, digest in code.items()))


def MicroGate_IsRobot3Identity(cpu_serial, machine_id):
    if not isinstance(cpu_serial, str) or not isinstance(machine_id, str):
        return False
    return hashlib.sha256((cpu_serial + '\0' + machine_id).encode('utf-8')).hexdigest() == _DEVICE_IDENTITY_SHA256


def local_boot_id():
    try:
        value = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        return value if re.fullmatch('[0-9a-f-]{36}', value) else None
    except OSError:
        return None


class MicroGate:
    def __init__(self, session, clock, boot_id=None):
        self.session, self.clock = session, clock
        self.boot_id = boot_id if boot_id is not None else local_boot_id()
        self.deployment = None
        self.capability = None
        self.permit = None
        self.used_nonces = set()

    def clear(self):
        self.capability = None
        self.permit = None

    def configure(self, record):
        # Parsing a file never transmits a probe or grants a live capability.
        self.clear()
        self.deployment = record if isinstance(record, dict) else None

    def deployment_ready(self):
        r = self.deployment or {}
        files = r.get('verified_files_sha256', {})
        return bool(self.boot_id and r.get('installed') is True and
                    r.get('reload_confirmed') is True and r.get('robot_id') == 3 and
                    MicroGate_IsRobot3Identity(r.get('cpu_serial'), r.get('machine_id')) and
                    (r.get('verified_boot_id') == self.boot_id or
                     (r.get('installation_evidence_scope')=='persistent_mcu_files_until_next_deployment' and
                      isinstance(r.get('verified_boot_id'),str) and re.fullmatch('[0-9a-f-]{36}',r['verified_boot_id']))) and
                    r.get('contract') == CONTRACT and
                    any(_matches_readback(files, code) for code in (
                        REQUIRED_CODE_SHA256, SMALL_STRAFE_CODE_SHA256, *_CMD0_ROUTE_CODE_VARIANTS,
                    )) and
                    type(r.get('policy_single_enabled')) is bool and
                    files.get('dat/craic_grab_safety.lua')==POLICY_SHA256[r['policy_single_enabled']])

    def small_strafe_deployment_ready(self):
        r = self.deployment or {}
        return (self.deployment_ready() and r.get('mission_abi') == 1 and r.get('wire_abi') == 42
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in (SMALL_STRAFE_CODE_SHA256, *_CMD0_ROUTE_CODE_VARIANTS)))

    def cmd0_route_deployment_ready(self):
        r = self.deployment or {}
        return (self.deployment_ready() and r.get('mission_abi') == 1 and r.get('wire_abi') == 42
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in _CMD0_ROUTE_CODE_VARIANTS))

    def medium_turn_deployment_ready(self):
        r = self.deployment or {}
        return (self.cmd0_route_deployment_ready()
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in (MEDIUM_TURN_CODE_SHA256, RIGHT5_ROUTE_CODE_SHA256, *_PAIR_ROUTE_CODE_VARIANTS)))

    def cmd0_right5_deployment_ready(self):
        r = self.deployment or {}
        return (self.cmd0_route_deployment_ready()
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in (RIGHT5_ROUTE_CODE_SHA256, *_PAIR_ROUTE_CODE_VARIANTS)))

    def pair_deployment_ready(self):
        r = self.deployment or {}
        return (self.cmd0_route_deployment_ready()
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in _PAIR_ROUTE_CODE_VARIANTS))

    def pickup_backward4_deployment_ready(self):
        r = self.deployment or {}
        return (self.cmd0_route_deployment_ready()
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in _PICKUP_BACKWARD_CODE_VARIANTS))

    def pickup_backward6_deployment_ready(self):
        r = self.deployment or {}
        return (self.cmd0_route_deployment_ready()
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in _PICKUP_BACKWARD_CODE_VARIANTS[1:]))

    def pickup_backward8_deployment_ready(self):
        r = self.deployment or {}
        return (self.cmd0_route_deployment_ready()
                and any(_matches_readback(r.get('verified_files_sha256'), code)
                        for code in _PICKUP_BACKWARD_CODE_VARIANTS[2:]))

    def repeated_single_ready(self):
        r=self.deployment or {}
        actions = r.get('verified_files_sha256',{}).get('lua/craic_actions.lua')
        return bool(self.deployment_ready() and r.get('single_grab_policy')=='repeat_after_success_empty_only'
                    and actions in (REQUIRED_CODE_SHA256['lua/craic_actions.lua'],
                                    PAIR_ROUTE_CODE_SHA256['lua/craic_actions.lua'],
                                    PICKUP_BACKWARD8_LOW_RELEASE_CODE_SHA256['lua/craic_actions.lua']))

    def mission_ready(self, epoch, recovery_ready):
        r=self.deployment or {}
        if not (self.repeated_single_ready() and r.get('mission_abi')==1 and r.get('wire_abi')==42
                and r.get('policy_single_enabled') is True and self.capability_ready(epoch)
                and self.permit_ready(epoch) and recovery_ready):
            return False
        if r.get('verified_boot_id')==self.boot_id:
            return True
        # Persistent readback proves which package was installed. A reboot
        # still needs this build's route probe and current-session permits.
        probe = self.capability.get('probe')
        if probe == PAIR_PROBE and self.cmd0_route_deployment_ready():
            return True  # The motionless pair probe also verifies the baseline route exports.
        if self.pickup_backward4_deployment_ready():
            return probe == PICKUP_BACKWARD4_PROBE
        if self.pair_deployment_ready():
            return probe == PAIR_PROBE
        if self.cmd0_right5_deployment_ready():
            return probe == RIGHT5_ROUTE_PROBE
        if self.medium_turn_deployment_ready():
            return probe == MEDIUM_TURN_PROBE
        return self.cmd0_route_deployment_ready() and probe == CMD0_ROUTE_PROBE

    def confirm(self, payload, epoch, recovery_ready):
        required = {'session', 'epoch', 'operator', 'guarded_single_trial', 'stop_limits_understood'}
        if not isinstance(payload, dict) or set(payload) != required:
            raise ValueError('exact guarded micro trial acknowledgment required')
        if not recovery_ready or payload['session'] != self.session or type(payload['epoch']) is not int or payload['epoch'] != epoch:
            raise ValueError('current recovery inspection and matching owner session/epoch required')
        if payload['guarded_single_trial'] is not True or payload['stop_limits_understood'] is not True:
            raise ValueError('guarded single-load trial and non-immediate-stop limits must be acknowledged')
        who = payload['operator']
        if not isinstance(who, str) or not 1 <= len(who.strip()) <= 80:
            raise ValueError('named human operator required')
        self.permit = dict(session=self.session, epoch=epoch, boot_id=self.boot_id,
                           operator=who.strip(), confirmed_monotonic=self.clock())
        return self.status(epoch, recovery_ready)

    def permit_ready(self, epoch):
        p = self.permit
        return bool(p and p['session'] == self.session and p['epoch'] == epoch and
                    p['boot_id'] == self.boot_id and self.boot_id and
                    0 <= self.clock()-p['confirmed_monotonic'] <= 900)

    def capability_ready(self, epoch):
        c = self.capability
        return bool(self.deployment_ready() and c and c['session'] == self.session and
                    c['epoch'] == epoch and c['boot_id'] == self.boot_id and
                    0 <= self.clock()-c['verified_monotonic'] <= 900)

    def small_strafe_capability_ready(self, epoch):
        return (self.small_strafe_deployment_ready() and self.capability_ready(epoch)
                and self.capability.get('probe') in (SMALL_STRAFE_PROBE, CMD0_ROUTE_PROBE, MEDIUM_TURN_PROBE, RIGHT5_ROUTE_PROBE, *_PAIR_PROBES))

    def cmd0_route_capability_ready(self, epoch):
        return (self.cmd0_route_deployment_ready() and self.capability_ready(epoch)
                and self.capability.get('probe') in (CMD0_ROUTE_PROBE, MEDIUM_TURN_PROBE, RIGHT5_ROUTE_PROBE, *_PAIR_PROBES))

    def medium_turn_capability_ready(self, epoch):
        return (self.medium_turn_deployment_ready() and self.capability_ready(epoch)
                and self.capability.get('probe') in (MEDIUM_TURN_PROBE, RIGHT5_ROUTE_PROBE, *_PAIR_PROBES))

    def cmd0_right5_capability_ready(self, epoch):
        return (self.cmd0_right5_deployment_ready() and self.capability_ready(epoch)
                and self.capability.get('probe') in (RIGHT5_ROUTE_PROBE, *_PAIR_PROBES))

    def pair_ready(self, epoch, recovery_ready):
        return self.mission_ready(epoch, recovery_ready) and self.pair_capability_ready(epoch)

    def pair_capability_ready(self, epoch):
        return (self.cmd0_route_deployment_ready() and self.capability_ready(epoch)
                and self.capability.get('probe') in _PAIR_PROBES)

    def pickup_backward4_ready(self, epoch, recovery_ready):
        return (self.pickup_backward4_deployment_ready() and self.mission_ready(epoch, recovery_ready)
                and self.capability.get('probe') == PICKUP_BACKWARD4_PROBE)

    def pickup_backward6_ready(self, epoch, recovery_ready):
        return (self.pickup_backward6_deployment_ready()
                and self.pickup_backward4_ready(epoch,recovery_ready))

    def pickup_backward8_ready(self, epoch, recovery_ready):
        return (self.pickup_backward8_deployment_ready()
                and self.pickup_backward4_ready(epoch,recovery_ready))

    def small_strafe_ready(self, epoch, recovery_ready):
        r = self.deployment or {}
        return (r.get('policy_single_enabled') is True and recovery_ready
                and self.permit_ready(epoch) and self.small_strafe_capability_ready(epoch))

    def cmd0_route_ready(self, epoch, recovery_ready):
        r = self.deployment or {}
        return (r.get('policy_single_enabled') is True and recovery_ready
                and self.permit_ready(epoch) and self.cmd0_route_capability_ready(epoch))

    def medium_turn_ready(self, epoch, recovery_ready):
        r = self.deployment or {}
        return (r.get('policy_single_enabled') is True and recovery_ready
                and self.permit_ready(epoch) and self.medium_turn_capability_ready(epoch))

    def cmd0_right5_ready(self, epoch, recovery_ready):
        return self.cmd0_route_ready(epoch, recovery_ready) and self.cmd0_right5_capability_ready(epoch)

    def admit(self, op, nonce, epoch, recovery_ready):
        if op not in NAMES:
            return None
        if not nonce or nonce in self.used_nonces:
            return 'micro_nonce_missing_or_replayed'
        if not self.deployment_ready():
            return 'micro_MCU_verified_installation_and_initial_reload_required'
        if op == SMALL_STRAFE_PROBE and not self.small_strafe_deployment_ready():
            return 'small_strafe_MCU_verified_installation_required'
        if op == CMD0_ROUTE_PROBE and not self.cmd0_route_deployment_ready():
            return 'cmd0_route_MCU_verified_installation_required'
        if op == MEDIUM_TURN_PROBE and not self.medium_turn_deployment_ready():
            return 'medium_turn_MCU_verified_installation_required'
        if op == RIGHT5_ROUTE_PROBE and not self.cmd0_right5_deployment_ready():
            return 'cmd0_right5_MCU_verified_installation_required'
        if op == PAIR_PROBE and not self.cmd0_route_deployment_ready():
            return 'horizontal_pair_probe_requires_verified_baseline_route'
        if op == PICKUP_BACKWARD4_PROBE and not self.pickup_backward4_deployment_ready():
            return 'pickup_backward4_MCU_verified_installation_required'
        if op in SMALL_STRAFE_OPS and not self.small_strafe_ready(epoch, recovery_ready):
            return 'small_strafe_live_capability_recovery_and_guarded_trial_required'
        if op in CMD0_ROUTE_OPS and not self.cmd0_route_ready(epoch, recovery_ready):
            return 'cmd0_route_live_capability_recovery_and_guarded_trial_required'
        if op == RIGHT5_ROUTE_OP and not self.cmd0_right5_ready(epoch, recovery_ready):
            return 'cmd0_right5_live_capability_recovery_and_guarded_trial_required'
        if op in MEDIUM_TURN_OPS and not self.medium_turn_ready(epoch, recovery_ready):
            return 'medium_turn_live_capability_recovery_and_guarded_trial_required'
        if op in PAIR_BODY_OPS and not self.pair_ready(epoch, recovery_ready):
            return 'horizontal_pair_current_native_capability_required'
        if op in PICKUP_BACKWARD4_OPS and not self.pickup_backward4_ready(epoch, recovery_ready):
            return 'pickup_backward4_current_native_capability_required'
        if op in PICKUP_BACKWARD6_OPS and not (self.pickup_backward6_ready(epoch, recovery_ready)
                or self.pair_ready(epoch, recovery_ready)):
            return 'pickup_backward6_current_native_capability_required'
        if op in PICKUP_BACKWARD8_OPS and not self.pickup_backward8_ready(epoch, recovery_ready):
            return 'pickup_backward8_current_native_capability_required'
        if op in MOTION_OPS and not (self.deployment.get('policy_single_enabled') is True and recovery_ready and self.permit_ready(epoch) and self.capability_ready(epoch)):
            return 'micro_live_capability_recovery_and_guarded_trial_required'
        self.used_nonces.add(nonce)
        return None

    def complete_probe(self, c, epoch):
        op = c['op']
        if (op not in PROBE_OPS or not self.deployment_ready() or c['epoch'] != epoch or
                (op == SMALL_STRAFE_PROBE and not self.small_strafe_deployment_ready()) or
                (op == CMD0_ROUTE_PROBE and not self.cmd0_route_deployment_ready()) or
                (op == MEDIUM_TURN_PROBE and not self.medium_turn_deployment_ready()) or
                (op == RIGHT5_ROUTE_PROBE and not self.cmd0_right5_deployment_ready()) or
                (op == PAIR_PROBE and not self.cmd0_route_deployment_ready()) or
                (op == PICKUP_BACKWARD4_PROBE and not self.pickup_backward4_deployment_ready()) or
                c.get('request_nonce') not in self.used_nonces or c.get('operation_seq', 0) <= 0 or
                not all(c.get(k) is True for k in ('dispatched', 'started', 'complete', 'drained')) or
                c.get('cancelled') is not False):
            self.capability = None
            return
        self.capability = dict(session=self.session, epoch=epoch, boot_id=self.boot_id,
                    request_nonce=c['request_nonce'], operation_seq=c['operation_seq'], probe=op,
            started=True, complete=True, verified_monotonic=self.clock(),
            evidence='current exclusive dispatch START and COMPLETE; not physical health')

    def status(self, epoch, recovery_ready):
        return dict(contract=CONTRACT, boot_id=self.boot_id,
                    deployment_readback_ready=self.deployment_ready(),
                    small_strafe_deployment_readback_ready=self.small_strafe_deployment_ready(),
                    small_strafe_capability_verified=self.small_strafe_capability_ready(epoch),
                    small_strafe_ready=self.small_strafe_ready(epoch,recovery_ready),
                    cmd0_route_deployment_readback_ready=self.cmd0_route_deployment_ready(),
                    cmd0_route_capability_verified=self.cmd0_route_capability_ready(epoch),
                    cmd0_route_ready=self.cmd0_route_ready(epoch,recovery_ready),
                    medium_turn_deployment_readback_ready=self.medium_turn_deployment_ready(),
                    medium_turn_capability_verified=self.medium_turn_capability_ready(epoch),
                    medium_turn_ready=self.medium_turn_ready(epoch,recovery_ready),
                    cmd0_right5_deployment_readback_ready=self.cmd0_right5_deployment_ready(),
                    cmd0_right5_capability_verified=self.cmd0_right5_capability_ready(epoch),
                    cmd0_right5_ready=self.cmd0_right5_ready(epoch,recovery_ready),
                    pair_deployment_readback_ready=self.pair_deployment_ready(),
                    pair_live_capability_verified=self.pair_capability_ready(epoch),
                    pickup_backward4_deployment_readback_ready=self.pickup_backward4_deployment_ready(),
                    pickup_backward4_ready=self.pickup_backward4_ready(epoch,recovery_ready),
                    pickup_backward6_deployment_readback_ready=self.pickup_backward6_deployment_ready(),
                    pickup_backward6_ready=self.pickup_backward6_ready(epoch,recovery_ready),
                    pickup_backward8_deployment_readback_ready=self.pickup_backward8_deployment_ready(),
                    pickup_backward8_ready=self.pickup_backward8_ready(epoch,recovery_ready),
                    deployment_evidence_scope=(self.deployment or {}).get('installation_evidence_scope','same_boot_legacy_record'),
                    initial_reload_boot_id=(self.deployment or {}).get('verified_boot_id'),
                    current_boot_file_readback_claimed=False,
                    capability_verified=self.capability_ready(epoch),
                    capability=dict(self.capability) if self.capability_ready(epoch) else None,
                    guarded_trial_confirmed=self.permit_ready(epoch),
                    ready=bool(self.capability_ready(epoch) and self.permit_ready(epoch) and recovery_ready),
                    native_immediate_stop=False, physical_health_verified=False,
                    angle_evidence='source_nominal', nominal_degrees=5, measured_degrees=None)
