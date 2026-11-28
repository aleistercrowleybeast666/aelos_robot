"""Command 0's estimated Tag 1 to Tag 2 trial, inside its existing session."""

from math import isfinite, radians, sin

from action_chains_v5 import COMMAND0_OUTBOUND
from cmd0_motion import (COMMAND0_ENTRY_FORWARD_STEPS, Cmd0Motion_ContinuousReady,
                         Cmd0Motion_ForwardActions, Cmd0Motion_ForwardStepCount, Cmd0Motion_LegacyReady,
                         Cmd0Motion_SmallStrafeProfile, SMALL_SIDE_ACTIONS)
from checkpoint_errors import VisualCamera_GetFreshnessCheck, VisualCameraUnavailable
from checkpoint_scan import (Checkpoint_FromBoundary, Checkpoint_Observe, Checkpoint_OptionalScanTime,
                             Checkpoint_ReadWindow, CheckpointScanResult)
from nav_observation import summarize, wrap
from checkpoint_alignment import RouteTagHeadingRebaseResult
from route_pose import (RoutePose_Load, RoutePose_Measure, RoutePose_PartialCheckpointGeometry,
                        RoutePose_PlanForwardCount, RoutePose_PlanTag1ForwardTrial, RoutePose_PlanTag2ForwardTrial,
                        TAG2_APPROACH_COUNTS, TAG2_FAR_APPROACH_LIMIT_M)


# Older tools import the checkpoint API from the entrance module.
EntranceTrial_BoundaryCheckpoint = Checkpoint_FromBoundary
EntranceTrial_ObserveCheckpoint = Checkpoint_Observe
EntranceTrialResult = CheckpointScanResult

_TAG1_FORWARD_TRIAL_OFFSETS = (-2, 2, 4, 5)
_TAG1_STANCE_REFERENCE_MAX_NS = 5_000_000_000
TAG1_CLEARANCE_YAW_DEG = 20.
_TAG1_CLEARANCE_SIDE_M = .10
_TAG1_CLEARANCE_STRIDE_UPPER_M = .10
_TAG1_CLEARANCE_DEPTH_M = (.11, .45)
_TAG1_CLEARANCE_EDGE_MARGIN_M = .01


def EntranceTrial_IsEntryAccepted(alignment):
    if not alignment:
        return False
    observation = alignment.get('scan') or {}
    heading = alignment.get('decision_fusion') or {}
    identified = (alignment.get('expected_id') == 1
            and alignment.get('fresh_target_detected') is True
            and (alignment.get('camera_info') or {}).get('ready') is True
            and observation.get('ids') == [1])
    if alignment.get('tag1_nominal_entry') is True:
        return identified
    return (identified and alignment.get('pose_accepted') is True
            and (alignment.get('heading_aligned') is True
                 or alignment.get('heading_residual_advisory') is True)
            and heading.get('source') == 'trusted_tag_heading'
            and heading.get('target_heading_deg') == 0
            and observation.get('pose_usable') is True)


def EntranceTrial_RecoverGroundHeading(robot, nav, alignment):
    """Recover yaw at the loaded Tag 1 stance when the taught tilt alone was rejected."""
    from corridor_trial import CorridorTrial_HeadingSnapshot
    from nav_observation import Observation_DistinctTagRows
    from relative_heading import reference_valid, relative_angles
    from route_pose import RoutePose_TrialLateralHint

    if not isinstance(alignment,dict):
        return alignment
    scan = alignment.get('scan') or {}
    if (not isinstance(scan,dict) or robot.load_kind not in ('single','double') or alignment.get('expected_id') != 1
            or scan.get('status') != 'id_seen_pose_untrusted'
            or scan.get('reason') != 'insufficient tilt-compatible relative poses'
            or scan.get('observed_ids') != [1]):
        return alignment
    visual=nav.cfg['visual_micro']
    if (visual.get('execute') is not True or visual.get('confirmed_target_id')!=1
            or not robot._allows_missing_magnetic()
            or nav.cfg['legs']['outbound'][0].get('tag_heading_deg')!=0.):
        return alignment
    evidence = alignment.get('tag_observation') or scan
    stance = robot.last_action_completed_ros_ns
    after = scan.get('after_ns')
    if (not isinstance(evidence,dict) or not isinstance(scan.get('observations'),(list,tuple))
            or type(stance) is not int or type(after) is not int or not 0 < stance <= after
            or evidence.get('camera_ready_for_motion') is False):
        return alignment
    camera_window=evidence.get('camera_window_status') or {}
    if not isinstance(camera_window,dict):
        return alignment
    restart_stamps=(scan.get('camera_restart_ns',0),camera_window.get('restart_barrier_ns',0))
    if any(type(t) is not int or t<0 for t in restart_stamps):
        return alignment
    window = dict(scan, camera_healthy=evidence.get('camera_healthy'),contract_match=evidence.get('contract_match'),
                  camera_restart_ns=max(restart_stamps))
    contract = dict(nav.cfg['pose_contract'])
    contract['max_ground_tilt_deg'] = min(15.,contract['max_ground_tilt_deg'])
    hint = RoutePose_TrialLateralHint(dict(alignment,scan=window),dict(nav.cfg,pose_contract=contract),1)
    live = VisualCamera_GetFreshnessCheck(nav.io)
    if (not hint or not .11<=hint['distance_m']<=.35 or max(map(abs,hint['lateral_bounds_m']))>.12
            or live is None or not live(hint['latest_stamp_ns'])):
        return alignment
    yaw = hint['heading_deg']
    stamp = hint['latest_stamp_ns']
    reference = nav.cfg.get('relative_heading_reference') or {}
    verification = None
    if abs(yaw)>=TAG1_CLEARANCE_YAW_DEG:
        return alignment
    valid = Observation_DistinctTagRows(row for row in scan['observations'] if isinstance(row,dict)
        and row.get('id')==1 and row.get('stamp_ns')==stamp and row.get('frame_id')==contract['frame']
        and not row.get('headers_disagree') and row.get('source_publisher')==contract['publisher']
        and row.get('source_topic')==nav.cfg['topic'])
    try:
        if stamp in valid and reference_valid(reference,1):
            twist, tilt = relative_angles([valid[stamp]['quaternion_raw'][k] for k in 'xyzw'],reference['quaternion_xyzw'])
            if tilt<=30. and abs(wrap(twist-yaw))<=5.:
                verification='taught_tag_twist'
    except (KeyError,TypeError,ValueError):
        return alignment
    if verification is None and not robot.magnetic_route_only:
        anchor = nav.mag.anchor or {}
        reading = CorridorTrial_HeadingSnapshot(robot.last_heading_report or {},after=True,load=robot.load_kind)
        completed = robot.last_action_completed_monotonic
        stream = getattr(nav.io,'capture_stream',None)
        if (reading and type(completed) in (float,int) and completed<reading[1]<=robot.clock()
                and robot.clock()-reading[1]<=_TAG1_STANCE_REFERENCE_MAX_NS/1e9
                and reading[2]==tuple(anchor.get(k) for k in ('capture_session','load_epoch','calibration_id'))
                and (not hasattr(nav.io,'capture_stream') or (getattr(nav.io,'motion_capture_verified',False)
                     and reading[2] == (getattr(stream,'session',None),getattr(nav.io,'magnetic_load_epoch',None),
                                        getattr(stream,'calibration_digest',None))))
                and ((anchor.get('tag_departure') or {}).get('tag_id')==1 or anchor.get('pickup_approach_heading'))
                and abs(wrap(reading[0]-yaw))<=5.):
            verification='same_stance_magnetic_heading'
    if verification is None:
        return alignment
    aligned=abs(yaw)<=visual['deadband_deg']
    window.update(status='usable',pose_usable=True,heading_only=True,position_usable=False,
                  heading_deg=yaw,latest_stamp_ns=stamp,relative_rejection=scan['reason'],
                  reason='corroborated near-field ground heading',heading_check=verification)
    return dict(alignment, scan=window,pose_accepted=True,fresh_target_detected=True,
                heading_aligned=aligned,heading_residual_advisory=not aligned,
                error_deg=yaw,decision_fusion={'source':'trusted_tag_heading','target_heading_deg':0.,
                    'error_deg':yaw,'predicted_only':False},
                reason_zh='Tag 1 持物俯仰与示教不同，已交叉确认近距地面码的水平航向。')


def _report_progress(robot, phase, result):
    report = getattr(robot.backend, 'report_progress', None)
    if report is not None:
        try:
            command = getattr(robot, 'route_command', '0')
            progress={'command': command, 'phase': phase, 'direction': 'outbound',
                    'outbound_only': getattr(robot,'outbound_only',command=='1'), 'shared_outbound': command in ('1','2'),
                    'journey': getattr(robot,'deliveries',0)+1,
                    'expected_tag': result['expected_id'], 'trial_target_tag': 5,
                    'target_steps': result['target_steps'], 'planned_steps': result['planned_steps'],
                    'completed_steps': result['completed_steps'],
                    'completed_actions': list(result['completed_actions']),
                    'delivery_planned': Cmd0Motion_ContinuousReady(robot.backend), 'route_evidence': 'estimated_trial'}
            if result.get('stationary_wait_reason_zh'):
                progress['reason_zh']=result['stationary_wait_reason_zh']
            if result.get('checkpoint_not_seen'):
                progress['checkpoint_not_seen']=result['checkpoint_not_seen']
            report(progress)
        except Exception:
            pass  # Reporting must not affect execution.


def EntranceTrial_PlanForward(target_steps, *, continuous, legacy):
    operations = Cmd0Motion_ForwardActions(target_steps, continuous=continuous, legacy=legacy)
    if operations:
        return operations, target_steps
    # The base MCU only exports 8/4/2; round an unsupported odd target down.
    from task_runner import segments
    planned_steps = target_steps - target_steps % 2
    return tuple(segments(planned_steps, 'single')), planned_steps


def _tag1_unresolved_entry_heading(original, current, recovery, deadband, completed_ns):
    initial = (original or {}).get('decision_fusion') or {}
    initial_error = initial.get('error_deg') if EntranceTrial_IsEntryAccepted(original) else None
    current_error = ((current.get('decision_fusion') or {}).get('error_deg')
                     if EntranceTrial_IsEntryAccepted(current) else None)
    large = any(type(yaw) in (int, float) and isfinite(yaw) and abs(yaw) >= TAG1_CLEARANCE_YAW_DEG
                for yaw in (initial_error, recovery.get('initial_heading_deg'), current_error))
    if not large:
        return False
    if not EntranceTrial_IsEntryAccepted(current):
        return True
    stamp = current['scan'].get('latest_stamp_ns')
    return not (type(stamp) is int and type(completed_ns) is int and stamp > completed_ns
                and type(current_error) in (int, float) and isfinite(current_error) and abs(current_error) <= deadband)


def _tag1_short_start_actions(operations, continuous, available):
    """Start with two steps. Retain the exact distance and existing MCU capability."""
    steps = Cmd0Motion_ForwardStepCount(operations)
    odd_tail = ()
    if steps % 2:
        odd_tail = ('cmd0_carry_forward5',) if continuous else ('carry_forward7',)
    even = steps - Cmd0Motion_ForwardStepCount(odd_tail)
    if even < 2:
        return operations
    fours, twos = divmod(even-2, 4)
    tail = ('carry_forward4',)*fours + ('carry_forward2',)*(twos//2) + odd_tail
    if 1+len(tail) > available:
        # Consolidate only the remaining path if the journey has few slots.
        eights, rest = divmod(even-2, 8)
        tail = ('carry_forward',)*eights + ('carry_forward4',)*(rest//4) + ('carry_forward2',)*((rest%4)//2) + odd_tail
    return ('carry_forward2',) + tail


def _tag1_recovery_geometry(robot, nav, scan):
    """Return current Tag 1 geometry when the signed position is consistent."""
    stance = robot.last_action_completed_ros_ns
    after, observed = scan.get('after_ns'), scan.get('now_ns')
    if (not all(type(value) is int for value in (stance, after, observed))
            or not 0 < stance <= after < observed
            or scan.get('camera_healthy') is not True or scan.get('contract_match') is not True):
        return None
    rows = [row for row in scan.get('observations', ())
            if type(row.get('stamp_ns')) is int and after < row['stamp_ns'] <= observed
            and observed - row['stamp_ns'] <= nav.cfg['max_age_s'] * 1e9]
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, 1)
    if (geometry is None or geometry['samples'] < nav.cfg['min_frames']
            or geometry['latest_stamp_ns'] != scan.get('latest_stamp_ns')
            or geometry['max_heading_deviation_deg'] > 3.
            or geometry['projection_bounds'][0] < .75):
        return None
    left, right = geometry['lateral_bounds_m']
    near, far = geometry['distance_bounds_m']
    if right-left > .025 or far-near > .025 or max(abs(left), abs(right)) > .24:
        return None
    return geometry


def _tag1_clearance_strafe(geometry, *, lateral_limit=_TAG1_CLEARANCE_SIDE_M,
                           stride_upper=_TAG1_CLEARANCE_STRIDE_UPPER_M):
    left, right = geometry['lateral_bounds_m']
    near, far = geometry['distance_bounds_m']
    direction = -1 if left > lateral_limit else 1 if right < -lateral_limit else 0
    if direction == 0:
        return None
    yaw = geometry['heading_deg']
    spread = geometry['max_heading_deviation_deg']
    shifts = tuple(direction * stride_upper * sin(radians(angle))
                   for angle in (yaw-spread, yaw+spread))
    depth_min, depth_max = _TAG1_CLEARANCE_DEPTH_M
    inside = near + min(0., *shifts) >= depth_min and far + max(0., *shifts) <= depth_max
    # Immediately outside a boundary, a lateral clearance may also recover depth.
    # All measured yaws must move inward, and a full stride must be able to reach the band.
    from_near = (depth_min-_TAG1_CLEARANCE_EDGE_MARGIN_M <= near < depth_min
                 and near + min(shifts) >= depth_min and far + max(shifts) <= depth_max)
    from_far = (depth_max < far <= depth_max+_TAG1_CLEARANCE_EDGE_MARGIN_M
                and far + max(shifts) <= depth_max and near + min(shifts) >= depth_min)
    if not (inside or from_near or from_far):
        return None
    return 'strafe_left_once' if direction < 0 else 'strafe_right_once'


def _tag1_observe_after_recovery(robot, nav, goal, recovery):
    """Read the new stance before another action can use its pose."""
    from visual_micro import VisualMicro_ObserveWithoutMagnetic, VisualMicroStopped

    after = Checkpoint_OptionalScanTime(nav.io, robot.check)
    live = VisualCamera_GetFreshnessCheck(nav.io)
    if after is None or live is None:
        recovery['reobserve_reason'] = 'Tag 1 的相机时钟或新帧校验暂不可用。'
        return None
    try:
        alignment = VisualMicro_ObserveWithoutMagnetic(nav.cfg, nav.io, 1, goal, after_ns=after,
            deadline=min(robot.deadline, robot.clock()+nav.cfg['settle_s']+2*nav.cfg['observe_s']),
            check=robot.check, clock=robot.clock, accept_trusted_residual=True)
    except (VisualMicroStopped, OSError) as exc:
        robot.check()
        recovery['reobserve_reason'] = str(exc)
        return None
    alignment = EntranceTrial_RecoverGroundHeading(robot,nav,alignment)
    scan = alignment['scan']
    stamp = scan.get('latest_stamp_ns')
    completed = robot.last_action_completed_ros_ns
    if (alignment.get('pose_accepted') is not True or scan.get('ids') != [1] or type(stamp) is not int
            or type(completed) is not int or stamp <= max(after, completed) or not live(stamp)):
        recovery['reobserve_reason'] = alignment['reason']
        return None
    alignment['heading_residual_advisory'] = not alignment.get('heading_aligned')
    recovery['reobservations'].append({'latest_stamp_ns': stamp, 'heading_deg': alignment['error_deg']})
    return alignment


def _tag1_center_departure(robot, nav, goal, alignment, result, reserved_actions):
    side = getattr(robot, 'corridor_side_action', None)
    fingerprint = (alignment.get('camera_info') or {}).get('fingerprint')
    if (side is None or goal.get('allow_strafe') is not True or not fingerprint
            or 'tag1_lateral_recovery' in result):
        return alignment
    geometry = _tag1_recovery_geometry(robot, nav, alignment['scan'])
    if geometry is None or geometry['max_abs_heading_deg'] >= TAG1_CLEARANCE_YAW_DEG:
        return alignment
    operation = _tag1_clearance_strafe(geometry)
    if operation is None and getattr(robot.backend, 'small_strafe_ready', lambda: False)():
        _, stride = Cmd0Motion_SmallStrafeProfile(robot.backend)
        small = _tag1_clearance_strafe(geometry, lateral_limit=.08, stride_upper=stride)
        if small:
            operation = SMALL_SIDE_ACTIONS[small]
    if not operation or robot.count + reserved_actions + 1 > robot.cfg['max_body_macros_per_journey']:
        return alignment
    stamp = alignment['scan'].get('latest_stamp_ns')
    live = VisualCamera_GetFreshnessCheck(nav.io)
    if live is None or not live(stamp):
        return alignment
    recovery = {'initial_heading_deg': alignment['decision_fusion']['error_deg'],
                'before_lateral_bounds_m': geometry['lateral_bounds_m'],
                'operation': operation, 'reobservations': []}
    result['tag1_lateral_recovery'] = recovery
    side(operation)
    current = _tag1_observe_after_recovery(robot, nav, goal, recovery)
    if current is None or (current.get('camera_info') or {}).get('fingerprint') != fingerprint:
        recovery['stance_unconfirmed'] = True
        return None
    updated = _tag1_recovery_geometry(robot, nav, current['scan'])
    if updated is not None:
        recovery['after_lateral_bounds_m'] = updated['lateral_bounds_m']
    return current


def _tag1_recover_departure(robot, nav, goal, alignment, result, *, reserved_actions):
    """Correct departure position and yaw using the current loaded Tag 1 stance."""
    turn = getattr(robot, 'Route_TurnFromTagHeading', None)
    visual = nav.cfg.get('visual_micro', {})
    if (turn is None or not alignment or not EntranceTrial_IsEntryAccepted(alignment)
            or alignment.get('tag1_nominal_entry') is True
            or visual.get('execute') is not True):
        return alignment
    error = alignment['decision_fusion'].get('error_deg')
    stamp = alignment['scan'].get('latest_stamp_ns')
    live = VisualCamera_GetFreshnessCheck(nav.io)
    deadband = visual['deadband_deg']
    completed = robot.last_action_completed_ros_ns
    if (type(error) not in (int, float) or not isfinite(error)
            or type(stamp) is not int or type(completed) is not int or stamp <= completed
            or live is None):
        return alignment
    if abs(error) <= deadband:
        current = _tag1_center_departure(robot, nav, goal, alignment, result, reserved_actions)
        if current is None or current is alignment:
            return current
        alignment = current
        error = alignment['error_deg']
        stamp = alignment['scan']['latest_stamp_ns']
        if abs(error) <= deadband:
            return alignment
    if not live(stamp):
        return alignment
    already_turned = alignment.get('correction_executed') is True
    advisory = alignment.get('heading_residual_advisory') is True
    if abs(error) < TAG1_CLEARANCE_YAW_DEG and (already_turned or not advisory):
        return _tag1_center_departure(robot, nav, goal, alignment, result, reserved_actions)
    if not advisory:
        return alignment
    recovery = {'initial_heading_deg': error, 'reobservations': []}
    result['tag1_heading_recovery'] = recovery
    needs_clearance_check = abs(error) >= TAG1_CLEARANCE_YAW_DEG
    centered = False
    if needs_clearance_check:
        geometry = _tag1_recovery_geometry(robot, nav, alignment['scan'])
        if geometry is not None:
            left, right = geometry['lateral_bounds_m']
            near, far = geometry['distance_bounds_m']
            depth_min, depth_max = _TAG1_CLEARANCE_DEPTH_M
            centered = (-_TAG1_CLEARANCE_SIDE_M <= left <= right <= _TAG1_CLEARANCE_SIDE_M
                        and depth_min <= near <= far <= depth_max)
            if centered:
                recovery['centered_turn_geometry'] = geometry
                recovery['reason_zh'] = 'Tag 1 横向与前后位置均在原地转向范围内，先按视觉航向转正。'
    if needs_clearance_check and not centered:
        side = getattr(robot, 'corridor_side_action', None)
        operation = (_tag1_clearance_strafe(geometry) if geometry is not None and side is not None
                     and goal.get('allow_strafe') is True else None)
        if (operation is None or robot.count+reserved_actions+2 > robot.cfg['max_body_macros_per_journey']
                or not live(stamp)):
            recovery['reason_zh'] = 'Tag 1 偏航较大，当前未确认可执行的让位侧跨，保留视觉余差。'
            return alignment
        label = '左跨' if operation == 'strafe_left_once' else '右跨'
        _report_progress(robot, 'cmd0_entry_reacquire', dict(result, expected_id=1,
            stationary_wait_reason_zh=f'Tag 1 横偏和航向偏差较大，先{label}一步，再用新画面修正航向。'))
        side(operation)
        recovery.update(clearance_action=operation, clearance_geometry=geometry)
        before_camera = (alignment.get('camera_info') or {}).get('fingerprint')
        alignment = _tag1_observe_after_recovery(robot, nav, goal, recovery)
        if alignment is None:
            return None
        error = alignment['error_deg']
        stamp = alignment['scan']['latest_stamp_ns']
        near, far = geometry['distance_bounds_m']
        depth_min, depth_max = _TAG1_CLEARANCE_DEPTH_M
        if abs(error) >= TAG1_CLEARANCE_YAW_DEG or not (depth_min <= near <= far <= depth_max):
            updated = _tag1_recovery_geometry(robot, nav, alignment['scan'])
            if (updated is None or updated['latest_stamp_ns'] <= geometry['latest_stamp_ns']
                    or not before_camera or (alignment.get('camera_info') or {}).get('fingerprint') != before_camera):
                recovery['reason_zh'] = 'Tag 1 让位后未取得同一相机的新位姿，保留视觉余差。'
                return alignment
            near, far = updated['distance_bounds_m']
            if not depth_min <= near <= far <= depth_max:
                recovery['reason_zh'] = 'Tag 1 让位后仍在转向距离范围外，保留视觉余差。'
                return alignment
            sign = 1 if operation == 'strafe_left_once' else -1
            low, high = sorted(sign*x for x in geometry['lateral_bounds_m'])
            low_after, high_after = sorted(sign*x for x in updated['lateral_bounds_m'])
            gain = high - max(abs(low_after), abs(high_after))
            spread = max(high-low, high_after-low_after)
            recovery.update(lateral_gain_m=gain, lateral_window_spread_m=spread)
            if (abs(error) >= TAG1_CLEARANCE_YAW_DEG and gain < .01
                    and not (0 <= low_after <= high_after < low and gain >= max(.002, spread))):
                recovery['reason_zh'] = 'Tag 1 让位后仍偏航较大，新画面未能区分横偏改善和帧间波动，保留视觉余差。'
                return alignment
    if abs(error) > deadband and (not already_turned or 'clearance_action' in recovery or centered):
        correction = turn(1, error, stamp, reserved_actions=reserved_actions)
        if correction is not None:
            recovery['visual_turn'] = correction
            camera = (alignment.get('camera_info') or {}).get('fingerprint')
            alignment = _tag1_observe_after_recovery(robot, nav, goal, recovery)
            if alignment is not None:
                if camera and (alignment.get('camera_info') or {}).get('fingerprint') != camera:
                    return None
                correction['after_error_deg'] = alignment['error_deg']
                if abs(alignment['error_deg']) <= deadband and 'clearance_action' not in recovery:
                    alignment = _tag1_center_departure(robot, nav, goal, alignment, result, reserved_actions)
        elif centered:
            recovery['reason_zh'] = 'Tag 1 可原地转向，但本次未发出微转，保留实测余差并先走短段。'
        elif not needs_clearance_check:
            return _tag1_center_departure(robot, nav, goal, alignment, result, reserved_actions)
    return alignment


def _tag1_forward_plan(nav, profile, load, scan, camera_info, nominal, candidates, trial_stride):
    if scan.get('heading_check') in ('same_stance_magnetic_heading','taught_tag_twist'):
        return ({'status':'heading_only','tag_id':1,'position_usable':False},
                {'status':'nominal','count':nominal,'reason':'本次恢复仅确认 Tag 1 水平航向，直行先用路线步数。'})
    pose = RoutePose_Measure(dict(scan, expected_load=load), nav.cfg, profile, 1, camera_info=camera_info)
    plan = RoutePose_PlanForwardCount(pose, profile, nominal)
    if (candidates and plan.get('reason') == 'same-load Tag 1 stride and next-checkpoint clearance unverified'):
        if 'ids' not in scan:
            scan = dict(scan, **summarize(scan, 1, nav.cfg, scan.get('after_ns', 0)))
        plan = RoutePose_PlanTag1ForwardTrial({'scan': scan, 'camera_info': camera_info}, nav.cfg,
                                            nominal, candidates, trial_stride)
    return pose, plan


def _tag1_paired_heading(robot, nav, goal, alignment):
    """Carry the measured Tag 1 offset into the straight without zeroing its yaw."""
    unavailable = RouteTagHeadingRebaseResult.UNAVAILABLE
    scan = alignment['scan']
    fused = alignment['decision_fusion']
    reference = (alignment.get('magnetic') or {}).get('reference') or {}
    anchor = nav.mag.anchor or {}
    stamp, after, observed = (scan.get(k) for k in ('latest_stamp_ns','after_ns','now_ns'))
    completed = robot.last_action_completed_ros_ns
    if (not reference or not anchor or not nav.cfg['post_action_heading'].get('enabled')
            or getattr(robot.backend, 'is_simulation', False)
            or anchor.get('source') != 'reliable_tag_alignment'
            or anchor.get('tag_id') != 1 or anchor.get('load') != robot.load_kind
            or robot.load_kind not in ('single','double') or not anchor.get('capture_verified')
            or not anchor.get('usable_for_heading') or not all(type(v) is int for v in (completed,after,stamp,observed))
            or not 0 < completed <= after < stamp <= observed
            or anchor.get('visual_stamp_ns') != stamp or stamp <= getattr(nav.io,'restart_barrier',0)
            or scan.get('contract_match') is not True or scan.get('camera_healthy') is not True
            or not (alignment.get('camera_info') or {}).get('fingerprint')
            or anchor.get('visual_camera_fingerprint') != alignment['camera_info']['fingerprint']):
        return unavailable
    now_ns = Checkpoint_OptionalScanTime(nav.io, robot.check)
    if now_ns is None or not 0 <= now_ns - stamp <= _TAG1_STANCE_REFERENCE_MAX_NS:
        return unavailable
    stream = getattr(nav.io,'capture_stream',None)
    session = anchor.get('capture_session')
    calibration = anchor.get('calibration_id')
    epoch = anchor.get('load_epoch')
    scope = (getattr(robot.backend,'session',''), getattr(robot.backend,'epoch',-1))
    if (not session or not calibration or type(epoch) is not int or not scope[0]
            or type(scope[1]) is not int or scope[1] < 0
            or anchor.get('owner_observation_scope') != scope or not getattr(nav.io,'motion_capture_verified',False)
            or session != getattr(stream,'session',None) or calibration != getattr(stream,'calibration_digest',None)
            or epoch != getattr(nav.io,'magnetic_load_epoch',None)
            or any(anchor.get(k) != reference.get(k) for k in ('raw_heading_deg','offset_deg','capture_session',
                        'calibration_id','received_monotonic','load_epoch','norm'))):
        return unavailable
    yaw, goal_yaw = fused.get('error_deg'), goal.get('route_heading_deg')
    measured, offset = anchor.get('raw_heading_deg'), anchor.get('offset_deg')
    started, paired, receipt = (getattr(robot,'last_action_completed_monotonic',None),
                                anchor.get('paired_after_monotonic'),anchor.get('received_monotonic'))
    sign = nav.cfg['magnetic'].get('heading_sign')
    now = robot.clock()
    if (not all(type(v) in (int,float) and isfinite(v) for v in (yaw,goal_yaw,measured,offset,started,paired,receipt,now))
            or not started <= paired < receipt <= now or sign not in (-1,1)
            or now - receipt > _TAG1_STANCE_REFERENCE_MAX_NS / 1e9
            or abs(wrap(sign*measured + offset - goal_yaw - yaw)) > .01):
        return unavailable
    robot.check()
    departure = {'tag_id':1,'route_heading_deg':goal_yaw,'error_deg':yaw,'observed_at_ns':stamp}
    nav.mag.anchor = dict(anchor,source='relative_stationary_segment_start',route_heading_deg=goal_yaw,
                          absolute_route_heading_known=True,tag_departure=departure)
    guard = robot._heading_controller(nav)
    guard.latched = False
    robot.magnetic_route_only = False
    robot.last_heading_report = None
    robot.events.append({'segment_heading_reference':{'status':'tag_departure_reference_established',
                        'reference_source':'same_stance_tag_magnetic_pair','tag_departure':departure,
                        'reference':dict(nav.mag.anchor)}})
    return RouteTagHeadingRebaseResult.APPLIED


def _tag1_same_stance_reference(robot, nav, plan, scan, fresh, camera_info, completed_ns):
    """A settled reference can size the existing straight while the stance stays fixed."""
    latest = plan.get('latest_stamp_ns')
    after, observed = scan.get('after_ns'), scan.get('now_ns')
    current = fresh.get('camera_info') or {}
    now = Checkpoint_OptionalScanTime(nav.io, robot.check)
    return (plan.get('source') == 'tag1_forward_trial'
            and all(type(stamp) is int for stamp in (completed_ns, after, latest, observed))
            and 0 < completed_ns == robot.last_action_completed_ros_ns <= after < latest <= observed
            and now is not None and 0 <= now - latest <= _TAG1_STANCE_REFERENCE_MAX_NS
            and fresh.get('camera_healthy') is True and fresh.get('contract_match') is True
            and current.get('ready') is True and current.get('fingerprint') == camera_info['fingerprint']
            and all(row.get('id') == 1 for row in fresh.get('observations', ())))


def _tag2_boundary_pose_rows(robot, nav, boundary):
    after = boundary.get('after_ns')
    completed = robot.last_action_completed_ros_ns
    now = Checkpoint_OptionalScanTime(nav.io, robot.check)
    if (not boundary.get('camera_info_ready') or not all(type(stamp) is int for stamp in (after, completed, now))
            or not 0 < completed <= after <= now):
        return []
    restart = boundary.get('camera_restart_ns', 0)
    observed = boundary.get('now_ns', now)
    if type(restart) is not int or type(observed) is not int or boundary.get('stance_completed_ns', completed) != completed:
        return []
    return [row for row in boundary['scan'].get('observations', ())
            if isinstance(row, dict) and type(row.get('stamp_ns')) is int
            and max(after,restart) < row['stamp_ns'] <= min(observed,now)
            and now-row['stamp_ns'] <= nav.cfg['max_age_s']*1e9]


def _tag2_boundary_near_hint(robot, nav, boundary, near_limit, *, rows=None):
    """A close glimpse can request a rescan from farther away, never authorize right strafe."""
    rows = _tag2_boundary_pose_rows(robot, nav, boundary) if rows is None else rows
    geometry = RoutePose_PartialCheckpointGeometry(rows, nav.cfg, 2)
    if geometry is None:
        return None
    near, far = geometry['distance_bounds_m']
    if (not .02 <= near <= far <= near_limit + .01
            or max(map(abs, geometry['lateral_bounds_m'])) > .10 or geometry['max_abs_heading_deg'] > 12.):
        return None
    return {'distance_m': far, 'latest_stamp_ns': geometry['latest_stamp_ns'],
            'stance_completed_ns': robot.last_action_completed_ros_ns, 'advisory_only': True}


def _tag2_partial_near_boundary(robot, nav, settings, action, boundary):
    # A partial reading may cancel unwalked forward strides. Only the normal
    # checkpoint controller decides whether to retreat, turn or depart sideways.
    boundary = boundary or {}
    scan = boundary.get('scan') or {}
    camera = boundary.get('camera_info') or {}
    pinned = nav.cfg.get('visual_micro', {}).get('camera_info', {})
    expected = pinned.get('expected_fingerprint')
    if (pinned.get('source_confirmed') is not True or boundary.get('action') != action or boundary.get('target_id') != 2 or boundary.get('fault_diagnostic')
            or not all(boundary.get(k) is True for k in ('boundary_valid','contract_match','camera_healthy','camera_info_ready'))
            or boundary.get('stance_completed_ns') != robot.last_action_completed_ros_ns
            or not expected or camera.get('ready') is not True or camera.get('fingerprint') != expected
            or 2 not in scan.get('observed_ids', ()) or not settings.get('distance_window_m')):
        return None
    geometry = RoutePose_PartialCheckpointGeometry(_tag2_boundary_pose_rows(robot, nav, boundary), nav.cfg, 2)
    live = VisualCamera_GetFreshnessCheck(nav.io)
    if geometry is None or not live or not live(geometry['latest_stamp_ns']):
        return None
    near, far = geometry['distance_bounds_m']
    # A singleton only justifies re-reading a genuinely near marker. A pair of
    # independent, agreeing frames can also protect the configured target band.
    limit = (settings['distance_window_m'][0] + .01 if geometry['samples'] == 1 else
             settings['target_distance_m'] + settings['deadband_m'])
    if (not .02 <= near <= far <= limit or far-near > .025 or geometry['max_heading_deviation_deg'] > 3.
            or geometry['projection_bounds'][0] < .65 or geometry['body_forward_bounds_m'][0] <= 0.
            or max(map(abs, geometry['lateral_bounds_m'])) > .20):
        return None
    return dict(geometry, distance_m=(near+far)/2, advisory_only=True)


def _tag2_boundary_replan(checkpoint, nav, settings, remaining, *, continuous=True):
    """Shorten the pending straight using a settled observation before another macro."""
    counts = TAG2_APPROACH_COUNTS if continuous else tuple(count for count in TAG2_APPROACH_COUNTS if count % 2 == 0)
    plan = RoutePose_PlanTag2ForwardTrial(checkpoint,nav.cfg,settings,candidate_counts=counts)
    if 'distance_bounds_m' not in plan:
        return None
    near, far = plan['distance_bounds_m']
    left, right = plan['lateral_bounds_m']
    if (not .02 <= near <= far <= TAG2_FAR_APPROACH_LIMIT_M or far-near > .025 or right-left > .025
            or plan['max_heading_deviation_deg'] > 3.):
        return None
    if (far <= settings['target_distance_m'] + settings['deadband_m']
            and plan['projection_bounds'][0] >= .65):
        return 0,plan,'Tag 2 已在目标距离内或更近，结束剩余直行，原地核对侧跨起点。'
    if plan['projection_bounds'][0] < .85:
        return None
    if plan.get('heading_to_correct_deg') is not None or plan.get('heading_reobserve_required'):
        return 0,plan,'已提前读到 Tag 2，先修正当前航向。'
    if plan.get('guarded_forward_steps') == 2:
        return 0,plan,'已提前读到偏远的 Tag 2，先按当前位姿短进两步，再确认右跨起点。'
    if plan['max_abs_lateral_m'] > .12:
        return 0,plan,'已提前读到 Tag 2，横偏较大，先按当前位置核对侧跨起点。'
    if plan.get('forward_would_cross_minimum'):
        return 0,plan,'Tag 2 下一组直行预计会越过最小起跨距离，结束剩余直行并核对当前位置。'
    if plan.get('confirmed_near') or (plan['in_departure_window'] and not plan['planned_steps']):
        return 0,plan,'已按 Tag 2 实测距离收短直行，保留当前扫码位置。'
    if 0 < plan['planned_steps'] < remaining:
        return plan['planned_steps'],plan,'已把 Tag 2 的实测距离合入剩余直行步数。'
    return remaining,plan,None


def EntranceTrial_Run(robot, *, camera_info=None, entry_alignment=None):
    """Continue from an already accepted Tag 1; scan at the next action boundary."""
    start, forward, end = COMMAND0_OUTBOUND[:3]
    if (start != {'tag': 1, 'direction': 'outbound'}
            or forward != {'op': 'carry_forward_group', 'steps': COMMAND0_ENTRY_FORWARD_STEPS}
            or end != {'tag': 2, 'direction': 'outbound'}):
        raise ValueError('Command 0 entry trial expects the Tag 1 to Tag 2 forward segment')

    expected_id = end['tag']
    route_steps = robot.cfg.get('route_steps', {})
    estimate = route_steps.get('continue_estimated_route') is True
    target_steps = route_steps.get('tag1_to_tag2_forward_steps', forward['steps'])
    continuous = Cmd0Motion_ContinuousReady(robot.backend)
    legacy = not continuous and Cmd0Motion_LegacyReady(robot.backend)
    operations, planned_steps = EntranceTrial_PlanForward(target_steps, continuous=continuous, legacy=legacy)
    result = {'from_tag': start['tag'], 'expected_id': expected_id, 'next_stage_started': False,
              'detected': False, 'position_confirmed': False, 'route_evidence': 'estimated_trial',
              'expected_load': robot.load_kind, 'completed_actions': [], 'observation_windows': 0,
              'completed_steps': 0, 'forward_protocol_complete': False,
              'nominal_target_steps': target_steps, 'target_steps': target_steps, 'planned_steps': planned_steps,
              'scan': None, 'status': CheckpointScanResult.MOTION_NOT_STARTED.value,
              'reason': 'entry trial preparing'}
    # Keep the latest snapshot once so cancellation retains diagnostics and a
    # prolonged reacquisition does not create an event for every window.
    robot.events.append({'entrance_trial': result})

    def set_status(status, reason):
        result.update(status=status.value, reason=reason)
        return result

    robot.check()
    if robot.load_kind != 'single' and not (robot.load_kind == 'double' and getattr(robot, 'pair_route_enabled', False)):
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'expected supported carry load required')
    if robot.count + len(operations) > robot.cfg['max_body_macros_per_journey']:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'insufficient body macro budget for entry segment')
    nav = robot.ensure_navigation_io()
    if nav is None:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'stationary Tag observation unavailable')
    points = nav.cfg['legs']['outbound']
    if [point['id'] for point in points[:2]] != [start['tag'], expected_id]:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'configured outbound Tag order differs from entry trial')

    if estimate and not EntranceTrial_IsEntryAccepted(entry_alignment):
        result['entry_fallback'] = 'Tag 1 找码后仍未取得位姿，按预设步数走向 Tag 2。'
        search = entry_alignment or {}
        if (type(search.get('tag1_search_completed_ns')) is int
                and search['tag1_search_completed_ns'] == robot.last_action_completed_ros_ns):
            net = search.get('tag1_reacquire_forward_steps',0) - search.get('tag1_reacquire_backward_steps',0)
            # Without a Tag 1 pose this is a nominal displacement only. Keep
            # the Tag 2 boundary scans active to supersede it as soon as visible.
            if net:
                remaining = max(2, target_steps - net)
                adjusted, exact = EntranceTrial_PlanForward(remaining,continuous=continuous,legacy=legacy)
                if exact == remaining and robot.count + len(adjusted) <= robot.cfg['max_body_macros_per_journey']:
                    operations, planned_steps, target_steps = adjusted, exact, remaining
                    result.update(target_steps=remaining,planned_steps=remaining,entry_search_net_steps=net,
                        entry_fallback=f'Tag 1 尚未定位，找码净位移 {net:+d} 步已合入路线，继续向 Tag 2 走 {remaining} 步。')
        entry_alignment = camera_info = None
    if camera_info is not None:
        snapshot = getattr(nav.io, 'preflight_health_snapshot', None)
        try:
            health = snapshot() if snapshot is not None else {}
        except OSError as exc:
            robot.check()
            result['camera_read_error'] = str(exc)
            health = {}
        current = health.get('camera_info', {})
        if (not health.get('camera_healthy') or not current.get('ready')
                or not camera_info.get('fingerprint') or current.get('fingerprint') != camera_info['fingerprint']):
            if not estimate:
                return set_status(CheckpointScanResult.CAMERA_UNAVAILABLE, 'chest camera stream or calibration changed before entry trial')
            result['entry_fallback'] = '胸部相机数据源已变化，舍弃旧位姿，按预设步数走向 Tag 2。'
            entry_alignment = camera_info = None

    original_alignment = entry_alignment
    entry_alignment = _tag1_recover_departure(robot, nav, points[0], entry_alignment, result,
                                             reserved_actions=len(operations))
    if entry_alignment is None and original_alignment is not None:
        if not estimate:
            return set_status(CheckpointScanResult.MOTION_NOT_STARTED,
                              'Tag 1 纠偏后未重新确认入口航向，保留持物并记录本次恢复动作。')
        result['entry_fallback'] = 'Tag 1 纠偏后暂不可见，沿预设路线进入下一段。'
    if entry_alignment is not None and entry_alignment is not original_alignment:
        camera_info = entry_alignment['camera_info']
    entry_stance_ns = robot.last_action_completed_ros_ns
    nominal_entry = bool(result.get('entry_fallback') or entry_alignment and entry_alignment.get('tag1_nominal_entry') is True)
    rebase = getattr(robot, 'Route_RebaseTagHeading', None)
    tag_rebased = False
    if rebase is not None and not nominal_entry and entry_alignment and EntranceTrial_IsEntryAccepted(entry_alignment):
        outcome = (_tag1_paired_heading(robot, nav, points[0], entry_alignment) if camera_info
                   else RouteTagHeadingRebaseResult.UNAVAILABLE)
        if outcome is RouteTagHeadingRebaseResult.APPLIED:
            result['entry_heading_source'] = 'same_stance_tag_magnetic_pair'
        else:
            stamp = entry_alignment['scan'].get('latest_stamp_ns')
            outcome = rebase(start['tag'], entry_alignment['decision_fusion'].get('error_deg'), stamp)
            live = VisualCamera_GetFreshnessCheck(nav.io)
            if (outcome is RouteTagHeadingRebaseResult.UNAVAILABLE and camera_info and estimate
                    and type(stamp) is int and live is not None and not live(stamp)
                    and callable(getattr(robot,'clock',None)) and getattr(robot,'deadline',None) is not None
                    and not getattr(robot.backend,'is_simulation',False)):
                refresh = {'reobservations':[]}
                result['entry_heading_reobserve'] = refresh
                current = _tag1_observe_after_recovery(robot, nav, points[0], refresh)
                if (current and entry_stance_ns == robot.last_action_completed_ros_ns
                        and (current.get('camera_info') or {}).get('fingerprint') == camera_info.get('fingerprint')
                        and EntranceTrial_IsEntryAccepted(current)):
                    if entry_alignment.get('correction_executed'):
                        current['correction_executed'] = True
                    current = _tag1_recover_departure(robot, nav, points[0], current, result,
                                                      reserved_actions=len(operations))
                    entry_stance_ns = robot.last_action_completed_ros_ns
                    entry_alignment = current
                    if current and current['camera_info'].get('fingerprint') == camera_info.get('fingerprint'):
                        camera_info = current['camera_info']
                        outcome = rebase(start['tag'], current['decision_fusion'].get('error_deg'),
                                         current['scan']['latest_stamp_ns'])
                        if outcome is RouteTagHeadingRebaseResult.APPLIED:
                            result['entry_heading_source'] = 'stationary_tag_reobservation'
                    else:
                        entry_alignment = camera_info = None
                        refresh['reobserve_reason'] = '入口纠偏后未重新取得同一相机的位姿，按预设路线继续。'
                elif robot.last_action_completed_ros_ns != entry_stance_ns:
                    entry_alignment = camera_info = None
                elif current:
                    entry_alignment = camera_info = None
                    refresh['reobserve_reason'] = '新观测的相机标定或入口位姿未通过核对。'
        result['tag_heading_rebase'] = outcome.value
        tag_rebased = outcome is RouteTagHeadingRebaseResult.APPLIED
    if nominal_entry:
        robot.prepare_heading_segment(fresh=False)
        result['entry_heading_source'] = 'retained_magnetic_reference'
        result['forward_plan'] = {'status': 'nominal', 'count': target_steps,
                                  'reason': result.get('entry_fallback', 'Tag 1 identified, but metric pose was not used')}
    elif not tag_rebased:
        # Establish a fresh relative zero if the Tag reading has expired or the
        # Mag stream cannot be paired with the confirmed departure stance.
        nav.mag.anchor = None
        result['entry_heading_source'] = 'relative_straight_only'
        robot.prepare_heading_segment(fresh=True)
    if (nav.cfg['post_action_heading'].get('enabled') and nav.mag.anchor is None
            and not getattr(robot,'magnetic_route_only',False) and not estimate):
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'fresh straight-segment heading reference unavailable')
    if entry_alignment is not None and not nominal_entry:
        profile = RoutePose_Load(robot.project)
        trial = route_steps.get('tag2_forward_trial') or {}
        trial_options = []
        if trial.get('enabled') is True:
            pairs = getattr(robot.backend, 'mission_pair_ops', None)
            supported = pairs() if robot.load_kind == 'double' and pairs is not None else None
            for offset in _TAG1_FORWARD_TRIAL_OFFSETS:
                count = target_steps + offset
                if not 2 <= count <= 80:
                    continue
                actions, exact_steps = EntranceTrial_PlanForward(count, continuous=continuous, legacy=legacy)
                if (exact_steps == count and robot.count + len(actions) <= robot.cfg['max_body_macros_per_journey']
                        and (supported is None or all('double_' + action in supported for action in actions))):
                    trial_options.append(count)
        trial_stride = trial.get('two_step_distance_m')
        entry_scan = entry_alignment.get('scan', {})
        pose, plan = _tag1_forward_plan(nav, profile, robot.load_kind, entry_scan, camera_info,
                                       target_steps, trial_options, trial_stride)
        result['entry_pose'] = pose
        result['forward_plan'] = plan
        adjusted_steps = plan['count']
        if adjusted_steps != target_steps:
            motion_ready = VisualCamera_GetFreshnessCheck(nav.io)
            stamp = plan.get('latest_stamp_ns', pose.get('latest_stamp_ns', 0))
            same_stance_reference = False
            if motion_ready is not None and not motion_ready(stamp):
                # Heading preparation can outlive the original camera window.
                # Reacquire once when a calibrated change would otherwise be usable.
                try:
                    fresh = Checkpoint_ReadWindow(nav.io, start['tag'], None, nav.cfg['observe_s'], robot.check)
                except VisualCameraUnavailable as exc:
                    robot.check()
                    result['camera_read_error'] = str(exc)
                    fresh, new_pose = {}, {}
                    new_plan = {'status': 'nominal', 'count': target_steps, 'reason': 'Tag 1 重扫暂时不可用'}
                else:
                    new_pose, new_plan = _tag1_forward_plan(nav, profile, robot.load_kind, fresh, fresh.get('camera_info'),
                                                           target_steps, trial_options, trial_stride)
                if (camera_info and entry_alignment.get('pose_accepted') is True
                        and 'distance_m' not in new_plan and new_plan['count'] == target_steps
                        and _tag1_same_stance_reference(robot, nav, plan, entry_scan, fresh, camera_info, entry_stance_ns)):
                    same_stance_reference = True
                    plan.update(reused_same_stance_reference=True, reobserve_reason=new_plan['reason'])
                else:
                    pose, plan = new_pose, new_plan
                result.update(entry_pose=pose, forward_plan=plan)
                adjusted_steps = plan['count']
                stamp = plan.get('latest_stamp_ns', pose.get('latest_stamp_ns', 0))
            if adjusted_steps != target_steps:
                adjusted_actions, exact_steps = EntranceTrial_PlanForward(adjusted_steps, continuous=continuous, legacy=legacy)
                if not same_stance_reference and (not motion_ready or not motion_ready(stamp)):
                    plan.update(status='nominal', count=target_steps, reason='Tag 1 forward estimate expired before dispatch')
                elif exact_steps != adjusted_steps or robot.count+len(adjusted_actions) > robot.cfg['max_body_macros_per_journey']:
                    plan.update(status='nominal', count=target_steps, reason='adjusted forward segment exceeds supported motion budget')
                else:
                    operations = adjusted_actions
                    result.update(target_steps=adjusted_steps, planned_steps=exact_steps)

    move_entry = getattr(robot, 'entry_forward_action', None)
    tag2_settings = route_steps.get('tag2_forward_trial') or {}
    live_tag2_trial = tag2_settings.get('enabled') is True
    use_boundary_plan = (live_tag2_trial and move_entry is not None
                         and callable(getattr(nav.io,'boundary_visual_snapshot',None)))
    if use_boundary_plan:
        native_splits = {('cmd0_carry_forward10',):('carry_forward','carry_forward2'),
                         ('cmd0_carry_forward15',):('carry_forward','cmd0_carry_forward5','carry_forward2')}
        grouped = native_splits.get(tuple(operations))
        if grouped and robot.count+len(grouped) <= robot.cfg['max_body_macros_per_journey']:
            operations = grouped
            result['entry_forward_grouping'] = list(grouped)
    lost_after_strafe = (result.get('tag1_lateral_recovery') or {}).get('stance_unconfirmed')
    if lost_after_strafe or _tag1_unresolved_entry_heading(original_alignment, entry_alignment,
            result.get('tag1_heading_recovery') or {}, nav.cfg.get('visual_micro', {}).get('deadband_deg', 8.),
            robot.last_action_completed_ros_ns):
        operations = _tag1_short_start_actions(operations, continuous, robot.cfg['max_body_macros_per_journey']-robot.count)
        result['entry_forward_grouping'] = list(operations)
        result['entry_heading_grouping_reason_zh'] = ('Tag 1 侧跨后失去位姿，先走短段并复查。' if lost_after_strafe
            else 'Tag 1 大偏航尚未确认转正，先走短段并在动作回执处检查当前航向。')
    result['initial_estimated_steps'] = result['planned_steps']
    set_status(CheckpointScanResult.MOVING_TO_CHECKPOINT, 'executing estimated Tag 1 to Tag 2 segment')
    _report_progress(robot, 'cmd0_next_leg', result)
    pending = list(operations)
    while pending:
        operation = pending.pop(0)
        try:
            if move_entry is None:
                robot.action(operation)
                result['completed_actions'].append(operation)
            else:
                next_action = pending[0] if pending else None
                move_entry(operation, next_action=next_action, completed_actions=result['completed_actions'])
        finally:
            result['next_stage_started'] = bool(result['completed_actions'])
            result['completed_steps'] = Cmd0Motion_ForwardStepCount(result['completed_actions'])
        if not use_boundary_plan or not pending:
            continue
        boundary = getattr(robot, 'last_boundary_visual', None)
        checkpoint = Checkpoint_FromBoundary(robot,nav,expected_id,action=operation,report=boundary)
        remaining = Cmd0Motion_ForwardStepCount(pending)
        if checkpoint is not None:
            result['tag2_seen_earlier_boundary'] = True
            decision = _tag2_boundary_replan(checkpoint,nav,tag2_settings,remaining,continuous=continuous)
        else:
            decision = None
        if decision is None:
            hint = _tag2_partial_near_boundary(robot,nav,tag2_settings,operation,boundary)
            if hint is None:
                continue
            decision = (0,hint,'动作边界已看到近处 Tag 2，留在当前站姿补帧，核对后再横移。')
        proposed, plan, reason = decision
        change = {'after_steps':result['completed_steps'], 'previous_remaining_steps':remaining,
                  'remaining_steps':remaining,'distance_m':plan['distance_m'],
                  'heading_deg':plan['heading_deg'],'lateral_bounds_m':plan['lateral_bounds_m']}
        if plan.get('advisory_only'):
            change.update(advisory_frames=plan['samples'], confirmed_for_strafe=False)
        result.setdefault('entry_boundary_observations',[]).append(change)
        if not reason or proposed == remaining:
            continue
        revised, exact_steps = EntranceTrial_PlanForward(proposed,continuous=continuous,legacy=legacy) if proposed else ((),0)
        if exact_steps != proposed or robot.count+len(revised) > robot.cfg['max_body_macros_per_journey']:
            continue
        pending = list(revised)
        change.update(remaining_steps=proposed,reason_zh=reason)
        result['planned_steps'] = result['completed_steps'] + proposed

    result['forward_protocol_complete'] = bool(operations) and result['completed_steps'] == result['planned_steps']
    if not result['forward_protocol_complete']:
        return set_status(CheckpointScanResult.MOTION_NOT_STARTED, 'Tag 1 到 Tag 2 的计划直行尚未全部收到动作回执。')

    set_status(CheckpointScanResult.WAITING_FOR_CHECKPOINT, 'stationary reacquisition of expected Tag 2')
    _report_progress(robot, 'cmd0_next_scan', result)
    last_action = result['completed_actions'][-1]
    cached = Checkpoint_FromBoundary(robot,nav,expected_id,action=last_action)
    if cached is not None:
        result.update(cached)
        notify = getattr(robot,'route_checkpoint_observed',None)
        if notify is not None:
            notify(expected_id)
        return set_status(CheckpointScanResult.CHECKPOINT_DETECTED, 'Tag 2 已在动作边界确认，直接复用当前位姿。')
    boundary = getattr(robot, 'last_boundary_visual', None) or {}
    glimpse = boundary.get('scan') or {}
    if (boundary.get('target_id') == expected_id and boundary.get('action') == last_action
            and boundary.get('boundary_valid') is True and boundary.get('contract_match') is True
            and boundary.get('camera_healthy') is True and expected_id in glimpse.get('observed_ids', ())):
        result['tag2_boundary_sighting'] = {'accepted_frames': glimpse.get('samples',0),
                                            'after_ns': boundary.get('after_ns')}
        if live_tag2_trial and 'distance_window_m' in tag2_settings:
            rows = _tag2_boundary_pose_rows(robot, nav, boundary)
            if rows:
                result['tag2_boundary_sighting'].update(partial_pose_rows=rows,
                    stance_completed_ns=robot.last_action_completed_ros_ns)
            near = _tag2_boundary_near_hint(robot, nav, boundary, tag2_settings['distance_window_m'][0], rows=rows)
            if near is not None:
                result['tag2_boundary_sighting']['near_advisory'] = near
    glimpse = result.get('tag2_boundary_sighting') or {}
    partial_boundary = glimpse.get('near_advisory') or glimpse.get('partial_pose_rows')
    observed, reason = Checkpoint_Observe(robot, nav, result,
        lambda: _report_progress(robot, 'cmd0_next_scan_wait', result), miss_policy='continue_estimated',
        advisory_windows=2 if partial_boundary or use_boundary_plan else None,
        quick_id=live_tag2_trial, reuse_settled_cache=live_tag2_trial)
    if estimate and observed is CheckpointScanResult.CAMERA_UNAVAILABLE:
        result['checkpoint_not_seen'] = {'tag_id': 2, 'policy': 'continue_estimated',
                                       'cause': 'camera_stream_unhealthy', 'detail': reason}
        observed = CheckpointScanResult.CHECKPOINT_NOT_SEEN
        reason = 'Tag 2 观测不可用，先在侧跨起点找码，再沿预设路线继续。'
    if observed is CheckpointScanResult.CHECKPOINT_NOT_SEEN:
        result['continue_estimated'] = True
        if result.get('tag2_boundary_sighting'):
            reason = '动作结束时读到 Tag 2，右跨前会复核前后距离。'
        elif result.get('tag2_seen_earlier_boundary'):
            result['checkpoint_not_seen']['seen_earlier_boundary'] = True
            reason = '直行途中已读到 Tag 2，继续移动后丢码，右跨前需确认当前位姿。'
    return set_status(observed, reason)
