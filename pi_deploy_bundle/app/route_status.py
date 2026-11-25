"""只读说明本地路线实际接线与门禁；不导入ROS、不执行路线或动作。"""
import json
from pathlib import Path


def inspect_project(project, mode='0', navigation=None):
    from checkpoint_nav import load
    from task_runner import segments
    from cmd0_motion import Cmd0Motion_LoadRouteSteps
    p=Path(project);c=load(p) if navigation is None else navigation
    estimate = str(mode) in ('0','1','2') and Cmd0Motion_LoadRouteSteps(p).get('continue_estimated_route') is True
    blockers=[]
    for n in (9,14,8,7,10):
        try:segments(n,'single')
        except (ValueError,RuntimeError) as exc:blockers.append({'forward_steps':n,'reason':str(exc)})
    pc=c.get('pose_contract',{});m=c.get('magnetic',{});enabled=c.get('enabled',False)
    heading_gate=enabled and pc.get('axes_confirmed',False) and pc.get('capture_time_confirmed',False)
    return {
        'scope':'local_configuration_only_not_device_readiness','body_commands':0,
        'command':str(mode),
        'current_command0':'tag1_to_tag5_then_configured_forward_approach_and_single_release',
        'command0_estimated_checkpoints':[1,2,3,4,5],
        'default_active_checkpoints':[],
        'command0_target_id_requires_explicit_verification':True,
        'relative_reference_does_not_measure_planned_turn_error':True,
        'configured_but_not_default_route':{'outbound':[1,2,3,4,5],'return':[5,6,7,1]},
        'pose_heading_gate_configured':bool(heading_gate),
        'pose_live_publisher_frame_time_still_required':True,
        'lateral_gate_configured':bool(heading_gate and pc.get('scale_confirmed',False)),
        'directional_action_gate_configured':bool(heading_gate and c.get('action_directions_confirmed',False)),
        'real_correction_ready':False,
        'real_correction_blockers':['minimum macro angle and load calibration not measured','sensor_topic capture freshness cannot be verified','finite native cancellation is not immediate','target ID and visual frame/axes/scale are unverified'],
        'mcu_guard_online_status_available':False,
        'physical_recovery_gate':'manual inspection bound to current owner session/epoch, expires in 300 seconds, cleared at finish/cancel',
        'ordinary_tag_loss':('reacquire_expected_tag_then_continue_remaining_route' if estimate else
                             'tag1_entry_gate_then_continue_through_missed_intermediate_tags_and_require_tag5_for_delivery'
                             if str(mode) in ('0','1','2','0-full')
                             else 'terminal_scan_without_onward_motion'),
        'magnetic':{
            'subscription_enabled_at_explicit_checkpoint':bool(enabled and m.get('enabled')),
            'refresh_requires':'relative stationary start of each straight segment; reliable Tag may replace reference after alignment; pickup/turn invalidate old reference',
            'refresh_can_pass_config_gate':bool(heading_gate and m.get('enabled')),
            'heading_sign_confirmed':m.get('heading_sign') in (-1,1),
            'continuous_magnetic_motion_control':False,
            'post_action_monitor_enabled':bool(c.get('post_action_heading',{}).get('enabled')),
            'post_action_correction_enabled':bool(c.get('post_action_heading',{}).get('execute')),
            'post_action_correction_scope':'bounded turns at eligible completed movement boundaries; requires verified heading and fresh samples',
            'checkpoint_observation_fallback_enabled':estimate,
            'capture_timestamp_available':False,
            'schema':'aelos_smart_ros/sensor_topic: uint16 mag degrees, int16 mag_x/y/z raw counts',
        },
        'recorded_outbound':{
            'source':'historical user-provided plan retained for offline comparison',
            'calls':['forward9','right3','forward14','left2','right1','forward8','left1','forward7','right3','forward10','place'],
            'unsupported_forward':blockers,'placement_exported':False,
            'facing_forward_equivalent_counts_known':False,
            'tag_id_to_segment_mapping_known':False,
        },
        'modes1and2':'shared visual pickup and outbound; command 1 delivers once, command 2 returns and repeats',
        'return_chain_available':str(mode)=='2',
        'command2_estimated_return_checkpoints':[5,6,7,1],
        'onsite_route_must_be_preserved':'preserve live route and navigation calibration; apply the current route_steps configuration',
    }


if __name__=='__main__':
    print(json.dumps(inspect_project(Path(__file__).resolve().parent),ensure_ascii=False,indent=2))
