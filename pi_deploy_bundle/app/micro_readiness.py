"""Read-only readiness report. Never imports the SDK, opens I2C or sends a probe."""
import argparse
import json
from pathlib import Path


def report(project, owner_status=None, capture_report=None):
    project=Path(project)
    c=json.loads((project/'config/navigation.json').read_text(encoding='utf-8'))
    runtime=json.loads((project/'config/runtime.json').read_text(encoding='utf-8'))
    r=(owner_status or {}).get('owner',{})
    h=c['post_action_heading'];t=h.get('micro_trial',{});m=c['magnetic']
    micro=r.get('micro_trial',{});recovery=r.get('recovery_gate',{})
    target=runtime.get('transfer_test',{}).get('target_id')
    missing=[]
    if not r:missing.append('live owner boot/session/status not observed')
    if not micro.get('deployment_readback_ready'):missing.append('current-boot MCU micro code readback and reload not verified; do not probe old MCU')
    if not micro.get('capability_verified'):missing.append('fresh exclusive capability START/COMPLETE with nonce and sequence not observed')
    if not recovery.get('ready'):missing.append('current owner session cooling, mechanism, single target and guard-reload acknowledgment missing/expired')
    if not micro.get('guarded_trial_confirmed'):missing.append('current session guarded single trial and non-immediate-stop acknowledgment missing/expired')
    if not m.get('capture_freshness_verified'):missing.append('real bounded atomic DRDY capture validation pending')
    if not capture_report:missing.append('live capture statistics not supplied; config is not evidence of fresh samples')
    elif not capture_report.get('ready_for_capture_contract'):missing.append('provided capture report did not verify consecutive acquisitions')
    if m.get('heading_sign') not in (-1,1) or t.get('heading_sign_evidence')!='measured_r3':missing.append('local magnetic heading_sign unverified; legacy Tag sign is not a substitute')
    if not h.get('directions_confirmed') or t.get('direction_evidence')!='measured_r3':missing.append('local dedicated micro turn direction unverified; nominal amplitude is not direction evidence')
    v=c.get('visual_micro',{})
    if not h.get('execute') and not v.get('execute'):missing.append('both magnetic and visual micro execution remain disabled')
    visual_missing=[]
    if target is None or target!=v.get('confirmed_target_id') or not v.get('target_id_confirmed'):visual_missing.append('explicit runtime target and independently confirmed matching target ID missing')
    if not all(c['pose_contract'].get(k) for k in ('axes_confirmed','scale_confirmed','capture_time_confirmed')):visual_missing.append('Tag axis/scale/capture-time geometry unverified')
    if v.get('direction_evidence')!='measured_r3_tag':visual_missing.append('local ground-pose Tag response direction unverified')
    if not v.get('camera_info',{}).get('source_confirmed') or not v.get('camera_info',{}).get('expected_fingerprint'):visual_missing.append('actual CameraInfo source/frame/calibration not verified')
    visual_missing.append('fresh paired target, CameraInfo and magnetic windows are checked by live runtime, not this static report')
    geometry={k:c['pose_contract'].get(k) for k in ('axes_confirmed','scale_confirmed','capture_time_confirmed')}
    return dict(version='r3-visual-pickup-20261109-v5',read_only=True,body_commands=0,capability_probe_sent=False,
        owner={'session':r.get('session'),'epoch':r.get('epoch'),'boot_id':micro.get('boot_id'),
               'mode':r.get('mode'),'in_flight':r.get('in_flight'),'version':r.get('owner_version')},
        micro=micro or {'capability_verified':False},recovery=recovery or {'ready':False},
        magnetic={'heading_sign':m.get('heading_sign'),'heading_sign_evidence':t.get('heading_sign_evidence'),
                  'configured_contract':m.get('capture_contract'),'field_validation':m.get('field_capture_validation'),
                  'provided_capture_statistics':capture_report,'live_runtime_still_rechecks_every_sample':True},
        visual={'explicit_target_id':target,'confirmed_target_id':v.get('confirmed_target_id'),'geometry':geometry,'dedicated_micro_wired':True,'execute':v.get('execute',False),'camera_info_contract':v.get('camera_info'),'required_before_visual_execution':visual_missing,'fresh_frames_observed_by_this_command':False,
                'search_enabled':c['search_enabled'],'pose_correction_enabled':c['pose_correction_enabled'],
                'note':'Missing ID is never replaced with first observed ID. Image y=.60H is diagnostic only. Magnetic relative straight drift does not establish absolute route heading.'},
        angle={'evidence':t.get('angle_evidence'),'nominal_degrees':t.get('nominal_degrees'),'measured_degrees':None},
        execution_modes={'magnetic_execute':h.get('execute',False),'visual_execute':v.get('execute',False),'shared_max_actions':1,'shared_max_nominal_degrees':5,'both_in_one_journey_not_guaranteed':True},
        required_before_actual_execution=missing,automatic_enable=False,hardware_ready_claimed=False,
        sign_diagnostic='With support and motion separately authorized, record fresh stationary windows before/after a physically observed small left turn. Compare raw magnetic heading wrap; under right-positive convention a negative raw delta suggests +1, positive suggests -1. Reject noisy/ambiguous readings, verify both directions. Never write a sign automatically or infer it from nominal five degrees.')


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--project',type=Path,default=Path(__file__).resolve().parent)
    parser.add_argument('--owner-status',type=Path)
    parser.add_argument('--capture-report',type=Path)
    args=parser.parse_args(argv)
    owner=json.loads(args.owner_status.read_text(encoding='utf-8')) if args.owner_status else None
    capture=json.loads(args.capture_report.read_text(encoding='utf-8')) if args.capture_report else None
    if capture:capture=capture.get('report',capture)
    print(json.dumps(report(args.project,owner,capture),ensure_ascii=False,indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
