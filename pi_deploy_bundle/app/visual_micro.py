"""Finite explicit-target heading-only alignment via dedicated micro actions."""
import math
import hashlib
import json
import re
from nav_observation import MAGNETIC_INSUFFICIENT_SAMPLES, summarize, wrap, MagneticReference, mad
from micro_trial import OPS, active, response_limit


class VisualMicroStopped(RuntimeError):
    def __init__(self, reason, *, report=None):
        super().__init__(reason)
        self.report=report
        if report and report.get('reason_zh'):
            self.error_details={'error_code':'passive_tag_observation_failed','reason_zh':report['reason_zh']}


class _VisualObservationExpired(RuntimeError):
    """Optional correction was declined before any native request."""


class _VisualTurnNotReserved(RuntimeError):
    """The shared budget did not allow the pending visual correction."""


def validate(cfg):
    v=cfg.get('visual_micro')
    if v is None:return
    for k in ('enabled','execute','target_id_confirmed'):
        if type(v.get(k)) is not bool:raise ValueError('visual micro flag '+k)
    if type(v.get('max_reobserve_windows')) is not int or not 0<=v['max_reobserve_windows']<=2:
        raise ValueError('visual reacquisition must be at most two extra stationary windows')
    if v.get('search_policy')!='stationary_new_frames_only':raise ValueError('one-five-degree budget cannot promise physical search and return')
    if v.get('deadband_deg')!=8 or v.get('release_deg')!=4 or v.get('max_error_deg')!=30 or v.get('max_observed_step_deg')!=response_limit(cfg.get('post_action_heading',{})) or v.get('minimum_improvement_deg')!=1:
        raise ValueError('visual trial requires 8/4/30, post-check8, improvement1')
    if type(v.get('max_sensor_disagreement_deg')) not in (int,float) or not 0<v['max_sensor_disagreement_deg']<=3:
        raise ValueError('visual/magnetic change disagreement bound invalid')
    for k in ('max_heading_mad_deg','max_magnetic_mad_deg'):
        if type(v.get(k)) not in (int,float) or not 0<v[k]<=2:raise ValueError('small-micro stable-window bound '+k)
    info=v.get('camera_info',{})
    for k in ('source_confirmed','static_camera_info_verified'):
        if type(info.get(k)) is not bool:raise ValueError('CameraInfo flag '+k)
    if not isinstance(info.get('topic'),str) or not info['topic'].startswith('/'):
        raise ValueError('CameraInfo topic required')
    for k in ('max_age_s','max_sync_skew_s'):
        if type(info.get(k)) not in (int,float) or not math.isfinite(info[k]) or not 0<info[k]<=1.2:
            raise ValueError('CameraInfo timing bound '+k)
    if not v['execute']:return
    h=cfg['post_action_heading'];t=h.get('micro_trial',{})
    if not (v['enabled'] and h['enabled'] and t.get('enabled')):
        raise ValueError('visual execution requires same active dedicated magnetic micro trial')
    if not v['target_id_confirmed'] or type(v.get('confirmed_target_id')) is not int or not 0<=v['confirmed_target_id']<=1023 or v.get('direction_evidence')!='measured_r3_tag':
        raise ValueError('explicit target and local Tag response direction evidence required')
    from relative_heading import reference_valid
    relative=reference_valid(cfg.get('relative_heading_reference'),v.get('confirmed_target_id'))
    if not (cfg['pose_contract']['capture_time_confirmed'] and (relative or all(cfg['pose_contract'][k] for k in ('axes_confirmed','scale_confirmed')))):
        raise ValueError('visual axis, scale and capture-time contract must be verified')
    if (not info['source_confirmed'] or not isinstance(info.get('publisher'),str) or not info['publisher'].startswith('/') or
            not isinstance(info.get('frame_id'),str) or not info['frame_id'] or
            not isinstance(info.get('expected_fingerprint'),str) or not re.fullmatch('[0-9a-f]{64}',info['expected_fingerprint'])):
        raise ValueError('actual verified CameraInfo source/frame/fingerprint required')
    if v.get('signed_nominal_delta_deg')!=t.get('signed_nominal_delta_deg'):
        raise ValueError('Tag and magnetic micro direction conventions must match observed R3 response')
    if any(type(v.get('signed_nominal_delta_deg',{}).get(op)) not in (int,float) or abs(v['signed_nominal_delta_deg'][op])!=5 for op in OPS):
        raise ValueError('dedicated signed nominal five-degree Tag deltas required')


def VisualCamera_DiagnoseWindow(window, camera, expected_fingerprint):
    stream=window.get('camera_window_status', {})
    ar_waiting_for_first_packet=(stream.get('parameters_match') is True
                                 and 'ar_source_matches' in stream and stream['ar_source_matches'] is None)
    if (stream.get('parameters_match') is False or stream.get('ar_source_matches') is False
            or (not window.get('contract_match') and not ar_waiting_for_first_packet)):
        return ('camera/AR source or parameters changed during passive Tag checkpoint',
                '扫码话题来源或参数校验失败。',False)
    sources=(window.get('camera_info',{}), camera)
    for item in sources:
        fingerprint=item.get('fingerprint')
        if ((fingerprint is not None and fingerprint!=expected_fingerprint)
                or (item.get('ready') is True and fingerprint!=expected_fingerprint)):
            return ('actual CameraInfo calibration/source/frame changed',
                    '胸部相机标定或来源校验失败。',False)
        if item.get('ready') is not True and not item.get('can_reobserve'):
            return ('actual CameraInfo calibration/source/frame/time unavailable or changed',
                    '胸部相机标定、来源或时间校验失败。',False)
    if not window.get('camera_healthy') or not window.get('contract_match'):
        return ('camera/AR stream fault during passive Tag checkpoint',
                '胸部相机或扫码话题暂未收到符合要求的新帧。',True)
    if any(item.get('ready') is not True for item in sources):
        return ('CameraInfo/actual image timing unavailable during passive Tag checkpoint',
                '胸部相机图像与标定帧暂未同步。',True)
    return None


def VisualMicro_ObserveWithoutMagnetic(cfg, io, tag_id, goal, *, after_ns, deadline, check, clock,
                                     accept_trusted_residual=False):
    """Use only fresh calibrated camera heading at an explicit route checkpoint."""
    validate(cfg)
    v=cfg['visual_micro']
    result=dict(status='partial',expected_id=tag_id,reason='no fresh verified Tag heading',
        scan={'ids':[],'observed_ids':[],'observations':[]},camera_info={},events=[],
        observation_windows=0,scan_attempted=False,fresh_target_detected=False,pose_accepted=False,
        heading_aligned=False,position_controlled=False,ready_for_obstacle=False,physical_pose_confirmed=False,
        correction_actions=0,search_actions=0,correction_executed=False,visual_only=True,
        magnetic={'status':'not_updated','reason':'passive Tag heading after magnetic data loss'})
    if (type(tag_id) is not int or tag_id!=goal.get('id') or not v['target_id_confirmed']
            or tag_id!=v.get('confirmed_target_id')):
        result.update(status='blocked',reason='explicit target ID missing/mismatched; never select first observed ID')
        return result
    if type(after_ns) is not int or after_ns<0:
        raise VisualMicroStopped('invalid ROS barrier before passive Tag checkpoint')
    default_deadline=clock()+cfg['checkpoint_timeout_s']
    deadline=default_deadline if deadline is None else min(default_deadline,deadline)
    camera_contract=v['camera_info']
    capture_contract=cfg['pose_contract']
    expected=camera_contract.get('expected_fingerprint')
    if camera_contract.get('source_confirmed') is not True or not expected:
        raise VisualMicroStopped('actual CameraInfo calibration/source unavailable')
    current=getattr(io,'visual_camera_status',None)
    unavailable_windows=0
    last_issue=None

    def stop(issue):
        reason, reason_zh, _=issue
        raise VisualMicroStopped(reason,report=dict(result, status='blocked',reason=reason,reason_zh=reason_zh))

    check()
    if clock()+cfg['settle_s']>=deadline:return result
    io.settle(cfg['settle_s'],check)
    while clock()<deadline:
        check()
        remaining=deadline-clock()
        if remaining<=0:break
        window=io.observe(tag_id,after_ns,min(cfg['observe_s'],remaining),check)
        check()
        camera=current() if current is not None else window.get('camera_info',{})
        result.update(camera_info=camera,scan_attempted=True,
                      observation_windows=result['observation_windows']+1)
        window_now=window.get('now_ns')
        now_ros=io.ros_now_ns()
        if type(window_now) is not int or type(now_ros) is not int or not after_ns<=window_now<=now_ros:
            stop(('ROS clock moved backwards during passive Tag checkpoint',
                  '扫码时钟回退，本轮图像无法验证。',False))
        last_issue=VisualCamera_DiagnoseWindow(window,camera,expected)
        if last_issue:
            unavailable_windows+=1
            status=dict(window.get('camera_window_status',{}),reason=last_issue[0],
                        reason_zh=last_issue[1],camera_info_reason=camera.get('reason'),
                        recoverable=last_issue[2],window=result['observation_windows'])
            result['events'].append({'passive_camera_window':status})
            if not last_issue[2] or unavailable_windows>v['max_reobserve_windows']:
                stop(last_issue)
            after_ns=now_ros
            continue
        unavailable_windows=0
        obs=summarize(window,tag_id,cfg,after_ns)
        obs.update(now_ns=window_now,after_ns=max(after_ns,window.get('after_ns',0)),
                   camera_healthy=True,contract_match=True)
        trusted=(obs['ids']==[tag_id] and obs.get('pose_usable')
                 and (obs.get('position_usable') or obs.get('heading_only'))
                 and capture_contract.get('capture_time_confirmed') is True
                 and all(row.get('source_publisher')==capture_contract['publisher'] for row in obs['observations'])
                 and obs.get('heading_mad_deg',float('inf'))<=v['max_heading_mad_deg']
                 and 0<=now_ros-obs['latest_stamp_ns']<=cfg['max_age_s']*1e9)
        result.update(scan=obs,fresh_target_detected=tag_id in obs['observed_ids'],pose_accepted=bool(trusted))
        if trusted:
            error=wrap(obs['heading_deg']-goal['tag_heading_deg'])
            result.update(error_deg=error,decision_fusion=dict(status='observed',source='trusted_tag_heading',
                       error_deg=error,target_heading_deg=goal['tag_heading_deg'],predicted_only=False))
            if abs(error)<=v['deadband_deg'] and clock()<=deadline:
                result.update(status='heading_accepted_near_target',heading_aligned=True,
                              heading_acceptance='passive_tag_heading',
                              heading_acceptance_limit_deg=v['deadband_deg'],
                              reason='fresh calibrated Tag heading within arrival deadband without magnetic feedback')
                return result
            result['reason']='Tag heading outside arrival deadband without magnetic feedback'
            if accept_trusted_residual:return result
        else:
            result.pop('error_deg',None)
            result.pop('decision_fusion',None)
            result['reason']='no fresh verified Tag heading'
        after_ns=now_ros
    if last_issue:
        stop(last_issue)
    return result


class VisualMicro:
    def __init__(self,cfg,io,reference,budget,action,check,clock,load='single',motion_model=None,*,
                 defer_predicted_overshoot=False,allow_missing_magnetic=False,allow_double=False):
        validate(cfg)
        self.cfg,self.v,self.io=cfg,cfg['visual_micro'],io
        self.reference,self.budget,self.action,self.check,self.clock=reference,budget,action,check,clock
        self.load=load
        self.motion_model=motion_model
        self.defer_predicted_overshoot=defer_predicted_overshoot is True
        self.allow_missing_magnetic=allow_missing_magnetic is True
        self.allow_double=allow_double is True
        self.unreliable_turns=set()

    def predicted_overshoot(self,result,reason):
        if not self.defer_predicted_overshoot:self.stop(reason)
        result.update(status='partial',reason=reason+'; defer motion and retain fixed reference',
                      correction_deferred=True,heading_aligned=False)
        return result

    def stop(self,reason,*,report=None):
        self.budget.lock(reason);self.reference.anchor=None
        if report is not None:
            report.update(status='blocked',reason=reason,budget=self.budget.status())
        raise VisualMicroStopped('visual micro stopped at completed macro boundary: '+reason+'; no next route macro, no reverse retry or release; no native immediate stop',report=report)

    def magnetic_loss(self,reason,stage):
        if self.allow_missing_magnetic and self.cfg['magnetic'].get('capture_freshness_verified') is True:
            from heading_guard import MagneticDataUnavailable, MagneticLoss_IsRecoverable
            health=getattr(self.io,'magnetic_capture_health',lambda:{})()
            if MagneticLoss_IsRecoverable(health):
                raise MagneticDataUnavailable(reason,health,stage=stage)
        self.stop(reason)

    def pair(self,tag,after_ns,deadline,post=False):
        from heading_guard import MagneticDataUnavailable
        from magnetic_capture import MagneticRebindResult

        try:
            return self._pair_once(tag,after_ns,deadline,post)
        except MagneticDataUnavailable as first:
            retry=getattr(self.io,'retry_magnetic_after_action',None)
            reserve=self.cfg['settle_s']+self.cfg['observe_s']+self.cfg['magnetic']['collect_s']
            self.check()
            if (retry is None or self.clock()+reserve>=deadline
                    or retry() is not MagneticRebindResult.REBOUND):
                raise
            fresh_after=self.io.ros_now_ns()
            if type(fresh_after) is not int or fresh_after<after_ns:
                self.stop('ROS clock moved backwards during stationary magnetic retry')
            attempts=first.error_details['magnetic_sampling_attempts']
            try:
                pair=self._pair_once(tag,fresh_after,deadline,post)
            except MagneticDataUnavailable as second:
                second.error_details['magnetic_sampling_attempts']=attempts+second.error_details['magnetic_sampling_attempts']
                raise
            pair['magnetic_result']['magnetic_sampling_attempts']=attempts+[self.io.magnetic_capture_health()]
            return pair

    def _pair_once(self,tag,after_ns,deadline,post):
        self.check()
        self.io.set_magnetic_load(self.load)
        self.io.settle(self.cfg['settle_s'],self.check)
        after=self.clock()
        if after>=deadline:self.stop('visual checkpoint deadline expired')
        current=getattr(self.io,'visual_camera_status',None)
        retry_frames=(self.allow_missing_magnetic and self.defer_predicted_overshoot
                      and not self.budget.limit_total_angle and self.v['execute'])
        camera_rechecks=[]
        expected_camera=self.v['camera_info'].get('expected_fingerprint')
        while True:
            window=self.io.observe(tag,after_ns,min(self.cfg['observe_s'],deadline-after),self.check)
            camera=current() if current is not None else window.get('camera_info',{})
            if not retry_frames:
                if not window.get('camera_healthy'):self.stop('camera/AR stream fault, not ordinary Tag loss')
                if self.v['execute'] and not camera.get('ready'):
                    self.stop('actual CameraInfo calibration/source/frame/time unavailable or changed')
                break
            issue=VisualCamera_DiagnoseWindow(window,camera,expected_camera)
            if issue is None:break
            if not issue[2]:self.stop(issue[0])
            camera_rechecks.append({'reason':issue[0],'reason_zh':issue[1]})
            if (len(camera_rechecks)>self.v['max_reobserve_windows']
                    or self.clock()+self.cfg['observe_s']+self.cfg['magnetic']['collect_s']>=deadline):
                window=dict(window,observations=[])
                break
            new_after_ns=self.io.ros_now_ns()
            if type(new_after_ns) is not int or new_after_ns<after_ns:
                self.stop('ROS clock moved backwards during camera recheck')
            after_ns=new_after_ns
            after=self.clock()
        obs=summarize(window,tag,self.cfg,after_ns)
        if self.defer_predicted_overshoot and tag==1:
            obs.update(now_ns=window['now_ns'],after_ns=max(after_ns,window.get('after_ns',0)),
                       camera_healthy=window.get('camera_healthy'),contract_match=window.get('contract_match'))
        remaining=deadline-self.clock()
        if remaining<=0:self.stop('checkpoint time budget expired before magnetic verification')
        rows=self.io.magnetic_samples(after,min(self.cfg['magnetic']['collect_s'],remaining),self.check)
        self.check()
        probe=MagneticReference(self.cfg['magnetic'])
        result=probe.refresh(rows,0.,self.clock(),after,self.load,None)
        magnetic=probe.anchor
        stream_healthy=self.io.motion_capture_verified
        healthy=bool(magnetic and magnetic.get('capture_verified') and stream_healthy)
        if healthy and mad([float(r['heading_deg']) for r in rows],magnetic['raw_heading_deg'],True)>self.v['max_magnetic_mad_deg']:
            healthy=False;result={'status':'rejected','reason':'paired magnetic dispersion exceeds small-micro window bound'}
        if not healthy:
            if active(self.cfg):
                reason='new magnetic capture missing, changed, stale or invalid'
                insufficient=(result.get('reason')==MAGNETIC_INSUFFICIENT_SAMPLES
                              and len(rows)<self.cfg['magnetic']['min_samples']
                              and all(r.get('capture_source')=='verified_sensor_capture' for r in rows))
                if insufficient or (magnetic and magnetic.get('capture_verified') and not stream_healthy):
                    if self.allow_missing_magnetic and not window.get('contract_match') and not camera_rechecks:
                        self.stop('camera/AR stream fault, not ordinary Tag loss')
                    self.magnetic_loss(reason,'visual_post_action_heading' if post else 'visual_magnetic_pair')
                self.stop(reason)
            magnetic=None
        if obs.get('pose_usable') and not 0<=self.io.ros_now_ns()-obs['latest_stamp_ns']<=self.cfg['max_age_s']*1e9:
            obs=dict(obs,pose_usable=False,reason='target pose expired before paired sample completed')
        trusted=bool(obs.get('pose_usable') and (obs.get('position_usable') or obs.get('heading_only')) and camera.get('ready') and obs.get('heading_mad_deg',float('inf'))<=self.v['max_heading_mad_deg'])
        if magnetic:
            magnetic.update(decision_capture_times=[r.get('captured_monotonic') for r in rows],stable_samples=len(set(r.get('captured_monotonic') for r in rows)),calibration_id=rows[-1].get('calibration_id','') if rows else '',load_epoch=rows[-1].get('load_epoch') if rows else None,owner_observation_scope=self.motion_model.observation_scope if self.motion_model else ('',-1))
        # Ordinary Tag loss does not turn a verified completed action into a fault.
        # Fresh magnetic feedback must still pass direction/step/field checks below.
        return dict(scan=obs,magnetic=magnetic,magnetic_result=result,rows=rows,after=after,
                    camera_info=camera,trusted=trusted,camera_rechecks=camera_rechecks)

    def dispatch_check(self,pair,deadline,*,defer_expired=False):
        self.check()
        if self.clock()>=deadline:
            if defer_expired:raise _VisualObservationExpired
            self.stop('visual deadline expired before dispatch')
        obs=pair['scan'];rows=pair.get('rows',[])
        live_visual=getattr(self.io,'visual_motion_ready',None)
        camera=self.io.visual_camera_status()
        tag_age=self.io.ros_now_ns()-obs['latest_stamp_ns']
        if (live_visual is None or not live_visual(obs['latest_stamp_ns']) or
                not 0<=tag_age<=self.cfg['max_age_s']*1e9 or
                camera.get('fingerprint')!=pair['camera_info'].get('fingerprint')):
            same_camera=(camera.get('fingerprint')==pair['camera_info'].get('fingerprint')
                         or camera.get('fingerprint') is None and camera.get('can_reobserve'))
            if defer_expired and tag_age>=0 and same_camera and (camera.get('ready') or camera.get('can_reobserve')):
                raise _VisualObservationExpired
            self.stop('target window expired or live visual source/calibration changed before dispatch')
        now=self.clock();age=self.cfg['magnetic']['max_age_s']
        if not self.io.motion_capture_verified:
            self.magnetic_loss('paired magnetic decision window expired or current capture stream invalid before dispatch',
                               'visual_dispatch')
        if not rows or not all(
                type(r.get('captured_monotonic')) in (int,float) and pair['after']<r['captured_monotonic']<=now and
                0<=now-r['captured_monotonic']<=age and
                type(r.get('received_monotonic')) in (int,float) and 0<=now-r['received_monotonic']<=age for r in rows):
            if defer_expired and rows and all(
                    type(r.get('captured_monotonic')) in (int,float) and pair['after']<r['captured_monotonic']<=now
                    and type(r.get('received_monotonic')) in (int,float) and 0<=now-r['received_monotonic']
                    for r in rows):
                raise _VisualObservationExpired
            self.stop('paired magnetic decision window expired or current capture stream invalid before dispatch')

    def _defer_expired_observation(self,result):
        result.update(stationary_reobserve_required=True,correction_deferred=True,budget=self.budget.status(),
                      reason='visual decision expired; take a fresh stationary observation',
                      reason_zh='扫码纠偏读数已过期，原地重新读取。')
        return result

    def _reobserve_disputed_tag(self,tag_id,pair,deadline,report):
        """Check the same Tag once more at rest without issuing another turn."""
        remaining=deadline-self.clock()
        if remaining<=0:return None
        self.check()
        after_ns=self.io.ros_now_ns()
        if type(after_ns) is not int or after_ns<pair['scan']['latest_stamp_ns']:
            self.stop('ROS clock moved backwards during Tag recheck',report=report)
        window=self.io.observe(tag_id,after_ns,min(self.cfg['observe_s'],remaining),self.check)
        self.check()
        camera=self.io.visual_camera_status()
        issue=VisualCamera_DiagnoseWindow(window,camera,pair['camera_info']['fingerprint'])
        if issue is not None:
            if not issue[2]:self.stop(issue[0],report=report)
            return None
        scan=summarize(window,tag_id,self.cfg,after_ns)
        if (not scan.get('pose_usable') or not (scan.get('heading_only') or scan.get('position_usable'))
                or scan.get('heading_mad_deg',float('inf'))>self.v['max_heading_mad_deg']):
            return None
        return scan,camera

    def _commit_accepted_heading(self,goal,pair,deadline,result,visual_basis_id,*,prefer_tag,field_feedback):
        near_target=result['status']=='heading_accepted_near_target'
        if pair['magnetic'] is None:
            result['reason']='heading within release but magnetic sample unavailable; reference not updated'
            return result
        if active(self.cfg):
            try:self.dispatch_check(pair,deadline,defer_expired=field_feedback)
            except _VisualObservationExpired:
                if near_target:result['status']='partial'
                return self._defer_expired_observation(result)
        old=self.reference.anchor
        if old and (old.get('source')=='relative_stationary_segment_start' or
                    prefer_tag and old.get('load')==self.load):
            if abs(pair['magnetic']['norm']-old['norm'])/old['norm']>self.cfg['magnetic']['max_norm_fraction_change']:
                if near_target:
                    self.reference.anchor=None
                    result.update(heading_aligned=True,magnetic={'status':'not_updated',
                                  'reason':'magnetic magnitude changed; accept verified Tag heading alone'})
                    self.check()
                    return result
                if active(self.cfg):self.stop('magnetic magnitude changed before reliable visual reanchor')
                result['reason']='magnetic magnitude changed; no visual reanchor'
                return result
            self.reference.anchor=None
        obs=pair['scan']
        actual_heading=wrap(goal['route_heading_deg']+wrap(obs['heading_deg']-goal['tag_heading_deg']))
        refreshed=self.reference.refresh(pair['rows'],actual_heading,self.clock(),pair['after'],self.load,goal['id'])
        if refreshed.get('status')!='updated' or not (self.reference.anchor or {}).get('capture_verified'):
            self.reference.anchor=None
            if active(self.cfg) and not near_target:
                self.stop('visual alignment could not establish consistent fresh magnetic reference')
        if self.reference.anchor is not None:
            self.reference.anchor.update(visual_basis_id=visual_basis_id,
                owner_observation_scope=self.motion_model.observation_scope if self.motion_model else ('',-1))
        result.update(heading_aligned=True,magnetic=refreshed)
        if not near_target:
            result['reason']='reliable heading aligned; magnetic reference uses observed residual; x/y remain diagnostic, obstacle entry not authorized'
        self.check()
        return result

    def checkpoint(self,tag_id,goal,*,after_ns=0,deadline=None):
        default_deadline=self.clock()+self.cfg['checkpoint_timeout_s']
        deadline=default_deadline if deadline is None else min(default_deadline,deadline)
        events=[]
        result=dict(status='partial',expected_id=tag_id,reason='',events=events,correction_actions=0,search_actions=0,
            search_scope='bounded stationary new-frame observations, not a physical sweep',heading_aligned=False,
            position_controlled=False,ready_for_obstacle=False,physical_pose_confirmed=False,
            detector_quality='actual fresh ALVAR pose and CameraInfo only; corners/reprojection/exposure not available',
            magnetic={'status':'not_updated','reason':'no reliable visual alignment'})
        if type(tag_id) is not int or tag_id!=goal.get('id') or not self.v['target_id_confirmed'] or tag_id!=self.v.get('confirmed_target_id'):
            result.update(status='blocked',reason='explicit target ID missing/mismatched; never select first observed ID',scan={'ids':[],'observed_ids':[],'observations':[]})
            return result
        pair=self.pair(tag_id,after_ns,deadline);obs=pair['scan'];windows=1
        events.extend({'camera_recheck':issue} for issue in pair.get('camera_rechecks',()))
        fresh_target_seen=tag_id in obs['observed_ids']
        while obs['status']=='no_tag' and windows<=self.v['max_reobserve_windows'] and self.clock()<deadline:
            events.append({'stationary_reobserve':windows,'body_commands':0})
            pair=self.pair(tag_id,self.io.ros_now_ns(),deadline);obs=pair['scan'];windows+=1
            events.extend({'camera_recheck':issue} for issue in pair.get('camera_rechecks',()))
            fresh_target_seen=fresh_target_seen or tag_id in obs['observed_ids']
        result.update(scan=obs,observation_windows=windows,camera_info=pair['camera_info'],budget=self.budget.status(),
                      scan_attempted=True,fresh_target_detected=fresh_target_seen,
                      pose_accepted=pair['trusted'],correction_executed=False)
        if not pair['trusted']:
            result.update(status='partial' if obs['status']=='no_tag' else 'blocked',
                reason='bounded ordinary Tag loss; return original route' if obs['status']=='no_tag' else 'target pose/geometry/CameraInfo untrusted; no correction')
            return result
        error=wrap(obs['heading_deg']-goal['tag_heading_deg'])
        visual_before_error=error
        magnetic_before_error=error  # fresh Tag heading defines the starting point for relative magnetic turn verification
        residual_driven=not self.budget.limit_total_angle and self.motion_model is not None
        field_feedback=residual_driven and self.allow_missing_magnetic and self.defer_predicted_overshoot
        prefer_tag_heading=(self.defer_predicted_overshoot and residual_driven and self.v['execute']
                            and tag_id==1 and goal['tag_heading_deg']==0)
        visual_basis_id=hashlib.sha256(json.dumps(dict(reference=self.cfg.get('relative_heading_reference'),camera=self.v.get('camera_info'),goal=goal),sort_keys=True).encode()).hexdigest()
        anchor=self.reference.anchor
        magnetic_reference_error=None
        if (anchor and anchor.get('source')=='reliable_tag_alignment' and anchor.get('tag_id')==tag_id and anchor.get('load')==self.load and anchor.get('capture_verified') and anchor.get('calibration_id') and anchor.get('calibration_id')==pair['magnetic'].get('calibration_id') and anchor.get('capture_session')==pair['magnetic'].get('capture_session') and type(anchor.get('load_epoch')) is int and anchor.get('load_epoch')==pair['magnetic'].get('load_epoch') and anchor.get('visual_basis_id')==visual_basis_id and self.motion_model is not None and anchor.get('owner_observation_scope')==self.motion_model.observation_scope and 0<=self.clock()-anchor['received_monotonic']<=self.cfg['post_action_heading']['max_reference_age_s']):
            if abs(pair['magnetic']['norm']-anchor['norm'])/anchor['norm']>self.cfg['magnetic']['max_norm_fraction_change']:self.stop('magnetic magnitude changed before fused visual decision')
            magnetic_reference_error=wrap(self.cfg['magnetic']['heading_sign']*pair['magnetic']['raw_heading_deg']+anchor['offset_deg']-goal['route_heading_deg'])
        if prefer_tag_heading:
            result['decision_fusion']=dict(status='observed',source='trusted_tag_heading',error_deg=error,
                target_heading_deg=goal['tag_heading_deg'],magnetic_reference_error_deg=magnetic_reference_error,
                reason='Tag 1 defines the absolute heading target. Magnetic feedback verifies physical turn changes.',
                predicted_only=False)
        elif magnetic_reference_error is not None:
            from motion_prediction import fuse_heading_errors
            now=self.clock();basis='tag:'+str(tag_id)+':route:'+str(goal['route_heading_deg'])
            magnetic_before_error=magnetic_reference_error
            fused=fuse_heading_errors(dict(error_deg=magnetic_before_error,captured_at_s=min(r['captured_monotonic'] for r in pair['rows']),mad_deg=mad([float(r['heading_deg']) for r in pair['rows']],pair['magnetic']['raw_heading_deg'],True),target_basis=basis,valid=True,capture_verified=True,field_valid=True),dict(error_deg=error,captured_at_s=now-max(0.,(self.io.ros_now_ns()-obs['latest_stamp_ns'])/1e9),mad_deg=obs['heading_mad_deg'],target_basis=basis,valid=True,capture_verified=self.cfg['pose_contract']['capture_time_confirmed'],tag_id=tag_id,pose_usable=True),now_s=now,target_basis=basis,expected_tag_id=tag_id,max_age_s=min(self.cfg['magnetic']['max_age_s'],self.cfg['max_age_s']),max_skew_s=min(.5,self.cfg['magnetic']['max_age_s']),max_disagreement_deg=self.v['max_sensor_disagreement_deg'])
            if fused.get('reason')=='observation_time_skew':
                fused=dict(status='observed',error_deg=magnetic_before_error,source='fresh_magnetic_only',reason='visual pairing time skew; no fusion',predicted_only=False)
            result['decision_fusion']=fused
            if fused.get('status')!='observed':
                result.update(reason='same-target visual/magnetic observations disagree; no new correction, continue original route',status='partial');return result
            error=fused['error_deg']
        result.update(error_deg=error,x_m=obs.get('x_m'),forward_m=obs.get('forward_m'),position_note='x/y diagnostics only; no R3 lateral gain, no center/y=.60H target assumed')
        heading_errors=(error,visual_before_error,magnetic_before_error)
        aligned=all(abs(e)<=self.v['release_deg'] for e in heading_errors)
        if not aligned:
            if not self.v['execute']:
                result.update(status='blocked',reason='visual micro execution disabled');return result
            if self.load!='single' and not (self.allow_double and self.load=='double'):
                self.stop('visual micro requires expected single load')
            if not field_feedback and max(abs(error),abs(visual_before_error),abs(magnetic_before_error))>self.v['max_error_deg']:
                result.update(status='blocked',reason='visual heading outside bounded correction envelope');return result
            if abs(error)<=(self.v['release_deg'] if residual_driven else self.v['deadband_deg']):
                result['reason']='inside trigger deadband but outside release; no claim of alignment';return result
            if goal.get('allow_turn') is not True:
                result.update(status='blocked',reason='configured checkpoint forbids turns; no micro budget consumed');return result
            delta_map=self.v['signed_nominal_delta_deg']
            name=next(n for n,d in delta_map.items() if d*error<0);delta=delta_map[name]
            if field_feedback and name in self.unreliable_turns:
                result.update(reason='previous turn response unrepresentative; retain verified Tag residual',
                              correction_deferred=True)
                return result
            choice=None;bound=abs(delta)
            if self.motion_model is not None:
                from motion_prediction import select_turn
                session,epoch=self.motion_model.observation_scope
                choice=select_turn(error_deg=error,captured_at_s=self.clock()-max(0.,(self.io.ros_now_ns()-obs['latest_stamp_ns'])/1e9),now_s=self.clock(),candidates={n:dict(direction='left' if 'left' in n else 'right',action_type='micro',nominal_delta_deg=d,max_step_deg=self.v['max_observed_step_deg'],bootstrap_upper_deg=response_limit(self.cfg['post_action_heading']) if response_limit(self.cfg['post_action_heading'])>8 else abs(d),adaptive_feedback=response_limit(self.cfg['post_action_heading'])>8,bounded_crossing_feedback=residual_driven,stop_on_possible_overshoot=self.cfg['post_action_heading'].get('stop_on_possible_overshoot',False)) for n,d in delta_map.items()},estimator=self.motion_model.turn_estimator,load=self.load,session=session,epoch=epoch,calibration_id=pair['magnetic'].get('calibration_id',''),max_age_s=self.cfg['max_age_s'],deadband_deg=self.v['release_deg'] if residual_driven else self.v['deadband_deg'],release_deg=.5 if residual_driven else self.v['release_deg'])
                result['motion_prediction']=choice
                if choice.get('status')!='selected':
                    if (prefer_tag_heading
                            and choice.get('status')=='no_safe_candidate'
                            and choice.get('rejected',{}).get(name)=='insufficient_worst_case_improvement'
                            and all(abs(e)<=self.v['deadband_deg'] for e in heading_errors)):
                        result.update(status='heading_accepted_near_target',
                                      heading_acceptance='near_target_without_improving_step',
                                      heading_acceptance_limit_deg=self.v['deadband_deg'],
                                      reason='Verified Tag heading within arrival deadband. No available turn safely improves it.')
                        return self._commit_accepted_heading(goal,pair,deadline,result,visual_basis_id,
                                                             prefer_tag=prefer_tag_heading,field_feedback=field_feedback)
                    if self.cfg['post_action_heading'].get('stop_on_possible_overshoot') and 'possible_overshoot' in choice.get('rejected',{}).values():
                        return self.predicted_overshoot(result,'possible visual overshoot predicted')
                    result['reason']='no safe predicted visual correction';return result
                name=choice['action'];delta=delta_map[name];bound=abs(choice['budget_delta_deg'])
            if residual_driven:
                bounds=(choice['lower_abs_deg'],choice['upper_abs_deg'])
                if any(e*delta>=0 or max(abs(abs(e)-b) for b in bounds)>abs(e)-self.v['minimum_improvement_deg']
                       for e in heading_errors):
                    result.update(reason='no one step improves both live heading estimates',correction_deferred=True)
                    return result
            if not residual_driven and self.cfg['post_action_heading'].get('stop_on_possible_overshoot') and (choice['upper_abs_deg'] if choice else abs(delta))>min(abs(error),abs(visual_before_error),abs(magnetic_before_error)):
                return self.predicted_overshoot(result,'possible overshoot of visual or magnetic target')
            if not (choice and choice.get('adaptive_interval_verified') is True) and (choice['upper_abs_deg'] if choice else abs(delta))>abs(error)-self.v['release_deg']:
                result['reason']='predicted step could cross release margin';return result
            if not self.budget.available(delta,conservative_bound=bound):
                result.update(reason='shared magnetic/visual journey budget exhausted; visual correction deferred',correction_deferred=True);return result
            before=pair
            try:self.dispatch_check(before,deadline,defer_expired=field_feedback)
            except _VisualObservationExpired:return self._defer_expired_observation(result)
            reserved=False
            if not field_feedback and not self.budget.reserve('visual',name,delta,conservative_bound=bound):
                result['reason']='shared budget unavailable';return result

            def verify_before_turn():
                nonlocal reserved
                self.dispatch_check(before,deadline,defer_expired=field_feedback)
                if field_feedback and not reserved:
                    if not self.budget.reserve('visual',name,delta,conservative_bound=bound):
                        raise _VisualTurnNotReserved
                    reserved=True

            turn_started=self.clock()
            try:
                done=self.action(name,verify_before_turn)
            except _VisualObservationExpired:
                return self._defer_expired_observation(result)
            except _VisualTurnNotReserved:
                result.update(reason='shared budget unavailable',correction_deferred=True,budget=self.budget.status())
                return result
            turn_completed=self.clock()
            if done is None:
                reason=('remaining action/time budget unavailable; visual correction deferred' if field_feedback and not reserved
                        else 'remaining action/time budget unavailable; reserved micro not replayed')
                result.update(reason=reason,budget=self.budget.status());return result
            if field_feedback and not reserved:self.stop('visual correction completion missing pre-dispatch verification')
            result['correction_executed']=True
            after_ns=done.get('after_ns')
            if type(after_ns) is not int or after_ns<=0:self.stop('micro completion ROS barrier unavailable')
            pair=self.pair(tag_id,after_ns,deadline,post=True);obs=pair['scan']
            events.extend({'camera_recheck':issue} for issue in pair.get('camera_rechecks',()))
            b=before['magnetic'];a=pair['magnetic']
            if not a or not b:self.stop('paired magnetic verification unavailable')
            if field_feedback and (not a.get('capture_verified')
                    or any(a.get(key)!=b.get(key) for key in ('capture_session','load_epoch','calibration_id','owner_observation_scope'))
                    or min(a['decision_capture_times'])<=max(b['decision_capture_times'])):
                self.stop('paired magnetic observation scope or capture order changed across visual micro')
            if abs(a['norm']-b['norm'])/b['norm']>self.cfg['magnetic']['max_norm_fraction_change']:self.stop('magnetic magnitude changed across micro')
            magnetic_delta=wrap(self.cfg['magnetic']['heading_sign']*wrap(a['raw_heading_deg']-b['raw_heading_deg']))
            magnetic_error=wrap(magnetic_before_error+magnetic_delta)
            unusual_response=(magnetic_delta*delta<=0 or abs(magnetic_delta)>self.v['max_observed_step_deg']
                              or abs(magnetic_error)>abs(magnetic_before_error)-self.v['minimum_improvement_deg'])
            if field_feedback and unusual_response:
                self.unreliable_turns.add(name)
                result['observed_turn_deviation']={'action':name,'magnetic_delta_deg':magnetic_delta,
                                                  'learning_used':False}
            if not residual_driven and self.cfg['post_action_heading'].get('stop_on_possible_overshoot') and magnetic_before_error*magnetic_error<0:
                self.stop('observed magnetic target crossing after visual micro; no reverse or continuation')
            if response_limit(self.cfg['post_action_heading'])>8 and not self.budget.reconcile(
                    abs(magnetic_delta),feedback_only_response=field_feedback):
                self.stop('observed angle budget exhausted')
            if unusual_response and not field_feedback:
                self.stop('magnetic wrong-direction, oversized or less than one degree improvement in visual correction')
            if not pair['trusted']:
                self.reference.anchor=None
                result.update(scan=obs,after_error_deg=magnetic_error,correction_actions=1,budget=self.budget.status(),reason='ordinary target loss/untrusted pose after completed micro; fresh magnetic feedback verified; continue original route without visual reanchor',prediction_update=dict(predicted_error_deg=choice['predicted_error_deg'] if choice else wrap(error+delta),observed_error_deg=magnetic_error,source='fresh_magnetic_relative_to_last_verified_visual',prediction_not_new_sample=True))
                if self.motion_model and not unusual_response:
                    result['turn_observation']=self.motion_model.observe_turn(name,self.load,b,a,turn_started,turn_completed)
                return result
            visual_delta=wrap(obs['heading_deg']-before['scan']['heading_deg'])
            new_error=wrap(obs['heading_deg']-goal['tag_heading_deg'])
            if not residual_driven and self.cfg['post_action_heading'].get('stop_on_possible_overshoot') and visual_before_error*new_error<0:
                self.stop('observed visual target crossing; no reverse or continuation')
            events.append(dict(action=name,angle_evidence='source_nominal',nominal_delta_deg=delta,
                visual_delta_deg=visual_delta,magnetic_delta_deg=magnetic_delta,after_error_deg=new_error,
                physical_inflight_angle_limit_guaranteed=False))
            if not field_feedback and (visual_delta*delta<=0 or abs(visual_delta)>self.v['max_observed_step_deg']):
                self.stop('visual or magnetic wrong-direction/oversized correction')
            if not field_feedback and abs(new_error)>abs(visual_before_error)-self.v['minimum_improvement_deg']:
                self.stop('visual or magnetic correction did not improve by one degree')
            disagreement_limit=self.v['max_sensor_disagreement_deg']
            delta_gap=abs(wrap(visual_delta-magnetic_delta))
            if delta_gap>disagreement_limit:
                fused=dict(status='disagreement',reason='turn_delta_disagreement',error_deg=None)
            else:
                from motion_prediction import fuse_heading_errors
                now=self.clock();basis='tag:'+str(tag_id)+':goal:'+str(goal['tag_heading_deg'])
                fused=fuse_heading_errors(dict(error_deg=magnetic_error,captured_at_s=min(r['captured_monotonic'] for r in pair['rows']),mad_deg=mad([float(r['heading_deg']) for r in pair['rows']],a['raw_heading_deg'],True),target_basis=basis,valid=True,capture_verified=a.get('capture_verified',False),field_valid=True),dict(error_deg=new_error,captured_at_s=now-max(0.,(self.io.ros_now_ns()-obs['latest_stamp_ns'])/1e9),mad_deg=obs.get('heading_mad_deg',float('inf')),target_basis=basis,valid=True,capture_verified=self.cfg['pose_contract']['capture_time_confirmed'],tag_id=tag_id,pose_usable=pair['trusted']),now_s=now,target_basis=basis,expected_tag_id=tag_id,max_age_s=min(self.cfg['magnetic']['max_age_s'],self.cfg['max_age_s']),max_skew_s=min(.5,self.cfg['magnetic']['max_age_s']),max_disagreement_deg=disagreement_limit)
            if fused.get('reason')=='observation_time_skew' and not field_feedback:
                fused=dict(status='observed',error_deg=magnetic_error,source='fresh_magnetic_only',reason='visual pairing time skew; no fusion',predicted_only=False)
            result['prediction_update']=dict(predicted_error_deg=choice['predicted_error_deg'] if choice else wrap(error+delta),visual_error_deg=new_error,magnetic_error_deg=magnetic_error,fusion=fused,prediction_not_new_sample=True)
            result.update(scan=obs,after_error_deg=new_error,correction_actions=1,budget=self.budget.status(),
                          camera_info=pair['camera_info'],x_m=obs.get('x_m'),forward_m=obs.get('forward_m'))
            if fused.get('status')=='disagreement':
                comparison=dict(tag_id=tag_id,action=name,reason=fused['reason'],visual_delta_deg=visual_delta,
                    magnetic_delta_deg=magnetic_delta,delta_disagreement_deg=delta_gap,limit_deg=disagreement_limit,
                    visual_heading_deg=obs['heading_deg'],previous_visual_heading_deg=before['scan']['heading_deg'],
                    magnetic_heading_deg=a['raw_heading_deg'],previous_magnetic_heading_deg=b['raw_heading_deg'],
                    visual_residual_deg=new_error,magnetic_residual_deg=magnetic_error,learning_used=False,
                    visual_stamp_ns=obs['latest_stamp_ns'],previous_visual_stamp_ns=before['scan']['latest_stamp_ns'],
                    magnetic_capture_times=a['decision_capture_times'],previous_magnetic_capture_times=b['decision_capture_times'])
                result['heading_disagreement']=comparison
                if not field_feedback:
                    reason=('visual and magnetic heading changes disagree' if fused['reason']=='turn_delta_disagreement'
                            else 'paired same-target fusion rejected: '+str(fused['reason']))
                    self.stop(reason,report=result)
                if pair['camera_info'].get('fingerprint')!=before['camera_info'].get('fingerprint'):
                    self.stop('actual CameraInfo calibration/source/frame changed',report=result)
                self.unreliable_turns.add(name)
                self.reference.anchor=None
                review_complete=True
                if delta_gap>self.v['deadband_deg']:
                    refreshed=self._reobserve_disputed_tag(tag_id,pair,deadline,result)
                    review_complete=False
                    if refreshed is not None:
                        fresh_scan,camera=refreshed
                        review_complete=(abs(wrap(fresh_scan['heading_deg']-obs['heading_deg']))<=disagreement_limit)
                        new_error=wrap(fresh_scan['heading_deg']-goal['tag_heading_deg'])
                        comparison.update(recheck_visual_heading_deg=fresh_scan['heading_deg'],
                                          recheck_visual_stamp_ns=fresh_scan['latest_stamp_ns'])
                        result.update(scan=fresh_scan,camera_info=camera,after_error_deg=new_error,
                                      x_m=fresh_scan.get('x_m'),forward_m=fresh_scan.get('forward_m'))
                    comparison['stationary_recheck']='confirmed' if review_complete else 'inconclusive'
                comparison['selected_source']='trusted_tag_heading' if review_complete else 'route_advisory'
                comparison['selected_residual_deg']=new_error
                aligned=review_complete and abs(new_error)<=self.v['deadband_deg']
                result.update(status='heading_accepted_near_target' if aligned else 'partial',
                    heading_aligned=aligned,heading_acceptance='trusted_tag_after_disagreement' if aligned else None,
                    correction_deferred=True,stationary_recheck_inconclusive=not review_complete,
                    reason='trusted Tag heading retained after sensor disagreement' if review_complete
                           else 'stationary Tag recheck inconclusive; retain route checkpoint as advisory',
                    reason_zh=(f'Tag {tag_id} 航向与磁力计不一致，采用可信 Tag 航向。' if review_complete
                               else f'Tag {tag_id} 已确认，转角复核未稳定，按原路线接续。'),
                    magnetic={'status':'not_updated','reason':'turn measurements disagree'})
                self.check()
                return result
            if fused.get('status')!='observed':
                self.stop('paired same-target fusion rejected: '+str(fused.get('reason')),report=result)
            result['fused_error_deg']=fused['error_deg']
            if self.motion_model and not unusual_response:
                result['turn_observation']=self.motion_model.observe_turn(name,self.load,b,a,turn_started,turn_completed)
            arrival_errors=(new_error,) if prefer_tag_heading else (new_error,magnetic_error,fused['error_deg'])
            aligned=all(abs(e)<=self.v['release_deg'] for e in arrival_errors)
            self.reference.anchor=None  # never retain the old reference and undo an intentional visual micro
        if aligned:
            return self._commit_accepted_heading(goal,pair,deadline,result,visual_basis_id,
                                                 prefer_tag=prefer_tag_heading,field_feedback=field_feedback)
        else:
            result['reason']=('verified turn response differed from prediction; recheck the remaining heading'
                if result.get('observed_turn_deviation') else 'fresh one-step improvement; next step must recheck both headings' if residual_driven
                else 'one bounded micro improved heading but release not reached; no retry or reverse; x/y diagnostic only')
        self.check();return result
