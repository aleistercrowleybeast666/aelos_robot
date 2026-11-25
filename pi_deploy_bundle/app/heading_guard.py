"""Post-action magnetic guard. No hardware imports; uncalibrated execution is closed."""
import math
import copy
import json
import micro_trial
from cmd0_motion import FORWARD_ACTIONS as CMD0_FORWARD_ACTIONS
from magnetic_capture import MagneticRebindResult
from nav_observation import MAGNETIC_INSUFFICIENT_SAMPLES, MagneticReference, wrap

CARRY_HEADING_ACTIONS = frozenset(('carry_forward','carry_forward2','carry_forward4',
                                  'carry_forward7','carry_forward9',*CMD0_FORWARD_ACTIONS.values()))
CORRIDOR_HEADING_ACTIONS = frozenset(('corridor_departure', 'corridor_side_boundary', 'pickup_retreat_complete'))
_ENTRY_STRAIGHT_LIMIT_DEG = 15.
_STATIONARY_HEADING_SPREAD_DEG = 3.
_CORRIDOR_MEDIUM_LEFT_TRIGGER_DEG = 15.
_CARRY_STATIONARY_RECHECK_WINDOWS = 2
_FEEDBACK_SCOPE_FIELDS = ('capture_session', 'load_epoch', 'owner_observation_scope', 'calibration_id')
_STREAM_INTERRUPTION_REASONS = frozenset(('capture_stream_unhealthy', 'sample_stale'))
_UNSTABLE_MAGNETIC_WINDOWS = frozenset(('magnetic heading unstable', 'magnetic field magnitude unstable within window'))
_CARRY_NO_TURN_REASONS = frozenset(('unreliable_or_oversized_prediction',
    'insufficient_worst_case_improvement', 'insufficient_predicted_improvement'))


def _carry_prediction_rejected(report):
    prediction = report.get('motion_prediction', {})
    rejected = set(prediction.get('rejected', {}).values()) - {'wrong_direction'}
    return prediction.get('status') == 'no_safe_candidate' and bool(rejected) and rejected <= _CARRY_NO_TURN_REASONS


def validate(cfg):
    c = cfg.get('post_action_heading', {})
    if not c:
        return
    if type(c.get('stop_on_possible_overshoot',False)) is not bool:
        raise ValueError('heading guard flag: stop_on_possible_overshoot')
    for k in ('enabled', 'execute', 'directions_confirmed', 'gain_confirmed', 'safe_stop_confirmed', 'strafe_gain_confirmed'):
        if type(c.get(k)) is not bool:
            raise ValueError('heading guard flag: ' + k)
    for k in ('deadband_deg', 'release_deg', 'max_error_deg', 'max_reference_age_s',
              'minimum_improvement_deg', 'max_step_deg', 'cooldown_s', 'settle_s'):
        if type(c.get(k)) not in (int, float) or not math.isfinite(c[k]) or c[k] <= 0:
            raise ValueError('heading guard limit: ' + k)
    if not 0 < c['release_deg'] < c['deadband_deg'] < c['max_error_deg'] < 90:
        raise ValueError('heading guard deadbands')
    if (micro_trial.response_limit(c)==8 and (c['max_step_deg']>c['deadband_deg'] or c['max_step_deg']>8)) or (micro_trial.response_limit(c)>8 and c['max_step_deg']!=micro_trial.response_limit(c)):
        raise ValueError('turn is not a bounded small correction')
    if type(c.get('max_actions_per_journey')) is not int or not 0 <= c['max_actions_per_journey'] <= 16:
        raise ValueError('heading journey budget')
    micro_trial.validate(cfg)
    if c['execute'] and not micro_trial.enabled(c):
        if not all(c[k] for k in ('directions_confirmed', 'gain_confirmed', 'safe_stop_confirmed')):
            raise ValueError('magnetic execution requires direction/gain/safe-stop validation')
        if not cfg['magnetic'].get('capture_freshness_verified',False):
            raise ValueError('sensor capture freshness unverified; receipt-only monitoring')
        if type(cfg['magnetic'].get('heading_sign')) is not int or cfg['magnetic']['heading_sign'] not in (-1, 1):
            raise ValueError('magnetic sign unverified')
        gains = c.get('turn_delta_deg', {})
        for name in ('left_turn', 'right_turn'):
            v = gains.get(name)
            if type(v) not in (int, float) or not math.isfinite(v) or not 0 < abs(v) <= c['max_step_deg']:
                raise ValueError('measured turn delta required: ' + name)
        if gains['left_turn'] * gains['right_turn'] >= 0:
            raise ValueError('turn directions must oppose')


class MicroTrialStopped(RuntimeError):
    pass


def MagneticLoss_IsRecoverable(health):
    return (isinstance(health,dict) and health.get('configured') is True
            and (not health.get('fault') or health.get('recoverable_interruption') is True))


class MagneticDataUnavailable(MicroTrialStopped):
    def __init__(self, reason, health, *, stage='post_action_heading', attempts=None):
        super().__init__(reason)
        self.health=health
        self.report=None
        self.guard_reason=reason
        self.error_details=dict(error_code='magnetic_data_unavailable',stage=stage,
            reason_zh='磁力计暂时无法提供有效的新样本。',magnetic_sampling_attempts=attempts or [health])


class HeadingGuard:
    def __init__(self, cfg, io, reference, check, clock, budget=None, estimator=None):
        validate(cfg)
        self.cfg = cfg
        self.c = cfg['post_action_heading']
        self.io, self.reference, self.check, self.clock = io, reference, check, clock
        self.pending_decision=None
        self.count = 0
        self.total_nominal_deg = 0.
        self.budget=budget if budget is not None else micro_trial.MicroBudget(max_step_deg=micro_trial.response_limit(self.c))
        self.last_action = float('-inf')
        self.latched = False
        self.blocked = False
        self.allow_missing_magnetic = False
        self._nominal_retry_used = False
        self._unreliable_coarse_actions = set()
        self._confirmed_coarse_response = None
        from turn_estimator import TurnEstimator
        self.turn_estimator=estimator or TurnEstimator(max_age_s=cfg["magnetic"]["max_age_s"],max_norm_change=cfg["magnetic"]["max_norm_fraction_change"],max_micro_deg=micro_trial.response_limit(self.c),allow_provisional=micro_trial.response_limit(self.c)>8)
        self.last_sample=None;self.pending_turn_observation=None;self.observation_scope=("",-1)

    def begin_turn_observation(self,action,load,started_at,session,epoch):
        self.observation_scope=(session,epoch)
        self.pending_turn_observation=dict(action=action,load=load,before=self.last_sample,started=started_at,completed=None)

    def finish_turn_observation(self,completed_at):
        if self.pending_turn_observation is not None:self.pending_turn_observation['completed']=completed_at

    def observe_turn(self,action,load,before,after,started,completed,*,confirmed_right_overrun=False):
        from turn_estimator import TurnWindow
        if not before or not after:return dict(accepted=False,reason='no_existing_pre_turn_window',estimate=None)
        if not before.get('capture_session') or before.get('capture_session')!=after.get('capture_session'):
            return dict(accepted=False,reason='capture_session_changed',estimate=None)
        anchor=self.reference.anchor
        if anchor and abs(after['norm']-anchor['norm'])/anchor['norm']>self.cfg['magnetic']['max_norm_fraction_change']:
            return dict(accepted=False,reason='reference_magnitude_changed',estimate=None)
        if before.get('owner_observation_scope')!=self.observation_scope or after.get('owner_observation_scope')!=self.observation_scope:
            return dict(accepted=False,reason='owner_session_or_epoch_changed',estimate=None)
        if type(before.get('load_epoch')) is not int or before.get('load_epoch')!=after.get('load_epoch'):
            return dict(accepted=False,reason='load_epoch_changed',estimate=None)
        def window(sample):
            session,epoch=sample['owner_observation_scope']
            return TurnWindow(heading_deg=(self.cfg['magnetic']['heading_sign']*sample['raw_heading_deg'])%360.,sample_time_s=min(sample.get('decision_capture_times') or [float('nan')]),norm=sample['norm'],session=session,epoch=epoch,load=sample['load'],calibration_id=sample.get('calibration_id',''),stable_samples=sample.get('stable_samples',0),capture_verified=sample.get('capture_verified',False),norm_basis=sample.get('norm_basis','raw_xyz_legacy'))
        evidence={'confirmed_right_overrun':True} if confirmed_right_overrun else {}
        action_type='medium' if action in micro_trial.MEDIUM_OPS else 'micro' if action in micro_trial.OPS else 'macro'
        return self.turn_estimator.observe(direction='left' if 'left' in action else 'right',action_type=action_type,before=window(before),after=window(after),started_at_s=started,completed_at_s=completed,now_s=self.clock(),protocol_completed=completed is not None,**evidence)

    def fault(self,reason,report=None):
        self.blocked=True
        self.budget.lock(reason)
        if micro_trial.active(self.cfg) and micro_trial.enabled(self.c):
            exc=MicroTrialStopped('micro trial stopped at completed macro boundary: '+reason+'; no remaining route, no reverse retry, no release; native immediate stop unavailable')
            exc.report=copy.deepcopy(report)
            exc.guard_reason=reason
            raise exc

    def verify_pending_decision(self):
        self.check()
        if not micro_trial.enabled(self.c):return
        sample=self.pending_decision
        if sample:
            self._defer_stream_interruption('magnetic decision interrupted before native submission')
        now=self.clock();age=self.cfg['magnetic']['max_age_s']
        stamps=(*sample.get('decision_capture_times',()),*sample.get('decision_receipt_times',())) if sample else ()
        if (stamps and all(type(t) in (int,float) and math.isfinite(t) and 0<=now-t for t in stamps)
                and any(now-t>age for t in stamps)):
            self._defer_magnetic_data('magnetic correction sample expired before dispatch',
                                      stage='magnetic_decision')
        if (not sample or not getattr(self.io,'motion_capture_verified',False) or
                not sample.get('decision_capture_times') or not all(
                    type(t) in (int,float) and sample['decision_after_monotonic']<t<=now and 0<=now-t<=age
                    for t in sample['decision_capture_times']) or not all(
                    type(t) in (int,float) and 0<=now-t<=age for t in sample.get('decision_receipt_times',[]))):
            self.fault('magnetic decision window expired or capture invalid immediately before native submission')

    def _defer_stream_interruption(self, reason, reasons=None):
        if not self.allow_missing_magnetic or getattr(self.io, 'capture_stream', None) is None:
            return
        if reasons is not None and (not reasons or not set(reasons)<=_STREAM_INTERRUPTION_REASONS):
            return
        if self.io.motion_capture_verified and 'sample_stale' not in (reasons or ()):
            return
        self._defer_magnetic_data(reason)

    def _defer_magnetic_data(self,reason,*,stage='post_action_heading',report=None):
        if self.allow_missing_magnetic:
            health=getattr(self.io,'magnetic_capture_health',lambda:{})()
            if MagneticLoss_IsRecoverable(health):
                exc=MagneticDataUnavailable(reason,health,stage=stage)
                exc.report=copy.deepcopy(report)
                if report and report.get('reason_zh'):
                    exc.error_details['reason_zh']=report['reason_zh']
                raise exc

    def _defer_expired_target(self, failures, report=None):
        if ('target_reference_expired' in failures
                and set(failures)-{'target_reference_expired'} <= _STREAM_INTERRUPTION_REASONS):
            reason = 'magnetic heading reference expired before feedback could be verified'
            advisory = dict(report or {}, reason=reason, status='magnetic_reference_expired',
                            reason_zh='磁航向参考过期，按预定路线进入下一处扫码点。')
            self._defer_magnetic_data(reason,stage='magnetic_reference_age',report=advisory)

    def sample(self, load, action_completed_monotonic=None, *, stationary_retry=False):
        rebind = getattr(self.io, 'retry_magnetic_after_action', None)
        if ((action_completed_monotonic is None and not stationary_retry) or rebind is None or
                not (micro_trial.active(self.cfg) and micro_trial.enabled(self.c))):
            return self._sample_once(load, action_completed_monotonic)

        attempts = []
        reobserve_after = None
        for attempt in range(2):
            result, sample = self._sample_once(load, action_completed_monotonic,
                                               defer_missing=True, reobserve_after=reobserve_after)
            unstable = result.get('window_issue') in _UNSTABLE_MAGNETIC_WINDOWS
            if result.get('reason') != MAGNETIC_INSUFFICIENT_SAMPLES and not unstable:
                if attempts:
                    attempts.append(self.io.magnetic_capture_health())
                    result['magnetic_sampling_attempts'] = attempts
                return result, sample
            health = self.io.magnetic_capture_health()
            attempts.append(dict(health, window_issue=result['window_issue'])
                            if result.get('window_issue') else health)
            self.check()
            if attempt == 0 and (not health.get('fault') or rebind() is MagneticRebindResult.REBOUND):
                reobserve_after = self.clock()
                continue

            fault = health.get('fault')
            if fault == 'producer_restarted':
                detail = '磁力计采集进程已重启，先前的航向基准需要重新确认。'
            elif health.get('recoverable_interruption') and len(attempts) > 1:
                detail = '磁力计采样中断，重采后仍未取得足够的新样本。'
            elif health.get('recoverable_interruption'):
                detail = '磁力计采样中断，未能重新建立采样窗口。'
            elif fault:
                detail = '磁力计数据来源或采样校验未通过。'
            elif unstable:
                detail = '两次静止采样的磁读数仍不稳定。'
            else:
                detail = '磁力计重采后仍未取得足够的新样本。'
            reason = 'fresh magnetic sample unavailable: '+str(result)
            if self.allow_missing_magnetic and MagneticLoss_IsRecoverable(health):
                raise MagneticDataUnavailable(reason,health,
                    stage='stationary_heading' if stationary_retry or unstable else 'post_action_heading',attempts=attempts)
            try:
                self.fault(reason)
            except MicroTrialStopped as exc:
                exc.error_details = {
                    'error_code': 'magnetic_post_action_unavailable',
                    'stage': 'post_action_heading',
                    'reason_zh': '动作收到完成回执。'+detail,
                    'magnetic_sampling_attempts': attempts,
                }
                raise

    def _sample_once(self, load, action_completed_monotonic, *, defer_missing=False, reobserve_after=None):
        if hasattr(self.io,'set_magnetic_load'):self.io.set_magnetic_load(load)
        self.check()
        scope=self.observation_scope
        capture=getattr(self.io,'capture_stream',None)
        capture_session=getattr(capture,'session',None)
        load_epoch=getattr(self.io,'magnetic_load_epoch',None)
        now=self.clock()
        if action_completed_monotonic is None:
            self.io.settle(self.c['settle_s'], self.check)
            after=self.clock()
        else:
            if (type(action_completed_monotonic) not in (int,float) or
                    not math.isfinite(action_completed_monotonic) or
                    not 0<=action_completed_monotonic<=now):
                self.fault('invalid true action-completion boundary')
                return {'status':'rejected','reason':'invalid completion boundary'},None
            after=action_completed_monotonic+self.c['settle_s']
            remaining=max(0.,after-now)
            if remaining:self.io.settle(remaining,self.check)
            # A past boundary permits only already-acquired POST-settle frames,
            # never motion frames, receipts-as-capture, or expired history.
            after=max(after,self.clock()-self.cfg['magnetic']['max_age_s'])
        if reobserve_after is not None:
            after=max(after,reobserve_after)
        rows = self.io.magnetic_samples(after, self.cfg['magnetic']['collect_s'], self.check)
        self.check()
        if (self.observation_scope!=scope or getattr(self.io,'capture_stream',None) is not capture or
                (capture_session is not None and getattr(capture,'session',None)!=capture_session) or
                (load_epoch is not None and getattr(self.io,'magnetic_load_epoch',None)!=load_epoch)):
            self.fault('magnetic owner, capture session or load epoch changed during collection')
            return {'status':'rejected','reason':'observation scope changed'},None
        if any(r.get('expected_load',load)!=load for r in rows):
            self.fault('magnetic expected load changed')
            return {'status':'rejected','reason':'different expected load'},None
        now=self.clock()
        if action_completed_monotonic is not None or capture is not None:
            def eligible(row):
                t=row.get('captured_monotonic')
                return (type(t) in (int,float) and after<t<=now and
                        0<=now-t<=self.cfg['magnetic']['max_age_s'] and
                        row.get('capture_source')=='verified_sensor_capture' and
                        (capture_session is None or row.get('session')==capture_session) and
                        (load_epoch is None or row.get('load_epoch')==load_epoch))
            rows=[r for r in rows if eligible(r)]
            for key in ('session','load_epoch','calibration_id'):
                values={r.get(key) for r in rows}
                if len(values)>1:
                    self.fault('mixed magnetic '+key+' in post-action window')
                    return {'status':'rejected','reason':'mixed '+key},None
        probe = MagneticReference(self.cfg['magnetic'])
        result = probe.refresh(rows, 0., now, after, load, None)
        if result['status'] != 'updated':
            if (self.allow_missing_magnetic and result.get('reason') in _UNSTABLE_MAGNETIC_WINDOWS and rows
                    and all(r.get('capture_source')=='verified_sensor_capture' for r in rows)):
                if defer_missing:
                    return dict(result,window_issue=result['reason']),None
                self._defer_magnetic_data(result['reason'],stage='stationary_heading')
            if not (defer_missing and result.get('reason') == MAGNETIC_INSUFFICIENT_SAMPLES):
                self.fault('fresh magnetic sample unavailable: '+str(result))
            return result, None
        captured = [row.get('captured_monotonic') for row in rows]
        capture_scoped = bool(rows) and all(
            row.get('capture_source') == 'verified_sensor_capture'
            and type(stamp) in (int,float) and after < stamp <= now
            for row,stamp in zip(rows,captured))
        validation_time = self.clock()
        probe.anchor['capture_verified'] = capture_scoped and (
            validation_time-min(captured) <= self.cfg['magnetic']['max_age_s'])
        stream_unhealthy = capture is not None and not self.io.motion_capture_verified
        if capture_scoped and (not probe.anchor['capture_verified'] or stream_unhealthy):
            issue = ('capture_stream_became_unavailable' if stream_unhealthy
                     else 'capture_window_expired_during_validation')
            if defer_missing and MagneticLoss_IsRecoverable(self.io.magnetic_capture_health()):
                return {'status':'skipped','reason':MAGNETIC_INSUFFICIENT_SAMPLES,'window_issue':issue},None
            self._defer_magnetic_data(issue,stage='post_action_heading')
        if not probe.anchor['capture_verified'] or stream_unhealthy:
            self.fault('capture contract is unverified, stale, changed or unhealthy', {
                'magnetic_capture': getattr(self.io,'magnetic_capture_health',lambda:{})(),
                'capture_scoped': capture_scoped, 'stream_unhealthy': stream_unhealthy})
        probe.anchor.update(decision_after_monotonic=after,
            decision_capture_times=[r.get('captured_monotonic') for r in rows],
            decision_receipt_times=[r.get('received_monotonic') for r in rows])
        probe.anchor['stable_samples']=len(set(r.get('captured_monotonic') for r in rows))
        probe.anchor['calibration_id']=rows[-1].get('calibration_id','') if rows else ''
        probe.anchor['owner_observation_scope']=scope
        probe.anchor['action_completed_monotonic']=action_completed_monotonic
        probe.anchor['load_epoch']=rows[-1].get('load_epoch') if rows else None
        self.last_sample=dict(probe.anchor)
        recovered=[r['recovered_error_detail'] for r in rows if isinstance(r.get('recovered_error_detail'),dict)]
        if recovered:
            result['recovered_magnetic_errors']=recovered
        return result, probe.anchor

    def _pickup_approach_target(self, sample, approach):
        if (approach is None or approach.get('load')!='empty' or sample['load'] not in ('single','double')
                or not approach.get('capture_verified') or not approach.get('usable_for_heading')
                or not sample.get('capture_verified') or not sample.get('usable_for_heading')):
            return None
        if not all(approach.get(key) and approach[key]==sample.get(key)
                   for key in ('capture_session','owner_observation_scope','calibration_id','norm_basis')):
            return None
        if not (type(approach.get('load_epoch')) is int and type(sample.get('load_epoch')) is int
                and approach['load_epoch']<sample['load_epoch']
                and 0.<sample['received_monotonic']-approach['received_monotonic']<=self.c['max_reference_age_s']
                and abs(sample['norm']-approach['norm'])/approach['norm']<=self.cfg['magnetic']['max_norm_fraction_change']):
            return None
        # The target crosses the load change. The measurement anchor is still entirely from the new load.
        target = wrap(approach['route_heading_deg']-approach['offset_deg']+sample['offset_deg'])
        return {'route_heading_deg':target, 'empty_reading_deg':approach['raw_heading_deg'],
                'loaded_reading_deg':sample['raw_heading_deg'], 'loading_residual_deg':wrap(-target)}

    def before_straight(self, load, action_completed_monotonic=None, *, pickup_heading=None,
                        tag_departure=None):
        """Use same-stance Tag heading when supplied, otherwise establish relative zero."""
        recovered=None
        if micro_trial.active(self.cfg) and micro_trial.enabled(self.c) and (self.reference.anchor or getattr(getattr(self.io,'capture_stream',None),'session',None)) and not getattr(self.io,'motion_capture_verified',False):
            if not self.allow_missing_magnetic:
                self.fault('capture unhealthy before next straight segment')
            recovered=self.sample(load,stationary_retry=True)
        old = self.reference.anchor
        if tag_departure is None and old and old.get('load') == load:
            return {'status':'reference_retained', 'reference_source':old.get('source','reliable_tag')}
        if recovered is not None:
            result, sample = recovered
        elif tag_departure is not None and not callable(getattr(self.io,'retry_magnetic_after_action',None)):
            result, sample = self._sample_once(load,action_completed_monotonic,defer_missing=True)
        else:
            result, sample = self.sample(load,action_completed_monotonic=action_completed_monotonic,
                                         stationary_retry=self.allow_missing_magnetic)
        if sample is None:
            if tag_departure is None:
                self.blocked = True
            return {'status':'reference_unavailable', 'sample':result, 'correction_actions':0}
        if tag_departure is not None and (not sample.get('capture_verified')
                or not sample.get('usable_for_heading') or sample.get('offset_deg') is None):
            return {'status':'reference_unavailable', 'sample':result, 'correction_actions':0}
        self.reference.anchor = dict(sample, source='relative_stationary_segment_start',
                                     tag_id=None, route_heading_deg=0.,
                                     planned_turn_error_measured=False,
                                     absolute_route_heading_known=False)
        pickup_target = self._pickup_approach_target(sample, pickup_heading)
        if pickup_target is not None:
            self.reference.anchor.update(route_heading_deg=pickup_target['route_heading_deg'],
                                         pickup_approach_heading=pickup_target)
        self.latched = False
        status = 'relative_reference_established'
        claim = 'straight-segment drift only; planned turn error and absolute route heading unknown'
        if tag_departure is not None:
            target = tag_departure['route_heading_deg']
            error = tag_departure['error_deg']
            self.reference.anchor.update(tag_id=tag_departure['tag_id'], route_heading_deg=target,
                                         offset_deg=wrap(sample['offset_deg']+target+error),
                                         absolute_route_heading_known=True, tag_departure=dict(tag_departure))
            status = 'tag_departure_reference_established'
            claim = 'verified Tag residual at the current stance; magnetic samples track subsequent drift'
        return {'status':status, 'reference':dict(self.reference.anchor), 'claim':claim,
                'correction_actions':0}

    def _target_observation_faults(self,sample,load):
        anchor=self.reference.anchor;now=self.clock();reasons=[]
        if not sample or not anchor:return ['missing_target_or_sample']
        if (not anchor.get('capture_verified') or not anchor.get('usable_for_heading') or
                anchor.get('load')!=load):reasons.append('untrusted_target_reference')
        t=anchor.get('stationary_validated_monotonic',anchor.get('received_monotonic'))
        if type(t) not in (int,float) or not 0<=now-t<=self.c['max_reference_age_s']:
            reasons.append('target_reference_expired')
        if (sample.get('owner_observation_scope')!=self.observation_scope or
                sample.get('load')!=load or not sample.get('capture_verified')):
            reasons.append('untrusted_sample_scope')
        times=sample.get('decision_capture_times') or []
        if not times or not all(type(t) in (int,float) and 0<=now-t<=self.cfg['magnetic']['max_age_s'] for t in times):
            reasons.append('sample_stale')
        if getattr(self.io,'capture_stream',None) is not None and not self.io.motion_capture_verified:
            reasons.append('capture_stream_unhealthy')
        if sample.get('calibration_id')!=anchor.get('calibration_id'):
            reasons.append('calibration_changed')
        capture=getattr(self.io,'capture_stream',None)
        if (sample.get('capture_session')!=anchor.get('capture_session') or
                (capture is not None and sample.get('capture_session')!=getattr(capture,'session',None))):
            reasons.append('capture_session_changed')
        current_load_epoch=getattr(self.io,'magnetic_load_epoch',None)
        if (sample.get('load_epoch')!=anchor.get('load_epoch') or
                (current_load_epoch is not None and sample.get('load_epoch')!=current_load_epoch)):
            reasons.append('load_epoch_changed')
        try:
            fraction=abs(sample['norm']-anchor['norm'])/anchor['norm']
            if not math.isfinite(fraction) or fraction>self.cfg['magnetic']['max_norm_fraction_change']:
                reasons.append('magnetic_magnitude_changed')
        except (KeyError,TypeError,ZeroDivisionError):reasons.append('invalid_magnetic_norm')
        return reasons

    def _confirm_unexpected_response(self,load,before,initial,report,trigger,*,require_agreement=True,
                                     revalidating_target=False):
        """Take an independent same-scope stationary sample before replanning."""
        sign=self.cfg['magnetic']['heading_sign']
        evidence=dict(trigger=trigger,observed_change_deg=wrap(sign*(initial['raw_heading_deg']-before['raw_heading_deg'])),
                      heading_deg=initial['raw_heading_deg'],capture_times=initial['decision_capture_times'])
        report['first_stationary_verification']=evidence
        def faults(sample):
            return [reason for reason in self._target_observation_faults(sample,load)
                    if not revalidating_target or reason!='target_reference_expired']

        failures=faults(initial)
        if failures or not all(initial.get(key)==before.get(key) for key in _FEEDBACK_SCOPE_FIELDS):
            reason='unexpected turn observation rejected: '+','.join(failures or ['feedback_scope_changed'])
            if all(initial.get(key)==before.get(key) for key in _FEEDBACK_SCOPE_FIELDS):
                self._defer_expired_target(failures,report)
            self._defer_stream_interruption(reason,failures)
            self.fault(reason,report)
        previous=initial
        for _ in range(2 if require_agreement else 1):
            try:
                result,settled=self.sample(load,action_completed_monotonic=self.clock())
            except MicroTrialStopped as exc:
                report.update(status='correction_locked',reason=str(exc))
                exc.report=copy.deepcopy(report)
                raise
            if self.allow_missing_magnetic and not revalidating_target:
                result,settled=self._recheck_magnetic_field(load,result,settled,report)
            independent = (settled is not None
                and all(settled.get(key)==initial.get(key) for key in _FEEDBACK_SCOPE_FIELDS)
                and min(settled.get('decision_capture_times') or [float('-inf')])>max(previous['decision_capture_times']))
            failures = faults(settled)
            if independent:
                self._defer_expired_target(failures,report)
            if not independent or failures:
                self.fault('independent stationary verification unavailable; no further motion',report)
            evidence['settled_capture_times']=settled['decision_capture_times']
            evidence['settled_change_deg']=wrap(sign*(settled['raw_heading_deg']-before['raw_heading_deg']))
            if not require_agreement or abs(wrap(settled['raw_heading_deg']-previous['raw_heading_deg']))<=_STATIONARY_HEADING_SPREAD_DEG:
                return result,settled
            evidence['disagreement_recheck']=True
            previous=settled
        self._defer_magnetic_data('independent stationary heading observations disagree',
                                  stage='stationary_heading',report=report)
        self.fault('independent stationary heading observations disagree',report)

    def _recheck_magnetic_field(self,load,result,sample,report):
        """Re-read a field-strength excursion before dropping magnetic correction."""
        ignored = {'target_reference_expired'} if self.allow_missing_magnetic else set()
        if set(self._target_observation_faults(sample,load))-ignored != {'magnetic_magnitude_changed'}:
            return result,sample
        fresh_result,fresh=self.sample(load,action_completed_monotonic=self.clock())
        if (not fresh or not all(fresh.get(key)==sample.get(key) for key in _FEEDBACK_SCOPE_FIELDS)
                or min(fresh.get('decision_capture_times') or [float('-inf')])<=max(sample['decision_capture_times'])):
            self.fault('magnetic field recheck has no independent same-scope sample',report)
            return result,sample
        faults=self._target_observation_faults(fresh,load)
        report.setdefault('magnetic_field_rechecks',[]).append({
            'reference_norm':self.reference.anchor['norm'],'first_norm':sample['norm'],
            'first_heading_deg':sample['raw_heading_deg'],'rechecked_heading_deg':fresh['raw_heading_deg'],
            'rechecked_norm':fresh['norm'],'rechecked_capture_times':fresh['decision_capture_times'],
            'rechecked_faults':faults})
        if set(faults)-ignored == {'magnetic_magnitude_changed'}:
            report.update(status='magnetic_field_recheck_unusable',
                          reason='magnetic field variation persisted after stationary recheck',
                          reason_zh='原地复测后磁场仍偏离参考，暂用原路线和下一处扫码点。')
            self._defer_magnetic_data(report['reason'],stage='magnetic_field_recheck',report=report)
        return fresh_result,fresh

    def corridor_motion_sample_ready(self, load, sampled_at, completed_at):
        """Recheck the exact post-strafe observation without taking a new sample."""
        sample = self.last_sample or {}
        now = self.clock()
        if (self.blocked or not getattr(self.io, 'motion_capture_verified', False)
                or any(type(value) not in (int, float) or not math.isfinite(value)
                       for value in (sampled_at, completed_at))
                or not 0 <= completed_at < sampled_at <= now
                or sample.get('received_monotonic') != sampled_at
                or sample.get('action_completed_monotonic') != completed_at
                or self._target_observation_faults(sample, load)):
            return False
        after = sample.get('decision_after_monotonic')
        if type(after) not in (int, float) or not completed_at <= after < sampled_at:
            return False
        max_age = self.cfg['magnetic']['max_age_s']
        captures = sample.get('decision_capture_times') or ()
        receipts = sample.get('decision_receipt_times') or ()
        return bool(captures and len(captures) == len(receipts) and all(
            type(stamp) in (int, float) and after < stamp <= sampled_at and now - stamp <= max_age
            for stamp in (*captures, *receipts)))

    def revalidate_stationary_target(self, load, reserve_s):
        """Retain a corridor heading target through a stationary Tag wait."""
        anchor = self.reference.anchor
        if not anchor or anchor.get('source') != 'relative_stationary_segment_start':
            self.fault('corridor heading reference unavailable before motion')
            return None
        validated = anchor.get('stationary_validated_monotonic', anchor['received_monotonic'])
        age = self.clock() - validated
        if 0 <= age and age + reserve_s < self.c['max_reference_age_s']:
            return None

        previous = self.last_sample
        _, sample = self.sample(load,stationary_retry=True)
        reasons = [reason for reason in self._target_observation_faults(sample, load)
                   if reason != 'target_reference_expired']
        if (age < 0 or anchor.get('owner_observation_scope') != self.observation_scope
                or not previous or previous.get('owner_observation_scope') != self.observation_scope
                or not previous.get('capture_verified') or previous.get('load') != load):
            reasons.append('stationary_reference_scope_changed')
        change = None
        verification = {}
        if sample and previous:
            change = wrap(sample['raw_heading_deg'] - previous['raw_heading_deg'])
            if abs(change) > self.c['deadband_deg']:
                if not reasons and self._uses_route_feedback('corridor_departure',load):
                    _,sample=self._confirm_unexpected_response(load,previous,sample,verification,
                        'stationary_heading_changed',revalidating_target=True)
                    change=wrap(sample['raw_heading_deg']-previous['raw_heading_deg'])
                else:
                    reasons.append('stationary_heading_changed')
        if reasons:
            self.fault('corridor stationary reference rejected: ' + ','.join(reasons))
            return None

        anchor['stationary_validated_monotonic'] = sample['received_monotonic']
        return {'status': 'stationary_reference_revalidated', 'previous_validation_age_s': age,
                'stationary_change_deg': change, 'target_deg': anchor['route_heading_deg'], **verification}

    def coarse_turn_prediction(self,action,load,action_completed_monotonic=None,*,nominal_delta_deg=None):
        sample=self.last_sample;anchor=self.reference.anchor
        reasons=self._target_observation_faults(sample,load)
        refreshable=(reasons==['sample_stale'] or (sample is None and anchor is not None))
        if refreshable and action_completed_monotonic is not None:
            _,sample=self.sample(load,action_completed_monotonic=action_completed_monotonic)
            reasons=self._target_observation_faults(sample,load)
        if reasons:
            self._defer_expired_target(reasons)
            self._defer_stream_interruption('magnetic stream interrupted before configured turn',reasons)
            report=dict(status='correction_locked',action=action,reason='coarse turn observation rejected',
                        failures=reasons,use_original_macro=False,sample=sample,reference=anchor)
            self.fault('coarse turn observation rejected: '+','.join(reasons),report)
            return report
        error=wrap(self.cfg['magnetic']['heading_sign']*sample['raw_heading_deg']+anchor['offset_deg']-anchor['route_heading_deg'])
        if math.isclose(error, -180., abs_tol=1e-6) and action == 'left_turn' and self.planned_turn_recovery_enabled(load):
            # At the exact half-turn, keep the explicitly configured direction.
            error = 180.
        if self._uses_route_feedback('planned_turn_intermediate',load) and action in self._unreliable_coarse_actions:
            return dict(use_original_macro=False,error_deg=error,prediction_only=True,
                        reason='previous coarse response was unexpected; enter bounded magnetic feedback')
        session,epoch=self.observation_scope
        estimate=self.turn_estimator.control_estimate('left' if 'left' in action else 'right','macro',load,session,epoch,sample.get('calibration_id',''),now_s=self.clock())
        if estimate is None:
            nominal=nominal_delta_deg if nominal_delta_deg is not None else self.cfg.get('coarse_turn_nominal_deg',{}).get(action)
            expected_sign=-1 if 'left' in action else 1
            if (type(nominal) not in (int,float) or not math.isfinite(nominal) or
                    not 0<abs(nominal)<=90 or nominal*expected_sign<=0):
                if self._uses_route_feedback('planned_turn_intermediate',load):
                    return dict(use_original_macro=False,error_deg=error,prediction_only=True,
                                reason='coarse prediction unavailable; enter bounded magnetic feedback')
                if self.c.get('stop_on_possible_overshoot'):
                    self.fault('coarse response unknown; cannot exclude overshoot; no further motion')
                    return dict(use_original_macro=False,reason='coarse response unknown')
                return dict(use_original_macro=True,reason='macro observations insufficient and no valid explicit nominal; preserve original chain')
            estimate=dict(degrees=nominal,upper_abs_deg=abs(nominal),
                          evidence='explicit_source_nominal',sample_count=0,
                          physical_upper_bound_verified=False)

        if (self.c.get('stop_on_possible_overshoot') and not self.planned_turn_recovery_enabled(load)
                and (estimate['degrees']*error>=0 or estimate['upper_abs_deg']>abs(error))):
            report=dict(use_original_macro=False,error_deg=error,estimate=estimate,reason='possible coarse overshoot; stop without further correction')
            self.fault(report['reason'],report)
            return report
        # Only remove a remaining coarse macro; never introduce extra turns or change target.
        minimum_remaining=self.c['release_deg']
        allowed_upper=abs(error)-minimum_remaining
        correct_direction=estimate['degrees']*error<0
        safe=correct_direction and estimate['upper_abs_deg']<=allowed_upper
        if safe:
            reason=('trusted coarse prediction' if estimate.get('evidence')!='explicit_source_nominal'
                    else 'explicit nominal coarse prediction; not measured')
        elif correct_direction and estimate['upper_abs_deg']<=abs(error):
            reason='remaining coarse macro does not leave the configured margin; enter bounded micro feedback'
        else:
            reason='remaining coarse macro risks target crossing; enter bounded micro feedback'
        comparison=None
        if self.allow_missing_magnetic and self.planned_turn_recovery_enabled(load):
            comparison=self._compare_remaining_coarse_turn(action,load,sample,error,estimate)
            if comparison is not None:
                measured=comparison['cost_basis']=='verified_elapsed_cycles'
                coarse=comparison['coarse_and_cleanup_s' if measured else 'coarse_and_cleanup_cycles']
                micros=comparison['micro_only_s' if measured else 'micro_only_cycles']
                tied=math.isclose(coarse,micros)
                unit='estimated time' if measured else 'predicted turns'
                if coarse<micros and not tied:
                    safe=True
                    reason='configured coarse cycle plus bounded cleanup is cheaper in '+unit
                elif safe and micros<coarse and not tied:
                    safe=False
                    reason='micro feedback is cheaper than the remaining coarse cycle in '+unit
        report=dict(use_original_macro=safe,error_deg=error,
                    predicted_error_deg=wrap(error+estimate['degrees']),estimate=estimate,
                    minimum_remaining_deg=minimum_remaining,allowed_upper_abs_deg=allowed_upper,
                    reason=reason,prediction_only=True)
        if comparison is not None:
            report['turn_cost_comparison']=comparison
        return report

    def _compare_remaining_coarse_turn(self, action, load, sample, error, estimate):
        """Compare settled motion cycles; fresh feedback still decides every next action."""
        direction='left' if 'left' in action else 'right'
        sign=-1 if direction=='left' else 1
        distance=abs(error)
        band=self.c['deadband_deg']
        improvement=self.c['minimum_improvement_deg']
        if (distance<=band or self.budget.limit_total_angle or estimate.get('sample_count',0)<2
                or estimate.get('source') not in ('observed_provisional','observed_recent_three')):
            return None

        def bounded_interval(candidate, expected_sign, limit):
            if candidate is None:
                return None
            low,high,gain=(candidate.get(k) for k in ('lower_abs_deg','upper_abs_deg','degrees'))
            if (not all(type(v) in (int,float) and math.isfinite(v) for v in (low,high,gain))
                    or not improvement<=low<=gain*expected_sign<=high<=limit):
                return None
            return low,high

        interval=bounded_interval(estimate,sign,self.turn_estimator.bounds['macro'])
        if interval is None or sign*error>=0:
            return None
        low,high=interval
        under,over=distance-low,high-distance
        if max(under,over)>distance-improvement:
            return None

        session,epoch=self.observation_scope
        calibration=sample.get('calibration_id','')
        max_micro=self.c['max_step_deg']
        room=min(self.c['micro_trial'].get('max_actions_per_turn_group',4),
                 self.c['max_actions_per_journey']-self.count,self.budget.max_actions-self.budget.actions)
        if room<=0 or self.blocked or self.budget.locked:
            return None
        now=self.clock()
        forward=self.turn_estimator.control_estimate(direction,'micro',load,session,epoch,calibration,now_s=now)
        forward_bounds=bounded_interval(forward,sign,max_micro)
        if forward_bounds is None or not self.budget.available(sign*5.,max(5.,forward_bounds[1])):
            return None

        def approaching_cycles(residual):
            return math.ceil(max(0.,residual-band)/forward_bounds[0])

        reverse_cycles=0
        reverse_elapsed=0.
        if over>band:
            opposite='right' if direction=='left' else 'left'
            reverse=self.turn_estimator.control_estimate(opposite,'micro',load,session,epoch,calibration,now_s=now)
            bounds=bounded_interval(reverse,-sign,max_micro)
            if (bounds is None or reverse.get('source')!='adaptive_micro_low_pass'
                    or reverse.get('sample_count',0)<1
                    or not self.budget.available(-sign*5.,max(5.,bounds[1]))):
                return None
            reverse_low,reverse_high=bounds
            least_crossing=max(band,low-distance)
            if (reverse_high>2*least_crossing-improvement
                    or max(abs(least_crossing-reverse_high),abs(over-reverse_low))>band):
                return None  # A verified reverse step must improve every crossing outside the band.
            reverse_cycles=1
            reverse_elapsed=reverse.get('cycle_elapsed_s')
        forward_cleanup=approaching_cycles(under)
        cleanup=max(forward_cleanup,reverse_cycles)
        if cleanup>room:
            return None
        comparison=dict(micro_only_cycles=approaching_cycles(distance),coarse_and_cleanup_cycles=1+cleanup,
                        coarse_cleanup_cycles=cleanup,reverse_cleanup_cycles=reverse_cycles,
                        residual_bounds_along_turn_deg=[low-distance,high-distance],
                        target_band_deg=band,cost_basis='predicted_cycles',estimated_only=True)
        coarse_elapsed=estimate.get('cycle_elapsed_s')
        forward_elapsed=forward.get('cycle_elapsed_s')
        if all(type(elapsed) in (int,float) and math.isfinite(elapsed) and elapsed>=0
               for elapsed in (coarse_elapsed,forward_elapsed,reverse_elapsed)):
            comparison.update(cost_basis='verified_elapsed_cycles',
                              micro_only_s=comparison['micro_only_cycles']*forward_elapsed,
                              coarse_and_cleanup_s=coarse_elapsed+max(forward_cleanup*forward_elapsed,
                                                                     reverse_cycles*reverse_elapsed))
        return comparison

    def plan_turn(self,delta):
        a=self.reference.anchor
        if not a or not a.get('capture_verified'):raise MicroTrialStopped('planned turn requires fresh segment reference')
        if not isinstance(delta,(int,float)) or not 0<abs(delta)<=180:raise ValueError('planned group angle')
        a['route_heading_deg']=wrap(a['route_heading_deg']+delta)
        a['planned_group_target']=True
        self.latched=False
        self._unreliable_coarse_actions.clear()
        self._confirmed_coarse_response=None

    def planned_turn_recovery_enabled(self,load):
        anchor=self.reference.anchor or {}
        return (getattr(self,'planned_turn_recovery_scope',False) is True and self.carry_load_supported(load)
                and anchor.get('planned_group_target') is True and self.c.get('execute') is True
                and micro_trial.enabled(self.c) and micro_trial.adaptive(self.c)
                and self.c.get('micro_trial',{}).get('bounded_route_mode') is True)

    def carry_load_supported(self,load):
        return load=='single' or (load=='double' and getattr(self,'pair_route_enabled',False))

    def boundary_recovery_enabled(self,name,load):
        if name=='planned_turn_complete':return self.planned_turn_recovery_enabled(load)
        return (getattr(self,'boundary_recovery_command0',False) is True and self.carry_load_supported(load)
                and name in CARRY_HEADING_ACTIONS | CORRIDOR_HEADING_ACTIONS
                and micro_trial.enabled(self.c) and micro_trial.adaptive(self.c)
                and self.c.get('micro_trial',{}).get('bounded_route_mode') is True)

    def _uses_route_feedback(self,name,load):
        phase='planned_turn_complete' if name=='planned_turn_intermediate' else name
        return (self.allow_missing_magnetic and not self.budget.limit_total_angle
                and self.boundary_recovery_enabled(phase,load))

    def _correction_target_deg(self,name,recovery):
        # A straight carry may continue inside its existing deadband. Only the
        # final turn keeps the finer residual target used before the entrance scan.
        if name=='planned_turn_complete' and recovery and not self.budget.limit_total_angle:
            return self.c['minimum_improvement_deg']
        return self.c['deadband_deg']

    def _can_recheck_carry(self,report):
        error=report.get('error_deg')
        return (type(error) in (int,float) and abs(error)>self.c['deadband_deg']
                and not self.blocked and not self.budget.locked
                and self.count<self.c['max_actions_per_journey'] and self.budget.actions<self.budget.max_actions
                and (report.get('motion_prediction',{}).get('status')=='no_safe_candidate'
                     or report.get('reason')=='journey budget or cooldown'))

    def _allow_short_transfer_segment(self,first,repeat):
        if not getattr(self,'short_transfer_segment_pending',False):
            return
        if (first.get('motion_prediction',{}).get('status')!='no_safe_candidate'
                or repeat.get('motion_prediction',{}).get('status')!='no_safe_candidate'):
            return
        margin=_STATIONARY_HEADING_SPREAD_DEG
        limit=min(self.c['max_error_deg'],self.c['deadband_deg']+margin)
        a,b=first.get('error_deg'),repeat.get('error_deg')
        if (type(a) in (int,float) and type(b) in (int,float) and abs(a-b)<=margin
                and self.c['deadband_deg']<abs(a)<=limit and self.c['deadband_deg']<abs(b)<=limit):
            repeat.update(status='short_transfer_allowed',short_transfer_segment_allowed=True,
                          short_transfer_limit_deg=limit,continuation='carry_forward2',
                          reason='two fresh headings within the short transfer segment margin')

    def _allow_entry_followup(self, first, repeat, followup):
        if followup is None or not all(_carry_prediction_rejected(report) for report in (first, repeat)):
            return
        a, b = first['error_deg'], repeat['error_deg']
        limit = min(_ENTRY_STRAIGHT_LIMIT_DEG, self.c['max_error_deg'])
        if a*b <= 0 or max(abs(a), abs(b)) > limit or abs(a-b) > _STATIONARY_HEADING_SPREAD_DEG:
            return
        decision = followup()
        if decision is not None:
            repeat.update(status='entry_heading_followup', entry_followup=decision,
                          entry_heading_limit_deg=limit,
                          reason='stable entry heading; use planned short approach or stationary Tag 2 scan')

    def _allow_configured_carry_followup(self, report, followup, load):
        """Use a fresh verified heading at a completed route action boundary."""
        if not self.allow_missing_magnetic or followup is None or self.blocked or self.budget.locked:
            return False
        residual = report.get('after_error_deg', report.get('error_deg'))
        limit = min(_ENTRY_STRAIGHT_LIMIT_DEG, self.c['max_error_deg'])
        if (type(residual) not in (int, float)
                or not self.c['deadband_deg'] < abs(residual) <= limit
                or self._target_observation_faults(self.last_sample, load)):
            return False
        self.check()
        decision = followup()
        if decision is None:
            return False
        report.update(status='entry_heading_followup', entry_followup=decision,
                      entry_heading_limit_deg=limit,
                      reason_zh='航向余差在路段容差内，继续执行已确认的短段或原地扫码。',
                      reason='verified residual allows the configured stationary scan or short approach')
        return True

    def _allow_corridor_heading_followup(self, report, load):
        residual = report.get('after_error_deg', report.get('error_deg'))
        limit = min(_ENTRY_STRAIGHT_LIMIT_DEG, self.c['max_error_deg'])
        if (not self.allow_missing_magnetic or self.blocked or self.budget.locked
                or type(residual) not in (int, float)
                or not self.c['deadband_deg'] < abs(residual) <= limit
                or self._target_observation_faults(self.last_sample, load)):
            return False
        report.update(status='corridor_heading_followup', remaining_heading_error_deg=residual,
                      reason='verified corridor residual retained for the next movement boundary',
                      reason_zh='航向余差在通道容差内，继续到下一次航向检查。')
        return True

    def _retain_route_heading_residual(self,name,load,report,followup):
        """Preserve known heading after exhausting useful corrections at a route boundary."""
        residual=report.get('after_error_deg',report.get('error_deg'))
        if (not self._uses_route_feedback(name,load) or self.blocked or self.budget.locked
                or type(residual) not in (int,float) or not math.isfinite(residual)
                or self._target_observation_faults(self.last_sample,load)):
            return False
        self.check()
        decision=followup() if followup is not None else None
        if decision is not None:
            report['entry_followup']=decision
        report.update(status='route_heading_followup',remaining_heading_error_deg=residual,
                      reason='verified heading residual retained for the next configured route step',
                      reason_zh='已复测航向并记录余差，按预定路线进入下一段或扫码。')
        return True

    def _wait_carry_boundary(self, name, load, correct, reports, correction_limit, *, entry_followup=None,
                             excluded_turns=None):
        callback = getattr(self, 'carry_boundary_wait_report', None)
        last = reports[-1]
        residual = last.get('after_error_deg', last.get('error_deg'))
        if (not callable(callback) or self.blocked or self.budget.locked
                or type(residual) not in (int, float)
                or abs(residual)<=self.c['deadband_deg']
                or (abs(residual)>self.c['max_error_deg'] and not self._uses_route_feedback(name,load))):
            return None

        prediction = last.get('motion_prediction', {})
        rejection_reasons = set(prediction.get('rejected', {}).values()) - {'wrong_direction'}
        no_improving_step = (prediction.get('status') == 'no_safe_candidate' and rejection_reasons
                             and rejection_reasons <= {'insufficient_worst_case_improvement',
                                                       'insufficient_predicted_improvement'})
        prediction_rejected = _carry_prediction_rejected(last)
        entry_retry = entry_followup is not None and prediction_rejected
        limit_reached = (len(reports) == correction_limit and last.get('correction_actions') == 1
                         or last.get('reason') in ('journey budget or cooldown',
                                                   'shared magnetic/visual journey budget exhausted or locked',
                                                   'nominal journey angle budget exhausted',
                                                   'remaining action/time budget unavailable'))
        if not (no_improving_step or limit_reached or entry_retry
                or (self.allow_missing_magnetic and prediction_rejected)):
            return None

        waiting = {'status': 'stationary_carry_wait', 'action': name, 'error_deg': residual,
                   'motion_prediction':prediction,'budget':self.budget.status(),
                   'reason': 'sampling while stationary after finding no improving turn or reaching the correction budget'}
        try:
            callback(waiting)
        except Exception:
            pass  # Reporting must not affect execution.

        windows = 0
        windows_without_motion = 0
        prior = last
        turns = sum(r.get('correction_actions', 0) for r in reports)
        while True:
            self.check()
            try:
                observation = self.sample(load,stationary_retry=True)
            except MagneticDataUnavailable as exc:
                exc.report=dict(waiting,stationary_windows=windows,correction_steps=reports)
                raise
            failures = self._target_observation_faults(observation[1], load)
            if failures:
                self.fault('stationary carry observation rejected: '+','.join(failures),
                           dict(waiting, failures=failures, stationary_windows=windows))
            current = self._after_action_once(name, load, correct,
                                              allow_correction=(entry_retry or self.allow_missing_magnetic) and turns<correction_limit,
                                              sample_override=observation, excluded_turns=excluded_turns)
            windows += 1
            if current.get('correction_actions'):
                reports.append(current)
                turns += current['correction_actions']
                windows_without_motion = 0
            else:
                windows_without_motion += 1
            if not self._allow_configured_carry_followup(current,entry_followup,load) and entry_retry:
                self._allow_entry_followup(prior,current,entry_followup)
            prior = current
            error = current.get('after_error_deg', current['error_deg'])
            if abs(error) <= self.c['deadband_deg']:
                current = dict(current, status='stationary_carry_recovered',
                               reason='fresh stationary heading returned to the existing deadband')
            if abs(error) <= self.c['deadband_deg'] or current.get('entry_followup'):
                return dict(current, stationary_recovery={'initial_error_deg': residual, 'windows': windows},
                            correction_steps=reports, correction_actions=turns)
            if windows_without_motion>=_CARRY_STATIONARY_RECHECK_WINDOWS:
                if self._retain_route_heading_residual(name,load,current,entry_followup):
                    return dict(current,stationary_recovery={'initial_error_deg':residual,'windows':windows},
                                correction_steps=reports,correction_actions=turns)
                self.fault('stationary carry recheck found no verified route continuation',
                    dict(current,status='stationary_carry_unresolved',
                         reason='stationary carry recheck found no verified route continuation',
                         reason_zh='航向复测仍超出路段容差，且没有已确认的扫码或短段接续。',
                         stationary_recovery={'initial_error_deg':residual,'windows':windows},
                         correction_steps=reports,correction_actions=turns))
                return None

    def after_action(self,name,load,correct,allow_correction=True,action_completed_monotonic=None,*,
                     entry_followup=None,transfer_turn_pending=False,prefer_visual_followup=None):
        self._nominal_retry_used=False
        count=self.c.get('micro_trial',{}).get('max_actions_per_checkpoint',1) if self.c.get('micro_trial',{}).get('bounded_route_mode') else 1
        if name=='planned_turn_complete' and self.c.get('micro_trial',{}).get('bounded_route_mode'):count=self.c['micro_trial'].get('max_actions_per_turn_group',count)
        reports=[];reuse=None;no_progress=0
        excluded_turns={}
        recovery=self.boundary_recovery_enabled(name,load)
        route_feedback=self._uses_route_feedback(name,load)
        if transfer_turn_pending and recovery and self.c['execute'] and allow_correction:
            # Translation is finished. Keep the initial reference and let the planned turn use the current heading.
            report=self._after_action_once(name,load,correct,allow_correction=False,
                                           action_completed_monotonic=action_completed_monotonic)
            failures=self._target_observation_faults(self.last_sample,load)
            if failures:self.fault('planned transfer turn observation rejected: '+','.join(failures),report)
            return dict(report,status='transfer_turn_allowed',continuation='planned_transfer_turn',
                        reason='transfer forward complete; use the measured heading for the planned turn')
        correction_target=self._correction_target_deg(name,recovery)
        recheck=(recovery and name!='planned_turn_complete' and not self.budget.limit_total_angle
                 and self.c['execute'] and allow_correction)
        reobserved_decision=False
        for _ in range(count):
            try:
                try:
                    report=self._after_action_once(name,load,correct,allow_correction,reuse,action_completed_monotonic,
                                                   excluded_turns=excluded_turns,prefer_visual_followup=prefer_visual_followup)
                except MagneticDataUnavailable as exc:
                    health=exc.health
                    if (name not in CORRIDOR_HEADING_ACTIONS or reobserved_decision
                            or exc.error_details['stage']!='magnetic_decision' or health.get('fault')
                            or not self.io.motion_capture_verified
                            or health.get('fresh_rows',0)<health.get('required_rows',1)):
                        raise
                    # The expired window was rejected before submission. Replan from a fresh stationary sample.
                    reobserved_decision=True
                    report=self._after_action_once(name,load,correct,allow_correction,
                        sample_override=self.sample(load,stationary_retry=True),excluded_turns=excluded_turns)
                    report['expired_decision_reobserved']=health
                if recheck and report.get('correction_actions',0)==0 and self._can_recheck_carry(report):
                    if self._allow_configured_carry_followup(report,entry_followup,load):
                        reports.append(report)
                        break
                    first=report
                    recheck=False
                    cooldown=max(0.,self.c['cooldown_s']-(self.clock()-self.last_action))
                    if cooldown:self.io.settle(cooldown,self.check)
                    report=self._after_action_once(name,load,correct,allow_correction,
                                                   sample_override=self.sample(load,stationary_retry=True),
                                                   excluded_turns=excluded_turns)
                    report['stationary_recheck']={'first_error_deg':first['error_deg'],'reason':first.get('reason')}
                    self._allow_short_transfer_segment(first,report)
                    self._allow_entry_followup(first,report,entry_followup)
            except MicroTrialStopped as exc:
                if exc.report is None:
                    exc.report=dict(status='correction_locked',action=name,reason=str(exc),correction_actions=0)
                if getattr(exc,'error_details',None):
                    exc.report.update(exc.error_details)
                if reports:
                    failed=copy.deepcopy(exc.report)
                    exc.report['correction_steps']=reports+[failed]
                    exc.report['correction_actions']=sum(r.get('correction_actions',0) for r in reports)+failed.get('correction_actions',0)
                raise
            reports.append(report)
            if report.get('correction_actions',0)!=1:break
            if abs(report.get('after_error_deg',0))<=correction_target:break
            if recovery:
                if name!='planned_turn_complete' and self._allow_configured_carry_followup(report,entry_followup,load):
                    break
                no_progress=no_progress+1 if report.get('correction_diagnostic',{}).get('improvement_deg',0)<self.c['minimum_improvement_deg'] else 0
                if no_progress>=2:
                    if route_feedback or name=='planned_turn_complete' and getattr(self,'terminal_stationary_scan_scope',False):
                        break
                    self.fault(('bounded planned turn' if name=='planned_turn_complete' else 'bounded carry')+' recovery no-progress budget',report)
            reuse=self.last_verified_sample
        if recovery and name!='planned_turn_complete' and self.c.get('execute') and allow_correction:
            last=reports[-1]
            residual=last.get('after_error_deg',last.get('error_deg'))
            continuation=(last.get('best_reachable_with_current_steps') or last.get('short_transfer_segment_allowed')
                          or last.get('entry_followup')
                          or (name in CORRIDOR_HEADING_ACTIONS and self._allow_corridor_heading_followup(last,load)))
            if residual is None or (abs(residual)>self.c['deadband_deg'] and not continuation):
                if not self._allow_configured_carry_followup(last,entry_followup,load):
                    recovered = self._wait_carry_boundary(name, load, correct, reports, count,
                        entry_followup=entry_followup,excluded_turns=excluded_turns)
                    if recovered is not None:
                        return recovered
                    if not self._retain_route_heading_residual(name,load,last,entry_followup):
                        self.fault('bounded carry recovery unresolved; no safe corridor fallback configured',dict(last,correction_steps=reports))
        if name=='planned_turn_complete' and self.c.get('execute'):
            residual=reports[-1].get('after_error_deg',reports[-1].get('error_deg'))
            if residual is None or abs(residual)>self.c['deadband_deg']:
                last=reports[-1]
                reserve_limited=last.get('reason') in (
                    'shared magnetic/visual journey budget exhausted or locked',
                    'nominal journey angle budget exhausted','journey budget or cooldown',
                    'remaining action/time budget unavailable')
                cap_limited=len(reports)==count and last.get('correction_actions')==1
                locally_best=last.get('best_reachable_with_current_steps') is True
                no_improving_prediction=(not self.budget.limit_total_angle and
                    last.get('motion_prediction',{}).get('status')=='no_safe_candidate')
                if (recovery and getattr(self,'terminal_stationary_scan_scope',False) is True
                        and type(residual) in (int,float) and math.isfinite(residual)
                        and not self.blocked and not self.budget.locked
                        and (reserve_limited or cap_limited or locally_best or no_improving_prediction
                             or no_progress>=2 or route_feedback)):
                    return dict(last,status='stationary_scan_pending',turn_target_reached=False,
                        terminal_visual_scan_allowed=True,forward_translation_allowed=False,
                        remaining_heading_error_deg=residual,
                        correction_actions=sum(r.get('correction_actions',0) for r in reports),
                        correction_steps=reports)
                self.fault('planned turn target not reached within bounded correction budget; no forward translation',dict(last,correction_steps=reports))
        if len(reports)>1:
            result=dict(reports[-1]);result['correction_actions']=sum(r.get('correction_actions',0) for r in reports);result['correction_steps']=reports
            if (len(reports)==count and abs(result.get('after_error_deg',0))>self.c['deadband_deg']
                    and not result.get('entry_followup')
                    and result['status'] not in ('corridor_heading_followup','route_heading_followup')):
                result['reason']='checkpoint correction cap reached; target retained for next action'
            return result
        return reports[0]

    def _after_action_once(self, name, load, correct, allow_correction=True, sample_override=None,action_completed_monotonic=None,
                           *,excluded_turns=None,prefer_visual_followup=None):
        """Exactly one optional finite turn. Its verification is sampled, not recursively hooked."""
        if sample_override is None and name=='planned_turn_complete' and self._confirmed_coarse_response is not None:
            completed,result,confirmed=self._confirmed_coarse_response
            self._confirmed_coarse_response=None
            if completed==action_completed_monotonic and not self._target_observation_faults(confirmed,load):
                sample_override=result,confirmed
        result, sample = sample_override if sample_override is not None else self.sample(load,action_completed_monotonic=action_completed_monotonic)
        report = {'status': 'monitor_only', 'action': name, 'sample': result, 'correction_actions': 0}
        anchor = self.reference.anchor
        recovery=self.boundary_recovery_enabled(name,load) and allow_correction
        route_feedback=self._uses_route_feedback(name,load)
        if route_feedback:
            result,sample=self._recheck_magnetic_field(load,result,sample,report)
            report['sample']=result
        self._defer_expired_target(self._target_observation_faults(sample,load),report)
        if recovery:
            failures=self._target_observation_faults(sample,load)
            if failures:
                self._defer_stream_interruption('magnetic stream interrupted at completed action',failures)
                self.fault(('planned turn' if name=='planned_turn_complete' else 'carry')+' recovery observation rejected: '+','.join(failures),report)
        pending=self.pending_turn_observation;self.pending_turn_observation=None
        planned=name in ('planned_turn_intermediate','planned_turn_complete')
        before=pending.get('before') if pending is not None else None
        coarse_unexpected=False
        if pending is not None and planned and route_feedback and anchor and before and sample:
            sign=self.cfg['magnetic']['heading_sign']
            before_error=wrap(sign*before['raw_heading_deg']+anchor['offset_deg']-anchor['route_heading_deg'])
            after_error=wrap(sign*sample['raw_heading_deg']+anchor['offset_deg']-anchor['route_heading_deg'])
            change=wrap(sign*(sample['raw_heading_deg']-before['raw_heading_deg']))
            direction=-1 if pending['action']=='left_turn' else 1
            coarse_unexpected=(change*direction<=0 or abs(change)>self.turn_estimator.bounds['macro']
                               or before_error*after_error<0)
            if coarse_unexpected:
                if pending['completed'] is None:
                    self.fault('unexpected coarse turn has no verified completion',report)
                result,sample=self._confirm_unexpected_response(load,before,sample,report,'unexpected_coarse_response')
                self._unreliable_coarse_actions.add(pending['action'])
                if name=='planned_turn_intermediate':
                    self._confirmed_coarse_response=(pending['completed'],result,sample)
                report.update(sample=result,turn_observation=dict(accepted=False,
                              reason='unexpected_coarse_response_not_used_for_learning',
                              delta_deg=report['first_stationary_verification']['settled_change_deg']))
        if pending is not None:
            if not coarse_unexpected:
                report['turn_observation']=self.observe_turn(pending['action'],load,before,sample,pending['started'],pending['completed'])
            if (planned and not route_feedback
                    and self.planned_turn_recovery_enabled(load)
                    and report['turn_observation'].get('reason') in ('wrong_direction_or_zero','turn_out_of_bounds')):
                self.fault('coarse response failed: '+report['turn_observation']['reason'],report)
        if micro_trial.enabled(self.c) and name not in CARRY_HEADING_ACTIONS | CORRIDOR_HEADING_ACTIONS | {'planned_turn_complete'}:
            allow_correction=False  # Explicit corridor boundaries are distinct from native strafe actions.
        if self.c.get('stop_on_possible_overshoot') and anchor and sample and before and planned:
            def target_error(row):
                return wrap(self.cfg['magnetic']['heading_sign']*row['raw_heading_deg']+anchor['offset_deg']-anchor['route_heading_deg'])
            before_error=target_error(before);after_error=target_error(sample)
            if before_error*after_error<0:
                report.update(before_error_deg=before_error,after_error_deg=after_error,reason='observed coarse target crossing')
                if not self.planned_turn_recovery_enabled(load):
                    self.fault('observed coarse target crossing; no reverse or continuation',report)
                    return report
                # A normal overshoot is recoverable, not a wrong-way/unknown response.
                if not route_feedback and not report.get('turn_observation',{}).get('accepted'):
                    self.fault('coarse target crossing without verified turn response',report)
                report['coarse_crossing_recoverable']=True
        if name=='planned_turn_intermediate':
            failures=self._target_observation_faults(sample,load)
            if failures:
                report.update(status='correction_locked',reason='intermediate turn observation rejected',
                              failures=failures,observed_sample=sample,reference=anchor)
                self.fault('intermediate turn observation rejected: '+','.join(failures),report)
                return report
            report['reason']='planned turn group in progress; target preserved; correction after final macro';return report
        # Intentional turns and pickup invalidate the old direction, never oppose planned turns.
        if name in ('left_turn', 'right_turn', 'grab_single', 'stand'):
            self.reference.anchor = None
            if name=='grab_single' and sample and self.c.get('micro_trial',{}).get('bounded_route_mode'):
                self.reference.anchor=dict(sample,source='relative_stationary_segment_start',tag_id=None,route_heading_deg=0.,planned_turn_error_measured=False,absolute_route_heading_known=False)
            self.latched = False
            report['reason'] = 'planned heading/load change; next straight establishes a new stationary relative reference; turn angle error unknown'
            report['planned_turn_error_measured'] = False
            if name=='grab_single' and self.reference.anchor:
                report['reason']='new same-load reference established from post-pickup sample; no duplicate pre-straight sample'
            if anchor and sample:
                report['field_norm_fraction_change'] = abs(sample['norm']-anchor['norm'])/anchor['norm']
            return report
        if sample is None:
            self.blocked = True
            report['reason'] = 'invalid magnetic data; correction locked for journey'
            return report
        if not anchor or not anchor['usable_for_heading'] or anchor['load'] != load:
            report['reason'] = 'no same-load reference with verified heading sign'
            if anchor and sample and anchor.get('load') == load:
                report['raw_relative_error_deg'] = wrap(sample['raw_heading_deg']-anchor['raw_heading_deg'])
                report['raw_relative_error_direction_verified'] = False
            return report
        validated = anchor.get('stationary_validated_monotonic', anchor['received_monotonic'])
        if not 0 <= self.clock() - validated <= self.c['max_reference_age_s']:
            self.fault('heading reference expired')
            report['reason'] = 'heading reference expired'
            return report
        if abs(sample['norm'] - anchor['norm']) / anchor['norm'] > self.cfg['magnetic']['max_norm_fraction_change']:
            self.fault('magnetic magnitude changed: reference=%s current=%s fraction=%s limit=%s basis=%s raw_reference=%s raw_current=%s' % (anchor['norm'],sample['norm'],abs(sample['norm']-anchor['norm'])/anchor['norm'],self.cfg['magnetic']['max_norm_fraction_change'],sample.get('norm_basis'),anchor.get('raw_norm'),sample.get('raw_norm')))
            report['reason'] = 'magnetic magnitude changed; correction locked'
            return report
        error = wrap(self.cfg['magnetic']['heading_sign'] * sample['raw_heading_deg'] + anchor['offset_deg'] - anchor['route_heading_deg'])
        report['error_deg'] = error
        if prefer_visual_followup is not None:
            followup = prefer_visual_followup()
            if followup is not None:
                report.update(status='entry_heading_followup',entry_followup=followup,
                              reason_zh='当前动作结束后已读到有效 Tag 2 位姿，先用视觉更新路线。')
                return report
        residual_driven=recovery and not self.budget.limit_total_angle
        if abs(error) <= self.c['release_deg']:
            self.latched = False
        if abs(error) > self.c['deadband_deg']:
            self.latched = True
        if (abs(error)<=self._correction_target_deg(name,recovery) if residual_driven else
                (not self.latched or abs(error)<=self.c['deadband_deg'])):
            report['reason'] = 'within deadband/hysteresis'
            return report
        if abs(error) > self.c['max_error_deg'] and not route_feedback:
            self.fault('heading error exceeds configured trial envelope: error=%s limit=%s' % (error,self.c['max_error_deg']),report)
        if (not self.c['execute'] or self.blocked or not allow_correction or not self.carry_load_supported(load) or
                not sample.get('capture_verified',False) or not anchor.get('capture_verified',False)):
            report['reason'] = 'execution disabled, unsafe, capture freshness unverified, or calibration/load unavailable'
            return report
        if self.count >= self.c['max_actions_per_journey'] or self.clock() - self.last_action < self.c['cooldown_s']:
            report['reason'] = 'journey budget or cooldown'
            return report
        gains=micro_trial.deltas(self.c)
        chosen = next(n for n, delta in gains.items() if delta * error < 0)
        delta = gains[chosen]
        conservative_bound=abs(delta);choice=None
        if micro_trial.enabled(self.c):
            from motion_prediction import select_turn
            session,epoch=self.observation_scope
            candidates={n:dict(direction='left' if 'left' in n else 'right',action_type='micro',
                nominal_delta_deg=d,max_step_deg=self.c['max_step_deg'],
                bootstrap_upper_deg=micro_trial.response_limit(self.c) if micro_trial.response_limit(self.c)>8 else abs(d),
                adaptive_feedback=micro_trial.adaptive(self.c),bounded_crossing_feedback=recovery,
                stop_on_possible_overshoot=self.c.get('stop_on_possible_overshoot',False))
                for n,d in gains.items() if excluded_turns is None or n not in excluded_turns}
            # Medium corrections use stationary magnetic feedback at a configured carry
            # boundary or the end of a planned turn. Visual-only fine turns stay small.
            if (residual_driven and self.allow_missing_magnetic
                    and abs(error)>self.c['deadband_deg']):
                direction='right' if error<0 else 'left'
                medium_op=micro_trial.MEDIUM_OPS[int(direction=='right')]
                nominal=math.copysign(micro_trial.MEDIUM_NOMINAL_DEG,-error)
                cap=micro_trial.MEDIUM_MAX_RESPONSE_DEG
                medium_estimate=self.turn_estimator.control_estimate(direction,'medium',load,session,epoch,
                    sample.get('calibration_id',''),now_s=self.clock())
                upper=medium_estimate['upper_abs_deg'] if medium_estimate is not None else cap
                forecast_upper=(medium_estimate or {}).get('unclipped_upper_abs_deg',upper)
                # Accept a measured crossing if the full allowed turn can still
                # improve the error and leave at most a small overshoot.
                margin_limit=abs(error)+self.c['deadband_deg']+_STATIONARY_HEADING_SPREAD_DEG
                medium_left=(name in CORRIDOR_HEADING_ACTIONS and error>=_CORRIDOR_MEDIUM_LEFT_TRIGGER_DEG)
                if medium_left:
                    margin_limit=max(margin_limit,2*error-self.c['minimum_improvement_deg'])
                limit=min(cap,margin_limit)
                if (name in CORRIDOR_HEADING_ACTIONS and not medium_left
                        and abs(error)<=2*self.c['deadband_deg'] and chosen in candidates):
                    report['medium_candidate']={'action':medium_op,'status':'prefer_small_in_corridor'}
                elif excluded_turns is not None and medium_op in excluded_turns:
                    report['medium_candidate']={'action':medium_op,'status':excluded_turns[medium_op]}
                elif not getattr(self,'medium_turn_ready',lambda:False)():
                    report['medium_candidate']={'action':medium_op,'status':'capability_unavailable'}
                elif (type(upper) in (float,int) and type(forecast_upper) in (float,int)
                        and 0<upper<=limit and upper<=forecast_upper<=margin_limit):
                    gains={**gains,medium_op:nominal}
                    candidates[medium_op]=dict(direction=direction,action_type='medium',nominal_delta_deg=nominal,
                        max_step_deg=cap,bootstrap_lower_deg=micro_trial.MEDIUM_PRIOR_MIN_RESPONSE_DEG,
                        bootstrap_upper_deg=cap,bounded_crossing_feedback=True,prefer_on_tie=True)
                    report['medium_candidate']={'action':medium_op,'status':'considered','upper_abs_deg':upper}
                    if forecast_upper>upper:
                        report['medium_candidate']['unclipped_upper_abs_deg']=forecast_upper
                else:
                    report['medium_candidate']={'action':medium_op,'status':'predicted_response_exceeds_margin',
                        'upper_abs_deg':upper,'allowed_upper_abs_deg':limit,
                        'unclipped_upper_abs_deg':forecast_upper,'allowed_forecast_upper_abs_deg':margin_limit}
            choice=select_turn(error_deg=error,captured_at_s=min(sample['decision_capture_times']),now_s=self.clock(),
                candidates=candidates,estimator=self.turn_estimator,load=load,session=session,epoch=epoch,
                calibration_id=sample.get('calibration_id',''),
                deadband_deg=self.c['minimum_improvement_deg'] if residual_driven else self.c['deadband_deg'],
                release_deg=.5 if residual_driven else self.c['release_deg'],max_age_s=self.cfg['magnetic']['max_age_s'],
                allow_nominal_retry=self.allow_missing_magnetic and residual_driven and not self._nominal_retry_used,
                preferred_residual_band_deg=self.c['deadband_deg'] if residual_driven else 0.)
            report['motion_prediction']=choice
            if choice.get('status')!='selected':
                if residual_driven and choice.get('status')=='no_safe_candidate':
                    direction='left' if error>0 else 'right'
                    estimate=self.turn_estimator.control_estimate(direction,'micro',load,session,epoch,
                        sample.get('calibration_id',''),now_s=self.clock())
                    if (estimate and estimate.get('sample_count',0)>0
                            and 0<2*abs(error)<=estimate['lower_abs_deg']<=estimate['upper_abs_deg']<=self.c['max_step_deg']):
                        report.update(status='best_reachable_heading',best_reachable_with_current_steps=True,
                                      reason='verified residual is smaller than any available same-direction turn can improve',
                                      turn_lower_bound_deg=estimate['lower_abs_deg'])
                        return report
                if not route_feedback and self.c.get('stop_on_possible_overshoot') and 'possible_overshoot' in choice.get('rejected',{}).values():
                    self.fault('possible overshoot predicted; stop without reverse or continuation',report)
                report['reason']='prediction selection rejected: '+str(choice.get('reason') or choice['status']);return report
            chosen=choice['action'];delta=gains[chosen];conservative_bound=abs(choice['budget_delta_deg'])
        if micro_trial.enabled(self.c) and self.budget.limit_total_angle and self.total_nominal_deg+abs(delta)>self.c['micro_trial']['max_total_nominal_degrees']:
            report['reason']='nominal journey angle budget exhausted'
            return report
        permits_crossing=choice and (choice.get('adaptive_interval_verified') or choice.get('nominal_retry'))
        if not permits_crossing and (choice['upper_abs_deg'] if choice else abs(delta)) > abs(error) - self.c['release_deg']:
            report['reason'] = 'configured nominal/measured step risks overshoot'
            return report
        self.check()
        if getattr(self.io,'capture_stream',None) is not None and not self.io.motion_capture_verified:
            self._defer_stream_interruption('magnetic stream interrupted before correction')
            self.fault('capture stream no longer healthy before correction')
            report['reason']='capture stream no longer healthy before correction'
            return report
        budgeted=micro_trial.enabled(self.c)
        budget_available=not budgeted or self.budget.available(delta,conservative_bound,op=chosen)
        self.pending_decision=sample
        try:
            if budget_available:self.verify_pending_decision()
            if not budget_available or (budgeted and not self.budget.reserve(
                    'magnetic',chosen,delta,conservative_bound=conservative_bound)):
                report['reason']='shared magnetic/visual journey budget exhausted or locked'
                report['budget']=self.budget.status()
                return report
            turn_started=self.clock()
            accepted=correct(chosen)
            turn_completed=self.clock()
        finally:self.pending_decision=None
        if not accepted:
            report['reason'] = 'remaining action/time budget unavailable'
            return report
        if choice and choice.get('nominal_retry'):self._nominal_retry_used=True
        self.count += 1
        self.total_nominal_deg += abs(delta)
        self.last_action = self.clock()
        step_limit=micro_trial.MEDIUM_MAX_RESPONSE_DEG if chosen in micro_trial.MEDIUM_OPS else self.c['max_step_deg']
        report.update(status='corrected_once', correction_actions=1, correction=chosen)
        report['budget']=self.budget.status()
        report.update(angle_evidence='source_nominal' if micro_trial.enabled(self.c) else 'measured',
                      commanded_delta_deg=delta, physical_inflight_angle_limit_guaranteed=False)
        try:
            verified, post = self.sample(load,action_completed_monotonic=turn_completed)
        except MicroTrialStopped as exc:
            report.update(status='correction_locked',reason=str(exc),
                          correction_diagnostic=dict(failures=['post_sample_rejected'],
                          before_heading_deg=sample['raw_heading_deg'],before_error_deg=error,
                          commanded_delta_deg=delta,verification_error=str(exc)))
            exc.report=copy.deepcopy(report)
            raise
        if route_feedback:
            verified,post=self._recheck_magnetic_field(load,verified,post,report)
        self._defer_expired_target(self._target_observation_faults(post,load),report)
        sign = self.cfg['magnetic']['heading_sign']
        confirmed_feedback = False
        if recovery and post is not None and not self._target_observation_faults(post,load):
            initial_change = wrap(sign*(post['raw_heading_deg']-sample['raw_heading_deg']))
            initial_error = wrap(error+initial_change)
            trigger = None
            if route_feedback and initial_change*delta<=0:
                trigger='medium_nonresponse' if chosen in micro_trial.MEDIUM_OPS else 'small_wrong_way'
            elif abs(initial_change)>step_limit and (route_feedback or initial_change*delta>0):
                trigger='oversized_response'
            elif route_feedback and error*initial_error<0 and abs(initial_error)>self.c['deadband_deg']:
                trigger='unexpected_target_crossing'
            if trigger is not None:
                verified,post=self._confirm_unexpected_response(load,sample,post,report,trigger,
                    require_agreement=route_feedback)
                confirmed_feedback=route_feedback
        self.last_verified_sample=(verified,post)
        report['verification'] = verified
        failures=[]
        diagnostic=dict(before_heading_deg=sample['raw_heading_deg'],
            commanded_delta_deg=delta,before_error_deg=error,
            max_step_deg=step_limit,response_bound_is_diagnostic=route_feedback,
            minimum_improvement_deg=self.c['minimum_improvement_deg'],
            reference_norm=anchor['norm'],norm_fraction_limit=self.cfg['magnetic']['max_norm_fraction_change'])
        if post is None:
            failures.append('post_sample_missing')
        else:
            diagnostic.update(after_heading_deg=post['raw_heading_deg'],post_norm=post['norm'],
                post_capture_verified=post.get('capture_verified',False),
                norm_fraction_change=abs(post['norm']-anchor['norm'])/anchor['norm'])
            if not post.get('capture_verified',False):failures.append('post_capture_unverified')
            if diagnostic['norm_fraction_change']>self.cfg['magnetic']['max_norm_fraction_change']:
                failures.append('post_magnetic_magnitude_changed')
            change=wrap(sign*(post['raw_heading_deg']-sample['raw_heading_deg']))
            new_error=wrap(error+change)
            diagnostic.update(observed_change_deg=change,after_error_deg=new_error,
                              improvement_deg=abs(error)-abs(new_error))
            report['after_error_deg']=new_error
            if choice:report['prediction_update']=dict(predicted_error_deg=choice['predicted_error_deg'],observed_error_deg=new_error,innovation_deg=wrap(new_error-choice['predicted_error_deg']),posterior_error_deg=new_error,source='fresh_magnetic_observation',prediction_not_new_sample=True)
            if self.c.get('stop_on_possible_overshoot') and not recovery and error*new_error<0:
                failures.append('observed_target_crossing')
            if not confirmed_feedback and (change*delta<0 or (change==0 and not recovery)):
                failures.append('wrong_direction_or_zero_change')
            if not confirmed_feedback and abs(change)>step_limit:
                failures.append('observed_step_oversized')
            if diagnostic['improvement_deg']<self.c['minimum_improvement_deg'] and not recovery:
                failures.append('insufficient_improvement')
            if recovery:
                failures.extend(self._target_observation_faults(post,load))
                if not all(post.get(key)==sample.get(key) for key in _FEEDBACK_SCOPE_FIELDS):
                    failures.append('correction_observation_source_or_load_changed')
            if micro_trial.response_limit(self.c)>8:
                budget_ok=self.budget.reconcile(abs(change),feedback_only_response=route_feedback and not failures)
                if not budget_ok and 'observed_step_oversized' not in failures:
                    failures.append('observed_angle_budget_exhausted')
            if confirmed_feedback and not failures:
                status='observed_turn_feedback'
                exclusion='unexpected_response_at_this_boundary'
                if trigger=='medium_nonresponse':
                    status='medium_nonresponse_reobserved'
                    exclusion='unresponsive_at_this_boundary'
                elif trigger=='small_wrong_way':
                    status='small_wrong_way_reobserved'
                elif trigger=='oversized_response':
                    exclusion='overrun_at_this_boundary'
                if excluded_turns is not None:
                    excluded_turns[chosen]=exclusion
                report.update(status=status,excluded_turn=chosen,
                              reason='unexpected turn response independently verified; replan from measured heading',
                              reason_zh='转向实测与预期不符，已复测航向，按当前余差重新选择动作。')
        report['budget']=self.budget.status()
        diagnostic['failures']=failures
        report['correction_diagnostic']=diagnostic
        self.blocked=bool(failures)
        if self.blocked:
            try:
                self._defer_stream_interruption('magnetic stream interrupted after completed correction',failures)
            except MagneticDataUnavailable as exc:
                self.blocked=False
                exc.report=copy.deepcopy(report)
                raise
            report['status']='correction_locked'
            report['reason']='correction verification failed: '+','.join(failures)+'; no retry'
            self.fault(report['reason']+'; '+json.dumps(diagnostic,sort_keys=True),report)
        if not self.blocked and confirmed_feedback:
            report['micro_turn_observation']=dict(accepted=False,reason=f'{trigger}_not_used_for_learning')
        elif not self.blocked and ('first_stationary_verification' in report or report.get('magnetic_field_rechecks')):
            report['micro_turn_observation']=dict(accepted=False,reason='disputed_initial_response_not_used_for_learning')
        elif not self.blocked:
            report['micro_turn_observation']=self.observe_turn(chosen,load,sample,post,turn_started,turn_completed)
            if (choice and choice.get('nominal_retry') and report['micro_turn_observation']['accepted']
                    and diagnostic['improvement_deg']>=self.c['minimum_improvement_deg']):
                # A further trial needs a new measured response that actually improved the heading.
                self._nominal_retry_used=False
        self.check()
        return report
