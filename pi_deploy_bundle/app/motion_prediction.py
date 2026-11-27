"""Bounded prediction from real observations; this module never dispatches motion.

Callers still enforce load/capability, journey limits and post-action feedback.
A prediction is deliberately NOT returned as a fresh sensor observation.
"""
import math
from turn_estimator import TurnEstimator, wrap180

_NOMINAL_RETRY_DEG = 10.


def _num(x):return type(x) in (int,float) and math.isfinite(x)


def fuse_heading_errors(magnetic, visual, *, now_s, target_basis,
                        expected_tag_id, max_age_s=.8, max_skew_s=.3,
                        max_disagreement_deg=3.):
    """Inputs are dicts of error_deg/captured_at_s/mad_deg/target_basis/valid.

    Their errors must already refer to the same fixed target, not raw yaw.
    Vision additionally needs tag_id, capture_verified and pose_usable;
    magnet needs capture_verified and field_valid. Invalid vision may fall
    back to valid magnet; conflicting valid observations never get averaged.
    """
    if (not _num(now_s) or not isinstance(target_basis,str) or not target_basis or
            not all(_num(x) and x>0 for x in (max_age_s,max_skew_s,max_disagreement_deg))):
        return dict(status='invalid_request',reason='invalid_limits',error_deg=None)
    def valid(o,source):
        if not isinstance(o,dict):return False
        if o.get('valid') is not True or o.get('capture_verified') is not True:return False
        if o.get('target_basis')!=target_basis:return False
        if not all(_num(o.get(k)) for k in ('error_deg','captured_at_s','mad_deg')):return False
        if o['mad_deg']<0 or not 0<=now_s-o['captured_at_s']<=max_age_s:return False
        if source=='magnetic':return o.get('field_valid') is True
        return (type(expected_tag_id) is int and type(o.get('tag_id')) is int and
                o['tag_id']==expected_tag_id and o.get('pose_usable') is True)
    m=magnetic if valid(magnetic,'magnetic') else None
    v=visual if valid(visual,'visual') else None
    # Magnetic field failure is distinct from ordinary visual code loss.
    if isinstance(magnetic,dict) and magnetic.get('field_valid') is False:
        return dict(status='sensor_fault',reason='magnetic_field_invalid',error_deg=None)
    if m is None and v is None:
        return dict(status='no_observation',reason='no_valid_fresh_observation',error_deg=None)
    if m is not None and v is not None:
        if abs(m['captured_at_s']-v['captured_at_s'])>max_skew_s:
            return dict(status='disagreement',reason='observation_time_skew',error_deg=None)
        difference=wrap180(v['error_deg']-m['error_deg'])
        if abs(difference)>max_disagreement_deg:
            return dict(status='disagreement',reason='same_target_heading_disagreement',error_deg=None)
        # Modest visual influence; disagreement is gated before fusion.
        wm=1./max(1.,m['mad_deg'])**2
        wv=1./max(1.,v['mad_deg'])**2
        visual_weight=min(.5,wv/(wm+wv))
        error=wrap180(m['error_deg']+visual_weight*difference)
        stamp=min(m['captured_at_s'],v['captured_at_s'])
        return dict(status='observed',error_deg=error,captured_at_s=stamp,
                    target_basis=target_basis,source='magnetic_visual',predicted_only=False)
    o=m if m is not None else v
    return dict(status='observed',error_deg=wrap180(o['error_deg']),
                captured_at_s=o['captured_at_s'],target_basis=target_basis,
                source='magnetic' if m is not None else 'visual',predicted_only=False)


def select_turn(*, error_deg, captured_at_s, now_s, candidates, estimator,
                load, session, epoch, calibration_id, max_age_s=.8,
                deadband_deg=8., release_deg=4., min_improvement_deg=1.,
                allow_nominal_retry=False, preferred_residual_band_deg=0.):
    """Select at most one allowed op by conservative predicted residual.

    candidates: {op: {direction: 'left'|'right', action_type: 'macro'|'micro',
                      nominal_delta_deg: signed degrees, max_step_deg: limit}}.
    A caller may authorize one nominal micro-turn retry when a learned interval
    is too wide. The measured response limit still sets the retry's budget,
    and even the original interval must predict improvement.
    Within an explicit operational residual band, prefer the flagged action
    over chasing a smaller error. All candidates still pass the same gates.
    """
    if (not all(_num(x) for x in (error_deg,captured_at_s,now_s,max_age_s,
                                 deadband_deg,release_deg,min_improvement_deg,preferred_residual_band_deg)) or
            not 0<release_deg<deadband_deg or max_age_s<=0 or min_improvement_deg<=0
            or preferred_residual_band_deg<0):
        return dict(status='invalid_request',action=None)
    if not 0<=now_s-captured_at_s<=max_age_s:
        return dict(status='no_observation',action=None,reason='stale_or_future_observation')
    error=wrap180(error_deg)
    if abs(error)<=deadband_deg:return dict(status='within_deadband',action=None)
    accepted=[];rejected={}
    for name,c in candidates.items():
        direction=c.get('direction');kind=c.get('action_type')
        nominal=c.get('nominal_delta_deg');limit=c.get('max_step_deg')
        if (direction not in ('left','right') or kind not in ('macro','micro','medium') or
                not _num(nominal) or not _num(limit) or not 0<abs(nominal)<=limit<180 or
                (1 if direction=='right' else -1)*nominal<=0):
            rejected[name]='invalid_candidate';continue
        if nominal*error>=0:
            rejected[name]='wrong_direction';continue
        estimate=estimator.control_estimate(direction,kind,load,session,epoch,
                                            calibration_id,now_s=now_s)
        delta=estimate.get('degrees') if estimate else nominal
        bootstrap=c.get('bootstrap_upper_deg',abs(nominal))
        if not _num(bootstrap) or not abs(nominal)<=bootstrap<=limit:
            rejected[name]='invalid_bootstrap_bound';continue
        upper=estimate.get('upper_abs_deg') if estimate else bootstrap
        lower=estimate.get('lower_abs_deg') if estimate else c.get('bootstrap_lower_deg',abs(nominal))
        if not all(_num(value) for value in (delta,upper,lower)):
            rejected[name]='unreliable_or_oversized_prediction';continue
        original_prediction=None
        if (allow_nominal_retry is True and kind=='micro' and c.get('adaptive_feedback') is True
                and c.get('bounded_crossing_feedback') is True and estimate
                and estimate.get('source')=='adaptive_micro_low_pass'
                and type(estimate.get('sample_count')) is int and estimate['sample_count']>0
                and isinstance(estimator,TurnEstimator) and estimator.bounds['micro']==limit
                and delta*error<0 and min_improvement_deg<=lower<=abs(delta)<=limit<upper
                and min_improvement_deg<=_NOMINAL_RETRY_DEG<=limit
                and abs(abs(error)-upper)<=abs(error)-min_improvement_deg):
            # The estimator only learns responses within this same limit. The
            # original upper can exceed it due to forecast uncertainty.
            original_prediction=dict(delta_deg=delta,lower_abs_deg=lower,upper_abs_deg=upper,
                                     source=estimate['source'],sample_count=estimate['sample_count'])
            delta=math.copysign(_NOMINAL_RETRY_DEG,nominal)
            lower,upper=min(lower,abs(delta)),limit
        if delta*error>=0 or upper>limit or lower<min_improvement_deg:
            rejected[name]='unreliable_or_oversized_prediction';continue
        if c.get('stop_on_possible_overshoot') is True and not c.get('bounded_crossing_feedback') and upper>abs(error):
            rejected[name]='possible_overshoot';continue
        adaptive_interval_verified=False
        worst_residual=None
        pickup_trial=False
        # Only observed, opt-in adaptive intervals may reach/cross the target.
        # A nominal/bootstrap bound is not enough evidence to relax this gate.
        if c.get('bounded_crossing_feedback') is True:
            if not all(_num(v) for v in (lower,upper,delta)) or not 0<lower<=abs(delta)<=upper<=limit:
                rejected[name]='invalid_bounded_interval';continue
            worst_residual=max(abs(abs(error)-lower),abs(abs(error)-upper))
            if worst_residual>abs(error)-min_improvement_deg:
                # At pickup we have a fixed approach target and can measure the
                # outcome before the first lateral gait. A nominally beneficial
                # small turn may tie at the endpoint of its prediction interval.
                pickup_trial=(kind=='micro' and c.get('pickup_retreat_trial') is True
                              and abs(error+delta)<=preferred_residual_band_deg
                              and worst_residual<=abs(error)+1e-9)
                if (not pickup_trial and kind=='micro' and c.get('pickup_retreat_trial') is True
                        and c.get('pickup_residual_confirmed') is True and c.get('adaptive_feedback') is True):
                    # With no measured response for this load/direction the MCU's nominal
                    # stroke can be useful even when the broad startup interval is not.
                    # A learned response always supersedes the nominal one.
                    trial_delta = nominal if not estimate or estimate.get('source')=='adaptive_micro_prior' else delta
                    if lower <= abs(trial_delta) <= upper and abs(error + trial_delta) <= abs(error) - min_improvement_deg:
                        delta = trial_delta
                        pickup_trial = True
                if not pickup_trial:
                    rejected[name]='insufficient_worst_case_improvement';continue
            adaptive_interval_verified=original_prediction is None
        elif (c.get('adaptive_feedback') is True and estimate is not None
              and estimate.get('source') not in ('adaptive_micro_prior','adaptive_medium_prior')):
            if not all(_num(v) for v in (lower,upper,delta)) or not 0<=lower<=abs(delta)<=upper:
                rejected[name]='invalid_observed_interval';continue
            worst_residual=max(abs(abs(error)-lower),abs(abs(error)-upper))
            if upper>abs(error)+release_deg:
                rejected[name]='predicted_overshoot_beyond_release';continue
            if worst_residual>abs(error)-min_improvement_deg:
                rejected[name]='insufficient_worst_case_improvement';continue
            adaptive_interval_verified=True
        elif upper>abs(error)-release_deg:
            rejected[name]='predicted_release_margin_crossing';continue
        residual=wrap180(error+delta)
        if abs(residual)>abs(error)-min_improvement_deg:
            rejected[name]='insufficient_predicted_improvement';continue
        candidate=dict(action=name,predicted_error_deg=residual,
                       predicted_delta_deg=delta,upper_abs_deg=upper,lower_abs_deg=lower,
                       adaptive_interval_verified=adaptive_interval_verified,worst_predicted_residual_deg=worst_residual,
                       budget_delta_deg=math.copysign(max(abs(nominal),upper),delta),
                       evidence=estimate.get('source','observed_recent_three') if estimate else ('source_nominal_with_bootstrap_bound' if bootstrap>abs(nominal) else 'source_nominal'),
                       sample_count=estimate.get('sample_count',0) if estimate else 0,physical_upper_bound_verified=False,
                       predicted_only=True,observation_time_unchanged=captured_at_s)
        if original_prediction is not None:
            candidate.update(evidence='adaptive_micro_nominal_retry',sample_count=0,nominal_retry=True,
                             original_prediction=original_prediction)
        if pickup_trial:
            candidate['pickup_retreat_trial']=True
            if c.get('pickup_residual_confirmed'):
                candidate['pickup_residual_confirmed']=True
        accepted.append(candidate)
    if not accepted:return dict(status='no_safe_candidate',action=None,rejected=rejected)
    def score(item):
        residual=abs(item['predicted_error_deg'])
        return (item.get('nominal_retry',False),max(0.,residual-preferred_residual_band_deg),
                candidates[item['action']].get('prefer_on_tie') is not True,residual,item['upper_abs_deg'])
    chosen=min(accepted,key=score)
    return dict(chosen,status='selected',rejected=rejected)
