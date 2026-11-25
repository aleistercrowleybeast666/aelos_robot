"""Explicit limited trial using dedicated MCU micro exports, never the 45-degree macros."""
import math

OPS = ('single_micro_left', 'single_micro_right')
MEDIUM_OPS = ('single_medium_left', 'single_medium_right')
MEDIUM_NOMINAL_DEG = 10.
MEDIUM_PRIOR_MIN_RESPONSE_DEG = 6.
MEDIUM_MAX_RESPONSE_DEG = 28.
CONFIRMED_RIGHT_OVERRUN_LIMIT_DEG = 30.

def enabled(c):
    return c.get('micro_trial', {}).get('enabled') is True

def adaptive(c):
    return c.get('micro_trial', {}).get('adaptive_feedback') is True

def response_limit(c):
    return 20. if adaptive(c) else 8.

def active(cfg):
    return bool(cfg.get('post_action_heading',{}).get('execute') or cfg.get('visual_micro',{}).get('execute'))

def deltas(c):
    return c['micro_trial']['signed_nominal_delta_deg'] if enabled(c) else c['turn_delta_deg']

def validate(cfg):
    c = cfg['post_action_heading']
    t = c.get('micro_trial', {})
    if not enabled(c):
        return
    if t.get('angle_evidence') != 'source_nominal' or t.get('nominal_degrees') != 5 or t.get('measured_degrees') is not None:
        raise ValueError('micro angle must remain source_nominal 5; no measured claim')
    if t.get('source_nominal_authorized') is not True:
        raise ValueError('explicit nominal trial authorization required')
    if not (t.get('bounded_route_mode') is True and 1<=c['max_actions_per_journey']<=16 and t.get('max_total_nominal_degrees')==5*c['max_actions_per_journey'] and 1<=t.get('max_actions_per_checkpoint',0)<=4) and (c['max_actions_per_journey'] != 1 or t.get('max_total_nominal_degrees') != 5):
        raise ValueError('first micro trial budget must be one action / nominal five degrees')
    if t.get('bounded_route_mode') and (type(t.get('max_actions_per_turn_group',4)) is not int or not 1<=t.get('max_actions_per_turn_group',4)<=8):raise ValueError('bounded planned turn correction count')
    if (c['deadband_deg'] != 8 or c['release_deg'] != 4 or c['max_error_deg'] != (60 if t.get('bounded_route_mode') else 30) or
            c['max_step_deg'] != response_limit(c) or c['minimum_improvement_deg'] != 1):
        raise ValueError('first micro trial thresholds must be 8/4/30, post-check 8, improvement 1')
    if any(c.get('turn_delta_deg', {}).get(n) is not None for n in ('left_turn', 'right_turn')):
        raise ValueError('existing 45-degree route macros must not be labeled as five-degree micro actions')
    if cfg.get('search_enabled') or cfg.get('pose_correction_enabled'):
        raise ValueError('first magnetic micro trial does not authorize visual turn/strafe/search')
    if not active(cfg):
        return
    measured = c['directions_confirmed'] and t.get('direction_evidence') == 'measured_r3'
    assumed = (not c['directions_confirmed'] and t.get('direction_evidence') == 'source_nominal_unverified'
               and t.get('direction_assumption_authorized') is True and t.get('source_nominal_authorized') is True)
    if not (measured or assumed) or t.get('heading_sign_evidence') != 'measured_r3':
        raise ValueError('local micro direction: measured or explicitly authorized nominal assumption and measured magnetic sign required')
    for n in OPS:
        v = t.get('signed_nominal_delta_deg', {}).get(n)
        if type(v) not in (int, float) or not math.isfinite(v) or abs(v) != 5:
            raise ValueError('signed nominal micro delta required: '+n)
    if deltas(c)[OPS[0]] * deltas(c)[OPS[1]] >= 0:
        raise ValueError('local micro directions must oppose')
    if not cfg['magnetic'].get('capture_contract') or not cfg['magnetic'].get('capture_freshness_verified'):
        raise ValueError('fresh magnetic capture contract and field validation required')
    if type(cfg['magnetic'].get('heading_sign')) is not int or cfg['magnetic']['heading_sign'] not in (-1, 1):
        raise ValueError('magnetic heading_sign unverified')


class MicroBudget:
    """One journey, shared by all magnetic and visual calls; never refund attempts."""
    def __init__(self,max_actions=1,max_step_deg=8,*,limit_total_angle=True):
        if type(max_actions) is not int or not 1<=max_actions<=16:raise ValueError("bounded micro budget")
        if type(max_step_deg) not in (int,float) or not math.isfinite(max_step_deg) or not 8<=max_step_deg<=20:raise ValueError('bounded response limit')
        self.max_step_deg=float(max_step_deg)
        self.max_actions=max_actions
        self.limit_total_angle=limit_total_angle is not False
        self.actions=0;self.nominal_degrees=0.;self.accounted_degrees=0.;self.locked=False;self.reason=None;self.history=[]
    def available(self,delta,conservative_bound=None,*,op=None):
        bound=abs(delta) if conservative_bound is None else conservative_bound
        if op in MEDIUM_OPS:
            return (type(bound) in (int,float) and math.isfinite(bound)
                    and abs(delta)==MEDIUM_NOMINAL_DEG and MEDIUM_NOMINAL_DEG<=bound<=MEDIUM_MAX_RESPONSE_DEG
                    and self.max_step_deg==20. and not self.limit_total_angle
                    and not self.locked and self.actions<self.max_actions)
        return (type(bound) in (int,float) and math.isfinite(bound) and 5<=bound<=self.max_step_deg and not self.locked and self.actions<self.max_actions and abs(delta)==5 and (not self.limit_total_angle or self.accounted_degrees+bound<=5*self.max_actions))
    def reserve(self,source,op,delta,conservative_bound=None):
        if not (source in ('magnetic','visual') and op in OPS or source=='magnetic' and op in MEDIUM_OPS):
            raise ValueError('dedicated turn budget scope')
        if not self.available(delta,conservative_bound,op=op):return False
        self.actions+=1;self.nominal_degrees+=abs(delta)
        bound=abs(delta) if conservative_bound is None else conservative_bound
        self.accounted_degrees+=bound
        self.history.append(dict(source=source,op=op,signed_nominal_delta_deg=delta,evidence='source_nominal',conservative_accounting_deg=bound,attempt_reserved_before_dispatch=True))
        return True
    def reconcile(self,conservative_bound,*,confirmed_right_overrun=False,feedback_only_response=False):
        """Charge the observed response; opted-in route feedback uses the action count."""
        if not self.history or type(conservative_bound) not in (int,float) or not math.isfinite(conservative_bound) or conservative_bound<0:
            raise ValueError('finite observed response bound and reserved attempt required')
        row=self.history[-1]
        prior=row['conservative_accounting_deg']
        new=max(prior,float(conservative_bound))
        self.accounted_degrees+=new-prior
        row['conservative_accounting_deg']=new
        row['observed_response_bound_deg']=float(conservative_bound)
        tolerated_overrun=(confirmed_right_overrun is True and not self.locked and not self.limit_total_angle
                           and row['source']=='magnetic' and row['op']=='single_micro_right'
                           and self.max_step_deg==20. and self.max_step_deg<new<=CONFIRMED_RIGHT_OVERRUN_LIMIT_DEG)
        if tolerated_overrun:row['confirmed_right_overrun']=True
        route_feedback=(feedback_only_response is True and not self.locked
                        and not self.limit_total_angle and self.max_step_deg==20.
                        and row['source'] in ('magnetic','visual'))
        if route_feedback:row['feedback_only_response']=True
        response_limit=MEDIUM_MAX_RESPONSE_DEG if row['op'] in MEDIUM_OPS else self.max_step_deg
        if (new>response_limit and not (tolerated_overrun or route_feedback)
                or self.limit_total_angle and self.accounted_degrees>5*self.max_actions):
            self.lock('observed response exceeded bounded journey allowance')
            return False
        return True
    def lock(self,reason):self.locked=True;self.reason=str(reason)
    def status(self):
        return dict(max_actions=self.max_actions,max_response_deg=self.max_step_deg,max_nominal_degrees=5*self.max_actions,used_actions=self.actions,used_nominal_degrees=self.nominal_degrees,used_conservative_degrees=self.accounted_degrees,
                    locked=self.locked,reason=self.reason,history=list(self.history),measured_degrees=None,
                    total_angle_limit_enabled=self.limit_total_angle)
