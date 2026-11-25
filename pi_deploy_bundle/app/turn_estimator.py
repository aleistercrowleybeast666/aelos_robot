"""Read-only turn-gain observations; never selects or dispatches a motion.

Times are from one monotonic clock. Windows must already have passed the
existing magnetic stability/freshness checks. Do not synthesize their evidence.
"""
from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import dataclass
import math
from statistics import median
from micro_trial import CONFIRMED_RIGHT_OVERRUN_LIMIT_DEG, MEDIUM_MAX_RESPONSE_DEG, MEDIUM_NOMINAL_DEG, MEDIUM_PRIOR_MIN_RESPONSE_DEG


_CONTROL_MAX_AGE_S = 120.
MICRO_INITIAL_DEG = 10.
MICRO_FILTER_ALPHA = .15
_MICRO_PREDICTION_MARGIN_DEG = 3.


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def wrap180(value):
    return (value + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class TurnWindow:
    heading_deg: float
    sample_time_s: float
    norm: float
    session: str
    epoch: int
    load: str
    calibration_id: str
    stable_samples: int = 0
    capture_verified: bool = False
    norm_basis: str = 'calibrated_horizontal'


@dataclass(frozen=True)
class _TurnFilterState:
    magnitude_deg: float
    sample_count: int = 0
    observed_at_s: float | None = None


class TurnEstimator:
    """Keep scoped measured statistics and adaptive small/medium forecasts."""
    def __init__(self, max_age_s=0.8, max_norm_change=0.35,
                 min_angle_deg=1.0, max_macro_deg=90.0, max_micro_deg=8.0, allow_provisional=False,
                 *, micro_initial_deg=MICRO_INITIAL_DEG, micro_alpha=MICRO_FILTER_ALPHA):
        values = (max_age_s, max_norm_change, min_angle_deg,
                  max_macro_deg, max_micro_deg)
        if not all(_finite(v) and v > 0 for v in values):
            raise ValueError('finite positive estimator limits required')
        if not max_norm_change < 1 or not min_angle_deg < max_micro_deg <= max_macro_deg < 180:
            raise ValueError('invalid turn estimator bounds')
        if type(allow_provisional) is not bool: raise ValueError("adaptive flag must be bool")
        if not _finite(micro_initial_deg) or micro_initial_deg<=0 or not _finite(micro_alpha) or not 0<=micro_alpha<=1:
            raise ValueError('invalid micro turn low-pass filter')
        self.micro_initial_deg = micro_initial_deg
        self.micro_alpha = micro_alpha
        self.adaptive_feedback = allow_provisional
        self.max_age_s = max_age_s
        self.max_norm_change = max_norm_change
        self.min_angle_deg = min_angle_deg
        self.bounds = {'macro': max_macro_deg, 'micro': max_micro_deg, 'medium': MEDIUM_MAX_RESPONSE_DEG}
        self._bins = OrderedDict()
        self._last_end = {}
        self._observed_timing = {}
        self._filter_states = {}

    def reset(self):
        self._bins.clear()
        self._last_end.clear()
        self._observed_timing.clear()
        self._filter_states.clear()

    def _key(self, direction, action_type, window):
        return (window.session, window.epoch, window.calibration_id,
                window.load, action_type, direction)

    def estimate(self, direction, action_type, load, session, epoch, calibration_id):
        key = (session, epoch, calibration_id, load, action_type, direction)
        rows = self._bins.get(key, ())
        if len(rows) < (1 if self.adaptive_feedback else 3):
            return None
        degrees = median(rows)
        return {'degrees': degrees, 'sample_count': len(rows),
                'mad_deg': median(abs(x - degrees) for x in rows),
                'source': ('observed_provisional' if len(rows)<3 else 'observed_recent_three'), 'used_for_control': False,
                'physical_upper_bound_verified': False}

    def control_estimate(self, direction, action_type, load, session, epoch,
                         calibration_id, *, now_s, max_age_s=_CONTROL_MAX_AGE_S,
                         max_mad_deg=1., max_spread_deg=3., margin_deg=1.):
        """Conservative interval for prediction, not a physical angle guarantee."""
        key=(session,epoch,calibration_id,load,action_type,direction)
        limits=(now_s,max_age_s,max_mad_deg,max_spread_deg,margin_deg)
        if not all(_finite(x) for x in limits):return None
        if min(max_age_s,max_mad_deg,max_spread_deg,margin_deg)<=0:return None
        if self.adaptive_feedback and action_type in ('micro','medium'):
            if (direction not in ('left', 'right') or type(epoch) is not int or epoch < 0
                    or not all(isinstance(value, str) and value for value in (load, session, calibration_id))):
                return None
            return self._filtered_control_estimate(key, now_s, max_age_s, margin_deg)
        result=self.estimate(direction,action_type,load,session,epoch,calibration_id)
        if not result:return None
        timing=self._observed_timing.get(key,())
        stamp=min((completed for completed,_ in timing),default=float('-inf'))
        if not 0<=now_s-stamp<=max_age_s:return None
        values=[abs(x) for x in self._bins[key]]
        if not self.adaptive_feedback and (result['mad_deg']>max_mad_deg or max(values)-min(values)>max_spread_deg):return None
        if self.adaptive_feedback:margin_deg=max(3.,margin_deg)
        result=dict(result)
        result.update(lower_abs_deg=max(0.,min(values)-margin_deg),
                      upper_abs_deg=max(values)+margin_deg,
                      predicted_only=True,observed_at_s=stamp,
                      cycle_elapsed_s=median(elapsed for _,elapsed in timing))
        return result

    def _turn_prior(self, action_type):
        initial=MEDIUM_NOMINAL_DEG if action_type=='medium' else self.micro_initial_deg
        magnitude=max(self.min_angle_deg,min(initial,self.bounds[action_type]))
        return _TurnFilterState(magnitude)

    def _filtered_control_estimate(self, key, now_s, max_age_s, margin_deg):
        action_type=key[-2]
        cap=self.bounds[action_type]
        state=self._filter_states.get(key)
        elapsed = None
        unclipped_upper = None
        if state is None or not 0 <= now_s - state.observed_at_s <= max_age_s:
            state=self._turn_prior(action_type)
            nominal_lower=MEDIUM_PRIOR_MIN_RESPONSE_DEG if action_type=='medium' else state.magnitude_deg/2.
            lower=max(self.min_angle_deg,nominal_lower)
            upper=cap
            source=f'adaptive_{action_type}_prior'
        else:
            spread = max(margin_deg, _MICRO_PREDICTION_MARGIN_DEG)
            recent = [(abs(delta),duration) for delta,(stamp,duration) in
                      zip(self._bins[key],self._observed_timing[key]) if 0<=now_s-stamp<=max_age_s]
            values = [delta for delta,_ in recent]
            elapsed = median(duration for _,duration in recent)
            measured_min=min(values)
            observed_lower=max(0., measured_min-spread, min(measured_min,state.magnitude_deg-spread))
            # A fast first turn can put the observation-based lower bound
            # above the still-converging low-pass forecast.
            lower=min(state.magnitude_deg,observed_lower)
            upper=max(state.magnitude_deg,max(values))+spread
            if action_type=='medium' and cap < upper and max(values)<=cap:
                # This is a planned verification envelope. The actual response
                # still has to pass the post-turn cap before another action.
                unclipped_upper,upper=upper,cap
            source=(f'adaptive_{action_type}_overrun_observed' if max(values)>cap
                    else f'adaptive_{action_type}_low_pass')
        result={'degrees': state.magnitude_deg if key[-1] == 'right' else -state.magnitude_deg,
                'lower_abs_deg': lower, 'upper_abs_deg': upper, 'sample_count': state.sample_count,
                'source': source, 'observed_at_s': state.observed_at_s,
                'alpha': self.micro_alpha,
                'cycle_elapsed_s': elapsed,
                'predicted_only': True, 'physical_upper_bound_verified': False}
        if unclipped_upper is not None:
            result['unclipped_upper_abs_deg']=unclipped_upper
        return result

    def _update_turn_filter(self, key, magnitude, completed_at_s):
        state=self._filter_states.get(key)
        if state is None or completed_at_s - state.observed_at_s > _CONTROL_MAX_AGE_S:
            state=self._turn_prior(key[-2])
            self._bins[key].clear()
            self._observed_timing.pop(key, None)
        filtered = (1. - self.micro_alpha)*state.magnitude_deg + self.micro_alpha*magnitude
        self._filter_states[key]=_TurnFilterState(filtered,state.sample_count+1,completed_at_s)
        sign = 1 if key[-1]=='right' else -1
        return {'raw_delta_deg':sign*magnitude, 'before_deg':sign*state.magnitude_deg,
                'after_deg':sign*filtered, 'alpha':self.micro_alpha}

    def observe(self, *, direction, action_type, before, after, started_at_s,
                completed_at_s, now_s, protocol_completed=False, confirmed_right_overrun=False):
        def reject(reason):
            return {'accepted': False, 'reason': reason, 'estimate': None}
        if direction not in ('left', 'right') or action_type not in self.bounds:
            return reject('unsupported_action')
        if protocol_completed is not True:
            return reject('incomplete_operation')
        if not isinstance(before, TurnWindow) or not isinstance(after, TurnWindow):
            return reject('invalid_window')
        for w in (before, after):
            if not all(_finite(v) for v in (w.heading_deg, w.sample_time_s, w.norm)):
                return reject('nonfinite_sample')
            if not 0 <= w.heading_deg <= 360 or w.norm <= 0:
                return reject('invalid_sample')
            if (not isinstance(w.session, str) or not w.session or
                    type(w.epoch) is not int or w.epoch < 0 or
                    not isinstance(w.load, str) or not w.load or
                    not isinstance(w.calibration_id, str) or not w.calibration_id):
                return reject('missing_scope')
            if (w.capture_verified is not True or type(w.stable_samples) is not int or
                    w.stable_samples < 3):
                return reject('unverified_window')
            if w.norm_basis != 'calibrated_horizontal':
                return reject('uncalibrated_magnitude')
        if (before.session, before.epoch) != (after.session, after.epoch):
            return reject('session_or_epoch_changed')
        if before.load != after.load:
            return reject('load_changed')
        if before.calibration_id != after.calibration_id:
            return reject('calibration_changed')
        if not all(_finite(t) for t in (started_at_s, completed_at_s, now_s)):
            return reject('invalid_time')
        if not (before.sample_time_s <= started_at_s < completed_at_s <= after.sample_time_s <= now_s):
            return reject('invalid_time_order')
        if (started_at_s - before.sample_time_s > self.max_age_s or
                now_s - after.sample_time_s > self.max_age_s):
            return reject('stale_window')
        if abs(after.norm / before.norm - 1.0) > self.max_norm_change:
            return reject('magnetic_magnitude_changed')
        # Any duplicate or overlapping operation in the same owner scope is rejected,
        # even if a caller changes the action or load label.
        scope = (after.session, after.epoch)
        if started_at_s < self._last_end.get(scope, float('-inf')):
            return reject('duplicate_or_overlapping_operation')
        delta = wrap180(after.heading_deg - before.heading_deg)
        sign = 1 if direction == 'right' else -1
        if delta * sign <= 0:
            return reject('wrong_direction_or_zero')
        if abs(delta) < self.min_angle_deg:
            return reject('below_observation_resolution')
        overrun = (confirmed_right_overrun is True and self.adaptive_feedback and direction=='right'
                   and action_type=='micro' and after.load=='single' and self.bounds['micro']==20.
                   and self.bounds['micro']<delta<=CONFIRMED_RIGHT_OVERRUN_LIMIT_DEG)
        if abs(delta) > self.bounds[action_type] and not overrun:
            return reject('turn_out_of_bounds')
        key = self._key(direction, action_type, after)
        if key not in self._bins:
            self._bins[key] = deque(maxlen=3)
        update = None
        if self.adaptive_feedback and action_type in ('micro','medium'):
            update=self._update_turn_filter(key,abs(delta),completed_at_s)
        self._bins[key].append(delta)
        self._bins.move_to_end(key)
        self._last_end[scope] = completed_at_s
        # Only accepted operations contribute elapsed time. The cycle includes
        # the actual dispatch, native completion, settling, and fresh feedback.
        self._observed_timing.setdefault(key,deque(maxlen=3)).append((completed_at_s,now_s-started_at_s))
        while len(self._bins) > 64:
            old_key, _ = self._bins.popitem(last=False)
            self._observed_timing.pop(old_key, None)
            self._filter_states.pop(old_key,None)
        while len(self._last_end) > 64:
            del self._last_end[next(iter(self._last_end))]
        result = {'accepted': True, 'reason': 'confirmed_right_overrun' if overrun else 'recorded_only', 'delta_deg': delta,
                'sample_count': len(self._bins[key]),
                'estimate': self.estimate(direction, action_type, after.load,
                                          after.session, after.epoch, after.calibration_id)}
        if update is not None:result['filter_update']=update
        return result
