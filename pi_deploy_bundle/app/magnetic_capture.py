"""Bounded software acquisition evidence, not a hardware timestamp.

No ROS imports, no device construction and no calibration writes. The sole
existing sensor owner supplies its existing QMC object. A status handshake and
one 6-byte burst replace get_data/get_bearing's multiple independent reads.
"""
import copy
import hashlib
import json
import math
import threading
import time
import uuid
from enum import Enum

SCHEMA = 'aelos.qmc.capture.v1'
DRDY, OVL, DOR = 1, 2, 4
STATUS, DATA, CONTROL_1 = 6, 0, 9


class CaptureError(ValueError):
    pass


class MagneticRebindResult(Enum):
    REBOUND = 'rebound'
    UNAVAILABLE = 'unavailable'


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def calibration_id(calibration):
    return hashlib.sha256(json.dumps(calibration, sort_keys=True,
        separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def heading_from_xyz(xyz, calibration):
    """Preserve sensor_magnet v0/v1 formulas on exactly this acquisition."""
    x, y, z = xyz
    if calibration['yaml_version'] == 0:
        xx = (x - calibration['x_offset']) * calibration['x_scale']
        yy = -(y - calibration['y_offset']) * calibration['y_scale']
        if not finite(xx) or not finite(yy) or math.hypot(xx, yy) == 0:
            raise CaptureError('degenerate calibrated horizontal field')
        compass = math.atan2(yy, xx) * (180.0 / 3.14159265358979) + 90
        if compass > 360:
            compass -= 360
        if compass < 0:
            compass += 360
        return int(math.floor(360 - compass)) % 360
    if calibration['yaml_version'] != 1:
        raise CaptureError('unknown calibration format')
    c = calibration['matrix']
    xx = x*c[0][0] + y*c[0][1] + c[0][2]
    yy = x*c[1][0] + y*c[1][1] + c[1][2]
    d = calibration['declination_deg']
    if not all(finite(v) for v in (xx, yy, d)) or math.hypot(xx, yy) == 0:
        raise CaptureError('degenerate calibrated horizontal field')
    b = math.degrees(math.atan2(yy, xx)) % 360
    b = (b + d) % 360
    return (int(b) - 90) % 360


class CapturePolicy:
    def __init__(self, conversion_bound_s, transaction_bound_s,
                 publication_bound_s, transport_bound_s, poll_s=.002):
        for v in (conversion_bound_s, transaction_bound_s,
                  publication_bound_s, transport_bound_s, poll_s):
            if not finite(v) or v <= 0:
                raise ValueError('positive finite acquisition bounds required')
        if not poll_s < conversion_bound_s < transaction_bound_s <= 1:
            raise ValueError('inconsistent transaction/conversion bounds')
        self.conversion_bound_s = conversion_bound_s
        self.transaction_bound_s = transaction_bound_s
        self.publication_bound_s = publication_bound_s
        self.transport_bound_s = transport_bound_s
        self.poll_s = poll_s
        # Oldest represented measurement plus one publication slot and delivery.
        self.max_age_s = (conversion_bound_s + transaction_bound_s +
                          publication_bound_s + transport_bound_s)

    def wire(self):
        return {k: getattr(self, k) for k in (
            'conversion_bound_s', 'transaction_bound_s',
            'publication_bound_s', 'transport_bound_s', 'max_age_s')}


class AtomicQmcCapture:
    def __init__(self, device, calibration_getter, boot_id, build_id, policy,
                 clock=time.monotonic, sleep=time.sleep, session=None):
        if not boot_id or not build_id:
            raise ValueError('boot and source identity required')
        self.device = device
        self.calibration_getter = calibration_getter
        self.boot_id, self.build_id, self.policy = boot_id, build_id, policy
        self.clock, self.sleep = clock, sleep
        self.session = session or uuid.uuid4().hex
        self.seq = self.attempt = 0
        self.fault_generation = 0
        self.lock = threading.RLock()
        self.latest = None

    def _status(self):
        s = self.device.bus.read_byte_data(self.device.address, STATUS)
        if type(s) is not int or not 0 <= s <= 7:
            raise CaptureError('invalid_status')
        return s

    def _control(self):
        value = self.device.bus.read_byte_data(self.device.address, CONTROL_1)
        if type(value) is not int or value != 0x11:
            raise CaptureError('control_1_not_10hz_continuous_8g_osr512')
        return value

    def _burst(self):
        b = self.device.bus.read_i2c_block_data(self.device.address, DATA, 6)
        if len(b) != 6 or any(type(v) is not int or not 0 <= v <= 255 for v in b):
            raise CaptureError('short_or_invalid_burst')
        xyz = [b[i] | b[i+1] << 8 for i in (0, 2, 4)]
        return [v - 65536 if v >= 32768 else v for v in xyz]

    def snapshot(self):
        with self.lock:
            return copy.deepcopy(self.latest)

    def acquire(self, enabled=True):
        """Every attempt publishes a new outcome; failed attempts retain no XYZ.

        A pending measurement is discarded. Only a clean DRDY low -> high edge
        followed by one complete burst and clean low status can be valid. The
        conservative acquisition lower bound subtracts a full configured sensor
        conversion bound, covering an integration already underway at low.
        I2C calls themselves are kernel calls: late return is rejected, never
        claimed to be cancellable by this Python timeout.
        """
        with self.lock:
            self.attempt += 1
            start = self.clock()
            p = self.policy
            row = dict(schema=SCHEMA, boot_id=self.boot_id,
                producer_build_id=self.build_id, session=self.session,
                seq=self.seq, attempt=self.attempt, valid=False, error=None,
                fault_generation=self.fault_generation,
                xyz=None, heading_deg=None, calibration_id=None,
                read_started_monotonic=start, read_finished_monotonic=None,
                capture_not_before_monotonic=None, ready_observed_monotonic=None,
                time_semantics='software_bounded_acquisition_not_hardware_timestamp',
                status_before=None, status_after=None, control_1=None, bounds=p.wire())
            deadline = start + p.transaction_bound_s
            try:
                if not enabled:
                    raise CaptureError('calibration_or_sensor_disabled')
                cal = copy.deepcopy(self.calibration_getter())
                cid = calibration_id(cal)
                row['control_1'] = self._control()
                s = self._status()
                # Drain old data once, even on overflow/overrun, to allow the
                # next attempt to recover. This attempt still fails closed.
                if s & (DRDY | OVL | DOR):
                    self._burst()
                if s & (OVL | DOR):
                    raise CaptureError('overflow_or_overrun')
                s = self._status()
                if s != 0:
                    raise CaptureError('data_ready_not_cleared')
                cleared = self.clock()
                while True:
                    if self.clock() >= deadline:
                        raise CaptureError('no_data_timeout')
                    s = self._status()
                    if s & (OVL | DOR):
                        self._burst()
                        raise CaptureError('overflow_or_overrun')
                    if s & DRDY:
                        break
                    self.sleep(p.poll_s)
                ready = self.clock()
                xyz = self._burst()
                post = self._status()
                self._control()
                finished = self.clock()
                if post != 0:
                    raise CaptureError('ambiguous_or_overrun_burst')
                if not start <= cleared <= ready <= finished <= deadline:
                    raise CaptureError('transaction_time_bound_exceeded')
                if ready - cleared > p.conversion_bound_s:
                    raise CaptureError('conversion_time_bound_exceeded')
                if any(v in (-32768, 32767) for v in xyz) or not any(xyz):
                    raise CaptureError('saturated_or_zero_field')
                heading = heading_from_xyz(xyz, cal)
                if calibration_id(self.calibration_getter()) != cid:
                    raise CaptureError('calibration_changed_during_read')
                committed = self.clock()
                if committed > deadline:
                    raise CaptureError('transaction_time_bound_exceeded')
                self.seq += 1
                row.update(seq=self.seq, valid=True, xyz=xyz,
                    heading_deg=heading, calibration_id=cid, status_before=s,
                    status_after=post, read_finished_monotonic=committed,
                    capture_not_before_monotonic=cleared-p.conversion_bound_s,
                    ready_observed_monotonic=ready)
            except Exception as exc:
                row['error'] = str(exc) if isinstance(exc, CaptureError) else 'read_exception:' + type(exc).__name__
            if not row['valid']:
                self.fault_generation += 1
                row['fault_generation'] = self.fault_generation
                row['read_finished_monotonic'] = self.clock()
            self.latest = row
            return copy.deepcopy(row)


class CaptureStream:
    """Read-only consumer. Faults latch for this navigation instance.

    A new explicit journey creates a new instance, allowing recovery after an
    observed restart. Equal XYZ from distinct DRDY acquisitions are legitimate;
    duplicate event identities, not equal vectors, are rejected.
    """
    def __init__(self, boot_id, build_id, calibration_digest, policy,
                 publisher='/sensor_service_node', clock=time.monotonic):
        self.boot_id, self.build_id = boot_id, build_id
        self.calibration_digest, self.policy = calibration_digest, policy
        self.publisher, self.clock = publisher, clock
        self.session = None
        self.seq = self.attempt = 0
        self.fault_generation = None
        self.last_finished = 0
        self.last_started = None
        self.rows = []
        self.fault = None
        self.fault_detail = None
        self.last_outcome = None
        self._recovery_session = None

    def rebind_same_session(self):
        """Start a clean window while retaining the last verified source and replay floors."""
        stream = CaptureStream(self.boot_id, self.build_id, self.calibration_digest,
                               self.policy, self.publisher, self.clock)
        stream._recovery_session = self.session or self._recovery_session
        stream.seq = self.seq
        stream.attempt = self.attempt
        stream.last_finished = self.last_finished
        return stream

    def fail(self, reason):
        self.fault = self.fault or reason
        self.rows = []
        return None

    def accept(self, row, publisher):
        if self.fault:
            return None
        now = self.clock()
        try:
            if publisher != self.publisher:
                return self.fail('unexpected_publisher')
            if not isinstance(row, dict) or row['schema'] != SCHEMA:
                return self.fail('schema_mismatch')
            if row['boot_id'] != self.boot_id or row['producer_build_id'] != self.build_id:
                return self.fail('host_or_source_mismatch')
            if row['valid'] is not True or row['error'] is not None:
                detail=row.get('error_detail')
                if isinstance(detail,dict):
                    stage,register,number=(detail.get(key) for key in ('stage','register','errno'))
                    if (type(stage) is str and 0<len(stage)<=48 and stage.isidentifier()
                            and (register is None or type(register) is int and 0<=register<=255)
                            and (number is None or type(number) is int and 0<number<=4095)):
                        self.fault_detail=dict(stage=stage,register=register,errno=number)
                        for key in ('seq','attempt','read_started_monotonic','read_finished_monotonic'):
                            if finite(row.get(key)):
                                self.fault_detail[key]=row[key]
                return self.fail('invalid_acquisition:' + str(row.get('error')))
            if row['calibration_id'] != self.calibration_digest:
                return self.fail('calibration_mismatch')
            if row['time_semantics'] != 'software_bounded_acquisition_not_hardware_timestamp' or row['bounds'] != self.policy.wire():
                return self.fail('timing_contract_mismatch')
            session, seq, attempt = row['session'], row['seq'], row['attempt']
            generation = row['fault_generation']
            if not isinstance(session, str) or not session:
                return self.fail('invalid_session')
            if type(seq) is not int or type(attempt) is not int or not 0 < seq <= attempt:
                return self.fail('invalid_sequence')
            if type(generation) is not int or generation < 0:
                return self.fail('invalid_fault_generation')
            expected_session = self.session or self._recovery_session
            if expected_session is not None and session != expected_session:
                return self.fail('producer_restarted')
            if seq <= self.seq or attempt <= self.attempt:
                return self.fail('duplicate_or_reordered_acquisition')
            if self.session is not None:
                if generation != self.fault_generation:
                    return self.fail('producer_fault_since_binding')
                if attempt-self.attempt != seq-self.seq:
                    return self.fail('unobserved_failed_attempt')
            start, finish, lower, ready = [row[k] for k in (
                'read_started_monotonic', 'read_finished_monotonic',
                'capture_not_before_monotonic', 'ready_observed_monotonic')]
            p = self.policy
            if not all(finite(v) for v in (start, finish, lower, ready)):
                return self.fail('invalid_monotonic_times')
            if not (0 <= lower <= start <= ready <= finish <= now and
                    start > self.last_finished and
                    finish - start <= p.transaction_bound_s and
                    start - lower <= p.conversion_bound_s):
                return self.fail('stale_or_inconsistent_acquisition')
            if row['status_before'] != DRDY or row['status_after'] != 0 or row['control_1'] != 0x11:
                return self.fail('invalid_status_evidence')
            xyz, heading = row['xyz'], row['heading_deg']
            if not isinstance(xyz, list) or len(xyz) != 3 or any(type(v) is not int or not -32768 < v < 32767 for v in xyz) or not any(xyz):
                return self.fail('invalid_raw_field')
            if not finite(heading) or not 0 <= heading < 360:
                return self.fail('invalid_heading')
            # Only well-formed, correctly scoped frames qualify as recoverable delays.
            gap = p.publication_bound_s + p.transport_bound_s
            if self.last_outcome is not None and not 0 <= now-self.last_outcome['received_monotonic'] <= gap:
                return self.fail('capture_stream_outage')
            if self.last_started is not None and start-self.last_started > gap:
                return self.fail('capture_acquisition_outage')
            if now-finish > p.transport_bound_s:
                return self.fail('capture_transport_delay')
            if now-lower > p.max_age_s:
                return self.fail('stale_or_inconsistent_acquisition')
            result = dict(row, received_monotonic=now,
                captured_monotonic=lower, capture_source='verified_sensor_capture',
                capture_semantics='conservative_software_lower_bound')
            self.session, self.seq, self.attempt = session, seq, attempt
            self.fault_generation = generation
            self.last_finished = finish
            self.last_started = start
            self.rows.append(result)
            self.rows = self.rows[-64:]
            self.last_outcome = result
            return result
        except (ValueError, KeyError, TypeError, OverflowError):
            return self.fail('malformed_acquisition')

    def window(self, after):
        if self.fault:
            return []
        now = self.clock()
        if self.last_outcome is not None and not 0 <= now-self.last_outcome['received_monotonic'] <= self.policy.publication_bound_s+self.policy.transport_bound_s:
            self.fail('capture_stream_outage')
            return []
        # Expired rows are never returned, including during a source stall.
        return [dict(r) for r in self.rows if
                after < r['capture_not_before_monotonic'] <= now and
                0 <= now-r['capture_not_before_monotonic'] <= self.policy.max_age_s]
