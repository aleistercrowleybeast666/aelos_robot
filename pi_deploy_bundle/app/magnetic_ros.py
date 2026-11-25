"""ROS wire/config glue; importing this module never connects to ROS or I2C."""
import json
from pathlib import Path
from magnetic_capture import CapturePolicy, CaptureStream, SCHEMA


def make_stream(magnetic, clock, boot_id=None):
    c = magnetic.get('capture_contract')
    if c is None:
        # Legacy receipt monitoring remains available. A configuration flag
        # alone cannot make RosNavigationIO.motion_capture_verified true.
        return None
    if c.get('schema') != SCHEMA or c.get('topic') != '/aelos/magnetic_capture':
        raise ValueError('unsupported magnetic capture contract')
    if c.get('publisher') != '/sensor_service_node':
        raise ValueError('existing sole sensor owner required')
    for key in ('producer_build_id', 'calibration_id'):
        value = c.get(key)
        if not isinstance(value, str) or len(value) != 64 or any(v not in '0123456789abcdef' for v in value):
            raise ValueError('pinned source/calibration SHA256 required: ' + key)
    p = CapturePolicy(**c['policy'])
    if magnetic['max_age_s'] != p.max_age_s:
        raise ValueError('magnetic max age must equal acquisition contract age')
    if magnetic['collect_s'] < p.conversion_bound_s + p.transaction_bound_s + magnetic['min_samples']*p.publication_bound_s:
        raise ValueError('magnetic collection cannot cover fresh sample window')
    if boot_id is None:
        boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if not boot_id:
        raise ValueError('local Linux monotonic clock domain unavailable')
    return CaptureStream(boot_id, c['producer_build_id'], c['calibration_id'], p,
                         c['publisher'], clock)


def accept_message(stream, msg):
    try:
        payload = msg.data
        if not isinstance(payload, str) or len(payload) > 16384:
            return stream.fail('invalid_wire_payload')
        row = json.loads(payload)
        caller = (getattr(msg, '_connection_header', None) or {}).get('callerid', '')
        return stream.accept(row, caller)
    except (ValueError, TypeError, AttributeError):
        return stream.fail('malformed_wire_payload')
