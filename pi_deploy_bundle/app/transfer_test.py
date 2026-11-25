"""Finite command-0 boundaries; no resume, no obstacle or release commands."""
BOUNDARIES = ('empty_forward5', 'grab_single', 'carry_forward8', 'carry_forward2',
              'right_turn1', 'right_turn2', 'right_turn3', 'carry_forward4', 'terminal_scan')
MACRO_BOUNDARIES = BOUNDARIES[:-1]

class BoundaryReached(Exception):
    def __init__(self, name):
        self.boundary = name
        super().__init__(name)

def validate(cfg, mode=None):
    c = cfg.get('transfer_test', {})
    if not c:
        return
    if set(c) != {'enabled', 'stop_after', 'preflight_sensor_streams', 'pixel_diagnostic', 'target_id'}:
        raise ValueError('exact transfer_test options required')
    for k in ('enabled', 'preflight_sensor_streams', 'pixel_diagnostic'):
        if type(c[k]) is not bool:
            raise ValueError('transfer_test flag ' + k)
    if c['target_id'] is not None and (type(c['target_id']) is not int or not 0<=c['target_id']<=1023):
        raise ValueError('target_id must be explicitly verified integer or null')
    if c['stop_after'] not in BOUNDARIES:
        raise ValueError('unknown stable transfer boundary; no resume offset supported')
    if mode is not None and c['stop_after'] != 'terminal_scan' and mode not in ('0','stage1'):
        raise ValueError('stop_after applies only to command 0/stage1')
