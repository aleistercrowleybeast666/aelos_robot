"""Measure one explicitly requested held-load turn against Tag and compass."""
import time
from micro_trial import MEDIUM_MAX_RESPONSE_DEG, MEDIUM_NOMINAL_DEG, MEDIUM_OPS, response_limit
from nav_observation import summarize,circular_median,wrap

TURN_TEST_STEPS={
    'test-micro-left':('single_micro_left',-1),
    'test-micro-right':('single_micro_right',1),
    'test-medium-left':('single_medium_left',-1),
    'test-medium-right':('single_medium_right',1),
}

def execute_micro_step(plan,backend,io_factory=None):
    if plan['mode'] not in TURN_TEST_STEPS:raise ValueError('dedicated turn test only')
    op,sign=TURN_TEST_STEPS[plan['mode']]
    medium=op in MEDIUM_OPS
    nominal=MEDIUM_NOMINAL_DEG if medium else 5
    cfg=plan['navigation'];timeout=plan['config']['request_timeout_s'];io=None
    if getattr(backend,'is_simulation',False):
        return {'status':'syntax_only_no_motion','action':op,'body_action_requests':0,'nominal_degrees':nominal}
    visual=cfg.get('visual_micro',{})
    target=visual.get('confirmed_target_id')
    heading_sign=cfg.get('magnetic',{}).get('heading_sign')
    if medium and (visual.get('target_id_confirmed') is not True or type(target) is not int
                   or type(heading_sign) is not int or heading_sign not in (-1,1)):
        raise RuntimeError('medium turn trial requires a confirmed Tag ID and magnetic heading direction before motion')
    try:
        if medium:
            backend.verify_medium_turn(timeout)
        else:
            backend.verify_micro(timeout)
        if io_factory is None:
            from nav_ros import RosNavigationIO
            io_factory=RosNavigationIO
        io=io_factory(cfg);io.set_magnetic_load('single')
        def capture(check, *, settle_s=0.):
            check()
            if settle_s:
                io.settle(settle_s,check)
            barrier=io.ros_now_ns();after=time.monotonic()
            window=io.observe(target,barrier,5,check)
            obs=summarize(window,target,cfg,barrier)
            rows=io.magnetic_samples(after,cfg['magnetic']['collect_s'],check)
            if not window.get('camera_healthy') or not io.visual_camera_status().get('ready') or not obs.get('pose_usable'):raise RuntimeError('fresh target/camera evidence missing')
            if not io.motion_capture_verified or len(rows)<3:raise RuntimeError('fresh magnetic evidence missing')
            if time.monotonic()-after>10:raise RuntimeError('observation deadline')
            return {'magnetic':circular_median([x['heading_deg'] for x in rows]),'visual':obs['heading_deg'],'stamp_ns':obs['latest_stamp_ns']}
        before=capture(backend.check_manual_idle)
        backend.check_manual_idle();backend.call('begin_align',timeout)
        backend.check_micro()
        if medium and not backend.medium_turn_ready():
            raise RuntimeError('medium turn current-session capability expired before dispatch')
        backend.call(op,timeout)
        after=capture(backend.check,settle_s=cfg.get('settle_s',0.))
        md=wrap(heading_sign*wrap(after['magnetic']-before['magnetic']));vd=wrap(after['visual']-before['visual'])
        max_response=MEDIUM_MAX_RESPONSE_DEG if medium else response_limit(cfg.get('post_action_heading',{}))
        if not (1<=sign*md<=max_response and 1<=sign*vd<=max_response and abs(md-vd)<=3):
            raise RuntimeError('%s response outside direction/1..%gdeg/agreement bounds: mag=%s visual=%s'
                               %('medium turn' if medium else 'micro',max_response,md,vd))
        backend.check();backend.call('finish',timeout)
        return {'status':'single_medium_response_observed' if medium else 'single_micro_response_observed',
                'action':op,'body_action_requests':1,'nominal_degrees':nominal,
                'magnetic_delta_deg':md,'visual_delta_deg':vd,'max_response_deg':max_response,
                'before':before,'after':after,'position_controlled':False,'no_automatic_next_step':True,
                'native_immediate_stop':False}
    except BaseException:
        backend.cancel();raise
    finally:
        if io is not None:io.close()
