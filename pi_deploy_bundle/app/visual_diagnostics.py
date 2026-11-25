#!/usr/bin/env python3
"""Bounded ROS read-only observation. No I2C, MCU probe, action or config writes."""
import argparse
import json
from pathlib import Path
import time


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,default=Path(__file__).resolve().parent/'config/navigation.json')
    p.add_argument('--target-id',type=int)
    p.add_argument('--observe-s',type=float,default=3.)
    p.add_argument('--output',type=Path)
    a=p.parse_args(argv)
    if not 1<=a.observe_s<=10 or a.target_id is not None and not 0<=a.target_id<=1023:p.error('bounded observation 1..10s and optional explicit ID 0..1023')
    cfg=json.loads(a.config.read_text(encoding='utf-8'))
    from nav_ros import RosNavigationIO
    from nav_observation import summarize
    import rospy
    rospy.init_node('r3_visual_readonly_diagnostics',anonymous=True,disable_signals=True)
    io=RosNavigationIO(cfg)
    def check():
        if rospy.is_shutdown():raise RuntimeError('ROS shutdown')
    try:
        start=io.ros_now_ns();window=io.observe(a.target_id,start,a.observe_s,check)
        obs=summarize(window,a.target_id,cfg,start)
        stream=io.capture_stream
        result=dict(version='r3-visual-pickup-20261109-v5',read_only=True,body_commands=0,MCU_probe_sent=False,
            requested_target_id=a.target_id,automatic_target_selection=False,
            target_note='Observed IDs never confirm the requested target or block count',
            camera_healthy=window.get('camera_healthy',False),camera_info=window.get('camera_info'),
            observation=obs,magnetic_stream={'session':getattr(stream,'session',None),'fault':getattr(stream,'fault',None),
                'current_contract_verified':io.motion_capture_verified,'expected_load_physically_verified':False},
            unknown_detector_quality=['corner completeness','reprojection residual','actual exposure timestamp'],
            config_updated=False,physical_validation=False,position_controlled=False,ready_for_obstacle=False)
        encoded=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
        if a.output:a.output.write_text(encoded,encoding='utf-8')
        print(encoded,end='')
        return 0
    finally:io.close()


if __name__=='__main__':raise SystemExit(main())
