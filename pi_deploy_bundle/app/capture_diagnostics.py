"""Explicit read-only ROS subscription / saved-recording validation. No I2C."""
import argparse
import collections
import json
from pathlib import Path
import threading
import time

from magnetic_capture import CapturePolicy, CaptureStream


def analyze(records, magnetic, boot_id, now, required=20):
    contract=magnetic['capture_contract']
    policy=CapturePolicy(**contract['policy'])
    report=dict(samples=len(records),required_consecutive_valid=required,
                ready_for_capture_contract=False,body_commands=0,errors={})
    errors=collections.Counter()
    recovered=collections.Counter()
    for entry in records:
        try:
            row=entry['payload']
            if row.get('valid') is not True:
                errors[str(row.get('error'))]+=1
            elif isinstance(row.get('recovered_error_detail'),dict):
                detail=row['recovered_error_detail']
                recovered[f"{detail.get('stage')} errno={detail.get('errno')}"]+=1
        except (KeyError,TypeError,AttributeError):errors['malformed']+=1
    report['errors']=dict(errors)
    report['recovered_errors']=dict(recovered)
    if len(records)<required:
        report['reason']='not_enough_observations'
        return report
    tail=records[-required:]
    receipt=[0.]
    stream=CaptureStream(boot_id,contract['producer_build_id'],contract['calibration_id'],
                         policy,contract['publisher'],lambda:receipt[0])
    durations=[];deliveries=[];intervals=[];previous=None
    for entry in tail:
        try:
            receipt[0]=entry['received_monotonic']
            row=entry['payload']
            if stream.accept(row,entry['publisher']) is None:
                report['reason']=stream.fault
                return report
            durations.append(row['read_finished_monotonic']-row['read_started_monotonic'])
            deliveries.append(receipt[0]-row['read_finished_monotonic'])
            if previous is not None:intervals.append(row['read_started_monotonic']-previous)
            previous=row['read_started_monotonic']
        except (KeyError,ValueError,TypeError):
            report['reason']='malformed_recording'
            return report
    report.update(max_transaction_s=max(durations),max_delivery_s=max(deliveries),
                  max_publication_interval_s=max(intervals or [0]),
                  mean_publication_interval_s=sum(intervals)/len(intervals) if intervals else 0,
                  control_1=tail[-1]['payload']['control_1'],session=stream.session,
                  first_seq=tail[0]['payload']['seq'],last_seq=stream.seq)
    if report['max_publication_interval_s']>policy.publication_bound_s:
        report['reason']='publication_interval_exceeds_bound'
        return report
    if not 0<=now-receipt[0]<=policy.publication_bound_s+policy.transport_bound_s:
        report['reason']='last_recording_no_longer_live'
        return report
    report.update(ready_for_capture_contract=True,
        reason='consecutive atomic acquisitions verified; no motion or gain validation claimed')
    return report


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True,help='navigation.json or magnetic_config_fragment.json')
    parser.add_argument('--count',type=int,default=40)
    parser.add_argument('--timeout',type=float,default=15.)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if not 20<=args.count<=200 or not 3<=args.timeout<=60:
        parser.error('bounded count 20..200 and timeout 3..60 required')
    c=json.loads(Path(args.config).read_text())
    magnetic=c.get('magnetic',c)
    boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    records=[];done=threading.Event();lock=threading.Lock()
    import rospy
    from std_msgs.msg import String
    rospy.init_node('r3_capture_readonly_probe',anonymous=True,disable_signals=True)
    def callback(msg):
        now=time.monotonic()
        try:payload=json.loads(msg.data)
        except (ValueError,TypeError):payload={'valid':False,'error':'malformed_wire'}
        caller=(getattr(msg,'_connection_header',None) or {}).get('callerid','')
        with lock:
            records.append(dict(payload=payload,publisher=caller,received_monotonic=now))
            if len(records)>=args.count:done.set()
    sub=rospy.Subscriber(magnetic['capture_contract']['topic'],String,callback,queue_size=1)
    try:done.wait(args.timeout)
    finally:sub.unregister()
    with lock:captured=list(records)
    report=analyze(captured,magnetic,boot,time.monotonic())
    Path(args.output).write_text(json.dumps(dict(report=report,records=captured),indent=2)+'\n')
    print(json.dumps(report,sort_keys=True))
    return 0 if report['ready_for_capture_contract'] else 2


if __name__=='__main__':raise SystemExit(main())
