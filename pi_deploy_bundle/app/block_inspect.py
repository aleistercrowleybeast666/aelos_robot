#!/usr/bin/env python3
"""Offline image inspection only; never imports ROS or sends robot actions."""
import argparse
import json
from pathlib import Path
from block_vision import BlockDetector


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('image',type=Path)
    p.add_argument('--config',type=Path,default=Path(__file__).parent/'config'/'mission_v5.json')
    p.add_argument('--overlay',type=Path)
    a=p.parse_args(argv)
    try:
        import cv2
        import numpy as np
        config=json.loads(a.config.read_text(encoding='utf-8-sig'))
        cfg=config.get('vision',config)
        data=np.frombuffer(a.image.read_bytes(),dtype=np.uint8)
        image=cv2.imdecode(data,cv2.IMREAD_COLOR)
        if image is None:raise ValueError('image decode failed')
        result=BlockDetector(cfg).detect(image)
        result['offline_only']=True;result['physical_validation']=False
        result['image']=str(a.image);result['config']=str(a.config)
        if a.overlay:
            h,w=image.shape[:2]
            for item in result['candidates']:
                x,y,bw,bh=item['bbox'];left,top=int(x*w),int(y*h)
                cv2.rectangle(image,(left,top),(int((x+bw)*w),int((y+bh)*h)),(255,255,255),2)
                cv2.putText(image,item['color'],(left,max(12,top-6)),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1)
            cv2.putText(image,result['status']+': '+result['reason'],(8,22),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,255),1)
            if a.overlay.resolve()==a.image.resolve():raise ValueError('overlay may not overwrite source image')
            if a.overlay.exists():raise ValueError('overlay destination exists; choose a new file')
            ok,encoded=cv2.imencode('.png',image)
            if not ok:raise ValueError('overlay encoding failed')
            a.overlay.write_bytes(encoded.tobytes());result['overlay']=str(a.overlay)
        print(json.dumps(result,ensure_ascii=False,sort_keys=True));return 0
    except Exception as exc:
        print(json.dumps({'status':'inspection_error','reason':str(exc),'offline_only':True},ensure_ascii=False));return 1


if __name__=='__main__':raise SystemExit(main())
