"""原始Tag位置的中文摘要；不推断相机方向、路线方向或控制精度。"""
import json,math,statistics
from pathlib import Path

def reporting_options(project):
    p=Path(project)/'config/tag_reporting.json'
    row=json.loads(p.read_text(encoding='utf-8')) if p.exists() else {'min_frames':3,'reference_path':None,'expected_load':'unknown'}
    if type(row.get('min_frames')) is not int or not 1<=row['min_frames']<=5:raise ValueError('report min_frames must be 1..5')
    if row.get('expected_load') not in ('unknown','empty','single','stacked'):raise ValueError('invalid report load')
    name=row.get('reference_path')
    if name is not None:
        path=Path(name)
        if path.is_absolute() or '..' in path.parts or path.parts[:1]!=('config',) or path.suffix!='.json':raise ValueError('reference must be a project/config JSON path')
        reference=json.loads((Path(project)/path).read_text(encoding='utf-8'))
        if reference.get('schema')!='r3.taught-tag.v1' or reference.get('operator_confirmed') is not True:raise ValueError('reference requires explicit manual confirmation')
    return row

def enrich(scan,minimum_frames=3,reference=None,expected_load='unknown'):
    rows=scan.get('capture_history',scan.get('observations',[]));groups={}
    for row in rows:
        key=(row.get('id'),row.get('frame_id'));groups.setdefault(key,{})[(row.get('id'),row.get('stamp_ns'),row.get('frame_id'))]=row
    summaries=[]
    for (ident,frame),captures in groups.items():
        samples=list(captures.values());valid=[]
        for r in samples:
            p=r.get('position_raw_m',{});values=[p.get(k) for k in ('x','y','z')]
            if all(type(v) in (int,float) and math.isfinite(v) for v in values):valid.append((r,values))
        if not valid:continue
        columns=list(zip(*[v for r,v in valid]));median=dict(zip(('x','y','z'),map(statistics.median,columns)))
        summary={'id':ident,'frame_id':frame,'position_unit':'m','position_raw_m_median':median,
            'position_raw_m_min':dict(zip(('x','y','z'),map(min,columns))),'position_raw_m_max':dict(zip(('x','y','z'),map(max,columns))),
            'declared_distinct_frames':len(valid),'minimum_frames_met':len(valid)>=minimum_frames,
            'stability':'spread_report_only_no_calibrated_tolerance','pose_for_control_verified':False,
            'image_center':'unknown_without_pixels_or_verified_camera_transform','expected_load':expected_load,'physical_load_sensor_confirmation':False}
        summary['header_conflicts']=any(r.get('headers_disagree') for r,v in valid)
        if reference:
            if reference.get('operator_confirmed') is not True:summary['reference_comparison']='reference_not_manually_confirmed'
            elif (reference.get('target_id'),reference.get('frame_id'),reference.get('expected_load'))!=(ident,frame,expected_load):summary['reference_comparison']='ID/frame/load mismatch; no comparison'
            else:
                goal=reference.get('median',{})
                if all(type(goal.get(k+'_m')) in (int,float) and math.isfinite(goal[k+'_m']) for k in ('x','y','z')):
                    summary['raw_delta_from_taught_m']={k:median[k]-goal[k+'_m'] for k in ('x','y','z')};summary['reference_comparison']='raw_same_frame_only_not_control'
        else:summary['reference_comparison']='not_set; first observation is not a reference'
        summaries.append(summary)
    return dict(scan,tag_position_summary=summaries,minimum_report_frames=minimum_frames,
        summary_note='位置为消息原始frame下的米；不是已验证相机坐标/机器人位置。方差小不等于标定或物理成功。')

def enrich_for_project(scan,project,expected_load='unknown'):
    cfg=reporting_options(project);reference=None
    if cfg['reference_path'] is not None:reference=json.loads((Path(project)/cfg['reference_path']).read_text(encoding='utf-8'))
    return enrich(scan,cfg['min_frames'],reference,expected_load if expected_load!='unknown' else cfg['expected_load'])

def human_lines(scan):
    lines=['AR扫码：'+str(scan.get('status'))+'；'+str(scan.get('reason',''))]
    for s in scan.get('tag_position_summary',[]):
        p=s['position_raw_m_median'];n=s['declared_distinct_frames']
        lines.append('Tag {} | 原始frame={} | x={:.4f}, y={:.4f}, z={:.4f} m | 不同时间戳帧={}{}'.format(s['id'],s['frame_id'],p['x'],p['y'],p['z'],n,'（单帧，不能判断稳定）' if n==1 else '（多帧范围见JSON；未标定稳定阈值）'))
        if not s['minimum_frames_met']:lines.append('  帧数不足，不能作为已确认稳定参考。')
        if s['header_conflicts']:lines.append('  header存在差异，不可用于控制。')
    if not scan.get('tag_position_summary'):lines.append('没有可报告的有效位置；请看超时/误ID/过期原因和observed_ids。')
    entry=scan.get('entry_observation',{});pixels=entry.get('pixel_samples',[])
    if pixels:
        value=pixels[-1];u,v=value['tag_center_px']
        lines.append('投影像素u={:.2f}, v={:.2f}；图像x中心偏差={:.2f}px；{}；图像y不要求居中。'.format(u,v,value['center_error_x_px'],'已经人工核对投影' if value['projection_verified'] else '候选投影未核对，不能据此控制'))
    else:lines.append('是否在图像中心：目前无法判断；没有像素中心或已验证相机坐标变换。没有自动纠偏。')
    return lines
