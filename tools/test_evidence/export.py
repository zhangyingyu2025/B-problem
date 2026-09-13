"""Read-only evidence export from Client JSONL; never connects to a simulator."""
import argparse
import csv
import hashlib
from html import escape
import json
import math
import os
from pathlib import Path


def audit(path):
    events=[json.loads(line) for line in path.read_text(encoding='utf-8-sig').splitlines() if line.strip()]
    actions=[]; seen={}; warnings=[]
    for e in events:
        if e.get('kind')!='action': continue
        if e['response'].get('accepted') is not True: raise ValueError('unaccepted action in action log')
        key=e['payload']['request_id']
        signature={k:e[k] for k in ('path','payload','response','before','after')}
        if key in seen:
            if signature!=seen[key]: raise ValueError('conflicting duplicate request_id')
            continue
        seen[key]=signature; actions.append(e)
    if not actions or actions[0]['path']!='/enter': raise ValueError('missing initial enter')
    if sum(e['path']=='/enter' for e in actions)!=1: raise ValueError('multiple sessions in one log')
    if any(e['path']=='/exit' for e in actions[:-1]): raise ValueError('actions after exit')
    complete=actions[-1]['path']=='/exit' and actions[-1]['response'].get('exit_reason')=='user_exit'
    if not complete: warnings.append('未确认正常exit；表格时间留空，统计仅至最后确认动作')
    unresolved={e['payload']['request_id'] for e in events if e.get('kind') in ('transport_failure','invalid_accepted_response')}-seen.keys()
    if unresolved:
        complete=False; warnings.append('存在执行结果未确认请求，实际轨迹可能超出已确认轨迹')
    position=(0.,0.); channel=1; vt=0.; movement=0.; measures=switches=failures=0
    cleared=set(); points=[]
    for i,e in enumerate(actions):
        if tuple(e['before']['position'])!=position or e['before']['channel']!=channel:
            raise ValueError('discontinuous local state')
        now=e['response']['virtual_time_s']
        if not math.isfinite(now) or now<vt: raise ValueError('non-monotonic virtual time')
        if e['before']['virtual_time_s']!=vt or e['after']['virtual_time_s']!=now:
            raise ValueError('virtual state mismatch')
        if e['path'] in ('/measure','/clear'):
            p=e['payload']['position']; new=(p['x'],p['y']); c=e['payload']['channel']
            if any(not math.isfinite(x) for x in new): raise ValueError('nonfinite coordinate')
            movement+=math.dist(position,new); position=new
            if e['path']=='/measure':
                measures+=1; switches+=int(c!=channel);channel=c; outcome=e['response']['measure_result']
            else:
                outcome=e['response']['clear_result']
                if outcome=='success':
                    if c in cleared: raise ValueError('same unique channel cleared twice')
                    cleared.add(c)
                elif outcome=='no_target_in_range': failures+=1
                else: raise ValueError('unknown clear result')
            points.append({'sequence':i,'request_id':e['payload']['request_id'],'action':e['path'][1:],
                           'x_m':position[0],'y_m':position[1],'channel':c,'result':outcome,'virtual_time_s':now})
        elif e['path'] not in ('/enter','/exit'): raise ValueError('unknown action')
        if tuple(e['after']['position'])!=position or e['after']['channel']!=channel:
            raise ValueError('after state mismatch')
        vt=now
    expected=movement/5+5*measures+switches+5*len(cleared)+3*failures
    cost_error=vt-expected
    if abs(cost_error)>max(.001,len(points)*.51e-6):
        complete=False;warnings.append('虚拟时间与动作成本不一致，需人工核验')
    server_delta=(actions[-1]['response']['real_timestamp_ms']-actions[0]['response']['real_timestamp_ms'])/1000 if complete else None
    if server_delta is not None and server_delta<0: raise ValueError('negative server elapsed time')
    return {'log_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'normal_exit_and_audit_ok':complete,
            'warnings':warnings,'cleared_count':len(cleared),'cleared_channels':sorted(cleared),
            'virtual_time_s_last_confirmed':vt,'mean_time_per_clear_s':vt/len(cleared) if complete and cleared else None,
            'server_response_elapsed_s':server_delta,'client_elapsed_s':actions[-1].get('wall_elapsed_s') if complete else None,
            'movement_m':movement,'measurements':measures,'switches':switches,'clear_failures':failures,
            'cost_error_s':cost_error,'points':points,'end_position':position,
            'true_source_count':None,'clear_fraction':None}


def draw_svg(report,title):
    points=report['points']; bound=max(1950.,max((max(abs(p['x_m']),abs(p['y_m'])) for p in points),default=0)*1.12)
    def xy(x,y): return 430+340*x/bound,450-340*y/bound
    parts=['<svg xmlns="http://www.w3.org/2000/svg" width="860" height="900" viewBox="0 0 860 900">',
           '<rect width="860" height="900" fill="white"/>',
           '<style>text{font-family:"Microsoft YaHei",Arial,sans-serif;fill:#243746}</style>']
    def text(x,y,t,size=14): parts.append(f'<text x="{x}" y="{y}" font-size="{size}">{escape(str(t))}</text>')
    text(70,34,title,21)
    text(70,59,'按确认动作顺序连接；保留同点多频道动作，重合点不抖动',14)
    text(70,81,'清除标记表示机器狗操作位置，不表示干扰源真实坐标',14)
    step=500 if bound<=2500 else 10**math.floor(math.log10(bound))
    for v in range(-int(bound//step),int(bound//step)+1):
        q=v*step;x,y=xy(q,q)
        parts.append(f'<path d="M{x},110 V790 M90,{y} H770" stroke="#e7eaed" stroke-width="1"/>')
        text(x-15,810,f'{q:g}',12);text(35,y+4,f'{q:g}',12)
    x,y=xy(0,0)
    parts.append(f'<circle cx="{x}" cy="{y}" r="{340*1800/bound}" fill="none" stroke="#a2aab3" stroke-dasharray="5 4"/>')
    coords=[xy(0,0)]+[xy(p['x_m'],p['y_m']) for p in points]
    parts.append('<polyline points="'+' '.join(f'{x:.6f},{y:.6f}' for x,y in coords)+'" fill="none" stroke="#596675" stroke-width="1.4"/>')
    for p in points:
        x,y=xy(p['x_m'],p['y_m']); label=f"#{p['sequence']} {p['action']} ch{p['channel']} {p['result']} t={p['virtual_time_s']:.6f}s"
        parts.append('<g><title>'+escape(label)+'</title>')
        if p['action']=='measure': parts.append(f'<circle cx="{x}" cy="{y}" r="3" fill="#2879a5"/>')
        else:
            color='#bd4b37' if p['result']=='success' else '#c48600'
            parts.append(f'<path d="M{x-5},{y-5} l10,10 M{x-5},{y+5} l10,-10" stroke="{color}" stroke-width="2"/>')
            if p['result']=='success': text(x+7,y-6,f"C{p['channel']}",11)
        parts.append('</g>')
    x,y=xy(0,0);parts.append(f'<rect x="{x-5}" y="{y-5}" width="10" height="10" fill="#34856c"/>')
    x,y=xy(*report['end_position'])
    star=[(x+(8 if i%2==0 else 3.5)*math.sin(i*math.pi/5),y-(8 if i%2==0 else 3.5)*math.cos(i*math.pi/5)) for i in range(10)]
    parts.append('<polygon points="'+' '.join(f'{x},{y}' for x,y in star)+'" fill="#825599"/>')
    text(410,835,'x / m');text(14,104,'y / m')
    text(65,865,'● 检测   × 红：清除成功 / 黄：失败   ■ 起点   ★ 终点（异常时为最后确认位置）',13)
    text(65,888,f"清除 {report['cleared_count']}；移动 {report['movement_m']:.1f} m；虚拟时间 {report['virtual_time_s_last_confirmed']:.3f} s",13)
    parts.append('</svg>');return '\n'.join(parts)


def export(manifest_path,output,write_tables=True):
    config=json.loads(manifest_path.read_text(encoding='utf-8-sig'));base=manifest_path.parent
    return export_config(config,base,output,write_tables=write_tables)


def export_config(config,base,output,write_tables=True):
    if output.exists(): raise ValueError('output already exists; choose a new directory')
    prepared=[]
    for run in config['runs']:
        if run['mode'] not in ('formal','rehearsal','synthetic'): raise ValueError('explicit mode required')
        path=(base/run['actions']).resolve(); report=audit(path)
        case=run.get('case_code'); official_runtime=run.get('official_runtime_s')
        if official_runtime is not None and (not math.isfinite(official_runtime) or official_runtime<0 or not run.get('runtime_evidence')):
            raise ValueError('official runtime requires a nonnegative value and evidence description')
        report.update(mode=run['mode'],problem=run['problem'],case_code=case,actions_path=str(path))
        report['official_runtime_provided']=official_runtime is not None
        report['table_runtime_s']=official_runtime if official_runtime is not None else report['server_response_elapsed_s']
        report['runtime_source']=run.get('runtime_evidence') if official_runtime is not None else 'enter/exit响应real_timestamp_ms之差；为接口重建值，需与官方可用记录核验'
        report['encrypted_log']=None
        if run.get('encrypted_log'):
            encrypted=(base/run['encrypted_log']).resolve()
            report['encrypted_log']={'original_filename':encrypted.name,'sha256':hashlib.sha256(encrypted.read_bytes()).hexdigest()}
        prepared.append(report)
    output.mkdir(parents=True)
    headers=['问题','测试类型','测试案例编码','清除干扰源个数','平均定位清除时间(s)','程序运行时间(s)','运行时间来源','日志审计通过']
    table=[];md=['# 测试结果（由客户端确认日志重建）','', '| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']
    for i,r in enumerate(prepared,1):
        mode={'formal':'正式','rehearsal':'演练','synthetic':'人工构造'}[r['mode']]
        def number(v):return '' if v is None else f'{v:.6f}'
        row=[r['problem'],mode,r['case_code'] or '待从模拟器填写',r['cleared_count'],number(r['mean_time_per_clear_s']),number(r['table_runtime_s']),r['runtime_source'],r['normal_exit_and_audit_ok']]
        table.append(row);md.append('| '+' | '.join(str(x).replace('|','\\|') for x in row)+' |')
        (output/f'run-{i:02d}.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        (output/f'run-{i:02d}-trajectory.svg').write_text(draw_svg(r,f"{r['problem']} {mode}轨迹 · {r['case_code'] or '案例编码待补'}"),encoding='utf-8')
        with (output/f'run-{i:02d}-actions.csv').open('w',encoding='utf-8-sig',newline='') as f:
            names=['sequence','request_id','action','x_m','y_m','channel','result','virtual_time_s']
            w=csv.DictWriter(f,fieldnames=names);w.writeheader();w.writerows(r['points'])
    if write_tables:
        with (output/'table.csv').open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.writer(f);w.writerow(headers);w.writerows(table)
    md+=['','正式案例不返回真实总源数，本工具不计算清除比例。运行时间不是虚拟时间，也不是仅CPU耗时。','',
         '轨迹只包括收到accepted=true并通过适配器验证的动作。原始加密日志必须另行从模拟器导出并保持文件名；本工具输出不能替代它。']
    if write_tables:
        (output/'table.md').write_text('\n'.join(md)+'\n',encoding='utf-8')
    (output/'manifest-used.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return prepared


def update_ledger(reports, folder):
    """Merge sessions by immutable log hash; atomically rebuild cumulative tables."""
    folder.mkdir(parents=True, exist_ok=True)
    lock=folder/'.update.lock'
    # Exclusive creation prevents concurrent exports from losing a session.
    with lock.open('x'):
        pass
    try:
        registry=folder/'runs.json'
        rows=json.loads(registry.read_text(encoding='utf-8')) if registry.exists() else []
        # Preserve case codes entered by the operator in Excel between exports.
        table_path=folder/'table.csv'
        if table_path.exists():
            with table_path.open(encoding='utf-8-sig',newline='') as f:
                manual={x['原始日志']:x['测试案例编码'].strip() for x in csv.DictReader(f)}
            for r in rows:
                code=manual.get(r['actions_path'])
                if code and code!='待从模拟器填写':r['case_code']=code
        scopes={(r['problem'],r['mode']) for r in rows+reports}
        if len(scopes)>1:
            raise ValueError('cumulative table must contain only one problem and one test mode')
        for report in reports:
            r={k:v for k,v in report.items() if k!='points'}
            existing=next((x for x in rows if x['log_sha256']==r['log_sha256']),None)
            if existing is not None:
                if (existing['problem'],existing['mode'])!=(r['problem'],r['mode']):
                    raise ValueError('same log cannot be relabeled as another problem/mode')
                if existing.get('case_code') and not r.get('case_code'):
                    r['case_code']=existing['case_code']
                if existing.get('official_runtime_provided') and not r.get('official_runtime_provided'):
                    for key in ('table_runtime_s','runtime_source','official_runtime_provided'):
                        r[key]=existing[key]
                existing.update(r)
            else:
                rows.append(r)
        headers=['记录序号','问题','测试类型','测试案例编码','清除干扰源个数','平均定位清除时间(s)','程序运行时间(s)','运行时间来源','日志审计通过','原始日志']
        table=[]
        for i,r in enumerate(rows,1):
            number=lambda v: '' if v is None else f'{v:.6f}'
            table.append([i,r['problem'],{'rehearsal':'演练','formal':'正式','synthetic':'人工构造'}[r['mode']],r.get('case_code') or '待从模拟器填写',r['cleared_count'],number(r['mean_time_per_clear_s']),number(r['table_runtime_s']),r['runtime_source'],r['normal_exit_and_audit_ok'],r['actions_path']])
        (folder/'runs.json.tmp').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        with (folder/'table.csv.tmp').open('w',encoding='utf-8-sig',newline='') as f:
            writer=csv.writer(f);writer.writerow(headers);writer.writerows(table)
        md=['# 累计测试结果','', '| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']
        md+=['| '+' | '.join(str(x).replace('|','\\|').replace('\n',' ') for x in row)+' |' for row in table]
        (folder/'table.md.tmp').write_text('\n'.join(md)+'\n',encoding='utf-8')
        # Registry is authoritative; rerunning regenerates tables if interrupted.
        for name in ('runs.json','table.csv','table.md'):
            os.replace(folder/(name+'.tmp'),folder/name)
        return rows
    finally:
        lock.unlink()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    source=p.add_mutually_exclusive_group(required=True)
    source.add_argument('--manifest',type=Path);source.add_argument('--actions',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--problem',choices=['B3','B4'],default='B3')
    p.add_argument('--mode',choices=['rehearsal','formal','synthetic'])
    p.add_argument('--case-code')
    p.add_argument('--ledger',type=Path,help='Persistent cumulative table directory; same log is not counted twice')
    a=p.parse_args()
    if a.actions:
        if not a.mode:p.error('--actions requires explicit --mode; never infer formal/rehearsal')
        rows=export_config({'runs':[{'problem':a.problem,'mode':a.mode,'case_code':a.case_code,
                                   'actions':str(a.actions.resolve())}]},Path.cwd(),a.output,write_tables=not bool(a.ledger))
    else:rows=export(a.manifest,a.output,write_tables=not bool(a.ledger))
    if a.ledger:update_ledger(rows,a.ledger)
    print(json.dumps([{k:r[k] for k in ('problem','mode','case_code','cleared_count','mean_time_per_clear_s','table_runtime_s','normal_exit_and_audit_ok')} for r in rows],ensure_ascii=False,indent=2))
