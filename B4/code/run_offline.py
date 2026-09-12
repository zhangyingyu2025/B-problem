"""Reproducible B4 synthetic batches. No network transport or official runner."""
import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'B3/code'))
from protocol_adapter import Client
from offline_environment_b4 import OfflineTransportB4, make_case, PATTERNS, ERROR_MODES
from solver_g0 import G0Solver, RobotPort


def quantile(xs,q):
    xs=sorted(xs); u=(len(xs)-1)*q; i=int(u)
    return xs[i]+(xs[min(i+1,len(xs)-1)]-xs[i])*(u-i)


def run_case(case,variant='G0',error_mode='fixed_field',log_path=None):
    if variant!='G0': raise ValueError('unsupported variant')
    transport=OfflineTransportB4(case,error_mode=error_mode)
    client=Client(transport,'offline-test',log_path=log_path,session_id=f"b4-{case['seed']}-{variant}")
    start=time.perf_counter(); cpu=time.process_time(); error=None; solver=None
    try:
        client.enter(); solver=G0Solver(RobotPort(client)); solver.run_all()
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}'
        client.record({'kind':'B4_failure','error':error})
    finally:
        if client.active:
            try: client.exit()
            except Exception as exc: error=(error or '')+f'; exit: {exc}'
    wall=time.perf_counter()-start; cpu=time.process_time()-cpu
    # Truth is used below only by the evaluator, after the policy terminates.
    truth={s['channel']:s for s in case['sources']}; n=len(truth)
    by_kind={'omni':[],'directional':[]}; seen=set(); absent_m=existing_m=0
    for e in client.events:
        if e.get('kind')!='action' or e['path']!='/measure': continue
        ch=e['payload']['channel']
        if ch not in truth: absent_m+=1
        elif ch not in seen:
            existing_m+=1
            if e['response']['measure_result']!='no_signal':
                seen.add(ch); by_kind[truth[ch]['kind']].append(e['response']['virtual_time_s'])
    m=dict(client.stats); m['virtual_time_s']=client.virtual_time_s
    costs={'move':m['movement_m']/5,'measure':m['measurements']*5,'switch':m['switches'],
           'clear_success':m['clear_successes']*5,'clear_fail':m['clear_failures']*3}
    accounting_error=abs(sum(costs.values())-client.virtual_time_s)
    if accounting_error>(m['measurements']+m['clear_attempts'])*.5001e-6+1e-7:
        error=(error or '')+'; cost accounting mismatch'
    cleared=transport.cleared
    full=error is None and cleared==set(truth) and solver.discovery_complete()
    stats=solver.stats if solver else {}
    return {'seed':case['seed'],'variant':variant,'pattern':case['pattern'],'error_mode':error_mode,
            'N':n,'sources_total':n,'directional_sources_total':sum(s['kind']=='directional' for s in truth.values()),
            'omni_sources_total':sum(s['kind']=='omni' for s in truth.values()),
            'cleared_total':len(cleared),'full_clear':full,'error':error,
            's_per_source':client.virtual_time_s/n,'metrics':m,'cost_s':costs,
            'CPU_s':cpu,'wall_s':wall,'accounting_error_s':accounting_error,
            'absent_channel_measurements':absent_m,
            'existing_channel_measurements_before_detection_including_detection':existing_m,
            'first_detection_times_by_type':by_kind,
            'proven_directional_count':sum(t.type_state=='directional' for t in solver.tracks.values()) if solver else 0,
            'stats':stats,'case_sha256':hashlib.sha256(json.dumps(case,sort_keys=True).encode()).hexdigest()}


def summarize(rows):
    n=sum(r['N'] for r in rows); ts=[r['s_per_source'] for r in rows]
    full=all(r['full_clear'] for r in rows)
    out={'runs':len(rows),'sources_total':n,'cleared_total':sum(r['cleared_total'] for r in rows),
         'full_clear_runs':sum(r['full_clear'] for r in rows),'qualified':full,
         'weighted_s_per_source':sum(r['metrics']['virtual_time_s'] for r in rows)/n if full else None,
         'mean_s_per_source':statistics.mean(ts) if full else None,
         'P95_s_per_source':quantile(ts,.95) if full else None,
         'CVaR95_s_per_source':statistics.mean(sorted(ts)[-max(1,math.ceil(len(ts)*.05)):]) if full else None,
         'max_s_per_source':max(ts) if full else None,
         'directional_sources_total':sum(r['directional_sources_total'] for r in rows),
         'omni_sources_total':sum(r['omni_sources_total'] for r in rows),
         'CPU_s':sum(r['CPU_s'] for r in rows),
         'fallback_trigger_runs':sum(r['stats'].get('fallback_triggers',0)>0 for r in rows)}
    for key in ('movement_m','measurements','switches','clear_failures'):
        out['mean_'+key]=statistics.mean(r['metrics'][key] for r in rows)
    for key in ('certificate_sites_visited','fan_steps','fan_invariant_failures','guaranteed_clear_count',
                'guaranteed_clear_failures','fallback_clear_cover_count'):
        out['total_'+key]=sum(r['stats'].get(key,0) for r in rows)
    fans=sum(r['stats'].get('fan_steps',0) for r in rows)
    out['fan_first_branch_success_rate']=sum(r['stats'].get('fan_first_success',0) for r in rows)/fans if fans else None
    out['fan_second_branch_required_rate']=sum(r['stats'].get('fan_second_required',0) for r in rows)/fans if fans else None
    for kind in ('omni','directional'):
        times=[t for r in rows for t in r['first_detection_times_by_type'][kind]]
        out['mean_time_to_first_detection_'+kind]=statistics.mean(times) if times else None
    for key in ('absent_channel_measurements','existing_channel_measurements_before_detection_including_detection','proven_directional_count'):
        out['total_'+key]=sum(r[key] for r in rows)
    out['mean_cost_s']={k:statistics.mean(r['cost_s'][k] for r in rows) for k in ('move','measure','switch','clear_success','clear_fail')}
    out['by_N']={str(k):{'runs':len(sub),'qualified':all(r['full_clear'] for r in sub),
                        'P95_s_per_source':quantile([r['s_per_source'] for r in sub],.95),
                        'max_s_per_source':max(r['s_per_source'] for r in sub)}
                 for k in sorted({r['N'] for r in rows}) if (sub:=[r for r in rows if r['N']==k])}
    return out


def job(spec):
    seed,pattern,variant,error_mode,directory=spec
    case=make_case(seed,pattern=pattern)
    folder=Path(directory); folder.mkdir(parents=True,exist_ok=True)
    case_file=folder/f'{seed}-case.json'
    case_file.write_text(json.dumps(case,indent=2)+'\n',encoding='utf-8')
    result=run_case(case,variant,error_mode,folder/f'{seed}-{variant}-actions.jsonl')
    (folder/f'{seed}-{variant}-result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--start',type=int,required=True);p.add_argument('--count',type=int,required=True)
    p.add_argument('--variant',default='G0',choices=['G0']);p.add_argument('--pattern',default='random_mixed',choices=PATTERNS)
    p.add_argument('--error-mode',default='fixed_field',choices=ERROR_MODES)
    p.add_argument('--workers',type=int,default=4);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.count<=0 or args.workers<=0: p.error('positive count and workers required')
    if any(64100000<=s<=64139999 for s in range(args.start,args.start+args.count)):
        p.error('HOLDOUT locked until an explicitly frozen performance candidate')
    directory=args.output.with_suffix('')
    if args.output.exists() or directory.exists(): p.error('refusing to overwrite prior batch')
    directory.mkdir(parents=True)
    contract={'start':args.start,'count':args.count,'pattern':args.pattern,'error_mode':args.error_mode,'variant':args.variant}
    (directory/'contract.json').write_text(json.dumps(contract,indent=2)+'\n',encoding='utf-8')
    specs=[(s,args.pattern,args.variant,args.error_mode,str(directory)) for s in range(args.start,args.start+args.count)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        rows=list(pool.map(job,specs))
    report={'contract':contract,'summary':summarize(rows),'runs':rows}
    args.output.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report['summary'],indent=2),flush=True)
    return 0 if report['summary']['qualified'] else 1


if __name__=='__main__':
    raise SystemExit(main())
