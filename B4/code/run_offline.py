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
from solver_g1 import G1Solver, G1DeferredSolver
from solver_g2 import G2OpportunisticSolver, G2RouteFirstSolver, G2BetterRouteSolver, G2Detour250Solver, G2Detour500Solver, G2Detour800Solver, G2WideSolver, G2Range1000Solver, G2Angle20Solver, G2SixSolver, G2TwoSolver, G2Six1000Solver, G2Six1100Solver, G2Six1400Solver, G2SixAngle10Solver, G2EightSolver
from solver_g3 import G3SpineSolver, G3Spine300, G3Spine450, G3Spine800, G3Spine1200
from solver_g4 import G4TransitSolver, G4Transit2, G4Transit6, G4Transit8
from solver_g5 import G5Info1, G5Info15, G5Info2, G5Info2R1000, G5Info15R1000
from solver_g6 import G6Cap4,G6Cap5,G6Cap6,G6Cap7,G6Cap8
from solver_g7 import G7Fine9,G7Fine7,G7Mid5,G7Fine9K3,G7Fine9K5
from solver_g8 import G8Probe1,G8Probe2,G8Probe3,G8Probe4
from solver_g9 import G9P1,G9P2,G9P3,G9P5
from solver_g10 import G10D0,G10D100,G10D200,G10D300,G10D500,G10NoDefer
from solver_g11 import G11SafeClear
from solver_g12 import G12TransitClear
from solver_g13 import G13FullClearRegion
from solver_g14 import G14TransitFullClear
from solver_g15 import G15TightMesh, G15SafeMesh
from solver_g16 import G16SurrogateMesh, G16Aggressive, G16Conservative, G16CheapProbe
from solver_g17 import G17TSPN, G17OneRound, G17ThreeRounds
from solver_g18 import G18Convex21,G18P8,G18T6,G18P8T6,G18Wide,G18Compact21,G18CompactT6,G18T8,G18A,G18B,G18D,G18B8,G18R100,G18R250,G18R500
from solver_g34 import G34R40,G34R60,G34R80,G34R100,G34R120,G34R180,G34R250,G34R350,G34R500,G34R800
from solver_g35 import G35SafeBackbone,G36SafeBackboneDisks,G37SafeRouteClear,G38S75,G38S100,G38S125,G39P8,G39P10,G39P12,G39P15,G40PassiveSafe
from solver_g31 import G31A,G31B,G31C,G31D,G31E,G31F
from solver_g30 import G30B2,G30B3,G30B5,G30B75,G30B10,G30B15,G30B20,G30B25,G30B30,G30B45,G30B4,G30B45x,G30B55,G30B6,G30B65,G30B8,G30B5F98,G30B5F99,G30B5F1001,G30B5F1002,G30B6F1003

SOLVERS={'G0':G0Solver,'G1':G1Solver,'G1D':G1DeferredSolver,'G2':G2OpportunisticSolver,'G2R':G2RouteFirstSolver,'G2T':G2BetterRouteSolver,'G2D250':G2Detour250Solver,'G2D500':G2Detour500Solver,'G2D800':G2Detour800Solver,'G2W':G2WideSolver,'G2R1000':G2Range1000Solver,'G2A20':G2Angle20Solver,'G2SIX':G2SixSolver,'G2TWO':G2TwoSolver,'G2S1000':G2Six1000Solver,'G2S1100':G2Six1100Solver,'G2S1400':G2Six1400Solver,'G2SA10':G2SixAngle10Solver,'G2EIGHT':G2EightSolver,'G3':G3SpineSolver,'G3S300':G3Spine300,'G3S450':G3Spine450,'G3S800':G3Spine800,'G3S1200':G3Spine1200,'G4':G4TransitSolver,'G4T2':G4Transit2,'G4T6':G4Transit6,'G4T8':G4Transit8,'G5I1':G5Info1,'G5I15':G5Info15,'G5I2':G5Info2,'G5I2R':G5Info2R1000,'G5I15R':G5Info15R1000,'G6C4':G6Cap4,'G6C5':G6Cap5,'G6C6':G6Cap6,'G6C7':G6Cap7,'G6C8':G6Cap8,'G7F9':G7Fine9,'G7F7':G7Fine7,'G7M5':G7Mid5,'G7F9K3':G7Fine9K3,'G7F9K5':G7Fine9K5,'G8P1':G8Probe1,'G8P2':G8Probe2,'G8P3':G8Probe3,'G8P4':G8Probe4,'G9P1':G9P1,'G9P2':G9P2,'G9P3':G9P3,'G9P5':G9P5,'G10D0':G10D0,'G10D100':G10D100,'G10D200':G10D200,'G10D300':G10D300,'G10D500':G10D500,'G10N':G10NoDefer,'G11':G11SafeClear,'G12':G12TransitClear,'G13':G13FullClearRegion,'G14':G14TransitFullClear,'G15':G15TightMesh,'G15S':G15SafeMesh,'G16':G16SurrogateMesh,'G16A':G16Aggressive,'G16C':G16Conservative,'G16P':G16CheapProbe,'G17':G17TSPN,'G17R1':G17OneRound,'G17R3':G17ThreeRounds,'G18':G18Convex21,'G18P8':G18P8,'G18T6':G18T6,'G18P8T6':G18P8T6,'G18W':G18Wide,'G18C':G18Compact21,'G18CT6':G18CompactT6,'G18T8':G18T8,'G18A':G18A,'G18B':G18B,'G18D':G18D,'G18B8':G18B8,'G18R100':G18R100,'G18R250':G18R250,'G18R500':G18R500,'G30B2':G30B2,'G30B3':G30B3,'G30B5':G30B5,'G30B75':G30B75,'G30B10':G30B10,'G30B15':G30B15,'G30B20':G30B20,'G30B25':G30B25,'G30B30':G30B30,'G30B45':G30B45,'G30B4':G30B4,'G30B45x':G30B45x,'G30B55':G30B55,'G30B6':G30B6,'G30B65':G30B65,'G30B8':G30B8,'G30B5F98':G30B5F98,'G30B5F99':G30B5F99,'G30B5F1001':G30B5F1001,'G30B5F1002':G30B5F1002,'G30B6F1003':G30B6F1003,'G31A':G31A,'G31B':G31B,'G31C':G31C,'G31D':G31D,'G31E':G31E,'G31F':G31F,'G34R40':G34R40,'G34R60':G34R60,'G34R80':G34R80,'G34R100':G34R100,'G34R120':G34R120,'G34R180':G34R180,'G34R250':G34R250,'G34R350':G34R350,'G34R500':G34R500,'G34R800':G34R800,'G35':G35SafeBackbone,'G36':G36SafeBackboneDisks,'G37':G37SafeRouteClear,'G38S75':G38S75,'G38S100':G38S100,'G38S125':G38S125,'G39P8':G39P8,'G39P10':G39P10,'G39P12':G39P12,'G39P15':G39P15,'G40':G40PassiveSafe}


def quantile(xs,q):
    xs=sorted(xs); u=(len(xs)-1)*q; i=int(u)
    return xs[i]+(xs[min(i+1,len(xs)-1)]-xs[i])*(u-i)


def run_case(case,variant='G0',error_mode='fixed_field',log_path=None):
    if variant not in SOLVERS: raise ValueError('unsupported variant')
    transport=OfflineTransportB4(case,error_mode=error_mode)
    client=Client(transport,'offline-test',log_path=log_path,session_id=f"b4-{case['seed']}-{variant}")
    start=time.perf_counter(); cpu=time.process_time(); error=None; solver=None
    try:
        client.enter(); solver=SOLVERS[variant](RobotPort(client)); solver.run_all()
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
    p.add_argument('--variant',default='G0',choices=sorted(SOLVERS));p.add_argument('--pattern',default='random_mixed',choices=PATTERNS)
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
