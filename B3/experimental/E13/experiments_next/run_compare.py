import argparse, json, statistics, sys, time, traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
HERE=Path(__file__).resolve().parents[1]; ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT/'B3/code')); sys.path.insert(0,str(HERE))
from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client
from solver import build_e13
from solver_next import build_next
from solver_slot import build_slot
from solver_cap import build_cap
from solver_adaptive import build_adaptive
from solver_proxy import build_proxy
from solver_combo import build_combo
from solver_ensemble import build_ensemble
from solver_forced import build_forced
from solver_diag import build_diag


def run_one(seed, variants, pattern='random'):
    case=make_case(seed,pattern=pattern); mode='endpoints' if pattern=='boundary' else 'fixed_field'
    rows=[]
    for name in ['R']+variants:
        transport=OfflineTransport(case,error_mode=mode)
        # /tmp-ish log but unique; caller output is not required for exploratory batches.
        log=Path('/tmp')/f'e13next-{seed}-{name}.jsonl'
        if log.exists(): log.unlink()
        client=Client(transport,'offline-test',log,session_id=f'{seed}-{name}')
        err=None; solver=None
        try:
            client.enter(); solver=(build_e13(client,'R') if name=='R' else build_slot(client,name) if name.startswith('D') else build_cap(client,name) if name.startswith('C') else build_adaptive(client,name) if name.startswith('A') else build_proxy(client,name) if name.startswith('P') else build_combo(client,name) if name.startswith('X') else build_ensemble(client,name) if name.startswith('E') else build_forced(client,name) if name.startswith('F') else build_diag(client,name) if name.startswith('Q') else build_next(client,name)); solver.run_all(); client.exit()
        except Exception:
            err=traceback.format_exc()
            if client.active:
                try: client.exit()
                except Exception: pass
        complete=bool(not err and solver is not None and solver.discovery_complete() and
                      set(solver.tracks)==transport.cleared=={s['channel'] for s in case['sources']})
        rows.append({'seed':seed,'name':name,'N':len(case['sources']),'ok':complete,'vt':client.virtual_time_s,
                     'per':client.virtual_time_s/len(case['sources']),'move':client.stats['movement_m'],
                     'meas':client.stats['measurements'],'switch':client.stats['switches'],
                     'fails':client.stats['clear_failures'],'risky':getattr(solver,'risky_clear_tasks',0) if solver else None,
                     'supp':getattr(solver,'stats',{}).get('supplemental_points',0) if solver else None,'err':err})
        try: log.unlink()
        except Exception: pass
    return rows


def summarize(rows,name):
    r=[x for x in rows if x['name']==name]; src=sum(x['N'] for x in r)
    return {'runs':len(r),'all':all(x['ok'] for x in r),'weighted':sum(x['vt'] for x in r)/src,
            'meanmove':statistics.mean(x['move'] for x in r),'meanmeas':statistics.mean(x['meas'] for x in r),
            'meanfails':statistics.mean(x['fails'] for x in r),'maxper':max(x['per'] for x in r)}


def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--start',type=int,required=True); ap.add_argument('--count',type=int,required=True)
    ap.add_argument('--variants',nargs='+',required=True); ap.add_argument('--workers',type=int,default=4); ap.add_argument('--pattern',default='random')
    ap.add_argument('--output',type=Path); a=ap.parse_args()
    rows=[]
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        fs=[ex.submit(run_one,a.start+i,a.variants,a.pattern) for i in range(a.count)]
        for f in as_completed(fs):
            rr=f.result(); rows+=rr; print([(x['seed'],x['name'],round(x['per'],2),round(x['move']),x['ok']) for x in rr],flush=True)
    out={n:summarize(rows,n) for n in ['R']+a.variants}
    # paired deltas
    by={(x['seed'],x['name']):x for x in rows}
    for n in a.variants:
        ds=[by[(s,n)]['per']-by[(s,'R')]['per'] for s in range(a.start,a.start+a.count)]
        moves=[by[(s,n)]['move']-by[(s,'R')]['move'] for s in range(a.start,a.start+a.count)]
        out[n]['mean_delta_per']=statistics.mean(ds); out[n]['median_delta_per']=statistics.median(ds); out[n]['mean_delta_move']=statistics.mean(moves)
        out[n]['wins']=sum(d<0 for d in ds)
    print(json.dumps(out,indent=2),flush=True)
    if a.output:
        a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps({'summary':out,'rows':rows},indent=2),encoding='utf-8')

if __name__=='__main__':main()
