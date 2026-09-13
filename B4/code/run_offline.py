"""Run the final B4 solver against the bundled deterministic offline simulator."""
import argparse, hashlib, json, statistics, sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
if str(HERE) not in sys.path: sys.path.insert(0,str(HERE))
from .offline_environment_b4 import OfflineTransportB4, make_case
from .protocol import Client
from .solver import B4Solver, RobotPort

ROOT=HERE.parent
_FROZEN_DEV_PATH=ROOT/'results/dev50_cases.json'
_FROZEN_DEV=None

def _case_sha256(case):
    canonical=json.dumps(case,sort_keys=True,separators=(',',':'),ensure_ascii=False)
    return hashlib.sha256(canonical.encode('utf-8')).hexdigest()

def _load_frozen_dev():
    global _FROZEN_DEV
    if _FROZEN_DEV is None:
        payload=json.loads(_FROZEN_DEV_PATH.read_text(encoding='utf-8'))
        _FROZEN_DEV=payload['cases']
    return _FROZEN_DEV

def canonical_case(seed, pattern='random_mixed'):
    """Return the frozen DEV case when available, otherwise generate one.

    Freezing DEV inputs makes reproduction independent of Python stdlib
    implementation details in random.sample()/randrange().
    """
    if pattern=='random_mixed':
        item=_load_frozen_dev().get(str(seed))
        if item is not None:
            case=item['case']
            actual=_case_sha256(case)
            if actual!=item['sha256']:
                raise RuntimeError(f'frozen case SHA256 mismatch for seed {seed}: {actual}')
            return case
    return make_case(seed,pattern=pattern)

def run_case(seed, pattern='random_mixed', error_mode='fixed_field'):
    case=canonical_case(seed,pattern)
    transport=OfflineTransportB4(case,error_mode=error_mode)
    client=Client(transport,'offline-test',session_id=f'b4-final-{seed}')
    solver=None; error=None
    try:
        client.enter(); solver=B4Solver(RobotPort(client)); solver.run_all()
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}'
    finally:
        if client.active:
            try: client.exit()
            except Exception as exc: error=(error or '')+f'; exit: {exc}'
    truth={s['channel'] for s in case['sources']}
    full=(error is None and transport.cleared==truth and solver.discovery_complete())
    return {
        'seed':seed,'pattern':pattern,'N':len(truth),'full_clear':full,'error':error,
        'virtual_time_s':client.virtual_time_s,'s_per_source':client.virtual_time_s/len(truth),
        'case_sha256':_case_sha256(case),
        'metrics':dict(client.stats),'solver_stats':dict(solver.stats) if solver else {},
    }

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--start-seed',type=int,default=64000000)
    ap.add_argument('--count',type=int,default=1)
    ap.add_argument('--pattern',default='random_mixed')
    ap.add_argument('--output',type=Path)
    args=ap.parse_args()
    rows=[run_case(args.start_seed+i,args.pattern) for i in range(args.count)]
    total_sources=sum(r['N'] for r in rows)
    summary={
        'runs':len(rows),'full_clear_runs':sum(r['full_clear'] for r in rows),
        'sources_total':total_sources,
        'weighted_s_per_source':sum(r['virtual_time_s'] for r in rows)/total_sources,
        'mean_s_per_source':statistics.mean(r['s_per_source'] for r in rows),
    }
    payload={'summary':summary,'rows':rows}
    text=json.dumps(payload,ensure_ascii=False,indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text+'\n',encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
