"""Paired E13 experiments with source-blind online clients."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT/'B3/code')); sys.path.insert(0, str(HERE))
from solver import build_e13
from e12_coverage_hook import E11, build_offline_solver
from e12_risk_experiment import dump, verify_protected, costs_from_actions
from e12_risk_report import summarize, pair_summary, write_csv, tail
from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client


def job(spec, output):
    seed, pattern, variants = spec
    case = make_case(seed, pattern=pattern)
    out = Path(output); dump(out/f'{seed}-case.json', case)
    mode = 'endpoints' if pattern == 'boundary' else 'fixed_field'
    rows = []
    for name in ['E11']+variants:
        log = out/f'{seed}-{name}-actions.jsonl'
        transport = OfflineTransport(case, error_mode=mode)
        client = Client(transport, 'offline-test', log, session_id=f'{seed}-{name}')
        started = time.perf_counter(); old_record = client.record
        def record(e):
            old_record(e)
            if time.perf_counter()-started > 120:
                raise TimeoutError('E13 experiment budget exceeded')
        client.record = record
        error = None; engine = None
        try:
            client.enter()
            engine = build_offline_solver(client, baseline=True) if name == 'E11' else build_e13(client, name)
            engine.run_all()
            client.exit()
        except Exception:
            error = traceback.format_exc()
            if client.active:
                try: client.exit()
                except Exception: pass
        complete = bool(not error and engine is not None and engine.discovery_complete()
                        and set(engine.tracks) == transport.cleared == {s['channel'] for s in case['sources']})
        costs, reasons = costs_from_actions(client.events)
        metrics = dict(client.stats); metrics['virtual_time_s'] = client.virtual_time_s
        k0 = set()
        for e in client.events:
            if e.get('kind') != 'action' or e['path'] not in ('/measure', '/clear'): continue
            p = e['payload']['position']
            if (p['x'], p['y']) != (0, 0): break
            if e['path'] == '/measure' and e['response']['measure_result'] != 'no_signal': k0.add(e['payload']['channel'])
        record_result = {'seed': seed, 'strategy': name, 'pattern': pattern, 'N': len(case['sources']), 'k0': len(k0),
             'all_cleared': complete, 'qualified_run': complete and abs(sum(costs.values())-client.virtual_time_s)<1e-4,
             'time_per_source_s': client.virtual_time_s/len(case['sources']), 'metrics': metrics,
             'costs': costs, 'costs_by_reason': reasons, 'wall_s': time.perf_counter()-started,
             'cleared': len(transport.cleared), 'rotation': getattr(engine, 'rotation', None), 'error': error,
             'b1plus_updates': getattr(engine, 'b1plus_updates', 0), 'b1plus_empty': getattr(engine, 'b1plus_empty', 0),
             'guaranteed_clear_failures': getattr(engine, 'guaranteed_clear_failures', 0),
             'supplement_count': getattr(engine, 'stats', {}).get('supplemental_points', 0),
             'joint_supp': getattr(engine, 'joint_supp', 0), 'action_log': str(log.relative_to(ROOT))}
        if record_result['guaranteed_clear_failures']:
            record_result['qualified_run'] = False
        dump(out/f'{seed}-{name}-result.json', record_result); rows.append(record_result)
    return rows


def main():
    p = argparse.ArgumentParser(); p.add_argument('--start', type=int, required=True); p.add_argument('--count', type=int, required=True)
    p.add_argument('--variants', nargs='+', default=['A']); p.add_argument('--pattern', default='random')
    p.add_argument('--output', type=Path, required=True); args = p.parse_args()
    if args.output.exists(): raise ValueError('refuse to overwrite experiment')
    verify_protected(); args.output.mkdir(parents=True)
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.rglob('*.py')}
    for source in HERE.rglob('*.py'):
        target = args.output/'source_snapshot'/source.relative_to(HERE)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
    dump(args.output/'contract.json', {'start': args.start, 'count': args.count, 'variants': args.variants,
                                     'pattern': args.pattern, 'hashes': hashes})
    specs = [(args.start+i, args.pattern, args.variants) for i in range(args.count)]
    rows = []
    with ProcessPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(job, s, str(args.output.resolve())) for s in specs]
        for future in as_completed(futures):
            result = future.result(); rows += result
            dump(args.output/'results.json', rows)
            print([(r['seed'], r['strategy'], r['qualified_run'], round(r['time_per_source_s'], 2), r['error']) for r in result], flush=True)
    summary = {name: summarize([r for r in rows if r['strategy'] == name]) for name in ['E11']+args.variants}
    dump(args.output/'summary.json', summary); print(json.dumps(summary), flush=True)
    verify_protected()


if __name__ == '__main__': main()
