"""Post-run hindsight diagnostics. Hidden source truth is never sent to solver."""
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT/'B3/code')); sys.path.insert(0, str(HERE))
from run_batch import job
from e12_coverage_hook import E11
from e12_risk_experiment import dump


def analyze(case, result):
    events = [json.loads(x) for x in (ROOT/result['action_log']).read_text(encoding='utf-8').splitlines()]
    structural = []
    for event in events:
        if event.get('kind') != 'action' or event['path'] not in ('/measure', '/clear'):
            continue
        reason = event.get('decision_reason', '')
        if 'joint_route_transit' in reason or 'mec_center_recovery' in reason:
            continue
        pos = event['payload']['position']; p = (pos['x'], pos['y'])
        if not structural or p != structural[-1][2]:
            structural.append((event['path'], len(structural), p))
    route = E11.e5.fast_open_route((0., 0.), structural)
    structural_length = E11.e5.route_cost((0., 0.), structural)
    hindsight = E11.e5.route_cost((0., 0.), route)
    rotation = result['rotation']
    cover = [('cover', i, (1130*math.cos(math.radians(rotation+60*i)), 1130*math.sin(math.radians(rotation+60*i)))) for i in range(6)]
    true_nodes = cover+[('source_oracle', s['channel'], (s['x'], s['y'])) for s in case['sources']]
    oracle_route = E11.e5.fast_open_route((0., 0.), true_nodes)
    oracle = E11.e5.route_cost((0., 0.), oracle_route)
    return {'seed': case['seed'], 'N': len(case['sources']), 'actual_movement_m': result['metrics']['movement_m'],
            'actual_structural_order': structural, 'structural_length_m': structural_length,
            'hindsight_order': route, 'hindsight_length_m': hindsight,
            'pure_structural_reordering_saving_m': structural_length-hindsight,
            'transit_vs_structural_difference_m': result['metrics']['movement_m']-structural_length,
            'source_oracle_order': oracle_route, 'source_oracle_length_m': oracle,
            'warning': 'feasible hindsight heuristic, not an optimality lower-bound certificate; ignores online causality'}


if __name__ == '__main__':
    out = ROOT/'B3/结果/e13_regret10'
    if out.exists(): raise ValueError('do not overwrite benchmark')
    out.mkdir(parents=True)
    specs = [(53000000+i, 'random', []) for i in range(10)]
    with ProcessPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(job, spec, str(out)) for spec in specs]
        results = [f.result()[0] for f in futures]
    rows = []
    for result in results:
        assert result['qualified_run']
        case = json.loads((out/f"{result['seed']}-case.json").read_text(encoding='utf-8'))
        rows.append(analyze(case, result))
    dump(out/'benchmark.json', rows)
    sources = sum(r['N'] for r in rows)
    summary = {key: sum(r[key] for r in rows)/len(rows) for key in
               ('actual_movement_m', 'structural_length_m', 'hindsight_length_m', 'source_oracle_length_m',
                'pure_structural_reordering_saving_m', 'transit_vs_structural_difference_m')}
    summary['pure_reordering_saving_s_per_source'] = sum(r['pure_structural_reordering_saving_m'] for r in rows)/5/sources
    summary['N'] = sources
    dump(out/'summary.json', summary)
    print(json.dumps(summary), flush=True)
