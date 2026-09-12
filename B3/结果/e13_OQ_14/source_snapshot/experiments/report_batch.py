"""Post-run paired audit and regret decomposition; never called by online code."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT/'B3/code'))
sys.path.insert(0, str(HERE))
from e12_risk_experiment import dump, costs_from_actions, verify_protected
from e12_risk_report import summarize, tail, write_csv, quantile
from regret_benchmark import analyze


def read(p):
    return json.loads(p.read_text(encoding='utf-8'))


def audited_row(row, case):
    log = ROOT/row['action_log']
    raw = log.read_bytes()
    events = [json.loads(x) for x in raw.decode('utf-8').splitlines()]
    actions = [e for e in events if e.get('kind') == 'action']
    costs, _ = costs_from_actions(events)
    assert abs(sum(costs.values())-row['metrics']['virtual_time_s']) < 1e-4
    cleared = {e['payload']['channel'] for e in actions if e['path'] == '/clear'
               and e['response'].get('clear_result') == 'success'}
    if row['qualified_run']:
        assert cleared == {s['channel'] for s in case['sources']}
        assert actions[0]['path'] == '/enter' and actions[-1]['path'] == '/exit'
        assert all(e['response']['accepted'] for e in actions)
    recover_channel = None
    ray_distance = 0.
    for action in actions:
        if action['path'] == '/measure' and 'mec_center_recovery' in action.get('decision_reason', ''):
            recover_channel = action['payload']['channel']
        elif action['path'] == '/clear' and action['payload']['channel'] == recover_channel:
            ray_distance += math.dist(action['before']['position'], action['after']['position'])
            if action['response']['clear_result'] == 'success':
                recover_channel = None
        else:
            recover_channel = None
    return {**row, 'chosen_template': 6,
            'case_sha256': hashlib.sha256(json.dumps(case, sort_keys=True).encode()).hexdigest(),
            'action_log_sha256': hashlib.sha256(raw).hexdigest(),
            'recovery_ray_m': ray_distance,
            'activity_group': 'zero' if not row['b1plus_updates'] else '1-9' if row['b1plus_updates'] < 10 else '10+',
            'movement_per_source_m': row['metrics']['movement_m']/row['N']}


def summary(rows):
    s = summarize(rows)
    sources = sum(r['N'] for r in rows)
    s['movement_per_source_m'] = sum(r['metrics']['movement_m'] for r in rows)/sources
    s['mean_recovery_ray_m'] = statistics.mean(r['recovery_ray_m'] for r in rows)
    s['mean_supplement_count'] = statistics.mean(r['supplement_count'] for r in rows)
    s['max_wall_s'] = max(r['wall_s'] for r in rows)
    s['guaranteed_clear_failures'] = sum(r['guaranteed_clear_failures'] for r in rows)
    s['b1plus_empty'] = sum(r['b1plus_empty'] for r in rows)
    return s


def paired(rows, baseline, hindsight):
    pairs = []
    for row in rows:
        base = baseline[row['seed']]
        assert row['case_sha256'] == base['case_sha256'] and row['N'] == base['N']
        regret = base['metrics']['movement_m'] - hindsight[row['seed']]['hindsight_length_m']
        gain = base['metrics']['movement_m'] - row['metrics']['movement_m']
        p = {k: row[k] for k in ('seed', 'strategy', 'N', 'k0', 'activity_group', 'chosen_template')}
        p.update({'qualified_pair': row['qualified_run'] and base['qualified_run'],
                  'delta_time_per_source_s': row['time_per_source_s']-base['time_per_source_s'],
                  'delta_total_s': row['metrics']['virtual_time_s']-base['metrics']['virtual_time_s'],
                  'baseline_clear_failures': base['metrics']['clear_failures'],
                  'candidate_clear_failures': row['metrics']['clear_failures'],
                  'movement_gain_m': gain, 'hindsight_gap_m': regret,
                  'regret_capture_ratio': gain/regret if regret > 1e-7 else None})
        for key in ('movement_m', 'measurements', 'switches', 'clear_attempts', 'clear_failures'):
            p['delta_'+key] = row['metrics'][key]-base['metrics'][key]
        for key in row['costs']:
            p['delta_'+key] = row['costs'][key]-base['costs'][key]
        pairs.append(p)
    return pairs


def pair_stats(rows):
    valid = [r for r in rows if r['qualified_pair']]
    if not valid:
        return {'pairs': len(rows), 'qualified_pairs': 0}
    ds = [r['delta_time_per_source_s'] for r in valid]
    gap = sum(r['hindsight_gap_m'] for r in valid)
    worst = max(valid, key=lambda r: r['delta_time_per_source_s'])
    return {'pairs': len(rows), 'qualified_pairs': len(valid),
            'win_rate': sum(x < 0 for x in ds)/len(ds),
            'median_delta_s_per_source': statistics.median(ds),
            'P95_delta_s_per_source': quantile(ds, .95),
            'worst_delta_s_per_source': worst['delta_time_per_source_s'], 'worst_seed': worst['seed'],
            'aggregate_regret_capture_ratio': sum(r['movement_gain_m'] for r in valid)/gap if gap > 1e-7 else None}


def generate(folder):
    raw = read(folder/'results.json')
    contract = read(folder/'contract.json')
    expected = set(range(contract['start'], contract['start']+contract['count']))
    names = ['E11'] + contract['variants']
    assert len(raw) == len(expected)*len(names)
    assert {(r['seed'], r['strategy']) for r in raw} == {(s, n) for s in expected for n in names}
    rows = [audited_row(r, read(folder/f"{r['seed']}-case.json")) for r in raw]
    baseline = {r['seed']: r for r in rows if r['strategy'] == 'E11'}
    bench_path = folder/'regret_benchmark.json'
    if bench_path.exists():
        bench = read(bench_path)
    else:
        bench = [analyze(read(folder/f'{seed}-case.json'), baseline[seed]) for seed in sorted(expected)]
        dump(bench_path, bench)
    hindsight = {r['seed']: r for r in bench}
    absolute = {name: summary([r for r in rows if r['strategy'] == name]) for name in names}
    pairs = paired([r for r in rows if r['strategy'] != 'E11'], baseline, hindsight)
    ps = {name: pair_stats([r for r in pairs if r['strategy'] == name]) for name in contract['variants']}
    strata = []
    for name in contract['variants']:
        candidate = [r for r in rows if r['strategy'] == name]
        for field in ('N', 'k0', 'activity_group', 'chosen_template', 'clear_failures'):
            value_of = lambda r: r['metrics'][field] if field == 'clear_failures' else r[field]
            for value in sorted({value_of(r) for r in candidate}, key=str):
                sub = [r for r in candidate if value_of(r) == value]
                seeds = {r['seed'] for r in sub}
                sr = {'strategy': name, 'dimension': field, 'value': value,
                      **pair_stats([r for r in pairs if r['strategy'] == name and r['seed'] in seeds])}
                sr.update({'E11_'+k: v for k, v in summary([baseline[s] for s in seeds]).items()})
                sr.update({'E13_'+k: v for k, v in summary(sub).items()})
                strata.append(sr)
    dump(folder/'paired_audit.json', {'absolute': absolute, 'paired': ps, 'stratified': strata,
        'regret_note': 'Same-node hindsight heuristic ignores causality; ratio combines geometry, task and order changes, not a causal estimate of pure ordering improvement.',
        'recovery_note': 'Ray movement after mec_center_recovery until success or another action. Excludes center approach; reconstructed identically for every variant, supersedes inconsistent early telemetry.'})
    write_csv(folder/'paired.csv', pairs); dump(folder/'paired.json', pairs)
    write_csv(folder/'stratified_paired.csv', strata)
    write_csv(folder/'summary_extended.csv', [{'strategy': n, **absolute[n], **ps.get(n, {})} for n in names])
    flat = []
    for r in rows:
        flat.append({**{k: r[k] for k in ('seed', 'strategy', 'N', 'k0', 'all_cleared', 'qualified_run',
                      'time_per_source_s', 'recovery_ray_m', 'supplement_count', 'b1plus_updates', 'b1plus_empty',
                      'chosen_template', 'movement_per_source_m', 'wall_s', 'case_sha256', 'action_log_sha256')}, **r['metrics']})
    write_csv(folder/'runs_extended.csv', flat)
    worst = sorted(pairs, key=lambda r: r['delta_time_per_source_s'], reverse=True)[:10]
    write_csv(folder/'worst10_cost_deltas.csv', worst)
    return absolute, ps


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('folders', type=Path, nargs='+'); args = p.parse_args()
    verify_protected()
    for folder in args.folders:
        absolute, ps = generate(folder)
        print(folder.name, json.dumps({'absolute': absolute, 'paired': ps}), flush=True)
