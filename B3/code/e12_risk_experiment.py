"""Pre-registered offline comparison. No simulator or solver changes."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import traceback

from coverage_templates import E11_TEMPLATE, SEVEN_TEMPLATE
from e12_coverage_hook import build_offline_solver
from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'B3/结果/e12_development'
THRESHOLDS = {'T2': 2, 'T4': 4, 'T6': 6}
VOLATILE = {'request_id', 'session_id', 'wall_elapsed_s', 'real_timestamp_ms'}


def dump(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def canonical(value):
    if isinstance(value, dict):
        return {k: canonical(v) for k, v in value.items() if k not in VOLATILE}
    if isinstance(value, (list, tuple)):
        return [canonical(v) for v in value]
    return value


def object_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def selector_for(name):
    if name == 'six':
        return lambda k0: E11_TEMPLATE
    threshold = THRESHOLDS[name]
    return lambda k0: SEVEN_TEMPLATE if k0 >= threshold else E11_TEMPLATE


def verify_protected():
    paths = {}
    frozen = json.loads((ROOT/'B3/结果/e12_phase_a/e11_frozen_manifest.json').read_text(encoding='utf-8'))
    for item in frozen['files']:
        paths[item['path']] = item['sha256_lf']
    phase_a = json.loads((ROOT/'B3/结果/e12_phase_a/artifact_manifest.json').read_text(encoding='utf-8'))
    for item in phase_a['files']:
        if item['path'].endswith(('coverage_templates.py', 'e12_coverage_hook.py')):
            paths[item['path']] = item['sha256_lf']
    for name, expected in paths.items():
        actual = hashlib.sha256((ROOT/name).read_bytes().replace(b'\r\n', b'\n')).hexdigest()
        if actual != expected:
            raise RuntimeError('protected source changed: '+name)
    return paths


def provenance():
    hashes = verify_protected()
    for name in ('e12_risk_experiment.py', 'e12_risk_report.py'):
        p = Path(__file__).with_name(name)
        if p.exists():
            hashes[p.relative_to(ROOT).as_posix()] = hashlib.sha256(p.read_bytes().replace(b'\r\n', b'\n')).hexdigest()
    return hashes


def definitions(phase):
    if phase == 'development':
        return [{'case': make_case(seed), 'error_mode': 'fixed_field', 'group': 'random'}
                for seed in range(27182800, 27183000)]
    groups = [('random', 'fixed_field'), ('boundary', 'endpoints'), ('cluster', 'fixed_field'),
              ('origin', 'fixed_field'), ('random', 'endpoints')]
    result = []
    for i in range(100):
        pattern, mode = groups[i//20]
        result.append({'case': make_case(42424200+i, pattern=pattern), 'error_mode': mode,
                       'group': pattern+'/'+mode})
    return result


def costs_from_actions(actions):
    totals = {'movement_s': 0., 'measurement_s': 0., 'switch_s': 0.,
              'clear_success_s': 0., 'clear_failure_s': 0.}
    by_reason = {}
    for action in actions:
        if action.get('kind') != 'action' or action['path'] not in ('/measure', '/clear'):
            continue
        reason = action.get('decision_reason') or action['path']
        row = by_reason.setdefault(reason, {**dict.fromkeys(totals, 0.), 'actions': 0})
        c = dict.fromkeys(totals, 0.)
        c['movement_s'] = math.dist(action['before']['position'], action['after']['position'])/5
        if action['path'] == '/measure':
            c['measurement_s'] = 5
            c['switch_s'] = int(action['before']['channel'] != action['after']['channel'])
        else:
            c['clear_success_s' if action['response']['clear_result'] == 'success' else 'clear_failure_s'] = 5 if action['response']['clear_result'] == 'success' else 3
        row['actions'] += 1
        for key, value in c.items():
            totals[key] += value
            row[key] += value
    return totals, by_reason


def run_one(spec, name, folder):
    case = spec['case']; seed = case['seed']
    log = folder/f'{seed}-{name}.jsonl'
    if log.exists():
        raise RuntimeError('refuse to overwrite '+str(log))
    transport = OfflineTransport(case, error_mode=spec['error_mode'])
    client = Client(transport, 'offline-test', log, session_id=f'{seed}-{name}')
    start = time.perf_counter()
    # A wall guard observes execution without altering any solver decision.
    original_record = client.record
    def timed_record(event):
        original_record(event)
        if time.perf_counter()-start > 120:
            raise TimeoutError('pre-registered per-run 120 second budget')
    client.record = timed_record
    solver = None; error = None
    try:
        client.enter()
        solver = build_offline_solver(client, baseline=name == 'E11',
                                      selector=None if name == 'E11' else selector_for(name))
        solver.run_all()
        client.exit()
    except Exception:
        error = traceback.format_exc()
        if client.active:
            try:
                client.exit()
            except Exception:
                error += '\nexit: '+traceback.format_exc()
    actions = [e for e in client.events if e.get('kind') == 'action']
    trace = canonical(client.events)
    origin_channels = set()
    for e in actions:
        if e['path'] not in ('/measure', '/clear'):
            continue
        p = e['payload']['position']
        if (p['x'], p['y']) != (0, 0):
            break
        if e['path'] == '/measure' and e['response']['measure_result'] in ('near', 'direction'):
            origin_channels.add(e['payload']['channel'])
    costs, reasons = costs_from_actions(actions)
    n = len(case['sources'])
    channels = {s['channel'] for s in case['sources']}
    discovered = set(solver.tracks) if solver is not None else set()
    discovery = solver.discovery_complete() if solver is not None else False
    chosen = solver.coverage_template.n if solver is not None and hasattr(solver, 'coverage_template') else 6
    evidence = (solver is not None and (len(discovered) == 16 or
                all(solver.checked[c] == set(range(len(solver.sites))) for c in set(range(1, 21))-discovered)))
    success = not error and discovery and channels == transport.cleared == discovered
    guards = {'all_cleared': bool(success), 'station_evidence': bool(evidence),
              'enter_exit': bool(actions) and actions[0]['path'] == '/enter' and actions[-1]['path'] == '/exit',
              'all_accepted': all(e['response']['accepted'] is True for e in actions),
              'accounting': abs(sum(costs.values())-client.virtual_time_s) < 1e-4}
    metrics = dict(client.stats)
    metrics['virtual_time_s'] = client.virtual_time_s
    result = {'seed': seed, 'strategy': name, 'case_hash': object_hash(case), 'group': spec['group'],
              'error_mode': spec['error_mode'], 'N': n, 'k0': len(origin_channels), 'chosen_template': chosen,
              'selector_invoked': getattr(solver, 'k0', None) is not None,
              'rotation': getattr(solver, 'rotation', None), 'all_cleared': bool(success),
              'qualified_run': all(guards.values()), 'guards': guards,
              'cleared_count': len(transport.cleared), 'discovered_count': len(discovered),
              'uncleared_channels': sorted(channels-transport.cleared), 'metrics': metrics,
              'time_per_source_s': client.virtual_time_s/n, 'costs': costs, 'costs_by_reason': reasons,
              'action_count': len(actions), 'canonical_sha256': object_hash(trace),
              'action_log': log.relative_to(ROOT).as_posix(), 'action_log_sha256': hashlib.sha256(log.read_bytes()).hexdigest(),
              'wall_s': time.perf_counter()-start, 'error': error}
    dump(folder/f'{seed}-{name}.json', result)
    return result, trace


def run_case_job(spec, names, folder):
    rows = []; base_trace = None; base_metrics = None
    for name in names:
        result, trace = run_one(spec, name, Path(folder))
        if name == 'E11':
            base_trace, base_metrics = trace, result['metrics']
        else:
            result['six_action_equivalent'] = None
            if result['chosen_template'] == 6:
                result['six_action_equivalent'] = trace == base_trace and result['metrics'] == base_metrics
                result['qualified_run'] = result['qualified_run'] and result['six_action_equivalent']
        dump(Path(folder)/f"{spec['case']['seed']}-{name}.json", result)
        rows.append(result)
    return rows


def run_batch(phase, output, workers=4):
    output = Path(output)
    if output.exists():
        raise ValueError('use a new output directory; existing trials are immutable')
    hashes = provenance()
    specs = definitions(phase)
    names = ('E11', 'six') if phase == 'equivalence' else ('E11', 'T2', 'T4', 'T6')
    if phase == 'development':
        gate = json.loads((OUT/'equivalence_gate.json').read_text(encoding='utf-8'))
        if gate['cases'] != 100 or not gate['passed'] or gate['provenance'] != hashes:
            raise RuntimeError('100-case exact equivalence gate missing or source changed')
    output.mkdir(parents=True)
    dump(output/'dataset.json', specs)
    dump(output/'provenance.json', hashes)
    trace_folder = ROOT/'B3/结果/offline_logs'/('e12-risk-'+phase+'-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f'))
    trace_folder.mkdir(parents=True)
    rows = []; disqualified = set(); completed = 0
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_case_job, spec, names, str(trace_folder)): spec['case']['seed'] for spec in specs}
        for future in as_completed(futures):
            case_rows = future.result()
            for row in case_rows:
                rows.append(row)
                # Copy per-run result as a portable, tracked record; action logs remain local.
                dump(output/'runs'/f"{row['seed']}-{row['strategy']}.json", row)
                if not row['qualified_run']:
                    disqualified.add(row['strategy'])
                    with (output/'events.jsonl').open('a', encoding='utf-8') as f:
                        f.write(json.dumps({'event': 'disqualified', 'seed': row['seed'], 'strategy': row['strategy'],
                                            'guards': row['guards'], 'six_action_equivalent': row.get('six_action_equivalent'),
                                            'error': row['error']}, ensure_ascii=False)+'\n')
            completed += 1
            dump(output/'progress.json', {'completed_cases': completed, 'total_cases': len(specs), 'disqualified': sorted(disqualified)})
            print(f'{phase}: {completed}/{len(specs)} seed={case_rows[0]["seed"]} disqualified={sorted(disqualified)}', flush=True)
    rows.sort(key=lambda r: (r['seed'], names.index(r['strategy'])))
    dump(output/'results.json', rows)
    verify_protected()
    passed = not disqualified and len(rows) == len(specs)*len(names)
    if phase == 'equivalence':
        gate = {'cases': len(specs), 'runs': len(rows), 'passed': passed,
                'exact_pairs': sum(r.get('six_action_equivalent') is True for r in rows),
                'provenance': hashes, 'result_file': str((output/'results.json').relative_to(ROOT))}
        dump(OUT/'equivalence_gate.json', gate)
    else:
        dump(output/'completion.json', {'cases': len(specs), 'runs': len(rows), 'qualified': passed,
                                       'disqualified': sorted(disqualified), 'stop': 'development complete; user review required'})
    return passed


def main():
    p = argparse.ArgumentParser()
    p.add_argument('phase', choices=('equivalence', 'development'))
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--workers', type=int, choices=range(1, 5), default=4)
    args = p.parse_args()
    if not run_batch(args.phase, args.output.resolve(), args.workers):
        raise SystemExit(2)


if __name__ == '__main__':
    main()
