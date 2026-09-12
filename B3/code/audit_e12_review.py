"""Read saved E12 evidence; verify exact traces and add total-time paired tables.

No strategy execution, simulator connection, threshold selection or source edits.
"""
import hashlib
import json
import statistics
from collections import Counter

from e12_risk_experiment import ROOT, OUT, canonical, dump, object_hash, verify_protected
from e12_risk_report import pair, quantile, write_csv


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def events(row):
    raw = (ROOT / row['action_log']).read_bytes()
    assert hashlib.sha256(raw).hexdigest() == row['action_log_sha256']
    trace = canonical([json.loads(line) for line in raw.decode('utf-8').splitlines()])
    assert object_hash(trace) == row['canonical_sha256']
    return trace


def total_summary(rows):
    assert rows and all(r['qualified_pair'] for r in rows)
    ds = [r['delta_total_virtual_s'] for r in rows]
    worst = max(rows, key=lambda r: r['delta_total_virtual_s'])
    return {
        'pairs': len(rows), 'win_rate': sum(d < 0 for d in ds) / len(ds),
        'tie_rate': sum(d == 0 for d in ds) / len(ds),
        'median_delta_total_s': statistics.median(ds),
        'P95_delta_total_s': quantile(ds, .95),
        'worst_delta_total_s': worst['delta_total_virtual_s'],
        'worst_seed_by_total_s': worst['seed'],
    }


def main():
    protected = verify_protected()
    gate = read(OUT / 'equivalence_gate.json')
    eq_path = ROOT / gate['result_file']
    eq = read(eq_path)
    eq_specs = read(eq_path.parent / 'dataset.json')
    assert len(eq_specs) == 100
    assert {s['case']['seed'] for s in eq_specs} == set(range(42424200, 42424300))
    groups = Counter(s['group'] for s in eq_specs)
    assert len(groups) == 5 and set(groups.values()) == {20}
    eq_lookup = {(r['seed'], r['strategy']): r for r in eq}
    assert len(eq_lookup) == len(eq) == 200
    for spec in eq_specs:
        seed = spec['case']['seed']
        baseline, candidate = (eq_lookup[seed, name] for name in ('E11', 'six'))
        for row in (baseline, candidate):
            assert row['qualified_run'] and row['all_cleared']
            assert row['chosen_template'] == 6
            assert row['case_hash'] == object_hash(spec['case'])
            assert row['error_mode'] == spec['error_mode']
        assert events(baseline) == events(candidate), seed
        assert baseline['metrics'] == candidate['metrics'], seed

    folder = OUT / 'development200'
    rows = read(folder / 'results.json')
    specs = read(folder / 'dataset.json')
    lookup = {(r['seed'], r['strategy']): r for r in rows}
    assert len(lookup) == len(rows) == 800
    assert {s['case']['seed'] for s in specs} == set(range(27182800, 27183000))
    recomputed = []
    six_count = 0
    for spec in specs:
        seed = spec['case']['seed']
        base = lookup[seed, 'E11']
        base_trace = events(base)
        for name in ('E11', 'T2', 'T4', 'T6'):
            row = lookup[seed, name]
            assert row['case_hash'] == object_hash(spec['case'])
            assert row['error_mode'] == spec['error_mode'] == 'fixed_field'
            assert row['qualified_run'] and row['all_cleared']
            if name == 'E11':
                continue
            trace = events(row)
            recomputed.append(pair(base, row))
            if row['chosen_template'] == 6:
                assert trace == base_trace and row['metrics'] == base['metrics']
                six_count += 1
    recomputed.sort(key=lambda r: (r['seed'], r['strategy']))
    assert recomputed == sorted(read(folder / 'paired.json'), key=lambda r: (r['seed'], r['strategy']))
    assert len(read(folder / 'worst10_unique_cases.json')) == 10
    for row in read(folder / 'worst10_unique_cases.json'):
        assert (folder / 'worst_actions' / f"{row['seed']}-{row['strategy']}.csv").exists()

    summaries = []
    for name in ('T2', 'T4', 'T6'):
        candidate = [r for r in recomputed if r['strategy'] == name]
        summaries.append({'strategy': name, 'dimension': 'all', 'value': 'all', **total_summary(candidate)})
        for field in ('N', 'k0', 'chosen_template'):
            for value in sorted({r[field] for r in candidate}):
                subset = [r for r in candidate if r[field] == value]
                summaries.append({'strategy': name, 'dimension': field, 'value': value, **total_summary(subset)})
    write_csv(folder / 'paired_total_time_summary.csv', summaries)
    dump(folder / 'paired_total_time_summary.json', summaries)
    result = {
        'protected_files_verified': len(protected),
        'equivalence_pairs_reverified_from_raw_events': 100,
        'equivalence_groups': dict(groups),
        'development_runs_verified': len(rows), 'paired_rows_recomputed': len(recomputed),
        'development_six_exact_pairs': six_count,
        'unique_worst_cases_with_action_tables': 10,
        'total_time_summary_rows': len(summaries),
        'new_strategy_runs': 0,
        'scope': 'Saved E12 evidence only; unrelated E13 artifacts untouched.',
    }
    dump(folder / 'review_audit.json', result)
    print(json.dumps(result, ensure_ascii=False))
    print(json.dumps([r for r in summaries if r['dimension'] == 'all'], ensure_ascii=False))


if __name__ == '__main__':
    main()
