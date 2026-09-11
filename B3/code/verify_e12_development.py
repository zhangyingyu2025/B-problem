"""Independent post-run evidence checks. Does not execute any strategy."""
import hashlib
import json
import math
from pathlib import Path
from e12_risk_experiment import ROOT, OUT, dump, verify_protected


def verify():
    protected = verify_protected()
    execution = json.loads((OUT/'execution_sources.json').read_text(encoding='utf-8'))
    for name, expected in execution['files'].items():
        assert hashlib.sha256((ROOT/name).read_bytes().replace(b'\r\n', b'\n')).hexdigest() == expected, name
    folder = OUT/'development200'
    rows = json.loads((folder/'results.json').read_text(encoding='utf-8'))
    specs = json.loads((folder/'dataset.json').read_text(encoding='utf-8'))
    cases = {s['case']['seed']: s['case'] for s in specs}
    assert len(rows) == 800 and set(cases) == set(range(27182800, 27183000))
    seen = set(); largest_error = 0.; failures = []
    for row in rows:
        key = (row['seed'], row['strategy'])
        assert key not in seen; seen.add(key)
        raw = ROOT/row['action_log']
        assert hashlib.sha256(raw.read_bytes()).hexdigest() == row['action_log_sha256']
        events = [json.loads(x) for x in raw.read_text(encoding='utf-8').splitlines()]
        actions = [e for e in events if e.get('kind') == 'action']
        ids = [e['payload']['request_id'] for e in actions]
        assert len(ids) == len(set(ids)), key
        # Full decision log is preserved even for any disqualified trial.
        if not row['qualified_run']:
            failures.append(key)
            continue
        assert actions[0]['path'] == '/enter' and actions[-1]['path'] == '/exit'
        source_channels = {s['channel'] for s in cases[row['seed']]['sources']}
        found, cleared = set(), set()
        measure_count = switch_count = clear_count = clear_fail = 0
        distance = 0.
        for action in actions:
            assert action['response']['accepted'] is True
            if action['path'] not in ('/measure', '/clear'):
                continue
            p = action['payload']['position']; before = action['before']
            distance += math.hypot(p['x']-before['position'][0], p['y']-before['position'][1])
            ch = action['payload']['channel']
            if action['path'] == '/measure':
                measure_count += 1; switch_count += ch != before['channel']
                if action['response']['measure_result'] in ('near', 'direction'):
                    found.add(ch)
            else:
                clear_count += 1
                if action['response']['clear_result'] == 'success':
                    assert ch not in cleared
                    cleared.add(ch)
                else:
                    clear_fail += 1
        assert cleared == found == source_channels, key
        m = row['metrics']
        assert measure_count == m['measurements'] and switch_count == m['switches']
        assert clear_count == m['clear_attempts'] and clear_fail == m['clear_failures']
        assert math.isclose(distance, m['movement_m'], abs_tol=1e-7, rel_tol=0)
        total = distance/5+measure_count*5+switch_count+(clear_count-clear_fail)*5+clear_fail*3
        error = abs(total-m['virtual_time_s']); largest_error = max(largest_error, error)
        assert error < 1e-4
        if row['strategy'] != 'E11':
            threshold = int(row['strategy'][1:])
            if row['selector_invoked']:
                assert row['chosen_template'] == (7 if row['k0'] >= threshold else 6)
            if row['chosen_template'] == 6:
                assert row['six_action_equivalent'] is True
    assert seen == {(seed, name) for seed in cases for name in ('E11', 'T2', 'T4', 'T6')}
    paired = json.loads((folder/'paired.json').read_text(encoding='utf-8'))
    assert len(paired) == 600
    evidence = {'runs_verified': len(rows), 'paired_rows': len(paired),
                'protected_files': len(protected), 'raw_action_hashes_verified': True,
                'max_accounting_difference_s': largest_error, 'disqualified_runs': failures,
                'new_official_runs': 0, 'new_formal_runs': 0, 'locked_validation_runs': 0}
    dump(folder/'verification.json', evidence)
    print(json.dumps(evidence, ensure_ascii=False))


if __name__ == '__main__':
    verify()
