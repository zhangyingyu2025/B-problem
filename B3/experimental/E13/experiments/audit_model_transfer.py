"""Read-only diagnosis of saved R observations; never executes an online policy.

Run from the repository root. Counts constraint activity, not counterfactual
speed improvements. Does not read synthetic source positions or radii.
"""
from collections import Counter, defaultdict
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import subprocess

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE))
from b1plus import intersection


def distance_to_polygon(q, poly):
    if all((b[0]-a[0])*(q[1]-a[1])-(b[1]-a[1])*(q[0]-a[0]) >= -1e-9
           for a, b in zip(poly, poly[1:]+poly[:1])):
        return 0.0
    best = math.inf
    for a, b in zip(poly, poly[1:]+poly[:1]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        den = dx*dx+dy*dy
        t = max(0., min(1., ((q[0]-a[0])*dx+(q[1]-a[1])*dy)/den)) if den else 0.
        best = min(best, math.dist(q, (a[0]+t*dx, a[1]+t*dy)))
    return best


def audit_one(path):
    dirs, signals, silent = defaultdict(list), defaultdict(list), defaultdict(list)
    geometry, cleared = {}, set()
    stats = Counter()
    movements = Counter()
    first_clear = set()
    opportunities = []
    for line in path.read_text(encoding='utf-8').splitlines():
        event = json.loads(line)
        if event.get('kind') != 'action' or event.get('path') not in ('/measure', '/clear'):
            continue
        if not event['response'].get('accepted'):
            continue
        c = event['payload']['channel']
        p = event['payload']['position']; p = (p['x'], p['y'])
        result = event['response']
        reason = event.get('decision_reason', '')
        movements[reason] += math.dist(event['before']['position'], p)
        if event['path'] == '/clear':
            if c not in first_clear:
                first_clear.add(c)
                g = geometry.get(c)
                if g and g['status'] == 'bounded':
                    r = g['mec'][1]
                    stats['first_clear_R_le20' if r <= 20 else 'first_clear_R_20_80' if r <= 80 else 'first_clear_R_gt80'] += 1
                    if r <= 20:
                        # MEC center belongs to K. This triangle bound applies
                        # even when a near reply justified an old clear outside
                        # G20 of the (deliberately loose) reconstructed polygon.
                        shift = 20 + math.dist(p, g['mec'][0])
                        stats['fixed_order_clear_move_saving_upper_m'] += 2*shift
            if result['clear_result'] == 'success':
                cleared.add(c)
            continue
        if c in cleared:
            continue
        res = result['measure_result']
        if res == 'no_signal':
            if p not in silent[c]:
                silent[c].append(p)
            if not dirs[c]:
                continue
        elif res in ('direction', 'near'):
            if p not in signals[c]:
                signals[c].append(p)
            if res == 'direction':
                o = {'x': p[0], 'y': p[1], 'svd_deg': result['svd_deg']}
                if o not in dirs[c]:
                    dirs[c].append(o)
        if not dirs[c]:
            continue
        g = intersection(dirs[c], signals[c], silent[c])
        geometry[c] = g
        stats['updates'] += 1
        if g['status'] != 'bounded':
            stats['uncertain'] += 1
            continue
        poly = g['polygon']
        lower_active = any(distance_to_polygon(q, poly) < 1000-1e-6 for q in silent[c])
        near_active = any(distance_to_polygon((o['x'], o['y']), poly) < 5-1e-6 for o in dirs[c])
        upper_active = max(math.hypot(*v) for v in poly) > 1800+1e-6 or any(
            max(math.dist(q, v) for v in poly) > 1500+1e-6 for q in signals[c])
        # If every hull vertex survives every missing lower-distance constraint,
        # removing interiors cannot change this hull, its diameter, or its MEC.
        lower_hull_active = any(any(math.dist(q, v) < 1000-1e-6 for v in poly) for q in silent[c])
        near_hull_active = any(any(math.dist((o['x'], o['y']), v) < 5-1e-6 for v in poly) for o in dirs[c])
        for name, value in [('silence_disk_active', lower_active), ('silence_hull_active', lower_hull_active),
                            ('near_disk_active', near_active), ('near_hull_active', near_hull_active),
                            ('upper_disk_outer_slack_active', upper_active)]:
            stats[name] += int(value)
        if near_hull_active:
            stats['near_hull_one_bearing' if len(dirs[c]) == 1 else 'near_hull_multiple_bearings'] += 1
        if lower_hull_active:
            opportunities.append({'channel': c, 'request_id': event['payload']['request_id'],
                                  'directions': len(dirs[c]), 'R_m': g['mec'][1],
                                  'reason': reason})
    return dict(stats), dict(movements), opportunities


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='B3/结果/e13_OR_16')
    parser.add_argument('--output', required=True)
    parser.add_argument('--group-review', help='Extracted group B1 directory, optional')
    parser.add_argument('--archive', help='Original group B1 archive, optional')
    args = parser.parse_args()
    folder, output = ROOT/args.input, ROOT/args.output
    rows = [json.loads(p.read_text(encoding='utf-8')) for p in sorted(folder.glob('*-R-result.json'))]
    assert len(rows) == 100
    summary = json.loads((folder/'summary.json').read_text(encoding='utf-8'))['R']
    total = Counter(); moves = Counter(); per_case = []; costs = Counter(); inputs = {}
    hashes = hashlib.sha256()
    for index, row in enumerate(rows):
        path = folder/f"{row['seed']}-R-actions.jsonl"
        for file in (path, folder/f"{row['seed']}-R-result.json"):
            digest = hashlib.sha256(file.read_bytes()).hexdigest()
            hashes.update((file.name+'\0'+digest+'\n').encode('utf-8'))
        counts, movement, opportunities = audit_one(path)
        assert counts['updates'] == row['b1plus_updates']
        total.update(counts); moves.update(movement); costs.update(row['costs'])
        per_case.append({'seed': row['seed'], 'N': row['N'], 'counts': counts,
                         'silence_hull_opportunities': opportunities})
        if (index+1) % 20 == 0:
            print(f'read-only geometry audit {index+1}/100', flush=True)
    for file in [Path(__file__), HERE/'b1plus.py', HERE/'solver.py', folder/'summary.json']:
        inputs[str(file.relative_to(ROOT))] = hashlib.sha256(file.read_bytes()).hexdigest()
    n = summary['sources']; runs = summary['runs']; mean_n = n/runs
    gaps = {}
    for target in (225, 220):
        gap = summary['weighted_time_per_source_s']-target
        gaps[target] = {'gap_s_per_source': gap, 'saved_s_per_case': gap*mean_n,
                        'movement_saving_m_per_case_if_other_costs_fixed': gap*mean_n*5,
                        'movement_target_m_if_other_costs_fixed': summary['mean_movement_m']-gap*mean_n*5}
    report = {'role': 'observation-only replay of existing synthetic R logs; no new policy trial',
              'baseline': summary, 'source_hashes': inputs,
              'ordered_200_log_and_result_hash_digest_sha256': hashes.hexdigest(),
              'counts': dict(total), 'gaps': gaps,
              'cost_s_per_source': {k: v/n for k, v in costs.items()},
              'movement_m_per_case_by_destination_reason': {k: v/runs for k, v in moves.items()},
              'geometry_notes': ['activity counts do not measure policy speed gains',
                                 'numeric activity thresholds 1e-6 m; no certified new geometry solver',
                                 'movement attributed to endpoint reason, not causal avoidable detour'],
              'analytic_checks': {
                  'outer_64_1800_slack_m': 1800*(1/math.cos(math.pi/64)-1),
                  'outer_64_1500_slack_m': 1500*(1/math.cos(math.pi/64)-1),
                  'inner_2880_1500_sagitta_m': 1500*(1-math.cos(math.pi/2880)),
                  'fixed_order_guaranteed_first_clear_move_saving_upper_s_per_source':
                      total['fixed_order_clear_move_saving_upper_m']/5/n,
                  'no_signal_counterexample': {'signal': [600,0], 'silent': [0,0],
                                              'candidate': [900,0], 'bearing_deg': 0,
                                              'pair_lhs': -1080000, 'pair_rhs': -360000,
                                              'candidate_satisfies_R_halfplanes': True,
                                              'candidate_violates_silence_distance_1000': True}},
              'cases': per_case}
    if args.group_review:
        review = Path(args.group_review).resolve()
        files = ['code/q1_model.py', 'code/q1_physical.py', 'tests/test_q1.py',
                 'README.md', '问题一_建模报告.txt', '第一问论文正文初稿.md',
                 '结果/frozen_numbers.json', '结果/arc_discretization_convergence.csv']
        before = {f: hashlib.sha256((review/f).read_bytes()).hexdigest() for f in files}
        tested = subprocess.run([sys.executable, '-B', '-X', 'utf8', '-m', 'unittest',
                                 'discover', '-s', str(review/'tests'), '-p', 'test_q1.py', '-v'],
                                capture_output=True, encoding='utf-8', check=False)
        assert tested.returncode == 0, tested.stderr
        assert before == {f: hashlib.sha256((review/f).read_bytes()).hexdigest() for f in files}
        report['group_B1_review'] = {'files_sha256': before, 'test_exit_code': tested.returncode,
                                     'test_output': tested.stdout+tested.stderr}
    if args.archive:
        archive = Path(args.archive)
        report['archive'] = {'name': archive.name, 'sha256': hashlib.sha256(archive.read_bytes()).hexdigest()}
    sys.path.insert(0, str(ROOT/'B3/code'))
    from e12_risk_experiment import verify_protected
    report['protected_files_verified'] = verify_protected()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('cases','source_hashes')}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
