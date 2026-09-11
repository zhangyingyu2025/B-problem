"""Deterministic development risk tables; never promotes a candidate."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
from e12_risk_experiment import ROOT, OUT, dump, object_hash, costs_from_actions


def quantile(values, p):
    xs = sorted(values)
    if not xs:
        return None
    h = (len(xs)-1)*p
    lo = math.floor(h); hi = math.ceil(h)
    return xs[lo]+(xs[hi]-xs[lo])*(h-lo)


def tail(values):
    if not values:
        return {k: None for k in ('median', 'P90', 'P95', 'P99', 'max', 'CVaR95', 'tail_count')}
    n = math.ceil(len(values)*.05)
    return {'median': statistics.median(values), 'P90': quantile(values, .90),
            'P95': quantile(values, .95), 'P99': quantile(values, .99), 'max': max(values),
            'CVaR95': statistics.mean(sorted(values, reverse=True)[:n]), 'tail_count': n}


def summarize(rows):
    qualified = all(r['qualified_run'] for r in rows)
    complete = [r for r in rows if r['qualified_run']]
    result = {'runs': len(rows), 'sources': sum(r['N'] for r in rows),
              'all_cleared': all(r['all_cleared'] for r in rows), 'qualified': qualified,
              'failed_runs': len(rows)-len(complete), 'performance_population': 'qualified_runs_only',
              'performance_runs': len(complete),
              'weighted_time_per_source_s': sum(r['metrics']['virtual_time_s'] for r in complete)/sum(r['N'] for r in complete) if complete else None,
              'mean_time_per_source_s': statistics.mean(r['time_per_source_s'] for r in complete) if complete else None,
              **tail([r['time_per_source_s'] for r in complete])}
    for key in ('movement_m', 'measurements', 'switches', 'clear_failures'):
        result['mean_'+key] = statistics.mean(r['metrics'][key] for r in complete) if complete else None
    return result


def pair(base, candidate):
    assert base['seed'] == candidate['seed'] and base['N'] == candidate['N']
    assert base['case_hash'] == candidate['case_hash'] and base['error_mode'] == candidate['error_mode']
    assert base['k0'] == candidate['k0']
    d = {'seed': base['seed'], 'strategy': candidate['strategy'], 'N': base['N'], 'k0': base['k0'],
         'chosen_template': candidate['chosen_template'], 'E11_rotation': base['rotation'],
         'E12_rotation': candidate['rotation'], 'qualified_pair': base['qualified_run'] and candidate['qualified_run'],
         'E11_time_per_source_s': base['time_per_source_s'], 'E12_time_per_source_s': candidate['time_per_source_s'],
         'delta_time_per_source_s': candidate['time_per_source_s']-base['time_per_source_s'],
         'delta_total_virtual_s': candidate['metrics']['virtual_time_s']-base['metrics']['virtual_time_s']}
    for key in ('movement_m', 'measurements', 'switches', 'clear_attempts', 'clear_failures'):
        d['delta_'+key] = candidate['metrics'][key]-base['metrics'][key]
    for key in base['costs']:
        d['E11_'+key] = base['costs'][key]
        d['E12_'+key] = candidate['costs'][key]
        d['delta_'+key] = candidate['costs'][key]-base['costs'][key]
    assert abs(sum(d['delta_'+k] for k in base['costs'])-d['delta_total_virtual_s']) < 2e-4
    return d


def pair_summary(rows):
    valid = [r for r in rows if r['qualified_pair']]
    ds = [r['delta_time_per_source_s'] for r in valid]
    worst = max(valid, key=lambda r: r['delta_time_per_source_s']) if valid else None
    result = {'pairs': len(rows), 'qualified_pairs': len(valid),
              'win_rate': sum(d < 0 for d in ds)/len(ds) if ds else None,
              'tie_rate': sum(d == 0 for d in ds)/len(ds) if ds else None,
              'median_delta_s_per_source': statistics.median(ds) if ds else None,
              'P95_delta_s_per_source': quantile(ds, .95), 'worst_delta_s_per_source': max(ds) if ds else None,
              'worst_seed': worst['seed'] if worst else None}
    for key in ('delta_movement_m', 'delta_measurements', 'delta_switches', 'delta_clear_failures'):
        result['mean_'+key] = statistics.mean(r[key] for r in valid) if valid else None
    return result


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with path.open('w', newline='', encoding='utf-8-sig') as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader(); writer.writerows(rows)


def flatten(r):
    return {**{k: r[k] for k in ('seed', 'strategy', 'N', 'k0', 'chosen_template', 'selector_invoked',
                                 'rotation', 'group', 'error_mode', 'case_hash', 'all_cleared', 'qualified_run',
                                 'cleared_count', 'discovered_count', 'time_per_source_s', 'action_log', 'error')},
            **r['metrics'], **r['costs']}


def generate(folder):
    rows = json.loads((folder/'results.json').read_text(encoding='utf-8'))
    specs = json.loads((folder/'dataset.json').read_text(encoding='utf-8'))
    assert len(rows) == 800 and len(specs) == 200
    expected = {s['case']['seed']: object_hash(s['case']) for s in specs}
    assert set(expected) == set(range(27182800, 27183000))
    lookup = {(r['seed'], r['strategy']): r for r in rows}
    assert len(lookup) == 800
    for seed, expected_hash in expected.items():
        for strategy in ('E11', 'T2', 'T4', 'T6'):
            assert lookup[seed, strategy]['case_hash'] == expected_hash
    summary = {name: summarize([r for r in rows if r['strategy'] == name]) for name in ('E11', 'T2', 'T4', 'T6')}
    pairs = [pair(lookup[seed, 'E11'], lookup[seed, name]) for seed in sorted(expected) for name in ('T2', 'T4', 'T6')]
    paired_summary = {name: pair_summary([p for p in pairs if p['strategy'] == name]) for name in ('T2', 'T4', 'T6')}
    for name in ('T2', 'T4', 'T6'):
        s, base = summary[name], summary['E11']
        paired_summary[name]['qualified_candidate'] = s['qualified']
        paired_summary[name]['P95_below_E11'] = s['P95'] < base['P95'] if s['qualified'] and base['qualified'] else None
        paired_summary[name]['CVaR95_below_E11'] = s['CVaR95'] < base['CVaR95'] if s['qualified'] and base['qualified'] else None
        paired_summary[name]['weighted_below_E11'] = s['weighted_time_per_source_s'] < base['weighted_time_per_source_s'] if s['qualified'] and base['qualified'] else None
    strata = []; stratified_pairs = []
    for field in ('N', 'k0', 'chosen_template', 'rotation'):
        for name in summary:
            group = [r for r in rows if r['strategy'] == name]
            for value in sorted(set(r[field] for r in group)):
                subset = [r for r in group if r[field] == value]
                strata.append({'dimension': field, 'value': value, 'strategy': name, **summarize(subset)})
        for name in ('T2', 'T4', 'T6'):
            field_p = 'E12_rotation' if field == 'rotation' else field
            group = [p for p in pairs if p['strategy'] == name]
            for value in sorted(set(p[field_p] for p in group)):
                subset = [p for p in group if p[field_p] == value]
                matched_baseline = [lookup[p['seed'], 'E11'] for p in subset]
                matched_candidate = [lookup[p['seed'], name] for p in subset]
                stratified_pairs.append({'dimension': field, 'value': value, 'strategy': name,
                    **pair_summary(subset), **{'E11_'+k: v for k, v in summarize(matched_baseline).items()},
                    **{'E12_'+k: v for k, v in summarize(matched_candidate).items()}})
    dump(folder/'summary.json', {'strategies': summary, 'paired': paired_summary, 'promotion': None,
                                'note': 'development only; user review required; no automatic candidate selection'})
    dump(folder/'paired.json', pairs); write_csv(folder/'paired.csv', pairs)
    write_csv(folder/'runs.csv', [flatten(r) for r in rows])
    write_csv(folder/'summary.csv', [{'strategy': k, **v} for k, v in summary.items()])
    write_csv(folder/'paired_summary.csv', [{'strategy': k, **v} for k, v in paired_summary.items()])
    dump(folder/'stratified.json', {'absolute': strata, 'paired': stratified_pairs})
    write_csv(folder/'stratified_absolute.csv', strata); write_csv(folder/'stratified_paired.csv', stratified_pairs)
    write_csv(folder/'N10_11_12_tail.csv', [r for r in strata if r['dimension'] == 'N' and r['value'] in (10, 11, 12)])
    worst_all = sorted(pairs, key=lambda r: (-r['delta_time_per_source_s'], r['seed'], r['strategy']))[:10]
    worst_each = [p for name in ('T2', 'T4', 'T6') for p in sorted([p for p in pairs if p['strategy'] == name],
                  key=lambda r: (-r['delta_time_per_source_s'], r['seed']))[:10]]
    write_csv(folder/'worst10_overall.csv', worst_all); write_csv(folder/'worst10_each_candidate.csv', worst_each)
    diagnostics = []
    for p in worst_each:
        base, cand = lookup[p['seed'], 'E11'], lookup[p['seed'], p['strategy']]
        detail = {'pair': p, 'E11_costs_by_reason': base['costs_by_reason'], 'E12_costs_by_reason': cand['costs_by_reason'],
                  'E11_action_log': base['action_log'], 'E12_action_log': cand['action_log']}
        diagnostics.append(detail)
        # Portable per-action cost table for the worst cases, in exact execution order.
        action_rows = []
        for r in (base, cand):
            events = [json.loads(x) for x in (ROOT/r['action_log']).read_text(encoding='utf-8').splitlines()]
            for i, event in enumerate(events):
                if event.get('kind') != 'action' or event['path'] not in ('/measure', '/clear'):
                    continue
                costs, _ = costs_from_actions([event])
                pos = event['payload']['position']
                action_rows.append({'strategy': r['strategy'], 'action_index': i, 'path': event['path'],
                    'x': pos['x'], 'y': pos['y'], 'channel': event['payload']['channel'],
                    'result': event['response'].get('measure_result', event['response'].get('clear_result')),
                    'reason': event.get('decision_reason'), **costs, 'total_action_s': sum(costs.values())})
        write_csv(folder/'worst_actions'/f"{p['seed']}-{p['strategy']}.csv", action_rows)
    dump(folder/'worst10_cost_breakdown.json', {'overall': worst_all, 'each_candidate': diagnostics})
    report = '# E12-Risk development 200：离线配对结果\n\n'
    report += '固定种子27182800–27182999，200个人工case，E11/T2/T4/T6各完整运行一次。同case、同fixed_field误差场；没有使用fixed12或unseen80调参，也未运行locked validation或模拟器。\n\n'
    report += 'P90/P95/P99采用h=(n−1)p线性插值；CVaR95为最大ceil(0.05n)项均值。分层小样本的CVaR可能等于max，均保留样本数和tail_count。胜率按Δ=(T候选−T基线)/N<0定义，严格相等计为平局。\n\n'
    report += '|策略|场数|源数|全清|weighted 秒/源|median|P90|P95|P99|max|CVaR95|平均移动m|测量|切换|清除失败|\n|---|---:|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n'
    for name, s in summary.items():
        vals = [s[k] for k in ('weighted_time_per_source_s', 'median', 'P90', 'P95', 'P99', 'max', 'CVaR95',
                              'mean_movement_m', 'mean_measurements', 'mean_switches', 'mean_clear_failures')]
        report += f"|{name}|{s['runs']}|{s['sources']}|{s['all_cleared']}|"+'|'.join(f'{v:.3f}' if v is not None else 'NA' for v in vals)+'|\n'
    report += '\n|候选|胜率|平局率|Δ中位数|Δ P95|最坏Δ|最坏seed|P95低于基线|CVaR95低于基线|\n|---|---:|---:|---:|---:|---:|---|---|---|\n'
    for name, s in paired_summary.items():
        report += f"|{name}|{s['win_rate']:.3%}|{s['tie_rate']:.3%}|{s['median_delta_s_per_source']:.3f}|{s['P95_delta_s_per_source']:.3f}|{s['worst_delta_s_per_source']:.3f}|{s['worst_seed']}|{s['P95_below_E11']}|{s['CVaR95_below_E11']}|\n"
    report += '\n## N=10、11、12的尾部\n\n|N|策略|场数|P95|CVaR95|max|tail场数|\n|---|---|---:|---:|---:|---:|---:|\n'
    for r in strata:
        if r['dimension'] == 'N' and r['value'] in (10, 11, 12):
            report += f"|{r['value']}|{r['strategy']}|{r['runs']}|{r['P95']:.3f}|{r['CVaR95']:.3f}|{r['max']:.3f}|{r['tail_count']}|\n"
    report += '\n## 最差10个候选退化组合\n\nΔ为秒/源，后五列是该场总成本差（秒），不是每源差。相同case在不同候选可重复出现；另有每个候选各10个退化case。\n\n|seed|候选|N|k0|模板|Δ|移动差|测量差|切换差|成功清除差|失败清除差|\n|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n'
    for r in worst_all:
        report += f"|{r['seed']}|{r['strategy']}|{r['N']}|{r['k0']}|{r['chosen_template']}|{r['delta_time_per_source_s']:.3f}|"+'|'.join(f"{r['delta_'+k]:.3f}" for k in ('movement_s', 'measurement_s', 'switch_s', 'clear_success_s', 'clear_failure_s'))+'|\n'
    report += '\n## 交付与解释边界\n\nsummary.csv/json为总表；runs/及results.json保留800场原始结果，runs.csv为平表；paired.csv/json为600组配对。stratified_absolute.csv与stratified_paired.csv覆盖N、k0、模板和旋转；配对分层中的E11统计使用候选同一子集，避免把全体六点E11与筛选后的七点候选混比。N10_11_12_tail.csv单列低源数尾部。worst10_each_candidate.csv及worst10_cost_breakdown.json保留每候选10个最差组合，worst_actions/提供逐动作成本明细。\n\n动作成本按L/5+5×measure+switch+5×clear_success+3×clear_failure独立重建；成本归因是动作记账，不证明改变某一局部动作必然改善整体路线。任何失败候选均不合格，events.jsonl即时记载；失败不因低平均时间而被掩盖。未全清场的部分耗时不能当作完成时间，若发生，性能指标仅对qualified_runs计算并明确样本数。\n\n已按预注册范围停止。上述表格只描述本development集，不自动选择候选，不外推官方表现，不针对最坏case继续调参。\n'
    (folder/'report.md').write_text(report, encoding='utf-8')
    return summary


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('folder', type=Path)
    args = p.parse_args(); print(json.dumps(generate(args.folder), ensure_ascii=False))
