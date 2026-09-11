"""Descriptive review tables only; never selects or runs a strategy."""
import json
from e12_risk_experiment import OUT, dump
from e12_risk_report import write_csv


def main():
    folder = OUT/'development200'
    pairs = json.loads((folder/'paired.json').read_text(encoding='utf-8'))
    summary = json.loads((folder/'summary.json').read_text(encoding='utf-8'))
    stratified = json.loads((folder/'stratified.json').read_text(encoding='utf-8'))['paired']
    ordered = sorted(pairs, key=lambda r: (-r['delta_time_per_source_s'], r['seed'], r['strategy']))
    seen = set(); worst = []
    for row in ordered:
        if row['seed'] in seen:
            continue
        seen.add(row['seed'])
        same = [r['strategy'] for r in pairs if r['seed'] == row['seed'] and
                r['delta_time_per_source_s'] == row['delta_time_per_source_s']]
        worst.append({**row, 'same_worst_delta_candidates': ','.join(same)})
        if len(worst) == 10:
            break
    write_csv(folder/'worst10_unique_cases.csv', worst)
    dump(folder/'worst10_unique_cases.json', worst)
    deltas = []
    for r in stratified:
        if r['dimension'] == 'N' and r['value'] in (10, 11, 12):
            deltas.append({'strategy': r['strategy'], 'N': r['value'], 'runs': r['pairs'],
                           **{'delta_'+k: r['E12_'+k]-r['E11_'+k] for k in ('P95', 'CVaR95', 'max')}})
    write_csv(folder/'N10_11_12_tail_deltas.csv', deltas)
    base = summary['strategies']['E11']
    text = '# 供复核的结论：不自动晋级\n\n'
    text += '所有候选均通过200场全清门槛。在本development集，T2是唯一同时降低总体P95与CVaR95的候选；T4虽有最低weighted mean但P95恶化，T6的两项尾部指标未改善。该结果不等同于在未知场景或官方模拟器中的优势。\n\n'
    text += '|候选|Δ weighted秒/源|Δ P95|Δ CVaR95|Δ max|\n|---|---:|---:|---:|---:|\n'
    for name in ('T2', 'T4', 'T6'):
        r = summary['strategies'][name]
        text += '|'+name+'|'+'|'.join(f'{r[k]-base[k]:.6f}' for k in
                                    ('weighted_time_per_source_s', 'P95', 'CVaR95', 'max'))+'|\n'
    text += '\nT2的总体P95改善仅约1.07秒/源。分层并非全面改善：N=11的P95、CVaR95、max全部恶化；N=12的P95也恶化。N=10的明显改善不能掩盖这些反向结果。下表全部使用候选与E11同一N子集。\n\n'
    text += '|候选|N|场数|Δ P95|Δ CVaR95|Δ max|\n|---|---:|---:|---:|---:|---:|\n'
    for r in deltas:
        text += f"|{r['strategy']}|{r['N']}|{r['runs']}|{r['delta_P95']:.3f}|{r['delta_CVaR95']:.3f}|{r['delta_max']:.3f}|\n"
    text += '\n最差退化按Δ而非绝对耗时排序，整体10个不重复seed保存在worst10_unique_cases.csv/json；另保留每候选10场和所有600组配对。seed 27182920的T2/T4退化46.566秒/源：全场增加约558.790秒，其中移动增加524.790秒、测量25秒、切换9秒；成功和失败清除成本差均0。它说明存在明显的路线退化，不构成针对该seed修改参数的授权。\n\n'
    text += '没有自动选择候选、没有扩大搜索空间、没有读取或运行locked validation、没有接官方模拟器。等待用户决定下一步。\n'
    (folder/'findings.md').write_text(text, encoding='utf-8')


if __name__ == '__main__':
    main()
