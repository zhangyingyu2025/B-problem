"""Build the retained-trial ledger from completed batches, without new runs."""
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT/'B3/code'))
from e12_risk_experiment import dump, verify_protected


if __name__ == '__main__':
    trials = []
    for folder in (ROOT/'B3/结果').glob('e13_*'):
        if not (folder/'summary.json').exists() or not (folder/'contract.json').exists():
            continue
        summary = json.loads((folder/'summary.json').read_text(encoding='utf-8'))
        contract = json.loads((folder/'contract.json').read_text(encoding='utf-8'))
        trials.append((contract['start'], folder.name, contract, summary))
    trials.sort()
    text = '# E13 持续实验台账\n\n所有场景为人工数据，不能视为官方演练结果。每行仅与同一批 E11 配对，不跨批挑选较低绝对分数。历史失败/无收益方向全部保留。\n\n'
    text += '|批次|种子起点|策略|场数|全清|weighted秒/源|P95|CVaR95|max|平均移动m|\n|---|---:|---|---:|---|---:|---:|---:|---:|---:|\n'
    for start, name, contract, summary in trials:
        for strategy in ['E11']+contract['variants']:
            r = summary[strategy]
            text += f"|{name}|{start}|{strategy}|{r['runs']}|{r['all_cleared']}|" + '|'.join(f'{r[k]:.3f}' for k in
                ('weighted_time_per_source_s', 'P95', 'CVaR95', 'max', 'mean_movement_m')) + '|\n'
    text += '\nB/C/D/E/F/G/H/K/L 未优于其直接对照，不推广。M 的100场确认集已全清，相对J有额外收益，但尚未达到220–225秒/源或11.5公里形变门槛。N是扩大信息触发范围的另一个消融，按其配对结果判断。\n\n'
    text += '每批 source_snapshot（第05轮起）和 contract.json 固定运行源码与种子。results.json 和 *-actions.jsonl 保留逐场结果与动作。paired_audit.json、stratified_paired.csv、regret_benchmark.json 给出成本审计、分层和事后基准；未生成这些文件的最新批次仍待报告审计。\n\n'
    text += '重现命令见各 round*.md 与 run_batch.py --help。原始 E11、B1、B2 和协议适配器受哈希核验保护。所有试验均未启用覆盖形变。当前无新增官方或正式测试结果。\n'
    (HERE/'study_log.md').write_text(text, encoding='utf-8')
    taskbook = Path('E:/Download/Codex_B3_直推220_完整研究结论与执行任务.md')
    dump(HERE/'project_state.json', {
        'goal': 'stable all-clear approximately 220-225 seconds/source', 'goal_complete': False,
        'stage': 'B1+ and event-aware online routing; paired synthetic experiments',
        'taskbook': str(taskbook), 'taskbook_sha256': hashlib.sha256(taskbook.read_bytes()).hexdigest(),
        'protected_files': verify_protected(),
        'completed_batches': [name for _, name, _, _ in trials],
        'best_confirmed_candidate': 'M; 100-case synthetic confirmation, not a final promotion',
        'coverage_deformation_enabled': False, 'new_official_runs': 0, 'new_formal_runs': 0,
        'remaining': ['improve online movement toward <=11.5km', 'failure-oriented stress tests',
                      'certified coverage deformation after gate', '220-225s/source with stable tails',
                      'fresh final confirmation and 20-30 official rehearsals'],
    })
    print('indexed', len(trials), 'completed batches')
