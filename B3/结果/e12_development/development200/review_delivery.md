# E12 development 200 复核交付

本次接续时，仓库已有完整的 100 场等价测试和 200×4 场配对运行产物。此次从原始日志重新核验，未重复运行策略、未调阈值；另行补齐整场总时间差的统计。当前工作树另有 E13 文件，本次未修改或运行。

## 验证结果

- 冻结 E11 与 Phase A 等 41 个受保护文件哈希一致。
- 100 对强制六点的完整规范化事件和最终 metrics 严格相等；仅排除请求身份和墙钟字段，位置不使用近似容差。
- random/fixed_field、boundary/endpoints、cluster/fixed_field、origin/fixed_field、random/endpoints 各 20 场，seeds 42424200–42424299。永久测试位于 `B3/tests/test_e12_equivalence100.py`。
- development 使用 seeds 27182800–27182999；800 次运行各自输入与保存的 synthetic case、fixed_field error mode 一致。600 条 paired 记录从逐场结果重算一致，另有 95 对六点回退动作严格相等。
- 四个策略各 200/200 场、2598/2598 源全清；无不合格运行。动作计数及移动距离重算通过，成本和虚拟时间最大绝对差 4.9783e-7 秒。
- 3 项统计回归测试通过。最差 10 个不重复 case 均有逐动作成本表。

## 结果与解释

[完整主表、低源数尾部和成本表](report.md)；[候选复核意见](findings.md)。

总体 P95/CVaR95 均下降的只有 T2，但其 N=11 的 P95、CVaR95、max 均恶化，N=12 的 P95 也恶化。T4 的 weighted mean 最低，但总体 P95 恶化。T6 总体 P95、CVaR95 未改善。因此这里仅呈现预注册候选比较，不晋级、不修改参数。

原表的时间差为每源时间差。补充的整场时间差定义为 ΔT=T_E12−T_E11，单位秒：

|候选|胜率|ΔT median|ΔT P95|最大 ΔT|对应 seed|
|---|---:|---:|---:|---:|---|
|T2|52.0%|-10.171|365.863|558.790|27182920|
|T4|48.5%|0.000|363.096|558.790|27182920|
|T6|31.0%|0.000|290.802|476.655|27182949|

ΔT 的 P95 与两组各自 P95 之差是不同统计量；不可相互替代。CVaR95 为最差 ceil(5%×n) 场的平均每源时间，分层样本数与尾部样本数均随表保留。

## 文件索引

|内容|文件|
|---|---|
|逐场原始结果 800 条|[results.json](results.json)、[runs.csv](runs.csv)、runs/|
|配对 600 条|[paired.csv](paired.csv)、[paired.json](paired.json)|
|总表与配对总表|[summary.csv](summary.csv)、[summary.json](summary.json)、[paired_summary.csv](paired_summary.csv)|
|N、k0、模板及旋转分层|[stratified_paired.csv](stratified_paired.csv)、[stratified.json](stratified.json)、[stratified_absolute.csv](stratified_absolute.csv)|
|N=10、11、12 尾部|[N10_11_12_tail.csv](N10_11_12_tail.csv)、[N10_11_12_tail_deltas.csv](N10_11_12_tail_deltas.csv)|
|整场总时间差与分层补充|[paired_total_time_summary.csv](paired_total_time_summary.csv)、[paired_total_time_summary.json](paired_total_time_summary.json)|
|最差 10 个不同 case|[worst10_unique_cases.csv](worst10_unique_cases.csv)、[worst10_unique_cases.json](worst10_unique_cases.json)|
|每候选最差 10 场成本分解|[worst10_cost_breakdown.json](worst10_cost_breakdown.json)、[worst10_each_candidate.csv](worst10_each_candidate.csv)、worst_actions/|
|本次复核记录|[review_audit.json](review_audit.json)、[verification.json](verification.json)|

原始动作 JSONL 路径及 SHA-256 保存在每条结果中，日志在本机 `B3/结果/offline_logs/` 下，受已有 .gitignore 排除；转交他人离线审计时需同时提供这些日志。

仓库根目录复核命令：

```powershell
python -X utf8 B3/code/verify_e12_development.py
python -X utf8 B3/code/audit_e12_review.py
python -X utf8 -m unittest discover -s B3/tests -p test_e12_risk_statistics.py -v
```

本次仅核验 E12 数据与补充报表，没有接官方模拟器，没有运行 locked validation，后续选择等待用户复核。
