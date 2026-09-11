# 本阶段运行说明

用户已授权 development 200，候选只限 E11/T2/T4/T6；不连接模拟器，不运行 locked validation，不自动选择候选。

在仓库根目录执行：

```powershell
python -X utf8 -m unittest discover -s B3/tests -p test_e12_risk_statistics.py -v
python -X utf8 -u -m unittest discover -s B3/tests -p test_e12_equivalence100.py -v
python -X utf8 -u B3/code/e12_risk_experiment.py development --output B3/结果/e12_development/development200 --workers 4
python -X utf8 B3/code/e12_risk_report.py B3/结果/e12_development/development200
python -X utf8 B3/code/verify_e12_development.py
python -X utf8 B3/code/e12_risk_findings.py
```

100场回归是永久unittest，默认会实际执行200次策略运行，而不是只验证旧摘要。每次自动创建新的等价测试目录，包含完整case、结果、原始日志链接与SHA-256；成功后写equivalence_gate.json。development入口要求该门槛通过且源码哈希相同。

development已有输出时拒绝覆盖。需要复现时请传新的输出目录；不要覆盖本轮报告。报告命令接受新的目录；本轮独立核验命令固定核验development200。

原始actions.jsonl位于B3/结果/offline_logs/e12-risk-*，遵循现有.gitignore，仍留在本机。逐场结果JSON、配对CSV/JSON、成本分解和数据集保存在本目录，可提交Git。转交复现审计时还需传送原始日志目录。

合格门槛独立于性能：任何未全清、证据不完整、计时不相符或六点分支动作不等价都判不合格。失败后剩余预注册场景仍执行，仅为诊断，不恢复该候选资格。对被判不合格的候选不得从合格子集的较低平均时间作推广结论。

P95和CVaR95均基于逐场T/N；weighted mean为总虚拟时间除以总源数。CVaR95采用最差ceil(5%×样本数)场均值。低样本分层请结合runs和tail_count阅读。按chosen_template分层的配对表同时重算同一子集的E11，以免引入样本选择差异。

本轮已完成：100场严格等价回归通过；development 200×4=800运行全部全清；另有95个development六点回退配对严格等价。独立审计最大计时差小于5e-7秒。主要结论见development200/findings.md，完整总表见development200/report.md。未自动晋级。
