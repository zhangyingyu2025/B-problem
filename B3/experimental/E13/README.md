# 当前开发主线：E13-R

仅保留 R 可执行策略。它继承冻结 E11，增加 B3 专用外包几何、历史无信号约束、有界风险清除，以及途中信息更新后的重规划。B1/B2、E11 和协议适配器未改。

最新确认集：seeds 52001500–52001599，100/100 场、1305/1305 源全清。weighted 238.400 秒/源，P95 293.131，CVaR95 303.054，平均移动 11.774 公里。该结果仍未达到220–225秒/源目标，也未通过稳定低于11.5公里的覆盖形变门槛。

## 保留的代码

|文件|用途|
|---|---|
|solver.py|R 唯一策略入口|
|event_route.py|新清除任务及目标第二次示向触发重规划|
|b1plus.py|B3 外包定位区域与 MEC|
|clear_risk.py|确定清除、有界风险和安全补测点|
|coverage_certificate.py|后续覆盖形变所需的严格证书，目前未启用形变|
|experiments/run_batch.py|冻结 E11 与 R 的离线配对运行|
|experiments/report_batch.py|逐场成本、尾部、分层和事后路径差距审计|
|experiments/regret_benchmark.py|事后基准工具，不参与在线决策|

仓库根目录运行新离线配对集，必须指定新的输出目录和事先登记的种子：

```powershell
python -X utf8 -u B3/experimental/E13/experiments/run_batch.py --start NEW_SEED --count COUNT --output B3/结果/NEW_RUN
python -X utf8 B3/experimental/E13/experiments/report_batch.py B3/结果/NEW_RUN
```

当前确认集完整数据保留在 `B3/结果/e13_OR_16/`。其中 O 是当时的配对对照，已不作为可执行选项保留；复现旧 O 需要恢复清理前代码版本。

## 清理和历史追溯

清理前完整版本：`bb530fc229369e001976789c2ae5a22fc8414314`。淘汰方案模块、专属测试、15个早期批次的大量中间数据从工作目录移除。精简结论见 [study_log.md](study_log.md)，数学说明见 [model_notes.md](model_notes.md)。完整历史可用 `git show bb530fc:仓库相对路径` 查看，或在独立目录检出该提交，避免覆盖当前主线。

`B3/结果/e13_OR_16/source_snapshot/` 的旧多方案副本已移到 Git 历史；执行源码对应上述提交，原始动作日志仍保留在确认集目录。

## 必要回归

```powershell
python -X utf8 -m unittest discover -s B3/tests -p 'test_e13_*.py' -v
```

包括几何/风险边界、真实观测下的真值保留和单调收缩、途中重规划，以及清理前100场 R 的严格动作等价。不能因清理删除这些验收依据。E12 的永久六点等价测试也继续保留。
