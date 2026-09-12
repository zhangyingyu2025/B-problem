# 确认集的运行源码

原始运行代码和完整多方案 source_snapshot 保存于 Git 提交：

`bb530fc229369e001976789c2ae5a22fc8414314`

工作目录只保留当前 R 主线，旧 source_snapshot 中重复和淘汰方案已清理。此目录的 case、原始动作 JSONL、逐场 JSON、配对表及分层报告保持不变。清理后的 R 通过 `B3/tests/test_e13_r_equivalence.py` 与这里100场旧动作序列进行严格比较；该测试不用于调参或选择新策略。
