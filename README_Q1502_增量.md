# E13-Q1502 增量包

这是建立在**当前最新 E13-R 仓库**上的实验增量，不覆盖 B1/B2/E11，也不替换已确认的 `E13-R`。

## 新增策略

`Q1502` = `R + C3 + rotation diagnostic`：

1. 原点执行现有初始扫描；
2. 从原点已经发现的 bearing 中，选择对最多两个源具有高视差的短基线方向；
3. 机器人移动到距原点 **150 m** 的诊断点；
4. 在该点重测最多 **2 个**已发现频道；
5. 用更新后的 B3-B1+ 几何任务重新比较六边形 rotation；
6. 后续使用 C3：每个源最多保留 3 条 opportunistic transit bearings，并继续保留 R 的 owner-second-bearing 取消过时 supplement 逻辑。

Q1502 不读取 synthetic true source 坐标/半径/真实源数量作为在线决策输入。

## 当前仅属离线候选

开发批次 seeds `53001400..53001419`，random + fixed_field：

- R：237.263 s/source，11.717 km/run，20/20 全清
- C3：233.973 s/source，11.745 km/run，20/20 全清
- Q1502：**227.345 s/source，11.311 km/run，20/20 全清**
- Q1502 对 R：15/20 胜，平均约 -9.58 s/source

这 20 场是探索结果，不应当成最终确认集。正式/官方 rehearsal 之前仍应扩大 held-out 和 stress 验证。

## 新增文件

```text
B3/experimental/E13/solver_diag.py
B3/experimental/E13/solver_cap.py
B3/experimental/E13/event_route_cap.py
B3/experimental/E13/experiments_next/run_compare.py
B3/tests/test_e13_q1502_smoke.py
```

其中 `Q1502` 依赖当前仓库已有的：

```text
B3/experimental/E13/solver.py          # E13-R
B3/experimental/E13/b1plus.py
B3/experimental/E13/clear_risk.py
B3/experimental/E13/event_route.py
B3/code/e12_coverage_hook.py
B3/code/run_e11_rehearsal.py
```

## 安装

将 ZIP **解压到仓库根目录**，保持目录结构合并。不要删除或覆盖 `solver.py` 的 R 基线。

## 先跑 smoke test

仓库根目录：

```powershell
python -X utf8 -m unittest B3.tests.test_e13_q1502_smoke -v
```

## 离线配对测试

例如再跑一批全新随机场：

```powershell
python -X utf8 -u B3/experimental/E13/experiments_next/run_compare.py `
  --start 53002000 `
  --count 20 `
  --variants Q1502 `
  --workers 4 `
  --output B3/结果/q1502_check20.json
```

输出会同场比较 `R` 和 `Q1502`。

## Git 提交建议

只提交本增量代码与测试即可；批量实验 JSON/动作日志不是必须提交。建议提交信息：

```text
Add E13 Q1502 diagnostic rotation experiment
```

当前 Q1502 **不要替换 R 为正式策略**，它首先是下一轮 220 目标研究候选。
