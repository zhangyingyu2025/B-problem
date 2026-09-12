# B3：全向干扰源搜索、定位与清除

**当前主线入口：** [E13-R 运行与验证说明](experimental/E13/README.md)。冻结 E11 继续作为基线；淘汰的 E13 实现已清理，历史可从 Git 提交 `bb530fc` 恢复。下方 A/B/C 内容是早期协议与基线记录，其依赖仍受 E11 冻结清单保护，不作为当前优化方案。

在合作仓库的 `B3/` 直接开发，复用兄弟目录 `B1/`、`B2/`。已经实现覆盖搜索、频道证据、在线B1、B2补测、有限清除兜底、A/B/C三个策略及HTTP协议层。**离线验证不等于官方演练；当前没有启动正式测试。**

## 运行

仅需Python标准库（Python 3.12已验证）。在仓库根目录：

```powershell
python -X utf8 -u B3/code/run_b3.py --mode offline --strategy C --seed 20260911 --output-dir B3/结果/demo-C
python -X utf8 -m unittest discover -s B3/tests -v
python -X utf8 -u B3/code/reproduce.py
python -X utf8 B3/code/plot_results.py
python -X utf8 B3/code/reproduce.py --verify-only
```

完整复现先核验B1/B2依赖，再运行全部回归及固定12场景×3策略离线对照。预计数分钟，预算10分钟；保存完整动作日志、场景、逐场统计与代码哈希。结果目录已有`actions.jsonl`时单次入口拒绝覆盖，应换新目录。完整复现的日志按批次保存，不覆盖旧日志。

## 官方演练接入

需要用户在模拟器中登录、选择**演练测试**并等待接口就绪，然后执行：

```powershell
python -X utf8 -u B3/code/run_b3.py --mode rehearsal --strategy A --robot-id 你的参赛队号 --rehearsal-ready --output-dir B3/结果/rehearsal_logs/run-001
```

默认`http://127.0.0.1:2026`，可用`--url`指定本机端口。HTTP协议没有查询当前演练/正式模式的接口，`--rehearsal-ready`表示操作者已检查界面，程序不能替代该确认。先用A验证官方全流程，再用B/C演练。CLI不提供正式测试模式。

官方接口不提供真实源数，官方结果中`clear_fraction`保留null；需演练结束后用界面给出的总数计算，不以已发现数冒充真实分母。正式日志不改名、不改内容；当前不消耗正式次数。

## 策略与保证

|层次|实现|
|---|---|
|保证发现|原点＋1130米正六边形六点；未知频道记录7站证据；发现16个可提前完成发现|
|在线定位|逐频道累计全部正常角度；near直接清除；no_signal不送B1|
|清除判断|当前位置已覆盖优先，否则MEC半径≤20米到圆心清除；保留数值内缩裕度|
|主动补测|只在收尾不足以清除时调用B2；用实际首次读数，最多一次/源|
|有限兜底|用边长≤28米网格覆盖首次物理矩形或可信B1包围盒交集，最多108次clear/源|

A先完成发现，B发现即追，C顺路补测与低绕路清除。C参数为预先固定启发式，本轮不进行为得到有利结果的事后调参。完整覆盖与兜底证明见[建模方案](建模方案.md)。

`grid_clear_attempts`包含有意的覆盖探查，失败一次耗3秒；它不同于`guaranteed_clear_failures`（MEC/near保证清除意外失败）。两者均计入总虚拟时间，不隐藏失败成本。

## 产物

- [sol原指南](B3_Astra_执行指南.md)及[协议审计与修订](题意与协议审计.md)
- [固定场景集](examples/comparison_cases.json)
- [逐场对照](结果/comparison.json)、[汇总数字](结果/frozen_numbers.json)
- [验证报告](结果/validation_report.md)、[覆盖与对照图](结果/figures/comparison.svg)
- [B1/B2依赖清单](dependency_manifest.json)、[原题版本](source_versions.json)

`frozen_numbers.json`仅为离线候选结果，不代表M8正式版本冻结。官方30–50次稳定演练、正式参数冻结、三次正式测试仍未完成。代码不会因发现10个源或遇到预算中止而输出全部完成。

实际运行源码由逐文件SHA-256标识；记录的Git HEAD是基线提交，若工作树尚未提交，不能仅用HEAD复现新代码。结果及源码应一起提交。原始官方题包不需要上传GitHub。
