# B3 当前交付：E13-R

**论文组员先读：[论文交接.md](论文交接.md)**。含三局正式结果、可使用的结论、统计口径及跨电脑重新生成表图的命令。

正式入口：`code/run_e13_formal.py`；演练入口：`code/run_e13_rehearsal.py`。正式完整操作脚本为仓库根目录的`tools/test_evidence/run_b3_formal.ps1`，后处理说明见`tools/test_evidence/README.md`。

## 保留结果

- `结果/e13_formal/`：2026-09-13正式测试原始动作、统计、案例编号、总表、轨迹及模拟器日志。
- `结果/e13_rehearsal/`：2026-09-13演练结果和独立总表。
- `结果/e13_formal.zip`：用户已有的正式结果压缩包；压缩包内容与目录后续变化不自动同步。

运行策略位于`experimental/E13/solver.py`，数学说明见`experimental/E13/model_notes.md`。E11目录及code中的部分历史命名模块仍是E13-R/B4实际依赖，不能仅凭名称删除。

## 2026-09-13清理

后续按引用核查删除7个旧独立工具：audit_e11_baseline、audit_e12_review、e12_risk_findings、verify_e12_development、run_e11_experimental、reproduce、plot_results。当前正式/演练入口不引用这些模块。旧版求解器和统计工具仍有保留测试引用，本次没有连带删除。现行表格与绘图工具位于`tools/test_evidence/`。

按用户要求删除旧离线结果、旧实验报告、淘汰的候选方案及缓存。正式和演练目录逐文件SHA256核验未变。旧100场R回归原始数据已删除，对应测试缺数据时明确跳过，不代表重新通过；B4的旧全量历史审计也需要恢复该数据才能执行。保留当前几何、协议与行为测试。未修改在线求解策略或连接模拟器。

删除清单与保留结果哈希位于仓库`.cache/b3-cleanup-20260913.json`；它只保存清单，不备份已删除的旧原始数据。已提交源码可从Git历史追溯，未跟踪的已删除结果不能保证恢复。
