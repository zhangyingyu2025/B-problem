E11 模拟器对接增量包
====================

前提：已经把“E11成果代码_解压到仓库根目录.zip”解压到仓库根目录，并能运行：
  python -X utf8 -u B3/code/run_e11_experimental.py --fixed12

本包只新增：
  B3/code/run_e11_rehearsal.py
不会修改原 A/B/C、run_b3.py 或 E11 实验代码。

建议先做一次离线适配层自检：
  python -X utf8 -u B3/code/run_e11_rehearsal.py `
    --mode offline `
    --seed 20260911 `
    --output-dir B3/结果/e11_adapter_check

参考结果：
  16/16 cleared
  virtual_time_s = 3472.998505
  mean_time_per_clear_s = 217.0624065625
  rotation_deg = 50

连接官方模拟器演练模式：
1. 在模拟器中明确选择“演练 / rehearsal”并启动接口。
2. PowerShell 检查端口：
   Test-NetConnection 127.0.0.1 -Port 2026
   应看到 TcpTestSucceeded : True
3. 在仓库根目录运行：
   python -X utf8 -u B3/code/run_e11_rehearsal.py `
     --mode rehearsal `
     --robot-id 202623001400 `
     --rehearsal-ready `
     --output-dir B3/结果/rehearsal_logs/run-011-E11

注意：
- 脚本没有 formal 模式；绝对不要在正式测试界面运行。
- 每次使用新的 output-dir，脚本拒绝覆盖 actions.jsonl/result.json。
- HTTP 无法识别模拟器当前到底是演练还是正式，--rehearsal-ready 是人工确认闸门。
- 跑完后同时记录模拟器页面显示的真实源数。
