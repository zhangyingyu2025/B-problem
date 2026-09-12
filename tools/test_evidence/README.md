# 从客户端日志生成测试表与轨迹

依据原题B题.pdf第1–4页、附件1第4.6节、附件2响应字段。题面要求正式表格和原名加密日志；未强制要求轨迹图。附件明确要求机器狗程序自行记录指令与响应，因此不必解密模拟器日志。

独立后处理工具，仅读文件，不连接模拟器，不改变B1/B2/B3/B4策略。Python标准库即可运行。

```powershell
python -B -X utf8 tools/test_evidence/export.py --manifest tools/test_evidence/example-rehearsal.json --output B4/results/evidence-demo
```

输出table.csv/table.md、逐局JSON、全部确认动作CSV、等比例SVG路径。SVG可以用浏览器查看或作为矢量图插入论文。图保留动作顺序及重访；同点多频道标记允许重合，不人为挪动。清除点是机器人坐标，不能称为源的精确真值。绘图范围包括全部坐标，虚线圆半径1800m。

正式测试后复制manifest，runs写入三局；`mode`改为`formal`，`problem`填B3或B4，`actions`指向对应日志，`case_code`按模拟器显示原样填写。所有相对路径相对manifest目录。`encrypted_log`可填原始加密文件路径，工具只记录原文件名和SHA256，不改名、不解密。输出目录必须是新目录。

## 四列口径

1. 测试案例编码：模拟器界面/日志列表显示的编码，不是随机种子、request_id或队号。HTTP字段未提供它，不能自动猜测。
2. 清除个数：唯一request_id对应success的频道数；重试不重复计数。
3. 平均定位清除时间：正常exit的virtual_time_s除以清除个数，包括全部移动、检测、切换及清除失败成本；不是各源首次发现时间平均值。
4. 程序运行时间：原题定义enter至测试结束的现实时间。默认保存enter/exit两响应real_timestamp_ms之差作为接口重建值，它接近但不保证精确等于内部计时起止；另保存客户端wall_elapsed_s供核验。如果官方界面/可用记录提供明确运行时间，可填`official_runtime_s`，同时必须填写`runtime_evidence`来源描述。不要直接采用旧result.json中的wall_time_s而不核对，其计时起点可能在enter后初始化之前/之后。

工具对缺失exit、未解决的传输失败、成本不一致标记不完整，不把部分时间当完整测试时间。正式测试不公开总源数，`true_source_count`、清除比例始终为空；全清的算法证书与官方真值核验须区别表述。

## 每局留存

- 独立目录的actions.jsonl和result.json、策略代码版本及哈希。
- 案例编码与结束页截图；若有官方现实运行时间也截图。
- 模拟器导出的原名加密日志（支撑材料要求，工具输出不能替代）。
- 本工具生成的表格、动作CSV和轨迹SVG。

本目录示例来自已有B3演练run-007，绝不是正式测试结果。未自动开始任何测试。若以前某局未保存客户端明文日志，仅有加密日志，则不能保证补出完整轨迹；可先核查控制台/本地备份，不应虚构路径。
