# B1 交会定位算法

本目录交付 B1 数学模型、标准库 Python 实现、自动测试及合成案例。不包含论文、B2–B4 搜索策略或模拟器调用。

主流程为 **独立二维线性可行性 → recession cone 有界性 → 顶点 → 直径 → 最远点对中点圆判定**。最小覆盖圆是独立验证器，并提供最小所需覆盖直径。

## 一键复现

Python 3.10 或更高版本，无需 pip 安装依赖。从项目根目录运行：

```powershell
python -X utf8 "求解/问题一/code/reproduce.py"
```

该入口运行全部测试，计算四个合成案例，生成 JSON 与 SVG，检查两次运行的结果完全一致，并核验原始资料哈希。测试或完整性检查失败时停止；不连接网络，不调用模拟器。

输入清单中的路径相对于项目根目录，整个项目可迁移；若只复制本目录，可单独运行算法和测试，但完整原始资料核验需要保留原目录结构。脚本可以从任意当前工作目录调用。

## 单个案例

```powershell
python -X utf8 "求解/问题一/code/run_b1.py" "求解/问题一/examples/equilateral_counterexample.json" --output "求解/问题一/结果/equilateral_counterexample.json" --svg "求解/问题一/图片/equilateral_counterexample.svg"
```

输入 JSON 示例（单一干扰源）：

```json
{
  "case_id": "my_case",
  "data_kind": "synthetic",
  "error_deg": 1.0,
  "observations": [
    {"x": -100, "y": -100, "svd_deg": 45},
    {"x": 100, "y": -100, "svd_deg": 135}
  ]
}
```

实测数据使用 `"data_kind": "measured"`；本目录示例全部是 `synthetic`。坐标单位米，角度为从东向逆时针旋转的度数，必须在 `[0,360)`。角度在内部转换为弧度，不可输入弧度值冒充度数。`error_deg` 默认 1；允许测试参数满足 `0 < error_deg < 90`。

每条观测必须恰含 `x,y,svd_deg`。`no_signal` 和 `near` 不能作为角度观测传入。分组频道由调用方完成。当前 CLI 不接受混合频道或直接输入模拟器响应。

完全重复观测自动去重，结果保留原索引；同地点不同读数不平均、不删除。空观测列表视为输入错误；一般半平面接口允许空约束集，表示全平面。

## Python 接口

在 `code` 目录或把它加入模块搜索路径后：

```python
from b1_geometry import solve_b1, solve_halfplanes, feasible_point, recession_direction

result = solve_b1([
    {"x": -100, "y": -100, "svd_deg": 45},
    {"x": 100, "y": -100, "svd_deg": 135},
])

# 一般半平面 ax*x + ay*y <= b：4×3 矩形
rectangle = solve_halfplanes([(1, 0, 4), (-1, 0, 0), (0, 1, 3), (0, -1, 0)])
assert rectangle["diameter_m"] == 5
assert rectangle["diameter_circle"]["covers"] is True
```

`feasible_point` 返回一个可行点或 `None`；`recession_direction` 只检查齐次系统，调用它不表示原区域已确认非空。主接口严格先检查原区域可行性。

## 输出状态与字段

| status | 含义 | 直径与圆 |
|---|---|---|
| `bounded` | 非空有界二维多边形 | 输出有限值 |
| `point` | 单点 | 直径与覆盖半径为 0 |
| `segment` | 线段 | 输出端点距离与中点圆 |
| `empty` | 约束不可行 | `null`，不是 0 |
| `unbounded` | 非空且有非零延伸方向 | `null`，数学直径为无穷 |
| `numerically_uncertain` | 有效交点病态、圆判定微小裕量或输出异常 | 不发布确定的直径和圆结论 |

- `feasible_point_exact`：可行点的精确有理数表示，针对输入浮点系数。
- `recession_direction_exact`：齐次系统可行的非零方向证据，最大绝对分量归一为 1。
- `recession_checks`：四个规范化系统的可行性记录；发现方向即可提前结束。
- `algebraic_status`：数值警告前，对所表示系数系统得到的代数状态。
- `vertices`：多边形顶点逆时针排列；退化情况为一个点或两个端点。
- `diameter_m`、`farthest_pair`：区域直径及一组最远点对。
- `diameter_circle.covers`：主判定；失败时列出未覆盖顶点。
- `minimum_enclosing_circle.diameter_m`：独立算法得到的最小所需覆盖直径。
- `circle_crosscheck_passed`：两种算法是否一致。

CLI 退出码：0 表示成功完成计算（包括正确得到空集或无界）；2 表示输入或文件错误；3 表示数值不确定。输入与两个输出必须是不同路径。JSON 禁止 NaN、Infinity 和重复键。输出文件可被同名重跑覆盖，建议放在 `结果/` 和 `图片/`。

## 查看模型、案例和验证

- `模型与算法说明.md`：完整模型、算法证明、覆盖圆充要条件、复杂度和限制。
- `数据审计.md`：题面与附件盘点、参数来源、数据缺口。
- `examples/`：等边三角形反例、双站交会、单站无界和背向空集。
- `结果/frozen_numbers.json`：通过测试后保存的合成结果。
- `结果/validation_report.md`：验证范围、发现的问题与修复记录。
- `结果/validation_results.json`、`test_log.txt`：机器可读结果与测试日志。
- `图片/*.svg`：等比例矢量核验图，可用浏览器打开。

单独运行测试：

```powershell
python -X utf8 -m unittest discover -s "求解/问题一/tests" -v
```

算法适用于今晚少量检测点的正确性验证：顶点枚举 `O(n³)`，独立最小覆盖圆穷举 `O(m⁴)`；有理数运算的位复杂度另计。它不是面向海量观测的高速半平面交实现。

关键限制：精确有理数运算保证输入浮点系数所表示的线性系统不会被容差悄悄扩张，不意味着三角函数或真实测向数据绝对精确。数值不确定时应检查输入几何并补充有效观测，不能直接修改物理误差界掩盖问题。
