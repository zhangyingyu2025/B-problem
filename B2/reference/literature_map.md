# B2 证据地图

核验日期：2026-09-11。用途：G1.5–G2模型决策，不是论文参考文献定稿。下列链接已打开原始全文或作者存档；搜索摘要不作为核心结论依据。三个外部作品具有不同目标，不能混用最优性结论。

|来源|已核验内容|本题用途与限制|
|---|---|---|
|本地B题.pdf 第1–3页；附件1 §2.2；附件2检测协议|单次读数、有界误差、源圆域、未知接收半径、near语义|最高优先级题设；B2没有实测输入|
|[Tokekar & Isler, Sensor Placement and Selection for Bearing Sensors with Bounded Uncertainty，作者2013版PDF](https://tokekar.com/pubs/tokekar2013asensor.pdf)，§III，第3页|以有界角误差楔形交集的最坏直径/面积描述定位不确定性|支持指标与对抗误差思路；研究方形环境多传感器布置，不直接给出本题单步、有限接收半径策略|
|本地2013参考论文.pdf，内页2012年TR12-006|同主题长版技术报告|版本补充，不算独立证据；不得仅按文件名写成2013正式出版版本|
|[Tekdas & Isler, Sensor Placement for Triangulation Based Localization，作者存档](https://www-users.cse.umn.edu/~isler/pub/tase10.pdf)，印刷页1，图1及§II-A|双站几何指标同时依赖距离与夹角；一般模型可纳入接收范围约束|说明只追求夹角不足；该文指标不等于B1精确直径，本文不借用其近似比|
|[Zhao, Chen & Lee, Optimal Sensor Placement for Target Localization and Tracking in 2D and 3D，arXiv:1210.7397v1](https://arxiv.org/abs/1210.7397v1)，摘要及[全文](https://arxiv.org/pdf/1210.7397)|统一处理测向、测距与RSS的布置优化，使用框架理论分析|备选方法背景；本题不凭此引入概率噪声或声称90°在未知源距下普遍最优|

相关工作与我们的差别：B2先固定一个已观测的示向度，仅选择第二点；未知固定R∈[1000,1500]带来接收保障约束。本项目自行推导C_safe、三个圆盘交集C_cert及角分离候选域。它们须由证明和失败测试支撑，不能标成参考文献直接结论。

尚未关闭的问题：完整物理K相对三角形T的保守损失、连续第二点搜索的最优性证据、三角函数上下界的数值实现。当前不声称全局最优、新颖性或优于文献方法。
