# G41-JCR Phase A 预登记（首次运行前）

任务书SHA256：5383f603ab0dda7c5aaa05a1f84b10432e0f6179ce26e68217b1b1291d0a7a4a。
实际起点HEAD、Python和未提交覆盖包状态见g41_start_audit.json；106个既有Python文件记录LF哈希。G40及其依赖不修改；run_offline仅注册G41。G40为唯一对照。

## 顺序与数据

Stage0：B4全套回归，G40 smoke64240980–64240999共20场。Stage1：fresh DEV20=64241000–64241019。Stage2=64241100–64241149；Stage3=64241200–64241249。五类stress各10场，依次64241300–309、310–319、320–329、330–339、340–349，模式all_directional/boundary/adversarial_fan/coverage_edge/backface_origin。随机/压力均fixed_field；数学测试及关闭JCR动作等价另用64241900起，不参与性能选择。

本地已有结果文件名/合同未发现上述段，覆盖包报告使用64002270–64002369，README建议64003000；本轮采用独立64241段避免冲突。锁定64100000–64139999，不运行、不读取。每个stage同case哈希、同error mode配对。人工构造，禁止称官方结果。

Stage1门槛：20/20全清，所有guarantee/invariant=0，weighted优于G40，movement不恶化超过1%，planner每局≤30s（额外硬限：累计规划30s后停用JCR，回G40；正常行为由固定工作量控制）。不通过停止批量实验并归因。
Stage2：50/50全清、weighted改善≥1%、movement改善；measurements+2/run或P95/CVaR95恶化>3%警告并暂缓，不擅自放宽。通过才Stage3，再stress。未通过保留G40，无阈值搜索。Phase B仅在Phase A晋级后研究。

## 单一实现

G41继承G40，保留其几何、被动机会测量、adaptive fan、center probe、guaranteed clear及fallback。仅对尚有certificate且已有未清除源的情况规划未来顶点对。G41关闭开关直接调用G40原循环。

成本单位为等效米：move1、measure25、switch5、失败clear15、成功clear25；不设奖励权重。J=L+sum(min(Cactive,Cpair))。初始路线复用fast_open_route，邻域2-opt、relocate、swap，最多4轮best strict improvement、每次最多4000次J评估；所有枚举排序固定。无pair时Cactive为路线无关常数，严格退化为长度排序。

Safe仅rho≤1000，side复用完整wedge bracket。方向分支的未来夹角下界同时考虑整个第一sector与第二次±1°误差，保守预测strip矩形覆盖个数；太平行则拒绝pair。first direction / first negative then second direction / double negative chord sweep三个分支取最大。方向分支使用有限strip圆覆盖；双负用弦界后的第一中心射线有限圆覆盖。每次局部扫清后，成本包含回到当前certificate顶点的路程上界，避免把local clear移动漏算；若直接前往下一任务，三角不等式保证不超该收费。

未来测量频道状态未知时采用最坏切换上界5m，已知同频道为0；不是声称每次必然切换。只有承诺本来会执行的相同测量才允许额外测量费用为0；当前第一版对未来pair测量保守收费，执行时若G40已完成该观测则复用，不重复测量。

Cactive为当前信息下，独立执行原G40定位/有限fallback的保守完成上界，不能用平均fan数。上界可能很松。比较两个上界不能证明真实时间一定改善；jcr_predicted_eq_m_saved只表示上界差，真实改善必须由配对实验检验。若因上界过松导致过量延期，按既定gate判失败，不再扫阈值。

执行仅对fan允许新增pair延期。每次真实信息变动重规划，已测第一负分支的承诺保留至第二点（若16源early stop则取消并回G40）。严格覆盖序列耗尽必须记录invariant并调用保守fallback；报告依旧判不合格，不能以fallback掩盖失败。不添加Phase B route-clear（对应统计为0）。

## 记录与文件体积

每场保留case、结果和G41事件/动作证据，raw仅在忽略的results下。小批结束后compact paired JSON/CSV、按N分层、时间成本分解、预测与实际方向、最差退化场。jcr_realized_movement_saved_m仅由离线reporter配对计算；在线无反事实真值，不伪造该值。所有JCR统计落stats，planner wall与模拟虚拟时间分开。

## 成本审计补记（不改变参数、seed或gate）

第一批DEV20完成后、未查看其性能作选择前，代码审计发现新增measure后接回原动作序列可能还需一次频道恢复。原来只计进入probe的switch，遗漏最坏5m恢复费用。attempt0保留原代码和原日志，但不作晋级实验；公式修正为每个新增probe计进入与恢复两端切换上界，已知同频道两端均0。补充永久回归后只重跑相同DEV20，G40原始对照复用。修正涉及费用记账，不调路线轮数、阈值或样本。
