# B2 中央案例连续全局下界证书

## 结论

本目录验证的是中央合成案例

\[
s_1=(0,0),\qquad \theta_1=0^\circ,\qquad
\alpha=1^\circ,\qquad 1000\le R\le1500,\qquad r_{\rm near}=5,
\]

在完整下支候选域 \(C_{0,-}\) 上的连续全局问题。这里所说的“全局”有明确作用域：

- 源集合使用物理集合
  \[
  K=D_0\cap W_1\cap\{G:5<r_1(G)\le1500\},
  \]
  不再用三角形 \(T\) 代替 \(K\)；
- \(p\) 在 \(C_{0,-}\) 中连续取值，不再只检查有限网格；
- 上支由精确镜像 \(y\mapsto-y\) 覆盖；
- 结论是连续最优值的一个严格数值夹逼，不是完整 \(C_{\rm safe}\)、非中央实例或证明助手级形式化结论。

当前结果为

\[
134.97447828420857\ {\rm m}
\le \inf_{p\in C_{0,-}}J_K(p)
\le 134.97448751394995\ {\rm m},
\]

夹逼宽度为

\[
9.229741367458566\times10^{-6}\ {\rm m}.
\]

下界由带定向舍入的 Decimal 区间分支定界得到；上界来自一个可行近边界点的自适应浮点包络。由于上界尚未用严格区间算术重新认证，本证书不声称已经证明“精确最优值等于某个闭式常数”。

## 与合并后主求解器的关系

主求解器合并了基础网格、双侧模式搜索和 `50/25/12.5/6.25 m` 多起点分层细化，并用 `outer_convergence`、`inner_convergence` 分别记录候选网格和固定点角度包络的稳定性。中央有限搜索报告的最坏直径上界约为 `135.80922861381495 m`，低于仅使用 `50/25/12.5/6.25 m` 分层网格时的 `136.111151 m`。

本目录的连续证书使用同一中央案例的物理集合 \(K\) 和限定候选域 \(C_{0,-}\)，给出约 `134.974478–134.974488 m` 的连续夹逼。它用于说明中央案例连续问题在该规定范围内能进一步收紧，同时不把主求解器的有限候选结果冒充为完整连续全局最优。

## 1. 物理活跃源

定义第一楔形 \(W_1\) 的下边界端点

\[
A=1500(\cos1^\circ,-\sin1^\circ).
\]

它满足：

- \(\|A\|=1500<1800\)，故 \(A\in D_0\)；
- \(A\) 位于 \(W_1\) 的下边界射线；
- \(5<1500\le1500\)。

因此 \(A\in K\)，是一个真实物理可行的目标位置，而不是凸外包络 \(H\) 中的人造点。

对下支候选点 \(p\)，记

\[
\beta=\arg(A-p),\qquad R_A=\|A-p\|.
\]

令第二次读数为

\[
\theta_A=\beta-\alpha.
\]

此时从 \(p\) 看 \(A\) 的真实方向恰为 \(\beta\)，且读数误差为 \(+\alpha\)，所以该读数物理可行。第二楔形为

\[
W_2=[\beta-2\alpha,\beta].
\]

在 \(P=W_1\cap W_2\) 中，\(A\) 是一个顶点。另一个顶点 \(C\) 位于 \(W_1\) 的上射线与 \(W_2\) 的下射线之交。联立两条射线得到

\[
r_C(\beta,R_A)
=
\frac{1500\sin(\beta-\alpha)+R_A\sin(2\alpha)}
     {\sin(\beta-3\alpha)}.
\]

固定 \(\beta\) 时，正弦系数

\[
\sin(2\alpha)>0,
\]

故 \(r_C\) 随 \(R_A\) 单调增加。

对固定 \(\beta\)，射线 \(A-p\) 与闭圆盘 \(\|p\|\le1000\) 相交。其最小距离为

\[
R_{A,\min}(\beta)
=
1500\cos(\beta+\alpha)
-\sqrt{1000^2-1500^2\sin^2(\beta+\alpha)}.
\]

所以每个区间盒只需对下式做外向区间外包：

\[
r_C(\beta,R_{A,\min}(\beta)).
\]

这已经把原来的二维连续问题严格约化为一维问题。

## 2. 角度外包

### 2.1 下界 \(\beta>3.1^\circ\)

代码中的下支判据为

\[
b<-1500\tan\alpha-(1500-a)\tan(3.1^\circ).
\]

记右侧阈值为 \(b_{\rm code}\)。真实方向满足 \(\beta>3.1^\circ\) 的边界为

\[
b_{\rm exact}
=
-1500\sin\alpha-(1500\cos\alpha-a)\tan(3.1^\circ).
\]

两式之差为

\[
b_{\rm code}-b_{\rm exact}
=
-1500(1-\cos\alpha)
 \left(\tan3.1^\circ-\tan1^\circ\right)<0.
\]

故 \(b<b_{\rm code}\) 蕴含 \(b<b_{\rm exact}\)，从而 \(\beta>3.1^\circ\)。证书中仍保守使用

\[
\beta\in[3^\circ,41^\circ],
\]

它严格包含真实下支范围。

### 2.2 上界 \(\beta<40.811^\circ\)

从 \(\|p\|\le1000\) 可得射线与圆盘有交点，因此判别式非负：

\[
1000^2-1500^2\sin^2(\beta+\alpha)\ge0.
\]

于是

\[
\sin(\beta+\alpha)\le\frac23.
\]

区间三角函数证书已经验证

\[
\sin41.811^\circ>\frac23.
\]

由于 \(\sin\) 在 \(0^\circ\) 至 \(90^\circ\) 上单调增加，

\[
\beta\le\arcsin(2/3)-1^\circ<40.811^\circ<41^\circ.
\]

## 3. 分支对称

中央案例中的 \(D_0\)、\(W_1\)、\(K\)、误差锥和 \(C_0\) 均关于 \(x\) 轴对称。因此

\[
(x,y)\mapsto(x,-y)
\]

保持最优值不变。下支证书自动给出上支证书，无需重复枚举。

## 4. 直径下界

设第二个顶点半径为 \(r\)。同一第一楔形上、下射线的夹角为 \(2\alpha\)，\(A\) 的半径为 \(1500\)，故

\[
D(r)^2=1500^2+r^2-2(1500)r\cos2^\circ.
\]

在本证书范围内

\[
r\ge1623.49551
>1500\cos2^\circ,
\]

所以 \(D(r)\) 关于 \(r\) 单调增加。分支定界只需要证明

\[
r_C(\beta,R_{A,\min}(\beta))\ge1623.49551.
\]

一旦该式成立，即可得到

\[
\operatorname{diam}(P)\ge D(1623.49551).
\]

## 5. 区间算术与搜索

`verify_central.py` 的实现链如下：

1. `Interval` 保存闭区间端点；
2. 加减乘除分别使用 `ROUND_FLOOR` 和 `ROUND_CEILING`；
3. 三角函数以 65 位中值点加 Lipschitz 半径外包，并用 \(10^{-55}\) 的 padding 覆盖级数与舍入误差；
4. 对 \([3^\circ,41^\circ]\) 做深度优先分支；
5. 若盒子的 \(r_C\) 下界已经达到目标值，则整盒安全剪枝；
6. 若盒子宽度降到 \(10^{-8}\) 仍未剪枝，则证书失败；
7. 当前结果为 `closed`，无未决盒子。

最新运行统计：

- 处理盒子：`158341`
- 剪枝盒子：`79171`
- 未决盒子：`0`
- 下界半径：`1623.49551 m`
- 下界直径：`134.9744782842085825... m`

## 6. 可行上界与最终夹逼

边界最小点附近取严格内向点

\[
p^\star=(798.3022225759216,-602.2570542802663),
\qquad \|p^\star\|=999.999999\ {\rm m}.
\]

程序验证 \(p^\star\in C_{0,-}\)，并用原 B2 自适应角度包络计算：

\[
J_K(p^\star)\le134.97448751394995\ {\rm m}.
\]

这个上界说明下界具有可达到性证据；它与下界共同给出前述约 \(9.23\times10^{-6}\) m 的连续夹逼。

注意：上界字段是 `adaptive float envelope`，不是经过定向舍入的区间证书。因此证书中
`exact_global_optimum_equality_proved` 明确为 `false`。

## 7. 已证明与未证明

已证明：

- 中央案例 `C0` 下支的完整连续下界；
- 上支由精确镜像覆盖；
- 活跃源 \(A\) 属于物理集合 \(K\)；
- 角度外包覆盖真实下支；
- 半径约化、单调性和直径下界链闭合。

未证明：

- 完整 \(C_{\rm safe}\setminus C_0\) 上的全局最优性；
- 非中央平移、旋转实例；
- 浮点上界的证明助手级形式化；
- “精确最小点唯一”以及精确闭式最优值等式。

## 8. 复现

在仓库根目录执行：

```powershell
& $python -X utf8 B2\continuous_proof\verify_central.py `
  --output B2\continuous_proof\certificate.json
```

其中 `$python` 指可用的 Python 3.11+ 解释器。运行测试：

```powershell
& $python -X utf8 -m unittest discover `
  -s B2\continuous_proof -p "test_*.py" -v
```

验证正式 B2 冻结结果未被本证书代码影响：

```powershell
& $python -X utf8 B2\code\reproduce.py --verify-only
```

## 9. 代码逻辑结构

```text
verify_central.py
├── Interval / directed arithmetic
│   ├── add, sub, mul, div
│   ├── sin_cos_interval_deg
│   └── sqrt_interval
├── radius_lower_bound
│   └── 对 beta 区间给 r_C 的严格下界
├── certify_lower_bound
│   └── 分支定界、剪枝、未决盒审计
├── diameter_lower_bound
│   └── 由 r_C 下界得到弦长下界
├── candidate_point / boundary_point
│   └── 构造可行的近边界上界点
├── endpoint_membership_certificate
│   └── 证明 A 是物理 K 中源
├── beta_domain_certificate
│   └── 记录角度上下界及其解析依据
└── build_report
    └── 汇总作用域、下界、可行上界、夹逼和限制
```
