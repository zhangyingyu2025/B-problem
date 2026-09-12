# G40 本地覆盖包使用说明

你的本地仓库路径：

`E:\DeskBox\jty1379\项目\数模\B-problem-jty`

本包按仓库根目录相对路径组织。

## 推荐操作

先在仓库根目录备份当前 B4：

```powershell
git status
git add B4
git commit -m "Checkpoint B4 before G40 overlay"
```

如果不想提交，也至少复制一份当前 `B4`。

然后把本压缩包中的：

```text
B4/code/*
B4/tests/test_b4_g40.py
```

覆盖/合并到你的仓库：

```text
E:\DeskBox\jty1379\项目\数模\B-problem-jty\B4\
```

本包的 `B4/code` 是经过 G40 验证时使用的完整代码目录，不是只放一个 solver 增量；
这样可以避免你本地还停留在 G1/G1D 时缺 G18/G31/G34/G40 的依赖链。

## 最关键入口

G40 solver 实际位于：

```text
B4/code/solver_g35.py
```

类：

```python
G40PassiveSafe
```

`B4/code/run_offline.py` 已注册：

```text
--variant G40
```

## 先做 smoke test

在仓库根目录运行一个你们当前允许的 DEV seed，例如：

```powershell
python -X utf8 -u B4/code/run_offline.py --variant G40 --start 64003000 --count 5 --output B4/results/g40_local_smoke
```

如果你当前 `run_offline.py --help` 的参数名与此略有差异，以：

```powershell
python -X utf8 B4/code/run_offline.py --help
```

为准。

然后跑 G40 新增测试：

```powershell
python -X utf8 -m unittest -v B4.tests.test_b4_g40
```

再跑已有 B4 回归。

## 注意

- 不要运行锁定 HOLDOUT：
  - 64100000–64100099
  - 64110000–64110049
  - 64120000–64120049
  - 64130000–64130049
- G40 保留 G34/G31/G18 的 fallback。
- 不要只拷贝 `solver_g35.py` 而漏掉依赖文件。
- 大的 raw results 不需要提交 Git。
