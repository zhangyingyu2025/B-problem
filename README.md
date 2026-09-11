# B-problem
dashumobuxiongdi

## 求解模块

- [B1：角度交会与圆覆盖](B1/README.md)
- [B2：第二检测点选择](B2/README.md)
- [B3：全向源搜索、定位与清除](B3/README.md)

后续在本克隆目录直接开发。B3调用兄弟目录中的B1/B2，禁止复制一份几何实现产生分叉。原始题包不需要提交。

```powershell
python -X utf8 B2/code/reproduce.py --verify-only
python -X utf8 -u B3/code/reproduce.py
```

B3默认只做自建离线仿真；官方演练需要登录模拟器和真实robot_id。当前不提供自动启动正式测试的入口。
