# 候选 006 发现集结果

源码提交：`807392a0c2fe46e8385856a4d1606ea875b68c69`。Ubuntu 中 16 个 Python 单测、PX4 SITL 与 Gazebo Classic 构建通过；自动阵风 `shadow` 和两个发现种子的 16 次 `off`/`active` 独立试验完成。种子 11、12 都是第 1 次完整尝试，没有触发无效真值重试。原始 [manifest.json](discovery/manifest.json)、[results.json](discovery/results.json)、[decision.json](discovery/decision.json)、[shadow.json](discovery/shadow.json) 保存在仓库。完整 ULog 和仿真日志归档于 `E:\MERIVUS\analysis\afcr-round5-discovery-006.tar.gz`，SHA-256：`26b65a4a334c952f52b3df4d88a191ba2e02c1063d72da92e456f550faf6e89f`。

| 种子 | 阵风事件 XY RMSE（m） | 停风后 XY RMSE（m） | 阵风事件 Z RMSE（m） | 正常/固定风/载荷修正 | 判定 |
| --- | ---: | ---: | ---: | --- | --- |
| 11 | 0.8502 → 0.7396 | 0.6832 → 0.5677 | 0.1611 → 0.1790 | 全程零输出 | 通过 |
| 12 | 0.8571 → 0.7520 | 0.6336 → 0.4930 | 0.1913 → 0.1502 | 全程零输出 | 通过 |

两个种子的所有 `off`/`active` 场景均通过真值峰值、分配器和飞行模式硬门。阵风平均电机输出平方和分别为 1.1953 → 1.1946、1.1948 → 1.1936，未出现 5% 以上增加。候选 006 因此按[第五轮协议](PROTOCOL.md)进入一次性保留集；发现集结果只能证明在这两个 Gazebo 种子下通过筛选，不能提前宣称泛化或实机收益。
