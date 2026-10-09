# 候选 005 第四轮发现集：真值坐标异常

源码提交：`6c85ff779f2351e24b9ac1ff78122db6c8b2f227`。Ubuntu 中 15 个 Python 单测和 PX4/Gazebo 构建通过；自动阵风 `shadow` 与 16 次 `off`/`active` 试验完成。正常、固定风和固定载荷三个 `active` 评分窗口的修正量均为零，且从候选启用到窗口结束的记录峰值也为零。原始 [manifest.json](manifest.json)、[results.json](results.json)、[decision.json](decision.json)、[shadow.json](shadow.json) 随仓库保留。完整 ULog 与仿真日志归档于 `E:\MERIVUS\analysis\afcr-round4-discovery-005.tar.gz`，SHA-256：`e7979e50059a206dfd864d35d54e1833df8548f1b5611afbf6d4ee0abeb696cc`。

旧评分器判定两个种子均 `candidate hard gate failed`。核查原始 ULog 后发现，种子 11 的正常 `active` 与种子 12 的载荷 `active` 的 `vehicle_local_position_groundtruth` 使用约 5,286,006/722,344/−490 m 的绝对坐标，PX4 本地位置和设定点仍在零附近。旧评分器直接相减，产生数百万米假误差；这些试验不具备厘米级位移评分所需的本地真值精度，整组配对无效。两次有效的阵风试验分别记录事件 XY 位移 0.8584 → 0.7539 m、0.8618 → 0.7895 m；种子 12 的约 0.0723 m 改善未达到 0.080 m 门槛。不能选择其中一组有利数据来宣称通过。

候选 005 不进入保留集。新的[真值坐标质量协议](../round-005/PROTOCOL.md)在所有工况统一拒绝异坐标原点或粗量化真值，重试整个种子而非挑选单个好看的会话。下一候选还须提高阵风改善余量，同时保持非阵风全程零修正。
