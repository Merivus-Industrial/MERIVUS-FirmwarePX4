# 候选 002 发现集结果

源码提交：`8d2f839dcf8d6b29bbc714abe8862ee6e169f1e2`。Ubuntu PX4 SITL 与 Gazebo Classic 构建通过，12 个 Python 单元测试通过；1 次自动 `shadow` 和 16 次 `off`/`active` 闭环试验完成。原始 [manifest.json](manifest.json)、[results.json](results.json)、[decision.json](decision.json)、[shadow.json](shadow.json) 随仓库保存。完整 ULog 与仿真日志归档在 `E:\MERIVUS\analysis\afcr-round2-discovery-8d2f.tar.gz`，SHA-256 为 `9ee2c6d67c84afded1e66084c4e2ece96a9f82bca96a0b674ab58bbae54bbde2`。

`shadow` 的 `AFCR_DA` 有 1875 个评分窗口样本，水平修正峰值 0.027/0.350 m/s²，Z 轴为零，均符合候选限幅。阵风窗口约 20% 样本触及水平限幅。所有 `off` 和 `active` 试验的安全硬门均通过。

| 种子 | 正常悬停 XY 锚点 RMSE（m） | 阵风事件 XY RMSE（m） | 阵风停风后 XY RMSE（m） | 预注册判定 |
| --- | ---: | ---: | ---: | --- |
| 11 | 0.0401 → 0.0507 | 0.8543 → 0.7468 | 0.6844 → 0.5699 | 正常场景增加约 26%，淘汰 |
| 12 | 0.0432 → 0.0510 | 0.8505 → 0.7697 | 0.6285 → 0.5072 | 正常场景增加约 18%，淘汰 |

候选 002 在阵风事件的 XY RMSE 上连续两次改善，正常悬停的 XY RMSE 也连续两次退化。按[预注册协议](PROTOCOL.md)，`retain_for_further_sitl=false`，未使用保留种子 21–23。单次种子 11 的快速筛选曾得到正常 XY 改善约 6%，但正式复测转为退化，说明厘米级正常场景差值还受试验波动影响；不能选择有利的一次来证明优化。

基线 ULog 中，正常悬停水平速度误差的第 95 百分位约 0.018 m/s，阵风窗口则约 1.7 m/s。这一数量级差异可用于下一候选的连续门控：让小误差时的补偿趋零，仅在明显受扰时介入。门控阈值必须在下次验证前固定，且仍须检查从阵风退出时的过渡与正常场景是否真正无回退。
