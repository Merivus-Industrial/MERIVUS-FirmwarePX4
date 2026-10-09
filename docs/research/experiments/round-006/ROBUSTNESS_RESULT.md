# Candidate 006 鲁棒性冻结试验结果

**判定：FAIL。** 本轮按[预注册矩阵](ROBUSTNESS_MATRIX.md)在 PX4 SITL / Gazebo Classic iris 运行 26 个用例，每项以相同 seed 的原生 `off` 与候选 `active` 独立会话比较，共 52 份 ULog。23 项 `PASS`、2 项 `PARTIAL`、1 项 `FAIL`，没有缺失用例。此判定不抹除第五轮 8 m/s 保留种子的通过结论，但禁止将其外推为跨扰动鲁棒或实机可用。

## 冻结身份与证据

| 项目 | 值 |
| --- | --- |
| 试验源码提交 | `fddffffe3b193da121e02f9b22fd1f9966e21df9` |
| Candidate JSON SHA-256 | `fdedda61eb85b7a31646cac47e0484c9bb2919874b29d6a4ba9a8ab5be81547e` |
| PX4 SITL 二进制 SHA-256 | `c4fb57e8cc9ffcea8a19ced309c4f22a08574bf5d98b7cf19ee2d0762624f908` |
| 风场插件 SHA-256 | `2f810ce4d4c18a22093a52d34855465082a3ccbbc0584669f8860b0b9f0a0129` |
| 原始目录 | `/home/cwkj/afcr-robustness-fddffff-matrix/` |
| 本地原始归档 | `E:/MERIVUS/analysis/afcr-round6/afcr-robustness-fddffff-matrix.tar.gz` |
| 归档 SHA-256 | `0f5f48065df59b47562d741533fd3e8148f870d48c1cdc8875b54b4429276edb` |

每项 `off`/`active` ULog 的完整路径见本地 `E:/MERIVUS/analysis/afcr-round6/classification.json` 的 `raw_ulog`，以及 `results.json`；所有 187 个 JSON/ULog 的相对路径、绝对路径、大小和 SHA-256 见 `evidence_index.json`。原始归档同时保留生成的 world、model、参数、控制台日志与评分窗口。归档已经在 Windows 端重新计算 SHA-256，与 Ubuntu 源文件一致。分析脚本版本晚于飞行提交，只读取冻结 ULog，不修改候选、基线或门限。

## 逐项结论

| 组 | 用例与结论 | 观察 |
| --- | --- | --- |
| 风速、方向、时长 | `w02_px_g05` PASS；`w04_nx_g1` PASS；`w06_py_g2` PASS；`w08_ny_g5` PASS；`w10_45_g1` PARTIAL；`w12_135_g2` FAIL | 六档风速、六方向与 0.5/1/2/5 s 已覆盖；高风速不满足全部预注册门限 |
| 标称与风型 | `nominal_iris_8` PASS；`steady_px_8` PASS；`steady_ny_12` PARTIAL；`periodic_45_8` PASS；`random_135_8` PASS | 固定风时修正全程为零；周期/随机风安全且未退化 |
| 质量 | `mass_m20`、`mass_m10`、`mass_p10`、`mass_p20` 均 PASS | 四种 base_link 质量偏移下事件 XY RMSE 均改善约 0.10–0.13 m |
| 惯量 | `inertia_m20`、`inertia_m10`、`inertia_p10`、`inertia_p20` 均 PASS | 四种惯量偏移下事件 XY RMSE 均改善约 0.09–0.12 m |
| 电机 | `motor_delay_2x`、`thrust_max_90pct` 均 PASS | 延迟加倍和最大转速 90% 的单因素偏移通过 |
| 传感器/估计 | `gps_pos_noise_2x`、`gps_vel_noise_2x`、`gps_fusion_delay_220ms`、`imu_noise_2x`、`imu_bias_2x` 均 PASS | 五种注入下事件 XY RMSE 各改善约 0.10–0.13 m |

`w12_135_g2` 的候选锚点 XY 最大位移为 **2.161 m**，越过预注册 **2.000 m** 安全硬门，因此为 `FAIL`。原生对照为 2.346 m，也超出包线；候选比原生改善约 0.185 m，仍不能把候选判为通过。候选/原生位置误差 95 分位分别为 1.909/2.059 m，最大位置误差 2.137/2.313 m；恢复时间 6.828/6.084 s，候选慢 0.744 s。该用例姿态最大倾角 32.10°、actuator 最大输出 0.694、激活频率 75.36%、最大修正范数 0.495 m/s²、每轴限幅样本占 13.65%。

`w10_45_g1` 的事件 XY RMSE 改善 **0.038 m**，低于强阵风逐项 **0.050 m** 门槛，因此为 `PARTIAL`。最大锚点位移候选/原生为 1.105/1.220 m；安全门没有失败。`steady_ny_12` 候选修正全程为零、候选安全门通过，原生对照 Z 锚点峰值约 1.017 m，越过 1 m 门限；独立会话中的候选 Z 峰值约 0.851 m。由于候选零修正，不能将两次运行的差异归功于算法，该工况标为 `PARTIAL`。

## 最差情况与控制活动

以下差值统一为 `active − off`；正值表示退化。所有原始值及每个用例的修正、速度误差、推断门状态、位置误差、速度、姿态、执行器和分配失败率见 `extended.json`、`results.json` 与原始 ULog。

| 指标 | 本轮最差观测 | 解释 |
| --- | --- | --- |
| XY 位置误差 95 分位差 | `steady_px_8` +0.0076 m | 两次会话候选均零修正；不能归因为候选 |
| 最大锚点位移差 | `steady_px_8` +0.0176 m | 同上，仍低于 +0.20 m 非退化门 |
| 停风后超调差 | `w02_px_g05` −0.0107 m | 有定义的事件用例中最差值仍为改善 |
| 2 s 持续收敛时间差 | `w12_135_g2` +0.744 s | 未超 +2 s 比较门，但该项硬门失败 |
| 激活样本占比 | 最高约 78%（`mass_m20`） | 非目标固定风两项均为 0 |
| 最大修正范数 | 0.495 m/s²（`w12_135_g2`） | 对角方向双轴同时限幅；每轴最大约 0.350 m/s²，总量低于 0.5 m/s² |
| 每轴候选限幅发生率 | 最高约 26%（`mass_m20`） | 说明部分收益依赖修正幅值上限，不能称有充足控制余量 |
| 分配器饱和发生率 | 26 项 `off`/`active` 均为 0 | 只适用于本轮模型、窗口与指标 |
| 修正差分斜率 95 分位 | 最高约 2.36 m/s³（`periodic_45_8`） | 仅为高频代理量，不能代替频谱或稳定裕度分析 |

部分用例在记录窗口内，候选与原生都未达到误差 ≤0.20 m 并连续保持 2 s；其 settling time 在原始 JSON 中为 `null`，没有用零代替。风速、风向、时长和模型因素只做了分层覆盖，未做全因子交互；每用例只有一个 seed，因此通过项也没有给出跨 seed 置信区间。

传感器组覆盖 Gazebo GPS 位置/速度噪声、IMU 噪声/偏置与 EKF2 GPS 融合延迟。**GPS 数据链路真实延迟和本地状态估计发布延迟没有在闭环注入。** `state_delay_replay.json` 仅将原生 ULog 按 0/20/50/100 ms 重放做开环敏感性检查，不能证明时延下的闭环稳定。100 ms 时标称速度误差 95 分位约 1.715 m/s（0 ms 约 1.655 m/s）；GPS 速度噪声场景约 1.713 m/s（0 ms 约 1.655 m/s）。当前 ULog 未直接记录候选 raw/gated residual 与内部 hover latch，门状态由对齐样本推断；[只读 Shadow 设计](SHADOW_MODE_DESIGN.md)将这些列为后续日志契约。

本轮安全边界另有独立失败，见[安全边界结果](SAFETY_BOUNDARY_RESULT.md)。因此本轮总判定保持 **FAIL**，Candidate 006 不进入实机闭环或真实飞行验证。
