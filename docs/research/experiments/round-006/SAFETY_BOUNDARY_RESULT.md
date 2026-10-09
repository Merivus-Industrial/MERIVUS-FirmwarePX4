# Candidate 006 安全边界回归结果

**判定：FAIL。** 十个预注册边界中，六项 `PASS`、两项 `PARTIAL`、两项 `FAIL`。failsafe 与 GPS 断流都已由 PX4 状态或 GPS topic 验证，禁用期仍出现非零候选修正，因此不能把 Candidate 006 称为 fail-silent，也不能进入实机闭环。

## 证据身份

安全试验使用 PX4 SITL / Gazebo Classic iris，基础源码提交 `4227a04a303af010c644c000c119f7272fe85a8e`，PX4 SITL 二进制 SHA-256 `ff994c8eb9220f26d3fa8f8d2a99cbd946cdcf0bc727d8c5dcf1abbd35286d00`；候选 JSON SHA-256 仍为 `fdedda61eb85b7a31646cac47e0484c9bb2919874b29d6a4ba9a8ab5be81547e`。各场景在 46–54 s 施加同一 8 m/s +X 阵风，使边界动作前的候选处于工作状态，然后在约 48 s 触发动作。

原始目录为 `/home/cwkj/afcr-safety-4227-full/`，本地完整归档为 `E:/MERIVUS/analysis/afcr-round6/afcr-safety-4227-full.tar.gz`，SHA-256 为 `050e485ed6636f3274309f0af0f94bb0a49bb6bc40ab5b4e475dcca45d66d22b`，已在 Windows 端复核。逐场景原始 ULog 和命令时间戳见 `E:/MERIVUS/analysis/afcr-round6/safety_results.json`；逐事件分析见 `safety_analysis.json`；32 个 JSON/ULog 的路径与 SHA-256 见 `safety_evidence_index.json`。归档还包含 PX4/Gazebo 控制台日志。每项起飞段另有约 700 个零修正样本，下面只列目标边界段。

| 边界 | 结果 | 禁用段 `AFCR_DA` 样本 | 最大单轴绝对修正 | 观察与依据 |
| --- | --- | ---: | ---: | --- |
| 起飞 | PASS | 700 | 0 | 起飞 nav_state 17 已出现 |
| 着陆 | PASS | 745 | 0 | 着陆 nav_state 18 已出现；动作前 2 s 修正峰值 0.350 m/s² |
| 手动摇杆输入 | PASS | 600 | 0 | 试验施加水平摇杆；动作前峰值 0.350 m/s² |
| Position→Altitude | PASS | 620 | 0 | nav_state 1 已出现；动作前峰值 0.350 m/s² |
| Position→Stabilized | PARTIAL | 0 | 无法测量 | nav_state 15 已出现、动作前峰值 0.350 m/s²，但模式切换后 `AFCR_DA` 没有可评分样本；不能把无样本当成零 |
| Stabilized→Position | PASS | 245 | 0 | nav_state 15→2 已出现；只评价重新进入后的 2 s 等待期 |
| RTL | PASS | 620 | 0 | nav_state 5 已出现；动作前峰值 0.350 m/s² |
| failsafe | **FAIL** | 919 | **0.276 m/s²** | `vehicle_status.failsafe=true` 已出现，禁用期仍非零；动作前峰值 0.350 m/s² |
| GPS loss | **FAIL** | 937 | **0.291 m/s²** | 注入后 `sensor_gps` 样本数为 0，候选仍非零；动作前峰值 0.350 m/s² |
| EKF reset | PARTIAL | 无 | 无法测量 | `ekf2 stop` 命令超时，原试验只有中断前 ULog，未观察到完整 reset 边界 |

逐样本复核排除了指令传播前样本造成的假阳性：从触发时刻后 0.5 s 开始，failsafe 用例约 644 个非零诊断样本，GPS loss 用例约 529 个非零诊断样本。failsafe 非零样本中包括 `failsafe=true` 且 nav_state 仍为 Auto Loiter 的阶段；GPS 断流后候选在切换到其他模式之前继续工作。原分析还要求状态实际出现或 GPS topic 实际断流，并按模式匹配诊断；详细时间戳和 X/Y 修正可从两份 ULog 复算。这里的 `AFCR_DA` 是当前 SITL 中候选参与位置控制时的修正诊断，因此其非零属于真实安全边界回归。

## 根因位置与未闭合证据

当前 `MulticopterPositionControl::researchHoverReady()` 只以 Position/Loiter、定点设定值、有限状态及悬停锁存判断研究入口，没有直接把 `vehicle_status.failsafe`、GPS 数据新鲜度、EKF reset counter 变化纳入同一个失效闭合合同。Auto Loiter 在故障过渡中仍属于允许集合，这与上面的非零 ULog 一致。要修复，应在研究入口的唯一门控层按 PX4 实际失效状态与估计器状态立即撤销锁存，并对故障前后时序做回归；本轮冻结 Candidate 006 的参数和试验结果，不通过改报告或重调参数掩盖失败。

Position→Stabilized 缺少诊断样本，说明当前从位置控制器发布的 `debug_vect` 在该模式下无法证明持续零输出；将来需由独立周期观察通道发布明确的 `candidate_active=false`、零修正和 gate reason。EKF reset 的首轮超时与试验器阻塞 PX4 命令时未维护 MAVLink 心跳有关，已保留失败的 ULog。随后只修复试验器，使其在等待 `ekf2 stop` 时继续处理 MAVLink，并用原候选、原 PX4 二进制单独补测；停止 EKF 后 SITL 锁步时间无法推进，仍为 `PARTIAL`。第二次原始证据目录 `/home/cwkj/afcr-safety-ekf-pumped/`，本地归档 `E:/MERIVUS/analysis/afcr-round6/afcr-safety-ekf-pumped.tar.gz`，SHA-256 `9ae42d99dea8784aa321a842deb298e81217b266b790c9f649cdcfb717920b6d`，含 5 个 JSON/ULog 的索引。原十项记录没有被覆盖。

本轮没有实机编译、刷写、闭环或飞行。后续只读[Shadow Mode 设计](SHADOW_MODE_DESIGN.md)要求显式记录门原因、估计状态年龄和 reset counter，且与原生 controller 输出隔离。只有解决两项非零回归及两个未闭合边界，并重新跑冻结安全矩阵，才能重新讨论更高验证等级。
