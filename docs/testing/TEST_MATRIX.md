# MERIVUS 测试矩阵

## 验证等级

| 等级 | 含义 | 典型证据 |
| --- | --- | --- |
| L0 Static | 路径、接口、格式、参数/uORB 生成输入和文档一致性 | `rg`、Git diff、GitNexus、格式/元数据检查 |
| L1 Host | 不依赖 PX4 固件的主机可执行检查 | 纯算法测试、脚本检查 |
| L2 Build | 目标固件或 SITL 完整构建 | 构建日志、目标、工具链、SHA |
| L3 Unit | 模块/库级自动测试 | 测试用例、输入输出和边界断言 |
| L4 SITL | 仿真闭环和故障/协议场景 | SITL 日志、状态转移、重复步骤 |
| L5 ULog Replay | 日志时间对齐、阈值与误报/漏报分析 | ULog、分析脚本、基线窗口 |
| L6 HITL | 飞控硬件参与的闭环仿真 | 硬件、固件 SHA、场景与结果 |
| L7 Bench | 拆桨/受控台架的传感器、通信和输出检查 | 设备清单、参数、日志、急停条件 |
| L8 Tethered Flight | 系留或等效受限飞行 | 风险评审、现场记录、退出条件 |
| L9 Flight Validation | 批准配置的完整飞行验证 | 发布身份、环境、逐项验收与复盘 |

达到某一级，并不表示较低级别的检查已经自动完成。每次验证记录都要写清实际执行了哪些项目。

## 功能最低矩阵

| 功能 | 开发合入最低目标 | 发布/实机最低目标 | 当前已知状态 |
| --- | --- | --- | --- |
| 纯文档/索引 | L0 | L0 | 本轮执行 L0，未构建 |
| FMUv6C BSP / V6C22 传感器 | L0 + L2 | L7；飞行发布需 L9 | `IMPLEMENTED_UNVERIFIED` |
| Hyper982 GNSS 二维速度 / EKF2 | L0 + L2 + L3 + L4；须含原始 NMEA 缺 Down 与完整三维两类输入 | L7 + L9 | 源码契约已修改；当前仓库未构建、未复测实际接收机 |
| GNSS 可选消费者 LPE / UAVCAN / Sagetech / 遥测 | L0 + L2 + L3，逐模块覆盖缺测和非法精度 | 启用前 L7；飞行发布需 L9 | LPE、UAVCAN、Sagetech 源码已处理，尚未构建；其他可选路径仍需审查 |
| GNSS 航向 / GSF | L0 + L2 + L4，核实有效航向及精度来源 | L7 + L9 | NMEA `sacc=0` 时 GSF 不启用；实机航向未核实 |
| HyperLte / MAVLink 带宽 | L0 + Unit + L4 | L7 + L9 | 代码/合同存在，未在本轮复验 |
| Swarm 两端协议 | L3/Mock + L4 | L7 → L8 → L9，按 1/2/6 机升级 | `IMPLEMENTED_UNVERIFIED` |
| FTC RLS estimator | L1 + L3 + L4 + L5 | 只观察时 L7；用于决策前 L9 数据集 | 核心 `HOST_VERIFIED`，集成未验证 |
| FTC fault classification | L3 + L4 + L5 | L7/L9 标定 | `IMPLEMENTED_UNVERIFIED` |
| FTC matrix shadow / authority | L3 + L4 + L5 | L6；只观察台架 L7 | `IMPLEMENTED_UNVERIFIED` |
| FTC impact / LOC | L3 + L4 + L5 | L7/L8/L9 分场景标定 | `IMPLEMENTED_UNVERIFIED` |
| FTC recovery candidate | L3 + L4 + L5 | L6 + L8；主动控制需 L9 | `IMPLEMENTED_UNVERIFIED`，主动路径默认关闭 |
| FTC active allocation/recovery | L3 + L4 + L5 + L6 | L7 + L8 + L9 | 门控软件路径已实现、默认关闭；`IMPLEMENTED_UNVERIFIED`，禁止实机启用 |
| 急停后的 rate 积分 | L0 + L2 + L3 + L4，覆盖输出授权时效及恢复 | L7 + L9 | 源码门控已实现，当前仓库未复测 |
| FMUv6C 产品构建容量 | L0 + L2，检查 Flash 与 CPU/栈 | L7 + L9 | default 已排除板内 SIH；新产物及资源数据待构建 |
| SITL FTC injection | L3 + L4 | 不适用实机 | `IMPLEMENTED_UNVERIFIED` |
| ULog topic contract | L2 + L4 + L5 | 相关功能发布前完成 | 源码订阅存在，尚无实际 ULog |
| 自主悬停控制候选（SITL 限定） | L1 + L2 + L4 + L5 | 实机需另行 L6 → L7 → L8 → L9 | Candidate 006 已执行 L1/L2/L4/L5；鲁棒性与安全边界 `FAIL`，禁止实机闭环 |

## 本轮验证边界

历史 Candidate 006 的研究验证与本次飞行可靠性源码变更分属不同版本，不能转用通过状态；[第六轮鲁棒性结果](../research/experiments/round-006/ROBUSTNESS_RESULT.md)与[安全边界结果](../research/experiments/round-006/SAFETY_BOUNDARY_RESULT.md)保留其失败证据。外部 v2 参考包的两次通用 SIH 保持指标也均未通过。当前仓库状态及后续检查见[实施记录](../development/FLIGHT_RELIABILITY_IMPLEMENTATION.md)；HITL、台架和实机测试未执行。
