# MERIVUS 项目状态

## 状态词

`DESIGN` → `SKELETON` → `IMPLEMENTED_UNVERIFIED` → `HOST_VERIFIED` → `BUILD_VERIFIED` → `SITL_VERIFIED` → `HITL_VERIFIED` → `BENCH_VERIFIED` → `FLIGHT_VERIFIED`。`DEPRECATED` 表示退出维护。状态只描述已有证据，不按代码量自动升级。

## 当前模块

| 模块/能力 | 状态 | 已有证据 | 仍缺 |
| --- | --- | --- | --- |
| FMUv6C/V6C22 BMI088 适配 | `IMPLEMENTED_UNVERIFIED` | BSP、HW type、SPI map、启动分流和验收合同 | 当前提交的 L2/L7/L9 |
| Hyper982 NMEA / RTK 配置 | `IMPLEMENTED_UNVERIFIED` | board defaults 与配置合同 | 当前设备的 bench/flight 证据 |
| HyperLte / TELEM1 MAVLink | `IMPLEMENTED_UNVERIFIED` | 57600/Normal/带宽合同与代码修订 | 当前链路 bench/flight 证据 |
| `swarm_node` | `IMPLEMENTED_UNVERIFIED` | 协议、状态机、startup 和 GroundStation 两端源码 | 本分支 L2/L4 及 1/2/6 机逐级验证 |
| FTC 估计、诊断、Shadow、方向权限、恢复、Supervisor | `IMPLEMENTED_UNVERIFIED` | 代码路径、Host 测试、7 次正常 SITL 与遥测链路 | 稳定 Estimator 准入、故障阳性与目标验证 |
| 主动分配 / 恢复仲裁 | `IMPLEMENTED_UNVERIFIED` | 唯一 owner、门控、限速、回退、重入 | 所有 ACTIVE 默认关闭；后续 SITL/HITL/台架 |
| Mass observe-only | `IMPLEMENTED_UNVERIFIED` | 独立推力尺度门、主机测试 | 尺度与模型标定；默认不可观测 |
| Inertia / CG 在线估计 | `NOT_IMPLEMENTED` | 明确无效的接口、状态和日志 | 可观测性模型与算法 |
| FTC MAVLink / GroundStation | `SITL_VERIFIED`（后端链路） | v2 生成/契约/构建、实际 VM—Windows 解码、stale 与恢复 | 最终 UI 视觉复核、目标链路带宽 |
| GitNexus | `PARTIAL/UNKNOWN` | 调用影响和差异分析已尝试 | 当前索引缺少运行时边，不能据零 caller 判安全 |

当前技术契约见 [FTC 架构](extreme_control/FTC_ARCHITECTURE.md)，测试与构建证据见 [验证总结](testing/FTC_VALIDATION_SUMMARY.md)。Active Allocation 与 Active Recovery 均保持 `IMPLEMENTED_UNVERIFIED`，默认关闭。
