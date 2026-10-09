# FTC uORB 与内部消息索引

## FTC 消息

所有 `.msg` 位于 `msg/`，并由 `msg/CMakeLists.txt` 进入 uORB 生成。下表中的 logger 订阅来自 `src/modules/logger/logged_topics.cpp`。FTC topic 使用固定订阅：即使 logger 启动时发布者尚未出现，也会保留并周期重试，避免晚启动模块的诊断缺失。

| Topic | Publisher | Subscriber | 用途与关键字段 | 日志 | 进入主控制 |
| --- | --- | --- | --- | --- | --- |
| `motor_health_status` | `motor_health_monitor` | `ftc_control_monitor`、`ftc_extreme_state_monitor`、`ftc_recovery`、`ftc_supervisor` | `effectiveness/health/confidence`、fault、mask、residual | 20 ms | 否 |
| `ftc_model_status` | `motor_health_monitor` | `ftc_supervisor`、`control_allocator` | 基线、可观测性、不确定度/年龄、模型有效位 | 20 ms | 门控动态分配 |
| `ftc_effectiveness_matrix` | `control_allocator` | `ftc_control_monitor`、`motor_health_monitor` | `B_nominal`、trim、linearization point、limits | 1000 ms | 只读来自主链；不反馈 |
| `ftc_allocation_shadow` | `ftc_control_monitor` | 无控制订阅者 | nominal/candidate motor、六轴 residual、saturation | 20 ms | 否 |
| `ftc_control_authority` | `ftc_control_monitor` | `ftc_extreme_state_monitor`、`ftc_recovery`、`ftc_supervisor` | roll/pitch/yaw/thrust authority、headroom、状态 | 20 ms | 否 |
| `ftc_extreme_state` | `ftc_extreme_state_monitor` | `ftc_recovery`、`ftc_supervisor` | impact/hard landing、LOC、score、reason | 20 ms | 否 |
| `ftc_recovery_status` | `ftc_recovery` | `ftc_supervisor`、`FtcRateInput` | 状态、候选、重入权重、回退 | 20 ms | 唯一 rate 输入仲裁 |
| `ftc_system_status` | `ftc_supervisor` | 无控制订阅者 | 顶层状态、有效位、reason、confidence | 100 ms | 否 |
| `ftc_simulation_status` | `simulator_mavlink` | 无运行时订阅者 | SITL 注入电机、目标/实际效能、间歇状态 | 20 ms | 仅仿真输出路径 |

| `ftc_allocation_status` | `control_allocator` | Supervisor、MAVLink | 实际应用 λ、active、fallback | 20 ms | 控制结果反馈 |
| `ftc_arbitration_status` | `FtcRateInput` | recovery、Supervisor、MAVLink | 实际权重和选定 rate/thrust | 20 ms | 控制结果反馈 |

## Swarm 消息与使用的 PX4 topic

| Topic | Publisher | Subscriber | 用途 | 进入控制 |
| --- | --- | --- | --- | --- |
| `swarm_command` | `MavlinkReceiver` | `swarm_node` | USER_1..4 转换后的 action、protocol、leader、source、member mask、session | 是，经过 swarm 状态机和安全门 |
| `follow_target` | `MavlinkReceiver` | `swarm_node` | GroundStation 转发的 UAV-1 全球位置和 `custom_state` 会话租约 | 是，生成跟随目标 |
| `offboard_control_mode` | `swarm_node` | PX4 Offboard/flight task 路径 | 声明 position setpoint 控制模式 | 是 |
| `trajectory_setpoint` | `swarm_node` | PX4 位置控制路径 | 起飞、主机轨迹和从机目标 | 是 |
| `vehicle_command` | `swarm_node` | Commander | 请求 Offboard、解锁或 AUTO_LOITER | 是 |
| `vehicle_command_ack` | `swarm_node` | MAVLink | PREPARE/COMMIT/RELEASE/ABORT 进度与结果 | 否；外部反馈 |

`SwarmCommand.msg` 是 MERIVUS 内部消息。`FOLLOW_TARGET`、`vehicle_command` 等是 PX4/MAVLink 既有消息，但本项目使用了 `FollowTarget.custom_state` 绑定 MERIVUS session。

## 消息变更检查

`ftc_recovery_status.fallback_reason` 的语义位 `1u << 4`（16）表示恢复中垂直状态重置计数变化或位置时间倒退，候选终止且 rate 仲裁硬退出。未修改该 topic 布局或 MAVLink 布局；它不是 `ftc_arbitration_status.fallback_reason` 的位定义。当前源码状态见[实施记录](../development/FLIGHT_RELIABILITY_IMPLEMENTATION.md)。

修改上述消息前至少检查：

1. `msg/CMakeLists.txt` 与生成结果；
2. 所有 publication/subscription 和数组长度；
3. logger topic 与 ULog 带宽；
4. MAVLink/GroundStation 的字段语义；
5. 旧日志、SITL 工具和参数元数据兼容性；
6. 是否意外新增 `actuator_motors` 或正常 setpoint writer。
