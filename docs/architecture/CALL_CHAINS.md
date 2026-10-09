# 关键调用链

本页只列出工程决策需要关注的入口和出口。链中的 `uORB`、MAVLink 和工作队列都是事件边界，不代表同一线程里的直接函数调用。

## 1. PX4 多旋翼主控制链

`sensor driver` → `sensors` → `EKF2::Run` → `MulticopterAttitudeControl::Run` → `MulticopterRateControl::Run` → `ControlAllocator::Run` → `publish_actuator_controls` → `MixingOutput::update` → `PWMOut::updateOutputs`

- 状态/数据：IMU、姿态、rate setpoint、torque/thrust、motor control、物理输出。
- 风险：这是实时飞行主链；GitNexus 对 `Run()` 的上游结果可能为 `UNKNOWN`，必须按 uORB 与调度关系验证。

## 2. allocator 名义矩阵导出

`ControlAllocator::Run` → `update_effectiveness_matrix_if_needed` → `ActuatorEffectiveness::getEffectivenessMatrix` → `ControlAllocation::setEffectivenessMatrix` → `publish_ftc_effectiveness_matrix`

- 出口：`ftc_effectiveness_matrix`，仅在 `FTC_CA_SHADOW=1` 时发布。
- GitNexus：`publish_ftc_effectiveness_matrix` 的直接 caller 已解析为 `update_effectiveness_matrix_if_needed`；后者上游影响 4 个 allocator 符号，风险为 `LOW`，但 uORB 下游不在普通调用计数内。

## 3. 电机效能与故障分类

`MotorHealthMonitor::Run` → `motorCount` / 数据门控 → `MotorEffectivenessEstimator::update` → `classifyFaults` → `publishModelStatus` → uORB/logger

- 出口：`motor_health_status`、`ftc_model_status`。
- 风险：参数、ESC 映射、低激励门和时间戳会同时改变诊断结论；`Run()` 的 GitNexus caller 为 `UNKNOWN`，实际入口来自工作队列调度。

## 4. FTC shadow 与控制权限

`FtcControlMonitor::Run` → `updateMatrix` → `calculateShadow` → `ControlAllocationSequentialDesaturation::allocate` → `axisAuthority` → uORB/logger

- 出口：`ftc_allocation_shadow`、`ftc_control_authority`。
- GitNexus：确认 `Run` 直接调用 `calculateShadow`，后者调用 `axisAuthority`；没有真实执行器出口。

## 5. 极端状态检测

`FtcExtremeStateMonitor::Run` → `quaternionError` → impact/hard-landing/LOC score → state debounce → `ftc_extreme_state`

- 输入：加速度、角速度、姿态误差、rate 误差、local position、allocator 状态、权限和电机健康。
- 风险：阈值未经 ULog/SITL 标定，正常激烈机动、硬着陆和传感器毛刺可能互相混淆。

## 6. 恢复候选

`FtcRecovery::Run` → eligibility/inhibit gates → `transition` → `generateCandidate` → `calculateLevelQuaternion` → `ftc_recovery_status`

- 状态：RATE_DAMPING、THRUST_VECTOR_RECOVERY、ATTITUDE_RECOVERY、ALTITUDE_STABILIZATION、CONTROL_REENTRY、EMERGENCY_LAND 等。
- 风险：候选经 FtcRateInput 唯一仲裁接入 rate 输入；模式/failsafe 只读门与超时回退已实现，需端到端验证。

## 7. FTC 顶层状态

`FtcSupervisor::Run` → 读取五类 FTC status → freshness/validity/reason 聚合 → `ftc_system_status` → logger

- 输入：motor health、model、authority、extreme state、recovery。
- 风险：Supervisor 只聚合事实，不拥有模式或执行器权限；不能把状态 `RECOVERY_ACTIVE` 当作真实控制已接管。

## 8. MAVLink 编队命令

`MavlinkReceiver::handle_message_command_both` → 校验 USER command 参数 → 发布 `swarm_command` → `SwarmNode::handleSwarmCommand` → `handlePrepareCommand` / `handleCommitCommand` / `handleReleaseCommand` / `handleAbortCommand` → `publishCommandAck`

- GitNexus：MAVLink handler 上游影响 4 个符号，并命中 `handle_message` 流程；跨 uORB 到 `swarm_node` 的边由源码确认。
- 风险：协议版本、24 位 session、member mask、source system/component 必须保持两端一致。

## 9. 编队控制循环

`SwarmNode::Run` → `handleSwarmCommand` → `updateFollowTarget` → `requestOffboard` / `requestArm` → `controlPosition` → `publishPositionSetpoint`; fault/timeout → `beginExitToHold` → `requestAutoHold`

- 出口：`offboard_control_mode`、`trajectory_setpoint`、`vehicle_command`、`vehicle_command_ack`。
- 风险：`SwarmNode::Run` 在 GitNexus 中无解析 caller，结果为 `UNKNOWN`；真实入口是 `swarm_node start` 与工作队列调度。

## 10. SITL 故障注入

`SimulatorMavlink::send_controls` → FTC injection gate/ramp/intermittent logic → 修改发往仿真器的目标电机通道 → 发布 `ftc_simulation_status`

- 入口参数：`FTC_SIM_EN/MOT/EFF/RAMP/INT`。
- 风险：仅用于 SITL。`FTC_SIM_EN=0` 时不得改变仿真控制；该机制不是实机故障注入接口。
