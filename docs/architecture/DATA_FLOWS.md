# 关键数据流

箭头表示源码能够确认的数据或控制关系；`~>` 表示跨越 uORB、调度器、串口或仓库边界，不是普通函数调用。GitNexus 能识别常规调用关系，遇到这些运行时边界时，还要结合 publisher/subscriber、启动脚本和协议字段判断。

## Flow 1：飞行控制与执行器

```text
BMI088 / ICM-42688-P
  -> sensor_* drivers
  ~> sensor_combined / vehicle_imu
  ~> EKF2::Run
  ~> vehicle_attitude / vehicle_local_position
  ~> MulticopterAttitudeControl::Run
  ~> vehicle_rates_setpoint
  ~> MulticopterRateControl::Run
  ~> vehicle_torque_setpoint + vehicle_thrust_setpoint
  ~> ControlAllocator::Run
  -> allocation method / effectiveness matrix
  ~> actuator_motors
  ~> mixer_module::MixingOutput
  -> PWMOut::updateOutputs or selected output driver
  -> PWM / DShot -> ESC -> motor
```

关键路径：`src/modules/sensors`、`ekf2`、`mc_att_control`、`mc_rate_control`、`control_allocator`、`src/lib/mixer_module`、`src/drivers/pwm_out`。FTC 没有替换这条主链。

## Flow 2：电机健康与模型观察

```text
actuator_motors
  + vehicle_angular_velocity.xyz_derivative
  + vehicle_acceleration
  + vehicle_status / vehicle_land_detected
  + optional esc_status
  ~> MotorHealthMonitor::Run
  -> MotorEffectivenessEstimator::update
  -> classifyFaults
  ~> motor_health_status
  ~> ftc_model_status
  ~> logger
```

估计更新受解锁、离地、数据新鲜度、最小推力和激励门限制。`ftc_model_status` 中效能可有效；质量、在线惯量和 CG 仍无有效估计，不能传播到 PX4 控制参数。

## Flow 3：动态矩阵、shadow 分配与权限

```text
ActuatorEffectiveness::getEffectivenessMatrix
  -> ControlAllocator::update_effectiveness_matrix_if_needed
  -> publish_ftc_effectiveness_matrix [FTC_CA_SHADOW=1]
  ~> ftc_effectiveness_matrix (B_nominal, trim, limits)

motor_health_status.lambda + B_nominal
  ~> FtcControlMonitor::updateMatrix
  -> B_dynamic(:, i) = B_nominal(:, i) * lambda_i
  -> ControlAllocationSequentialDesaturation [shadow instance]
  -> FtcControlMonitor::calculateShadow
  ~> ftc_allocation_shadow (nominal/candidate/residual/saturation)
  ~> ftc_control_authority
```

候选电机值只进入 `ftc_allocation_shadow`，没有发布到 `actuator_motors`。`FTC_CA_EN` 是保留参数，不存在主动切换 allocator 的实现。

## Flow 4：极端状态与恢复候选

```text
vehicle_acceleration + vehicle_angular_velocity
  + attitude / attitude_setpoint / rates_setpoint
  + vehicle_local_position + actuator_motors
  + control_allocator_status
  + ftc_control_authority + motor_health_status
  ~> FtcExtremeStateMonitor::Run
  ~> ftc_extreme_state (impact / hard landing / LOC)

ftc_extreme_state + ftc_control_authority + motor_health_status
  + attitude / rates / altitude / armed / landed
  ~> FtcRecovery::Run
  -> transition -> generateCandidate
  ~> ftc_recovery_status
  ~> FtcSupervisor::Run
  ~> ftc_system_status -> logger
```

恢复模块只发布 body rate、body thrust 和 attitude 候选字段。`FTC_REC_ACT=1` 不会把这些候选写入正常 setpoint、Commander 或执行器链。

## Flow 5：Hyper982 GNSS / RTK 到地面站

```text
Hyper982 UART1 @ 230400 8N1
  -> GPS1 /dev/ttyS0
  -> PX4 gps driver, GPS_1_PROTOCOL=6 (NMEA)
  ~> sensor_gps
  ~> vehicle_gps_position
  ~> EKF2 GNSS fusion
  ~> vehicle_global_position / vehicle_local_position
  ~> MAVLink GPS_RAW_INT / GLOBAL_POSITION_INT / GPS_STATUS
  -> TELEM1 /dev/ttyS5 @ 57600 8N1
  -> HyperLte serial passthrough -> TCP/network -> GroundStation
```

`EKF2_GPS_CTRL=15` 和 `GPS_YAW_OFFSET=90` 是当前产品配置合同；90° 只适用于主天线在右、从天线在左的安装。源码没有 Hyper982 专用协议驱动，接收机通过通用 NMEA 接口接入。

## Flow 6：编队事务与位置租约

```text
GroundStation SwarmController
  -> MAV_CMD_USER_1/2/3/4 (protocol=2, member mask, leader=1, session)
  -> MAVLinkReceiver::handle_message_command_both
  ~> swarm_command
  ~> SwarmNode::handleSwarmCommand
  -> PREPARE / COMMIT / RELEASE / ABORT state handlers
  ~> vehicle_command_ack -> MAVLink -> GroundStation

UAV-1 GPS_RAW_INT -> GroundStation
  -> FOLLOW_TARGET with source-bound session lease
  -> MAVLink receiver ~> follow_target
  -> SwarmNode::updateFollowTarget / projectFollowerTarget
  ~> offboard_control_mode + trajectory_setpoint
  ~> vehicle_command (Offboard / arm / AUTO_LOITER)
```

`swarm_node` 由 `ROMFS/px4fmu_common/init.d/rc.mc_apps` 启动。FOLLOW_TARGET 超过 3 秒、来源/会话不匹配或位置无效会触发退出到 Hold/Loiter；Hold/Loiter 不等于自动降落。

## Flow 7：FTC 状态到 GroundStation

```text
motor_health_status + ftc_model_status + ftc_system_status
  ~> MERIVUS_FTC_MOTOR_STATUS @ 5 Hz

ftc_control_authority + ftc_effectiveness_matrix
  + ftc_allocation_shadow + ftc_recovery_status + ftc_system_status
  ~> MERIVUS_FTC_CONTROL_STATUS @ 5 Hz

ftc_extreme_state + ftc_recovery_status
  ~> MERIVUS_FTC_EXTREME_STATUS @ 10 Hz

motor/model/shadow/extreme/system + ftc_simulation_status
  ~> MERIVUS_FTC_DIAGNOSTICS @ 1 Hz

PX4 MAVLink scheduler ~> serial/UDP/TCP ~> GroundStation VehicleFtcStatusFactGroup
```

这条链是只读传输。它没有反向命令，不连接 Commander、setpoint 或 `actuator_motors`。详细字段、带宽和模式边界见 [FTC MAVLink 契约](../reference/FTC_MAVLINK_CONTRACT.md)。
