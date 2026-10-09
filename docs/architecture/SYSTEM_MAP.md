# MERIVUS 系统地图

## 范围与证据

本图以 `E:\MERIVUS\FirmwarePX4` 为主仓库，GroundStation 与 HardwareFMUv6C 作为相邻系统边界。判断依据包括当前源码、板级配置、uORB 声明、启动脚本、Git 历史，以及提交 `294b2c60fc8f64007a84553bbf3c8106d9ef5dc9` 的 GitNexus 索引。该索引包含 830,568 个符号节点、1,033,775 条关系和 942 条流程。PX4 调度器、uORB 和串口属于运行时边界，所以相关连接还要与源码互相印证。

```text
MERIVUS
├── Flight Core
│   ├── Sensors / sensor processing
│   ├── EKF2 estimator
│   ├── Position / attitude / rate control
│   ├── Control Allocator
│   └── Mixer / output drivers / Commander safety
├── Hardware
│   ├── FMUv6C / Pixhawk 6C Mini V6C22
│   ├── BMI088 + ICM-42688-P + IST8310 + MS5611
│   └── GPS1 / TELEM1 / power / PWM or DShot
├── Communication & Navigation
│   ├── Hyper982 NMEA GNSS / RTK
│   ├── MAVLink
│   └── HyperLte 4G serial-to-network link
├── Swarm
│   ├── GroundStation SwarmController
│   ├── MAV_CMD_USER_1..4 + FOLLOW_TARGET
│   └── onboard swarm_node
├── FTC / Extreme Control
│   ├── motor health and model observation
│   ├── allocator matrix shadow and authority
│   ├── impact / loss-of-control observation
│   ├── disconnected recovery candidate
│   └── supervisor and logging
├── Simulation
│   ├── PX4 SITL
│   ├── simulator_mavlink FTC injection
│   └── SITL 限定悬停候选研究入口与真值评估
└── Tooling
    ├── CMake / Make / NuttX / arm-none-eabi
    ├── Git submodules and GitNexus
    └── GitHub Actions / metadata artifacts
```

## 一级系统

| 系统 | 职责与入口 | 主要路径 | 上下游 | 来源 | 成熟度 |
| --- | --- | --- | --- | --- | --- |
| Flight Core | 状态估计、位置/姿态/角速度控制、控制分配、安全状态 | `src/modules/ekf2`、`mc_*_control`、`control_allocator`、`commander`、`flight_mode_manager` | 传感器输入；向执行器输出 | 以 `PX4_UPSTREAM` 为主；allocator 有 FTC hook，mc_pos_control 有 SITL 限定研究入口 | PX4 成熟核心；研究入口尚无闭环验证 |
| Sensor processing | 驱动数据选择、校准并生成 `sensor_combined`、`vehicle_imu` 等 | `src/drivers`、`src/modules/sensors` | 硬件→EKF2、FTC | PX4 原生 + FMUv6C 板级适配 | 板级改动已实现，未在本轮实机复验 |
| FMUv6C BSP | V6C22 识别、总线映射、传感器启动、串口和产品默认参数 | `boards/px4/fmu-v6c` | 硬件→驱动、构建目标 | `BOARD_SPECIFIC`、`MERIVUS_MODIFIED_PX4` | `IMPLEMENTED_UNVERIFIED` |
| GNSS / RTK | Hyper982 通过 NMEA 输入定位与双天线航向配置 | GPS1、`src/drivers/gps`、`src/modules/sensors/vehicle_gps_position`、EKF2 | GNSS→EKF2/MAVLink | PX4 通用 GPS 栈 + MERIVUS 默认参数 | 配置合同已建立；本轮未复验设备 |
| MAVLink / 4G | uORB 与地面站之间的命令、状态和遥测；HyperLte 透传 TELEM1 | `src/modules/mavlink`、`boards/.../rc.board_defaults` | 飞控↔GroundStation | PX4 原生 + MERIVUS 带宽/遥测修订 | `IMPLEMENTED_UNVERIFIED` |
| Swarm | 1/2/6 机 PREPARE→COMMIT→RELEASE 事务、ABORT 与位置租约 | `src/modules/swarm_node`、`msg/SwarmCommand.msg`、相邻仓库 `GroundStation/custom/src/Swarm/SwarmController.*` | MAVLink↔swarm_node→Offboard/Commander | `MERIVUS_CUSTOM`，导入前来源 SHA 不完整 | `IMPLEMENTED_UNVERIFIED`；Mock/SITL 阶段 |
| FTC | 观察电机效能、故障、权限和极端状态，生成恢复候选，并经门控核心接口实现主动分配/恢复 | `src/modules/motor_health_monitor`、`src/modules/ftc_*` | 控制/传感 uORB→FTC topics→logger | `EXPERIMENTAL`、`MERIVUS_CUSTOM`；allocator/simulator/logger 为集中修改 | 见 [FTC 架构](../extreme_control/FTC_ARCHITECTURE.md) |
| Logging | 记录 PX4 与可选 FTC uORB topic | `src/modules/logger` | 全栈→ULog→离线分析 | PX4 原生 + FTC topic 列表修改 | topic 已接入源码，尚无实际 ULog 证据 |
| Simulation | SITL 传感器/执行器桥接与 FTC 电机效能注入 | `boards/px4/sitl`、`src/modules/simulation/simulator_mavlink` | 仿真器↔uORB | PX4 原生 + `EXPERIMENTAL` hook | 注入实现未完成 SITL 验证 |
| Build / CI | 固件、SITL、元数据和可追溯产物 | `CMakeLists.txt`、`Tools`、`.github/workflows`、`Documentation/merivus` | 源码→产物→QGC | PX4 工具链 + MERIVUS CI | 构建流程有合同；本轮未构建 |

## 跨仓库边界

- `GroundStation` 通过 MAVLink 命令、ACK、`GPS_RAW_INT` 与 `FOLLOW_TARGET` 和本固件交互；它不是 FirmwarePX4 子目录，也未纳入本次 GitNexus 单仓图。
- `HardwareFMUv6C` 保存 V6C02 硬件合同与 Altium 工程；本固件同时面向 Pixhawk 6C Mini V6C22。V6C02 与 V6C22 标识不可混用。
- 三个目录是独立 Git 仓库。本文档变更只提交到 `FirmwarePX4`。
