# MERIVUS FirmwarePX4

MERIVUS 是一个基于 PX4 v1.14 源码快照的多无人机飞控项目，主要面向 Holybro Pixhawk 6C Mini（FMUv6C/V6C22）和 PX4 SITL。仓库目前包含 FMUv6C 适配、Hyper982 GNSS/RTK 与 HyperLte 4G 配置、`swarm_node` 编队协议，以及默认关闭的极端环境/容错控制（FTC）研究链。

> 本仓库不是 PX4 官方发行版。产品历史从源码快照重新初始化，未保留精确的上游基线提交；上游和第三方许可证仍按原文件执行。

维护者：**[Ale-xl](https://github.com/Ale-xl)**。产品历史与作者归属见 [贡献指南](CONTRIBUTING.md)。

## 当前边界

- 产品固件目标：`px4_fmu-v6c_default`
- 仿真目标：`px4_sitl_default`
- 构建环境：Ubuntu 22.04，或已经配好 PX4 v1.14 工具链的 Ubuntu；Windows 负责编辑、Git、GitNexus 和 QGroundControl 刷写
- 编队：协议版本 `2`，支持包含 UAV-1 的单机、双机和六机分阶段验证
- FTC：已实现观察、Shadow、恢复候选以及默认关闭的主动分配/恢复路径；Active 状态为 `IMPLEMENTED_UNVERIFIED`
- 验证状态：已有 Host、SITL、构建和遥测链路证据；Estimator 尚未稳定建立真实 SITL Baseline，未进入故障注入或 Active 验证

## 系统概览

```text
Sensors -> EKF2 -> position/attitude/rate control -> Control Allocator -> ESC/motors
   |                                                   |
   +---------------- FTC observe/shadow ---------------+--> ULog

Hyper982 -> GPS1/NMEA -> EKF2/MAVLink -> TELEM1/HyperLte -> GroundStation

GroundStation -> MAVLink PREPARE/COMMIT/RELEASE/ABORT -> swarm_node
              <- ACK/GPS/FOLLOW_TARGET session lease  -> Offboard setpoints
```

完整模块、来源和数据流从 [MERIVUS 文档门户](docs/README.md) 进入：

- [系统地图](docs/architecture/SYSTEM_MAP.md)
- [代码来源地图](docs/architecture/CODE_OWNERSHIP_MAP.md)
- [关键数据流](docs/architecture/DATA_FLOWS.md)
- [FTC 架构与真实接管边界](docs/extreme_control/FTC_ARCHITECTURE.md)
- [当前模块状态](docs/PROJECT_STATUS.md)

## 快速构建

```bash
git clone --recursive https://github.com/Ale-xl/MERIVUS-FirmwarePX4.git FirmwarePX4
cd FirmwarePX4
make px4_fmu-v6c_default
```

固件产物：

```text
build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
```

首次环境准备、子模块、SHA-256、Windows/Ubuntu 分工和 QGroundControl 刷写见 [构建与刷写](docs/development/BUILD_AND_FLASH.md)。

## 主要产品能力

- FMUv6C V6C22：BMI088 + ICM-42688-P、IST8310、MS5611 的板级识别和启动合同。
- GNSS/通信：Hyper982 通过 GPS1 230400 8N1/NMEA 接入；HyperLte 通过 TELEM1 57600 8N1 透传 MAVLink Normal。
- 编队：GroundStation 和 `swarm_node` 共同实现 PREPARE → COMMIT → RELEASE 事务，失败或超时进入 ABORT/Hold。
- FTC：电机健康与模型观察、动态效能矩阵 shadow、剩余控制能力、impact/LOC、恢复候选和 Supervisor。

## 安全说明

- `FTC_` 检测入口默认关闭；`FTC_CA_EN=0`、`FTC_REC_ACT=0`、`FTC_SIM_EN=0`。主动路径已有实现但未经闭环验证。
- 编队 ABORT 请求 Hold/Loiter，不代表飞机已经安全降落。
- 自定义固件首次上电和输出测试必须拆桨或采取等效防护；实机测试需要现场授权、急停和逐级验证。
- 构建和部署必须记录主仓/子模块提交、工具链、构建目标及产物 SHA-256，不得在部署机直接改产物。

配套地面站：[Ale-xl/MERIVUS-GroundStation](https://github.com/Ale-xl/MERIVUS-GroundStation)。许可证见 [LICENSE](LICENSE)。
