# 硬件—软件对应关系

本页只写当前仓库或相邻硬件合同能支持的结论。FMUv6C 板型可以支持多种输出与电源器件；没有装机清单证据时，不把“驱动已编译”写成“实机已安装”。

| 硬件 | 接口/配置 | 软件入口与数据去向 | 证据与限制 |
| --- | --- | --- | --- |
| Pixhawk 6C Mini V6C22 | FMUv6C，`HW type V6C002002` | `boards/px4/fmu-v6c` → NuttX → PX4 modules | 当前固件产品目标；`V6C22` 定义在 `board_config.h` |
| STM32H743 | NuttX `CONFIG_ARCH_CHIP_STM32H743VI=y` | NuttX BSP、work queues、drivers | MCU 配置证据来自板级 defconfig |
| BMI088 | SPI1；gyro PC14/PE5，accel PC15/PE4；启动 rotation 4 | `src/drivers/imu/bosch/bmi088` → sensor accel/gyro → sensors/EKF2/FTC | V6C02/V6C22 启用；其他旧变体走 BMI055 |
| ICM-42688-P | SPI1；PC13/PE6；启动 rotation 6 | `src/drivers/imu/invensense/icm42688p` → 第二套 IMU → sensors/EKF2 | 与 BMI088 构成惯性冗余 |
| IST8310 | 内部 I2C4 `0x0c` | `src/drivers/magnetometer/isentek/ist8310` → vehicle magnetometer → EKF2 | I2C4 为兼容原因仍可能标为 external |
| MS5611 | 内部 I2C4 `0x77` | `src/drivers/barometer/ms5611` → vehicle air data → EKF2 | 板级启动脚本明确地址 |
| Hyper982 / UM982 类接收机 | UART1→GPS1 `/dev/ttyS0`，230400 8N1，NMEA | `src/drivers/gps` → `sensor_gps` → vehicle GPS → EKF2/MAVLink | 项目合同名为 Hyper982；源码没有专用 Hyper982 驱动，使用通用 NMEA。不能仅凭名字断言所有 UM982 配置等同 |
| 双天线 GNSS | `EKF2_GPS_CTRL=15`、`GPS_YAW_OFFSET=90` | GPS yaw → EKF2 attitude | 90° 只适用于主天线右、从天线左 |
| HyperLte 4G | UART1→TELEM1 `/dev/ttyS5`，57600 8N1，无 RTS/CTS | MAVLink Normal → serial passthrough → 模块 TCP/network → GroundStation | 飞控侧不是 TCP endpoint；57600 物理上限 5760 B/s，自动 tx rate 2880 B/s |
| ESC / motor | 由配置选择 PWM、DShot 等输出；`actuator_motors` | control allocator → mixer/output driver → ESC → motor；反馈可通过 `esc_status` 供 FTC 使用 | 实际 ESC 型号/协议没有统一装机证据；FTC 映射仍需验证 |
| 电池/电源模块 | board ADC、SMBus、INA226/228/238 驱动被目标启用 | `battery_status`、system power、Commander 和 logger | 启用表示软件支持，不证明某一 INA/SMBus 器件已安装 |
| PX4IO | `SYS_USE_IO=1`、FMUv6C IO 配置 | 输出/安全 IO 边界 | 修改需同时核对 bootloader、IO firmware 和输出测试 |

## 相邻 HardwareFMUv6C 仓库

`E:\MERIVUS\HardwareFMUv6C` 当前硬件合同面向完整 FMUv6C V6C02，预期 `V6C000002`、Board ID 56、BMI088 + ICM-42688-P。它与本仓主要目标 Pixhawk 6C Mini V6C22 的传感器组合相似，但 HW type 不同。硬件仓 ADR 还指出官方 v1.14.4 不含 V6C02/BMI088 原生支持，本地 v1.14 派生树属于回移植兼容；因此升级或自研板投产必须单独跑 firmware contract，不能用“固件能刷入”代替总线/引脚/电源兼容验证。
