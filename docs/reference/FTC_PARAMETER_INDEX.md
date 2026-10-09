# FTC 参数索引

本页只整理 MERIVUS 新增或修改的产品参数，不复制 PX4 全量参数库。定义与默认值以源码为准；已经保存到设备的旧参数不会因刷写自动改成板级默认值。

风险：`LOW` 为观察/显示或正常配置，`MEDIUM` 会改变估计、通信或仿真行为，`HIGH` 会改变飞行控制、模式或执行器相关行为。当前保留但无执行路径的参数仍按未来含义标注风险。

## FTC 参数

42 个参数定义在 `src/lib/parameters/ftc_shared_params.c`（5 个共享开关）、`src/modules/motor_health_monitor/motor_health_monitor_params.c`（32 个）和 `src/modules/simulation/simulator_mavlink/simulator_mavlink_params.c`（5 个）。共享开关纳入全局参数扫描，使编入 `control_allocator` 或 `mc_rate_control` 的板型都生成其引用的参数；监控模块只在 FMUv6C 和 SITL 产品配置中启用。主要读取位置是对应模块头文件的 `DEFINE_PARAMETERS`。所有控制/检测/注入入口默认关闭。

| 参数 | 系统/作用 | 默认值 | 改变控制 | 实验 | 风险 |
| --- | --- | ---: | --- | --- | --- |
| `FTC_MON_EN` | FTC 观察层总开关 | `0` | 否 | 是 | MEDIUM |
| `FTC_MIN_THR` | 估计所需最小平均电机指令 | `0.15` | 否 | 是 | MEDIUM |
| `FTC_LPF_TC` | 估计输入低通时间常数 | `0.20` | 否 | 是 | MEDIUM |
| `FTC_EXC_MIN` | RLS 最小激励 | `0.025` | 否 | 是 | MEDIUM |
| `FTC_BASE_T` | 健康基线学习时间 | `5.0` | 否 | 是 | MEDIUM |
| `FTC_EST_RATE` | 效能 `lambda` 最大变化率 | `0.30` | 否 | 是 | MEDIUM |
| `FTC_RES_THR` | 模型残差有效门限 | `1.0` | 否 | 是 | MEDIUM |
| `FTC_HLTH_MIN` | 电机退化门限 | `0.70` | 否 | 是 | MEDIUM |
| `FTC_FAIL_MIN` | 电机失效门限 | `0.25` | 否 | 是 | MEDIUM |
| `FTC_CONF_MIN` | 最小估计置信度 | `0.60` | 否 | 是 | MEDIUM |
| `FTC_FAIL_T` | 故障确认持续时间 | `1.0` | 否 | 是 | MEDIUM |
| `FTC_CA_SHADOW` | 名义矩阵导出与 shadow 分配 | `0` | 否 | 是 | MEDIUM |
| `FTC_EST_FORG` | RLS 遗忘因子 | `0.995` | 否 | 是 | MEDIUM |
| `FTC_EST_LMIN` | 电机效能下界 | `0.10` | 否 | 是 | MEDIUM |
| `FTC_EST_IXX` | 标称滚转惯量近似 | `0.02` | 否 | 是 | MEDIUM |
| `FTC_EST_IYY` | 标称俯仰惯量近似 | `0.02` | 否 | 是 | MEDIUM |
| `FTC_EST_IZZ` | 标称偏航惯量近似 | `0.04` | 否 | 是 | MEDIUM |
| `FTC_FAULT_P` | 故障分类概率门限 | `0.65` | 否 | 是 | MEDIUM |
| `FTC_FAULT_VIB` | 振动归一化门限 | `8.0` | 否 | 是 | MEDIUM |
| `FTC_FAULT_EXT` | 外部扰动门限 | `0.70` | 否 | 是 | MEDIUM |
| `FTC_CA_EN` | 有门控的实验性主动控制分配开关 | `0` | 开启后可能 | 是 | HIGH |
| `FTC_CA_ATT_MIN` | roll/pitch 最低权限 | `0.35` | 否 | 是 | MEDIUM |
| `FTC_CA_YAW_MIN` | yaw 最低权限 | `0.20` | 否 | 是 | MEDIUM |
| `FTC_CA_THR_MIN` | thrust 最低权限 | `0.25` | 否 | 是 | MEDIUM |
| `FTC_IMPACT_EN` | 冲击/硬着陆检测开关 | `0` | 否 | 是 | MEDIUM |
| `FTC_IMPACT_ACC` | 冲击加速度门限 | `30.0` | 否 | 是 | MEDIUM |
| `FTC_IMPACT_JRK` | 冲击 jerk 门限 | `120.0` | 否 | 是 | MEDIUM |
| `FTC_LOC_EN` | 失控检测开关 | `0` | 否 | 是 | MEDIUM |
| `FTC_LOC_THR` | LOC score 门限 | `0.70` | 否 | 是 | MEDIUM |
| `FTC_REC_EN` | 恢复状态机与候选生成开关 | `0` | 否 | 是 | HIGH |
| `FTC_REC_ACT` | 有门控的实验性恢复仲裁开关 | `0` | 开启后可能 | 是 | HIGH |
| `FTC_REC_RATE` | 候选最大角速度 | `2.0` | 否 | 是 | HIGH |
| `FTC_REC_KD` | 候选角速度阻尼增益 | `0.8` | 否 | 是 | HIGH |
| `FTC_REC_ALT` | 恢复最低高度 | `3.0` | 否 | 是 | HIGH |
| `FTC_EST_DELAY` | 指令到角响应的时间对齐 | `0.04` | 否 | 是 | MEDIUM |
| `FTC_EST_AGE` | 效能参数更新最大年龄 | `10.0` | 否 | 是 | MEDIUM |
| `FTC_THR_MAX` | 仅供质量观测的标定总推力 | `0` | 否 | 是 | MEDIUM |
| `FTC_SIM_EN` | SITL 故障注入总开关 | `0` | 仅仿真 | 是 | HIGH |
| `FTC_SIM_MOT` | 注入电机索引 | `1` | 仅仿真 | 是 | HIGH |
| `FTC_SIM_EFF` | 目标电机效能 | `1.0` | 仅仿真 | 是 | HIGH |
| `FTC_SIM_RAMP` | 效能渐变时间 | `0.0` | 仅仿真 | 是 | HIGH |
| `FTC_SIM_INT` | 间歇故障周期 | `0.0` | 仅仿真 | 是 | HIGH |

PX4 参数名最多 16 个字符，因此主动分配和恢复开关使用 `FTC_CA_EN`、`FTC_REC_EN`，不能在文档中改写为更长的别名。

## FMUv6C 产品默认参数

定义位置：`boards/px4/fmu-v6c/init/rc.board_defaults`。

| 参数 | 默认值 | 作用 | 风险/使用位置 |
| --- | ---: | --- | --- |
| `GPS_1_CONFIG` | `201` | GPS1 串口映射 | MEDIUM；PX4 GPS 启动配置 |
| `GPS_1_PROTOCOL` | `6` | 通用 NMEA | MEDIUM；Hyper982 接入合同 |
| `SER_GPS1_BAUD` | `230400` | GPS1 波特率 | MEDIUM；必须与 Hyper982 UART1 一致 |
| `EKF2_HGT_REF` | `1` | GPS 为高度参考 | HIGH；EKF2 |
| `EKF2_GPS_CTRL` | `15` | 位置、高度、速度和双天线航向 | HIGH；EKF2 GNSS 融合 |
| `GPS_YAW_OFFSET` | `90` | 双天线安装航向偏置 | HIGH；仅适用主天线右/从天线左 |
| `MAV_0_CONFIG` | `101` | MAVLink 使用 TELEM1 | MEDIUM；通信实例 |
| `MAV_0_MODE` | `0` | Normal 模式 | MEDIUM；完整 GCS 消息流 |
| `MAV_0_RATE` | `0` | 自动取物理带宽的一半 | MEDIUM；57600 8N1 时为 2880 B/s |
| `MAV_0_FLOW_CTRL` | `0` | 强制关闭流控 | MEDIUM；HyperLte 无 RTS/CTS |
| `MAV_0_FORWARD` | `0` | 不转发 MAVLink | LOW |
| `MAV_0_RADIO_CTL` | `0` | 不使用 radio status 自动限速 | LOW |
| `SER_TEL1_BAUD` | `57600` | TELEM1 波特率 | MEDIUM；必须与 HyperLte UART1 一致 |

`BAT1_V_DIV`、`BAT2_V_DIV`、`BAT1_A_PER_V`、`BAT2_A_PER_V` 和 `SYS_USE_IO` 也由该板脚本设定，但属于既有 FMUv6C 电源/IO 合同，尚无证据表明它们是本次 MERIVUS 参数设计新增项。

## Swarm 参数边界

`swarm_node` 当前没有 `PARAM_DEFINE_*` 产品参数。协议版本 `2`、主机 system ID `1`、成员数 `6`、3 秒租约超时、20 秒阶段超时和轨迹距离等是编译期常量/代码合同。未来若参数化，必须同时修改 GroundStation 协议、固件验证和本索引，不能只改单端。
