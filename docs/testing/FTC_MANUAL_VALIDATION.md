# FTC 人工验证

当前结论见 [验证总结](FTC_VALIDATION_SUMMARY.md)，安全边界见 [FTC 安全与限制](../extreme_control/FTC_SAFETY_AND_LIMITATIONS.md)。本文只规定后续复测顺序和停止条件。

本文用于当前“只观测、影子计算、恢复候选”阶段的人工联调，操作顺序固定为：

```text
Firmware build → SITL → GroundStation → FTC telemetry → fault injection → UI observation
```

本指南不验证、也不允许启用 allocator takeover、Recovery ACTIVE、PID 修改、Commander 自动切换或真实 setpoint 接管。连续效能注入只允许在使用 `simulator_mavlink` 的 jMAVSim/Gazebo Classic 中进行，不得用真实桨叶损伤代替。

## 测试前固定契约

- FTC 协议版本：`1`
- 两仓 `merivus_ftc.xml` SHA-256：`0ae936460c2489f33cebaaba018b2532dd258486d50d1fd835dbc91fc6fc064d`
- GroundStation 已验证的代码/二进制基线：`e4748463a5dd4a59f86e1ba9b22b1404c5fe34f4`
- 该基线的 `MERIVUS.exe` SHA-256：`FFA5CCDDFFE889A9F2EF8EF7873355520BD447BBBCD4CAC978E952AB750DDED1`
- FirmwarePX4 编写本指南时的代码基线：`3984b22216d6b07074e51e9d8ca1ce6bedbec0d0`

文档提交可以位于上述代码基线之后，但不能把旧二进制的构建证据自动套用到新的代码提交。每轮测试都要记录两个仓库的实际 HEAD、递归子模块 SHA、构建参数和产物 SHA-256。

四条消息的生成合同如下：

| ID | 消息 | 负载 | CRC | 默认频率 |
| ---: | --- | ---: | ---: | ---: |
| 60000 | `MERIVUS_FTC_MOTOR_STATUS` | 78 B | 29 | 5 Hz |
| 60001 | `MERIVUS_FTC_CONTROL_STATUS` | 24 B | 153 | 5 Hz |
| 60002 | `MERIVUS_FTC_EXTREME_STATUS` | 38 B | 136 | 10 Hz |
| 60003 | `MERIVUS_FTC_DIAGNOSTICS` | 65 B | 5 | 1 Hz |

百分比字段使用 `uint8_t`：`0..200` 表示 `0..100%`，步长 `0.5%`；`255` 表示不可用；`201..254` 保留。GroundStation 解码结果为 `0..100`，不可用解码为 `-1` 并显示 `N/A`。

## SAFE TEST CONFIGURATION

以下取值来自当前参数定义，不改变控制输出路径：

| 参数 | 安全测试值 | 当前默认值 | 作用与边界 |
| --- | ---: | ---: | --- |
| `FTC_MON_EN` | `1` | `0` | 启用观测模块与系统汇总 |
| `FTC_CA_SHADOW` | Phase 4 为 `0`，Phase 5 起为 `1` | `0` | 允许影子矩阵/分配计算，不发布执行器命令 |
| `FTC_CA_EN` | **`0`** | `0` | 保持主动控制关闭；当前也没有已连接的执行路径 |
| `FTC_IMPACT_EN` | `1` | `0` | 启用撞击检测 |
| `FTC_LOC_EN` | `1` | `0` | 启用 LOC 检测 |
| `FTC_REC_EN` | `1` | `0` | 启用恢复状态机与候选生成 |
| `FTC_REC_ACT` | **`0`** | `0` | 保持候选与控制仲裁断开 |
| `FTC_SIM_EN` | 基线阶段 `0`；仅 Phase 8 临时为 `1` | `0` | 只在兼容 SITL 边界注入效能 |

`FTC_REC_EN=1` 只允许状态机生成候选。当前 `ftc_recovery` 和 `ftc_supervisor` 都把 intervention 明确固定为 disconnected；无论界面处于何种恢复状态，都不等于执行器接管。

建议保留故障分类门限默认值：`FTC_HLTH_MIN=0.70`、`FTC_FAIL_MIN=0.25`、`FTC_CONF_MIN=0.60`、`FTC_FAIL_T=1.0 s`。分类使用估计值和置信度，不直接复制 `FTC_SIM_EFF`，因此不能用注入值机械推断最终故障枚举。

## 模块启动合同

多旋翼启动脚本 `ROMFS/px4fmu_common/init.d/rc.mc_apps` 在启动时发现 `FTC_MON_EN=1`，会按下列顺序自动启动全部五个模块：

```text
motor_health_monitor
ftc_control_monitor
ftc_extreme_state_monitor
ftc_recovery
ftc_supervisor
```

如果参数在本次 SITL 启动后才从 `0` 改为 `1`，启动脚本不会重跑，需要人工启动一次。始终先执行五条 `status`；已经运行时不要再次 `start`。

人工启动顺序：

```sh
motor_health_monitor start
ftc_control_monitor start
ftc_extreme_state_monitor start
ftc_recovery start
ftc_supervisor start
```

状态检查：

```sh
motor_health_monitor status
ftc_control_monitor status
ftc_extreme_state_monitor status
ftc_recovery status
ftc_supervisor status
```

人工停止时使用逆序：

```sh
ftc_supervisor stop
ftc_recovery stop
ftc_extreme_state_monitor stop
ftc_control_monitor stop
motor_health_monitor stop
```

## Firmware → GroundStation 数据闭包

| 观测项 | Firmware uORB 来源 | MAVLink 字段 | GroundStation C++ | QML 显示位置 |
| --- | --- | --- | --- | --- |
| Motor health | `motor_health_status.health[]` | 60000 `health_pct[]` | `ftcStatus.motors` 角色 `health` | 电机卡 `H`；详情“健康”；电机 Tooltip |
| Motor effectiveness | `motor_health_status.effectiveness[]` | 60000 `effectiveness_pct[]` | `motors` 角色 `effectiveness` | 电机卡 `E`；详情“效能”；Tooltip |
| Fault probability | `motor_health_status.fault_probability[]` | 60000 `fault_probability_pct[]` | `motors` 角色 `faultProbability` | 详情“故障概率”；Tooltip |
| Fault confidence | `motor_health_status.confidence[]` | 60000 `confidence_pct[]` | `motors` 角色 `confidence` | 电机 Tooltip“置信度” |
| Fault type | `motor_health_status.fault_type[]` | 60000 `fault_type[]` | `motors` 角色 `faultType`、`faultTypeText` | 详情分类；Tooltip；电机卡严重度边框 |
| Roll authority | `ftc_control_authority.roll_authority` | 60001 `roll_authority_pct` | `ftcStatus.rollAuthority` | 详情“滚转 / 俯仰” |
| Pitch authority | `ftc_control_authority.pitch_authority` | 60001 `pitch_authority_pct` | `ftcStatus.pitchAuthority` | 详情“滚转 / 俯仰” |
| Yaw authority | `ftc_control_authority.yaw_authority` | 60001 `yaw_authority_pct` | `ftcStatus.yawAuthority` | 详情“偏航 / 推力” |
| Thrust authority | `ftc_control_authority.thrust_authority` | 60001 `thrust_authority_pct` | `ftcStatus.thrustAuthority` | 详情“偏航 / 推力” |
| Impact | `ftc_extreme_state.impact_type/impact_score/impact_confidence/impact_severity` | 60002 对应 `impact_*` 字段 | `impactType/Text`、`impactScore`、`impactConfidence`、`impactSeverity` | 降级/事件状态；详情“撞击 / 失控分数” |
| LOC | `ftc_extreme_state.loc_state/loss_of_control_score/reason_mask` | 60002 `loc_state`、`loss_of_control_score_pct`、`loss_of_control_reason_mask` | `locState/Text/Severity`、`lossOfControlScore`、`lossOfControlReasonMask` | LOC 醒目警告；详情分数 |
| Recovery state | `ftc_recovery_status.state/progress/candidate_valid` | 60001/60002 `recovery_state`、`recovery_progress_pct`、candidate flag | `recoveryState/Text/Severity`、`recoveryProgress`、`recoveryCandidate` | Recovery 状态条；详情恢复状态；模式徽标 |
| FTC system state | `ftc_system_status.state` | 60000/60001 `system_state` | `systemState/Text/Severity` | 面板标题、降级警告、详情“系统” |
| Model quality | `ftc_model_status.model_quality` | 60000 `model_quality_pct` | `modelQuality` | 仅详情“模型质量” |
| Shadow / Candidate | `ftc_allocation_shadow.valid`、`ftc_recovery_status.candidate_valid` | 60001 `control_mode` 与 flags | `controlMode/Text/Severity`、`recoveryCandidate` | 模式徽标、详情模式、Candidate 说明 |

低频的 residual、matrix 派生量、mask 和内部置信度不堆放在主飞行页面；诊断摘要只在“详情”展开后出现。固件的 `control_mode` 发送路径只可能产生 `DISABLED/OBSERVE/SHADOW/CANDIDATE`，当前不会产生 `ACTIVE`。

## PHASE 0：确认版本与工作区

FirmwarePX4（Ubuntu）：

```bash
cd ~/src/FirmwarePX4
git status --short --branch
git branch --show-current
git rev-parse HEAD
git log --oneline -10
git submodule status --recursive
sha256sum src/modules/mavlink/message_definitions/v1.0/merivus_ftc.xml
```

GroundStation（Windows PowerShell）：

```powershell
Set-Location E:\MERIVUS\GroundStation
git status --short --branch
git branch --show-current
git rev-parse HEAD
git log --oneline -10
Get-FileHash -Algorithm SHA256 schemas\mavlink\merivus_ftc.xml
Get-FileHash -Algorithm SHA256 build\Desktop_Qt_5_15_2_MSVC2019_64bit-Release\staging\MERIVUS.exe
```

正常：两个工作树没有非预期改动；分支分别是 `research/extreme-control-v1` 与 `research/groundstation-ftc-ui`；两份 XML 哈希相同；若使用已验证二进制，其哈希为本指南顶部值。

异常说明：XML 哈希不同是协议阻断项；子模块不完整会破坏生成与构建复现；HEAD 与二进制来源不一致时，不能宣称测试属于当前源码。

## PHASE 1：编译 Firmware SITL

```bash
cd ~/src/FirmwarePX4
make px4_sitl_default
```

正常：构建完成并生成 `build/px4_sitl_default/bin/px4`；`merivus` 包装方言及 FTC uORB/MAVLink stream 编译、链接通过。

异常说明：若在 MAVLink 生成阶段失败，先核对递归子模块与 XML；若 FTC stream 或 uORB 头缺失，说明生成/构建依赖断链；不要跳过错误改用旧产物。

## PHASE 2：启动兼容的 SITL

本次连续效能注入使用 Gazebo Classic：

```bash
cd ~/src/FirmwarePX4
make px4_sitl gazebo-classic
```

也可使用同样经过 `simulator_mavlink` 的 jMAVSim。不要用直接 GZ bridge 或 SIH 验证本指南的 `FTC_SIM_*`，它们不经过当前注入点。

正常：仿真器与 PX4 shell 启动，车辆进入可连接状态，无持续启动错误。

异常说明：仿真器未启动属于宿主图形/仿真依赖问题；PX4 启动但没有 `simulator_mavlink` 时，连续效能注入不会作用于仿真 HIL actuator controls。

## PHASE 3：连接 GroundStation

Windows PowerShell：

```powershell
& 'E:\MERIVUS\GroundStation\build\Desktop_Qt_5_15_2_MSVC2019_64bit-Release\staging\MERIVUS.exe'
```

让 GroundStation 使用默认 SITL UDP 链路连接，不要同时运行第二个会占用同一端口的地面站实例。

正常：车辆出现在 Fly View；FTC 尚未启动时，FTC 面板显示“未收到 FTC 遥测”或 `N/A`，标准飞行与 ESC 显示不受影响。

异常说明：没有车辆首先是 UDP/防火墙/端口问题；车辆存在但后续 Inspector 不能识别 60000–60003，则核对是否启动了正确的 MERIVUS 二进制及方言生成头。

## PHASE 4：开启 Observe

在 PX4 shell 设置安全组合，先保持 Shadow 关闭：

```sh
param set FTC_MON_EN 1
param set FTC_CA_SHADOW 0
param set FTC_CA_EN 0
param set FTC_IMPACT_EN 1
param set FTC_LOC_EN 1
param set FTC_REC_EN 1
param set FTC_REC_ACT 0
param set FTC_SIM_EN 0
param show FTC_CA_EN
param show FTC_REC_ACT
```

随后先执行“模块启动合同”中的五条 `status`。若显示未运行，再按规定顺序各执行一次 `start`；若已经运行，说明启动脚本已处理，不要重复启动。

正常：五个模块都在运行；`FTC_CA_EN` 与 `FTC_REC_ACT` 都为 `0`；MAVLink Inspector 能看到四条 FTC 消息，频率约为 5/5/10/1 Hz；面板 Mode 为 `OBSERVE · 只观测`。

异常说明：只有部分消息通常表示对应 uORB 发布模块未运行；完全没有消息说明模块/流配置或链路有问题；出现 `ACTIVE` 是安全阻断项，应立即停止测试并保存日志。

## PHASE 5：开启 Shadow

```sh
param set FTC_CA_SHADOW 1
param show FTC_CA_SHADOW
param show FTC_CA_EN
param show FTC_REC_ACT
listener ftc_effectiveness_matrix 1
listener ftc_allocation_shadow 3
```

正常：控制分配器发布有效矩阵，`ftc_allocation_shadow.valid` 在输入就绪后成立；GroundStation Mode 变为 `SHADOW · 影子分配`，同时主动参数仍为 `0`。

异常说明：矩阵没有发布时，检查车辆构型与 control allocator；矩阵有效但 Shadow 仍不成立时，检查 `ftc_control_monitor status` 及其输入。不得通过开启 `FTC_CA_EN` 绕过问题。

## PHASE 6：检查电机 H/E

在 GroundStation 中仅在 SITL 环境起飞并保持安全、可控的悬停；等待默认 5 秒基线标定及足够激励，然后在 PX4 shell 执行：

```sh
motor_health_monitor status
listener motor_health_status 5
listener ftc_model_status 5
```

正常：`motor_health_status.state` 最终为 VALID、`model_valid=true`；四个电机的 H/E 是合理百分比且大致一致；电机卡显示 `H … · E …`，Tooltip 同时显示故障概率、置信度和分类。H 是当前观测分数，不是剩余寿命。

异常说明：未解锁、落地、平均油门低、输入过期、基线未完成或置信度不足都会保留 `N/A`/CALIBRATING/INVALID；这不是 0% 健康。先解决观测有效性，不要修改门限伪造 VALID。

## PHASE 7：检查 Control Authority

```sh
ftc_control_monitor status
listener ftc_control_authority 5
listener ftc_allocation_shadow 5
```

打开 FTC 面板“详情”。

正常：authority `valid=true`；Roll、Pitch、Yaw、Thrust 均显示 `0..100%` 的合理值；主面板只在退化时显示关键警告，内部矩阵、残差和 mask 不占用主飞行页面。

异常说明：四轴全部 `N/A` 表示控制消息未收到、已过期或 authority 无效；固定 0% 通常是输入/矩阵问题，不应当作正常显示。

## PHASE 8：执行三组 SITL 故障注入

`FTC_SIM_MOT` 是 **1-based**。例如 `FTC_SIM_MOT=1` 选择界面 M1；代码内部减 1 后写入 actuator index 0，`ftc_simulation_status.motor_index` 因而报告 0-based 的内部索引。先固定无斜坡、无间歇：

```sh
param set FTC_SIM_MOT 1
param set FTC_SIM_RAMP 0
param set FTC_SIM_INT 0
```

TEST A，正常链路：

```sh
param set FTC_SIM_EFF 1.0
param set FTC_SIM_EN 1
listener ftc_simulation_status 3
```

正常：target/applied effectiveness 为 1.0；M1 的 H/E 不应出现持续下降或故障标记。

TEST B，部分退化：

```sh
param set FTC_SIM_EFF 0.70
listener ftc_simulation_status 3
listener motor_health_status 5
```

正常：target/applied effectiveness 为约 0.70；估计器收敛后 M1 的 E/H 应相对其他电机下降。`FTC_HLTH_MIN` 使用严格小于 0.70 且还受置信度和持续时间约束，所以恰好 0.70 不保证设置 degraded mask；本场景重点验证趋势与链路。

TEST C，严重退化：

```sh
param set FTC_SIM_EFF 0.40
listener ftc_simulation_status 3
listener motor_health_status 5
```

正常：target/applied effectiveness 为约 0.40；在模型有效、置信度达标并超过 `FTC_FAIL_T` 后，M1 应呈明显退化趋势并可进入 degraded。0.40 仍高于默认 `FTC_FAIL_MIN=0.25`，因此不要求进入 failed。

每组都记录 `ftc_simulation_status`、`motor_health_status`、M1 卡片和 Tooltip。若选中的不是 M1，先核对 1-based 参数与内部 0-based 状态，禁止通过尝试真实桨叶损伤排查。

异常说明：target/applied 与参数不一致说明注入路径未生效；所有电机同时同比例变化通常不是单电机索引问题；估计器未 VALID 时 H/E 为 `N/A` 是有效性门控，不应把它记录成故障注入结果。

## PHASE 9：检查 LOC 与 Recovery Candidate

在 TEST C 仍生效时，仅在 SITL 中做受控姿态/油门操作，并检查：

```sh
ftc_extreme_state_monitor status
ftc_recovery status
ftc_supervisor status
listener ftc_extreme_state 5
listener ftc_recovery_status 5
listener ftc_system_status 5
param show FTC_CA_EN
param show FTC_REC_ACT
```

正常：LOC 状态和分数与姿态误差、角速度误差、饱和、控制裕度及推进退化共同变化。单独 0.40 效能注入并不保证达到 LOC 门限，这种情况下保持 NORMAL/DISTURBED 是可接受结果。若触发恢复候选，面板应醒目显示 LOC、Recovery 状态和 `CANDIDATE · 恢复候选`，并明确“不向执行器下发 FTC 命令”。

异常说明：GroundStation 显示 `ACTIVE`、`FTC_CA_EN/FTC_REC_ACT` 非 0、或状态输出不再含 `intervention: disconnected` 都是立即停止测试的安全阻断项。Candidate 没有出现时先检查 `eligible`、trigger 和 inhibit mask；不得通过开启 ACTIVE 参数强行触发。

本阶段结束立即恢复注入：

```sh
param set FTC_SIM_EN 0
listener ftc_simulation_status 3
```

正常：simulation status 回到 disabled，applied effectiveness 恢复 1.0。

## PHASE 10：断开 SITL，检查 3 秒 stale/N/A

确认 `FTC_SIM_EN=0` 后，在运行 SITL 的终端按 `Ctrl+C` 停止 PX4/仿真器，不要先关闭 GroundStation。观察至少 5 秒。

正常：每个消息族自最后一帧超过 3000 ms 后标记 stale；后台每 500 ms 检查一次，所以界面切换可能最多再延迟约 0.5 秒。Motor H/E、故障状态与 Tooltip 变为 `N/A`；Control Authority 与 Mode 变为 `N/A`；LOC/Recovery 在相应消息族过期后变为 `N/A`。只有所有曾收到的消息族都过期时，总体 `ftc.stale` 才为 true；局部显示始终按各自消息族独立失效。

异常说明：断流后仍长期显示旧百分比或旧告警，说明 stale 门控失效，是测试前必须修复的问题；某一消息族提前 `N/A` 而其他仍更新，优先检查该流频率或带宽，不要用其他消息的时间戳刷新它。

## 测试记录与通过条件

至少保存：两个仓库 HEAD、递归子模块 SHA、构建日志、`MERIVUS.exe` 哈希、两份协议哈希、四消息频率、五模块状态、三组注入的 uORB 输出、关键 UI 截图、断流时间与 `N/A` 截图。

只有以下条件同时成立，本轮人工集成测试才通过：

1. 两仓协议哈希一致且版本为 1。
2. Firmware SITL 构建与兼容仿真启动通过。
3. 四消息可识别，字段与 UI 一致，局部 stale 正确。
4. H/E、控制裕度、LOC、Recovery 的变化可由 uORB/MAVLink 证据解释。
5. 全程 `FTC_CA_EN=0`、`FTC_REC_ACT=0`，Mode 从未出现 ACTIVE。
6. 关闭注入后效能恢复，停止 SITL 后 3 秒进入 N/A。
