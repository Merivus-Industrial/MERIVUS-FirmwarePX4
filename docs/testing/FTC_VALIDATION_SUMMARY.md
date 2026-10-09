# FTC 验证总结

日期：2026-09-09 至 2026-09-10。原计划为完整软件验证，随后用户要求“收尾并整理总结之前的工作”，本轮据此结束，不再扩大测试范围。本文记录实际结果，不表示 FTC 全项目验收通过。

最终源码基准：Firmware `3b4304e25350c02452d68a1a598d4f7fdacc45c9`，GroundStation `e7444f32b2f81c699e8b881a1262ca8f1945f5ee`。Firmware 最终源码已通过 25/25 Host 单元测试，并在隔离 VM worktree 完成 SITL 与 FMUv6C 构建。GroundStation Release 生产构建基准为 `31638ea`，后续改动只涉及测试探针和文档。

## A. 项目最终架构

见 [当前架构](../extreme_control/FTC_ARCHITECTURE.md)。估计、健康诊断、动态矩阵/控制裕度、Shadow、恢复候选、实际仲裁、Supervisor 与地面站均有实现。实际控制仍由 PX4 的 rate controller 和唯一 ControlAllocator 执行。

## B. 本轮发现的缺口

普通机动的激励不足以更新估计；强机动虽可学习 Baseline，模型误差仍会造成 λ 偏移和较大 sigma。Iris 参数与锁定 SDF 的几何、偏航系数及输出非线性不一致。仿真命令比例被误当作物理 λ 真值的风险也已确认。

代码复核及最小复现发现：非法估计年龄、Recovery 垂直 NaN 输入、正常 NaN 输入下的仲裁归属、分配着陆门、Supervisor 状态优先级、地面站协议恢复与过期字段通知均有缺口。

## C. 修复内容

所有路径以 `E:/MERIVUS/FirmwarePX4` 或 `E:/MERIVUS/GroundStation` 为根。

| 文件 / 部分 | 修改与影响 |
| --- | --- |
| Firmware `ROMFS/px4fmu_common/init.d-posix/airframes/10015_gazebo-classic_iris` | 按 SDF 修正旋翼位置、KM=±0.06，CT=7.008，使用 THR_MDL_FAC=0.833333333 线性化高于 armed idle 的推力。仅影响 Iris 仿真机型。 |
| Firmware `src/modules/control_allocator/FtcAllocationPolicy.hpp` | 拒绝负数 age；增加 LANDED=512；着陆清零；active 状态与本周期 yaw 权重一致。 |
| Firmware `src/modules/control_allocator/ControlAllocator.cpp/.hpp` | 订阅着陆状态，缺失/过期或 landed 时撤销 FTC 分配资格。 |
| Firmware `src/modules/ftc_recovery/FtcRecoveryArbiter.hpp` | 正常输入非有限时立即释放 FTC 混合权重；不擅自构造正常 PX4 指令。 |
| Firmware `src/modules/ftc_recovery/FtcRecoveryController.hpp` | 检查有效垂直位置/速度及控制配置是否有限；拒绝非法候选。 |
| Firmware `src/modules/ftc_supervisor/FtcSystemPolicy.hpp`、`FtcSupervisor.cpp/.hpp` | 状态决策成为实际生产函数；故障优先于退化；SHADOW 需要真实有效候选；退出混合期间报告实际 Active。 |
| Firmware `src/modules/motor_health_monitor/MotorHealthMonitor.cpp`、`PropulsionFaultEvidence.hpp` | 删除仅凭全机振动/λ 对桨损、电机损伤或各电机不平衡的具体归因；证据不足时报告推进异常。 |
| Firmware `Tools/merivus/ftc_validation/`、`src/modules/ftc_recovery/FtcControlContractTest.cpp` | 保存独立会话、ULog 分析、41 场景定量 sweep、38 项策略回归与 SSH 遥测桥接。 |
| GroundStation `src/Vehicle/VehicleFtcStatusFactGroup.cc/.h` | 每类 stream 独立维护协议兼容性，可恢复且不能被其他正常 stream 掩盖；过期时通知所有受影响电机 role。 |
| GroundStation `test/FTC/` | 实际生产后端的 Qt 测试及实时 UDP 探针。 |

## D. Estimator 结果

41 组理想模型场景已输出 CSV。24 组 λ=1.00/0.95/0.90/0.80/0.70/0.50、M1–M4 静态真值场景，最后 10 秒最大 MAE 为 0.000001，其他电机最大误差 0.000003，最慢稳定时间 9.28 秒。这些数字仅适用于同一理想回归模型。

4 秒故障/4 秒恢复的间歇场景失败：最终 λ=0.421727，MAE=0.304411，其他电机误差最高 0.319358；平均 sigma=0.297927、confidence=0.175677，最后窗口有效比例为 0。不能把整个 sweep 报为全部通过。

7 次真实飞行的最终准入门均未通过。部分会话短暂出现 `valid=true`，最近 3 次飞行统计窗口占比约 5.39%–6.00%，但没有形成稳定有效估计。最近会话门检查时 λ≈[0.950,0.995,1.000,1.000]，sigma≈[0.371,0.324,0.265,0.385]，confidence≈[0.045,0.095,0.210,0.035]。这些 λ 是量化遥测快照，不代表飞行全过程精度。低激励限制已登记为 `KNOWN_LIMITATION`，不阻断本次软件收口。

## E. Fault Diagnosis 结果

4 项分类选择函数测试通过。7 次正常飞行统计窗口未出现 degraded/failed mask。因模型准入失败，没有开展真实推进故障阳性检测与定位，不能评价识别延迟或物理分类准确率。PROP_DAMAGE 与真实 ESC_POWER_FAILURE 尚无相应物理模型/硬件证据。

## F. Shadow 结果

确认使用动态矩阵和原生分配算法，Shadow 与实际执行器输出分离；ULog 包含候选及残差。尚未完成有效故障估计下 candidate 对 nominal 的定量优势检查，保持 `IMPLEMENTED_UNVERIFIED`。

## G. Active Allocation 结果

从未开启。估计器、误差、authority、Shadow 的前置门未全部通过，因此未执行 0.90→0.80→0.70 的 Active 对照。软件接入和 Host 回退测试不等同于闭环效果验证。

## H. Impact / LOC 结果

7 次正常飞行统计窗口无 Impact、无 LOC_LOSS_OF_CONTROL 误报。`iris-angular-06` 有短暂 LOC_DISTURBED（窗口约 0.14%），不能把它描述成实际失控。真实冲量、碰撞、重着陆与失控阳性场景尚未执行。物理扰动插件仅在产物区完成编译，源稿已归档，未保留为已验证的项目入口。

## I. Recovery Candidate 结果

既有状态序列与新安全回归通过 Host 测试。真实飞行中未取得有效 authority，未完成真实扰动触发下的候选轨迹验证。

## J. Active Recovery 结果

未开启，仍为 `IMPLEMENTED_UNVERIFIED`。没有证据证明实际降低角速度、恢复 body-Z 或控制下降。

## K. Dual Active 结果

未开启。Supervisor 的双 Active 标志与优先级有 Host 测试；没有双控制路径动态稳定性、竞争或震荡验证结果。

## L. Fallback / Re-entry 结果

16 项分配、恢复及仲裁 Host 回归通过，覆盖持续故障门、失效输入、超时、sigma、参数关闭、上锁/着陆/结构不支持、候选边界、倒置超时与正常 setpoint 匹配。软回退测试检查 λ 每步变化及最终精确回到 1；上锁/着陆直接清除适配。实际飞行 Active 退出、姿态/推力跳变及积分器 windup 尚未验证。

## M. Supervisor 结果

18 项实际生产策略测试通过，覆盖初始化、学习、不可观测、就绪、正常、Shadow、退化、故障、候选、各 Active、紧急下降、FAILED、停用混合和陈旧/未来时间戳。修正后的 Supervisor 参与 `live-07`、`ssh-live-08` 正常 SITL 回归。

## N. MAVLink 结果

两仓 v2 字段、生成头、后端与 QML 合同检查通过。CRC extra：60000=29、60001=153、60002=136、60003=5。真实后端解码 26092 包，0 CRC 错误。协议版本均为 2。

`ssh-live-08` 的独立记录链路，按仿真时间统计：

| 消息 | Hz | B/s（帧字节） | 最大间隔 |
| --- | ---: | ---: | ---: |
| MOTOR | 5.000 | 824.69 | 0.220 s |
| CONTROL | 5.000 | 440.68 | 0.204 s |
| EXTREME | 10.000 | 500.38 | 0.108 s |
| DIAGNOSTICS | 1.000 | 130.00 | 1.020 s |

合计约 1895.75 B/仿真秒。仿真加速倍率设为 2，不能将此表直接当作墙钟或串口带宽。尚未得到足以区分跨链路转发/序列重排的绝对丢包率，不宣称“零丢包”。

## O. GroundStation 结果

Qt Test 8 个结果项通过（6 个测试方法加初始化/清理）。修复前两个协议恢复测试失败，修复后通过。最新生产代码 Windows Release 构建通过。

实际 MERIVUS.exe 窗口通过可访问性观察到 UAV-1。SSH 双向测试桥向地面站发送 26092 个数据报、1390569 B，收到并转发地面站 296 个数据报、6391 B。实际 FTC 后端在约 63.496 s 进入 STALE，约 93.996 s 自动恢复；停流窗口内心跳计数仍从 188 增至 278。最终会话结束后再次 STALE 是预期结果。

Windows 截图接口报 `SetIsBorderRequired 0x80004002`，点击接口报 `coordinate input geometry is unavailable`。因此详情面板及卡片的最终视觉样式未自动确认；没有伪造截图或视觉通过结论。

## P. FMUv6C Build

最终源码 `3b4304e253` 上执行 `make px4_sitl_default -j4`、`make px4_fmu-v6c_default -j4`，均通过。使用隔离 VM worktree，由本地 Git bundle 导入，原 VM 开发目录未直接修改。

| 产物 | SHA-256 |
| --- | --- |
| FMUv6C ELF | `0cbdf8ef63f77a7788d7b7ff419602ce986c6f7fdff83324034fe21d18933c22` |
| FMUv6C BIN | `802b31a1cafd3e46b4d2da296202bd1a70156736ac1bd936b9dfd57c12ddce45` |
| FMUv6C PX4 | `9a4999cb614d6e75b46e0c622d60774035beb9782845c21dc99083c331ddc9a1` |
| SITL px4 | `4e2253d445f606f52cc0aab38a8775b32cbfe4fe42fad0aa788a2f5cda321e2b` |
| 源 Git bundle | 见最终归档 `MANIFEST.sha256`；bundle 包含本页所在的收口提交 |
| GroundStation MERIVUS.exe | `8b7cfe912342c079a20d929b06b94279a8e45d3ead1cd785ce79eb90355d88cb` |

工具链：Ubuntu g++ 9.4.0、arm-none-eabi-gcc 9.2.1，现有 CMake/Ninja；Windows Qt 5.15.2、MSVC 14.44。依赖使用仓库锁定 gitlink。固件文件在隔离目录 `build/px4_fmu-v6c_default/`，没有刷写真实硬件。

## Q. 性能 / 资源

最终 Flash 1964832 / 1966080 B（99.94%），剩余 1248 B；AXI SRAM 静态占用 61608 / 524288 B（11.75%）。没有扩大链接区。Flash 余量很小，后续代码变更必须重新构建检查。

FTC 常规观测 topic 约 50 Hz，Supervisor 10 Hz。矩阵为配置快照，不要求 50 Hz。CPU 峰值、H743 栈高水位和最坏 logger 负载尚未完成测量，不用静态 RAM 数字替代运行时资源验证。

## R. 自动测试汇总

- 最终策略测试 38/38：16 分配/恢复/仲裁、18 Supervisor、4 分类选择。
- Qt Test 8/8 结果项；FTC 遥测静态合同通过。
- Iris SDF/参数契约 1/1。
- Estimator sweep 41 个数值场景；其中间歇跟踪失败，不能折算为 41 PASS。
- 两个固件目标及 Windows Release 构建通过。

GitNexus 使用现有本地索引并刷新，做了核心影响与提交前差异检查。UNKNOWN、调度/uORB/Qt/QML 边界均结合源码复核；MCP 缓存仍显示旧索引时，提交前使用本地 CLI。没有将无调用边或新文件未入索引解释为无影响。

## S. SITL 测试汇总

7 次完成正常飞行与记录：baseline-02、strong-03、angular-04、iris-corrected-05、iris-angular-06、live-07、ssh-live-08。另一次 baseline-01 在 UDP 端口冲突时未开始飞行，随后使用独立端口。

各次 ULog 均含 11 个 FTC topic。最后飞行代码为 `72dd944388`；之后的物理分类收敛修改完成 Host 与构建验证，按用户收尾要求未再开新飞行。不能把最后构建提交写成已完整 SITL 回归。

## T. 未通过 / 尚未执行项目

稳定真实 Baseline、真实 λ 精度与间歇跟踪未通过。完整日志中还发现 `valid` 短暂为 true，但最终 `last_valid_timestamp=0` 的组合，需继续审计历史有效性不变量；本轮未为此再调整算法。

真实 EFF100/95/90/80/70 注入序列、Shadow 残差优势、Active Allocation、真实 Impact/LOC 阳性、Recovery Candidate 物理扰动、Active Recovery、Dual Active、实际 re-entry 与积分器测试尚未执行。原计划的全部专题文档拆分也未完成。这些属于后续软件工作，不标为“必须人工”。

## U. 人工剩余工作

见 [人工验证](FTC_MANUAL_VALIDATION.md)。仅包括窗口视觉、目标硬件、真实 ESC/推力测量、HITL/台架及以后单独批准的飞行。当前不要求用户马上执行。

## V. Remaining Risks

统一 λ 近似不能覆盖独立变化的推力/偏航力矩；Gazebo 还含转速动态、气动阻力与速度相关推力修正。源码确认这些项存在，但尚未量化各项对当前误差的贡献。历史有效性、sigma 量纲/残差放大、Baseline 增益验证仍有待进一步研究。模型无效时不应通过放宽门限获得表面 PASS。

## W. Future Research

在现有日志上建立物理一致的输入、陀螺耦合/气动项及延迟分析；验证 Baseline 增益和不确定度覆盖率；改善时变故障跟踪。Mass 需要独立推力标定，Inertia/CG 保持未实现可靠在线估计。

## X. Firmware Git

起点 `70193b156976d24f869dd359ac52f33d2403197d`，本轮分支 `codex/ftc-full-validation`。测试前 tag `archive/ftc-full-validation-pre-20260909`。代码按配置、安全门、Supervisor、着陆回退、诊断和工具分开提交；最终源码构建点为 `3b4304e253`。收尾文档在后续独立提交，不改生产代码。未 push、未 merge。

## Y. GroundStation Git

起点 `aea9833ebd319217446d62dc2c0440945d6da899`，同名分支和测试前 tag。生产修复 `31638ea`；实时探针 `5a1931a`；收尾说明独立提交。未 push、未 merge。

## Z. 最终结论与证据位置

本轮修复了可复现的软件安全门和状态显示问题，建立了真实自动飞行、数值分析、跨主机遥测及停流恢复证据。估计器仍未满足稳定注入准入，Active 保持关闭。项目尚未完成完整软件验收，本轮已按用户要求收尾。

最终证据统一归档到 `E:/MERIVUS-archive-final-20260917/`。`ARCHIVE_INDEX.md` 说明来源与取舍，`MANIFEST.sha256` 校验归档内容；其中保留 ULog、stdout、参数、源 HEAD、scenario、事件、CSV/JSON、补丁和 Git bundle，不保留依赖缓存与构建树。现有测试会话已退出，未操作真实硬件。
