# FTC MAVLink v2 契约

更新：2026-09-09。`protocol_version=2`；传输为 MAVLink 2。两仓核心 XML 逐字节一致，SHA-256 为 `27d637cb3da357295f9b3f59467be07a13e2b63f6da040b3f78a3af93d9d7a12`。Firmware 使用 development 包装，GroundStation 使用 all 包装；上游 common 不改动。生成器锁定 MAVLink 子模块 `18955a04c7c7467e00ea42b704addb4a9c12b53a`，固定 PYTHONHASHSEED=0。

## 兼容性和带宽

原有消息 ID、基础字段、MIN_LEN 和 CRC 保持，新增字段全部放在 MAVLink 2 extensions。线路布局可由旧解析器截断读取，但 v2 改变 ACTIVE/validity 语义，因此应用必须验证协议版本；旧版 v1 与新版 v2 应显示不兼容，不能静默推断安全状态。

| ID | 消息后缀 | MIN_LEN / LEN | CRC | 频率 | 最大未签名流量 |
| --- | --- | --- | --- | --- | --- |
| 60000 | MOTOR_STATUS | 78 / 153 | 29 | 5 Hz | 825 B/s |
| 60001 | CONTROL_STATUS | 24 / 78 | 153 | 5 Hz | 450 B/s |
| 60002 | EXTREME_STATUS | 38 / 38 | 136 | 10 Hz | 500 B/s |
| 60003 | DIAGNOSTICS | 65 / 117 | 5 | 1 Hz | 129 B/s |

以每帧 12 B 开销计算合计 1,904 B/s，13 B 签名块后 2,177 B/s；零尾裁剪可降低实际长度。57,600 baud、MAV_0_RATE=0 的标称预算约 2,880 B/s，FTC 最大未签名占约 66%，须与其他消息共享并在后续测量调度降频、延迟和丢包。流频率可由 PX4 标准调度器调整。

## 字段和语义

MOTOR_STATUS 增加 last_valid_timestamp、estimate_age、12 路 uncertainty/diagnosis_state、estimator_state、baseline_learned、current_observable。confidence 是协方差质量分数，历史效能允许用于诊断；health 仅在模型有效时提供。GroundStation 主 H/E 仍按模型有效性隐藏历史数值。

CONTROL_STATUS 增加三轴正/负方向权限、thrust_up/down、reachable_residual、allocation/recovery fallback、arbitration/reentry weight、实际 allocation/recovery active。ACTIVE_COMMAND_PATH 和 ACTIVE 模式只来自 Supervisor 聚合的实际分配/仲裁反馈，不从使能参数或候选有效性推断。

DIAGNOSTICS 增加条件数、真实预测残差、rigid_body_activity、更新/复位计数、mass/inertia/CG 及其可用状态和估计门标志。其余撞击/LOC 消息保持基本结构；恢复枚举追加 VERTICAL_SPEED_RECOVERY=11，已有数值不重排。

`uint8` 百分比 0..200 表示 0..100%，255 表示不可用；不能将不可用显示成 0。浮点不可用使用 NaN/明确有效位。

## GroundStation

每种消息独立维护接收时间与 3 s 超时。区分消息未收到、消息超时、学习中、当前不可观测、历史估计过期、模型不可用、有效、退化和故障。模型无效不能显示绿色正常。模式分别显示 OBSERVE、SHADOW、CANDIDATE、ACTIVE；控制模式消息过期时不能继续声称已接管。

主页面保持 H/E 和 FTC 摘要，Tooltip 显示 confidence/uncertainty/age/fault type；条件数、预测残差、方向权限和权重在详情页。质量、惯量、CG 的无效状态必须保留，不能为完成度提供假数值。

## 检查与当前边界

Firmware：`python3 Tools/merivus/verify_ftc_telemetry.py`。
GroundStation：`tools/dev/test-ftc-telemetry-contract.ps1` 和 `tools/dev/generate-merivus-mavlink.ps1 -Check`（使用锁定生成器及 Python future 依赖）。

协议检查、生成一致性和构建提供软件证据；实际 VM—Windows 链路已验证解码、3 秒过期与恢复，不能据此宣称绝对零丢包。UI 详情面板仍需人工视觉复核。所有 ACTIVE 默认关闭。
