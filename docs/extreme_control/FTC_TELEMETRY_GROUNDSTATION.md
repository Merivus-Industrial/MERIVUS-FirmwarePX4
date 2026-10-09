# FTC 遥测与 GroundStation

Firmware 通过 MAVLink 2 发布四类 MERIVUS FTC 消息：MOTOR、CONTROL、EXTREME 和 DIAGNOSTICS。GroundStation C++ 后端分别维护每类消息的协议版本、接收时间和 3 秒 stale 状态，QML 区分未收到、过期、学习中、不可观测、模型无效、有效、退化和故障。无效 H/E 显示 `N/A`，历史值不能伪装成当前健康。

实际 VM—Windows 链路解码 `26092` 个数据包且 CRC 错误为 0；后端约在仿真时 `63.496 s` 进入 STALE，并在 `93.996 s` 恢复。该记录不能证明端到端零丢包。Qt Test 共 8 个结果项通过，Windows Release 构建通过；由于截图/点击工具故障，详情面板最终视觉样式仍需人工复核。

字段、CRC、频率和兼容规则见 [MAVLink 契约](../reference/FTC_MAVLINK_CONTRACT.md)，内部 topic 见 [uORB 索引](../reference/FTC_UORB_INDEX.md)。
