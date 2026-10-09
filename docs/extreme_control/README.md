# FTC 工程文档

本目录是 FTC 设计的唯一入口。当前源码提交、验证结论和历史证据分别见 [验证总结](../testing/FTC_VALIDATION_SUMMARY.md)、[测试历史](../testing/FTC_TEST_HISTORY.md) 与 `E:/MERIVUS-archive-final-20260917/`。

- [系统架构](FTC_ARCHITECTURE.md)
- [电机效能估计](FTC_ESTIMATOR.md)
- [故障诊断](FTC_FAULT_DIAGNOSIS.md)
- [控制分配与 Shadow](FTC_CONTROL.md)
- [恢复候选与仲裁](FTC_RECOVERY.md)
- [遥测与地面站](FTC_TELEMETRY_GROUNDSTATION.md)
- [安全边界与已知限制](FTC_SAFETY_AND_LIMITATIONS.md)
- [参数索引](../reference/FTC_PARAMETER_INDEX.md)
- [uORB 索引](../reference/FTC_UORB_INDEX.md)
- [MAVLink 契约](../reference/FTC_MAVLINK_CONTRACT.md)

观察、估计、诊断、动态分配、恢复候选和独占仲裁已有实现。Active Allocation 与 Active Recovery 均为 `IMPLEMENTED_UNVERIFIED`，产品默认值保持 `FTC_CA_EN=0`、`FTC_REC_ACT=0`、`FTC_SIM_EN=0`。
