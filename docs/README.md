# MERIVUS 文档门户

这里是 FirmwarePX4 产品文档入口。PX4 上游通用资料仍在 `Documentation/`；MERIVUS 的架构、产品合同、验证状态和开发规则从本页进入。

## 项目与架构

- [当前模块状态](PROJECT_STATUS.md)
- [系统地图](architecture/SYSTEM_MAP.md)
- [代码来源地图](architecture/CODE_OWNERSHIP_MAP.md)
- [关键数据流](architecture/DATA_FLOWS.md)
- [关键调用链](architecture/CALL_CHAINS.md)
- [硬件—软件对应关系](hardware/HARDWARE_SOFTWARE_MAP.md)

## 悬停控制研究

- [自主飞控算法研究系统](research/AUTONOMOUS_FLIGHT_CONTROL_RESEARCH.md)

## FTC

- [FTC 文档入口](extreme_control/README.md)
- [系统架构](extreme_control/FTC_ARCHITECTURE.md)
- [Estimator](extreme_control/FTC_ESTIMATOR.md)
- [故障诊断](extreme_control/FTC_FAULT_DIAGNOSIS.md)
- [控制分配与 Shadow](extreme_control/FTC_CONTROL.md)
- [恢复候选与仲裁](extreme_control/FTC_RECOVERY.md)
- [遥测与 GroundStation](extreme_control/FTC_TELEMETRY_GROUNDSTATION.md)
- [安全边界与已知限制](extreme_control/FTC_SAFETY_AND_LIMITATIONS.md)
- [验证总结](testing/FTC_VALIDATION_SUMMARY.md)
- [人工验证](testing/FTC_MANUAL_VALIDATION.md)
- [测试历史](testing/FTC_TEST_HISTORY.md)
- [参数索引](reference/FTC_PARAMETER_INDEX.md)
- [uORB 索引](reference/FTC_UORB_INDEX.md)
- [MAVLink 契约](reference/FTC_MAVLINK_CONTRACT.md)

## 硬件、通信与编队

- [Pixhawk 6C Mini V6C22 合同](../Documentation/merivus/PIXHAWK_6C_MINI_V6C22.md)
- [RTK 与 4G 配置合同](../Documentation/merivus/RTK_AND_4G_CONFIGURATION.md)
- [构建与 CI 产物合同](../Documentation/merivus/CI_CONTRACT.md)
- [机载 `swarm_node` 协议](../src/modules/swarm_node/README.md)

## 开发与测试

- [构建与刷写](development/BUILD_AND_FLASH.md)
- [飞行可靠性实施与验收边界](development/FLIGHT_RELIABILITY_IMPLEMENTATION.md)
- [GNSS 速度维度与精度契约](reference/GNSS_VELOCITY_CONTRACT.md)
- [修改影响分析](development/CHANGE_IMPACT_GUIDE.md)
- [后续重构候选](development/REFACTOR_CANDIDATES.md)
- [L0–L9 测试矩阵](testing/TEST_MATRIX.md)
- [统一术语表](reference/GLOSSARY.md)
- [文档收口记录](DOCUMENT_AUDIT.md)

文档与源码冲突时以当前提交源码为准，并在同一变更中修正文档。验证状态只能由对应层级的可复现证据升级。
