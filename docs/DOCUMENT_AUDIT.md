# 文档收口记录

2026-09-17 对 MERIVUS 自有文档完成最终收口。PX4 上游文档结构保持不变。

FTC 设计由 `docs/extreme_control/` 的八份文件维护；验证由 `docs/testing/FTC_VALIDATION_SUMMARY.md`、`FTC_MANUAL_VALIDATION.md`、`FTC_TEST_HISTORY.md` 维护；参数、uORB 和 MAVLink 分别由 `docs/reference/FTC_PARAMETER_INDEX.md`、`FTC_UORB_INDEX.md`、`FTC_MAVLINK_CONTRACT.md` 维护。

原有分散的 Estimator、分配、权限、Impact、LOC、Recovery、Supervisor、SITL 计划、ULog 说明和阶段报告已经合并并删除。其历史仍可由 Git 和 `E:/MERIVUS-archive-final-20260917/` 追溯。归档中的旧文档、补丁和原型只用于审计，不是当前规范。

产品 CI、V6C22 硬件合同、RTK/4G 配置与 `swarm_node` 协议职责独立，继续保留在原位置。
