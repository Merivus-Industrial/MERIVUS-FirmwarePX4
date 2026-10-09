# FTC 测试历史

本页保留可追溯的时间线；当前结论以 [验证总结](FTC_VALIDATION_SUMMARY.md) 为准，原始文件在 `E:/MERIVUS-archive-final-20260917/`。

| 日期 | 阶段 | 结果 |
| --- | --- | --- |
| 2026-09-03 至 09-08 | 初始 FTC 集成与 Baseline 排查 | 建立模块、uORB、SITL 注入和早期 Host/构建记录；发现 estimator 在正常飞行中 confidence 低且 `valid=false`。 |
| 2026-09-09 至 09-10 | 全链路验证 | 完成 7 次正常飞行、41 个数值场景、38 项策略回归、FMUv6C/SITL 构建、GroundStation Qt/Release 和实际 MAVLink 停流恢复。估计器最终准入 7/7 失败，未执行故障注入或 Active。 |
| 2026-09-14 | Estimator 后续审计 | 保存输入审计、delay sweep、lifecycle 前后测试与未采用的 tracking prototype。没有把研究原型并入生产代码。 |
| 2026-09-17 | 最终收口 | 保留 `3b4304e253` 的有效性生命周期修复；停止算法扩展，合并文档和证据，清理阶段目录、构建树及 VM 旧副本。 |

关键证据摘要：理想静态模型 24 个场景精度良好；4 秒间歇跟踪失败；真实 SITL 的稳定 Baseline 未建立；正常飞行没有 Impact/LOC loss 误报；Shadow、Active Allocation 与 Active Recovery 缺少闭环效果证据；MAVLink v2 实际链路完成过期和恢复验证。

归档中的 `legacy-patches/` 仅保存历史原型和未采用源稿。它们不是最终源码，也不代表验证通过。
