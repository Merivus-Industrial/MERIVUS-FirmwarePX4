# FTC 安全边界与已知限制

当前交付是软件收口，不是飞行认证。以下约束必须长期保持：

- `FTC_CA_EN=0`：禁止 Active Allocation 接管。
- `FTC_REC_ACT=0`：禁止 Active Recovery 接管。
- `FTC_SIM_EN=0`：默认不向 SITL 注入故障。
- 只有 `ftc_model_status.valid=true`、`effectiveness_valid=true`、四电机 confidence 达门限、mask 为 0、authority 有效且无 saturation 时，才允许开展下一步仿真注入。
- Shadow 和 Candidate 表示候选计算，不表示执行器控制权。

已知限制：真实 SITL 的低激励和模型失配使 Baseline 不能稳定有效；统一 `lambda` 不能表达推力和偏航力矩分别变化；间歇故障跟踪未通过；真实故障阳性、Impact/LOC 阳性、Active、双 Active、实际 re-entry、HITL、台架和飞行均未验证；FMUv6C Flash 仅剩 `1248 B`，任何代码增长都必须重新构建检查。

后续验证应从 [人工验证](../testing/FTC_MANUAL_VALIDATION.md) 的安全状态开始。估计器门不满足时停止测试并保留日志，不通过放宽阈值、启用 Active 或制造真实损伤取得表面结果。
