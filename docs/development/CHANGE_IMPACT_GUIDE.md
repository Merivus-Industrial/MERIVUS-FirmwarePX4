# 修改影响分析指南

## 固定流程

1. 记录 `git status --short --branch`、当前 HEAD 和目标文件来源分类。
2. 确认 GitNexus 索引与 HEAD 一致；过期时运行 `npx gitnexus@latest analyze`。
3. 用 `query` 查相关执行流程，用 `context` 确认 symbol 的 callers/callees，再用 `impact(direction="upstream")` 分析修改影响。
4. GitNexus 返回 `HIGH`/`CRITICAL` 时先报告风险；返回 `UNKNOWN` 或零 caller 时必须用 `rg`、uORB publisher/subscriber、启动脚本和配置继续核对。
5. 修改后运行 `detect_changes(scope="all")`。`partial:true`、`truncated:true` 或 `risk:UNKNOWN` 不能作为通过结论。
6. 按 `docs/testing/TEST_MATRIX.md` 完成与风险相称的验证，并更新接口文档。

CLI fallback：

```powershell
node .gitnexus\run.cjs impact "symbolName" --direction upstream --repo .
node .gitnexus\run.cjs detect-changes --scope all --repo .
```

`.gitnexus/run.cjs` 属于本地缓存。新 clone 第一次使用前要先执行 analyze。本轮实测发现，npm 11 运行 `npx @latest --help` 时可能再次尝试联网；已经加载的 MCP 查询不需要这次重复下载。

## 关键修改面

| 修改面 | 修改前必须检查 | 最低验证建议 | 主要风险 |
| --- | --- | --- | --- |
| `control_allocator` | ActuatorEffectiveness、matrix/trim/limits、allocation method、`actuator_motors`、FTC matrix shadow、logger、SITL | L0 + L2 + L4；主动策略至少 L6 | 实时输出、机架几何、饱和优先级 |
| `mc_att_control` / `mc_rate_control` | vehicle attitude/rates、setpoint owner、VTOL virtual topics、allocator feedback | L2 + L3 + L4 + L6 | 飞行主闭环 |
| `commander` / failsafe | arming、nav state、vehicle command、Offboard、swarm ABORT、recovery 未来仲裁 | L2 + L4 + L6/L7 | 安全状态与模式所有权 |
| MAVLink handler/stream | message schema、command ACK、stream mode、带宽、secondary GPS、GroundStation | L0 + 相关单测 + L4/Mock | 两端协议和 57600 链路容量 |
| GNSS | GPS driver/protocol、`sensor_gps`、vehicle GPS selection、EKF2 fusion、MAVLink streams | L2 + L4 + L7 | 定位/航向/高度参考 |
| FMUv6C BSP | HW type、SPI/I2C map、startup scripts、driver config、V6C02/V6C22 合同 | L2 + L7 | 错误器件启动或错误刷写目标 |
| `swarm_node` | USER commands、session/source、FOLLOW_TARGET、Offboard setpoint、Commander、GroundStation `SwarmController` | L3/Mock + L4；实机前 L7/L8 | 多机一致性和整组 ABORT |
| FTC message | `.msg`、所有 pub/sub、logger、数组长度、旧 ULog | L0 + L2 + L4 | 隐式接口漂移 |
| FTC estimator | gates、time freshness、ESC mapping、参数、ULog | L3 + L4 + L5 | 误报故障或错误置信度 |
| FTC shadow/authority | `B_nominal`、normalize_rpy、limits、motor count、Sequential Desaturation | L3 + L4 + L5 | shadow 与真实 allocator 语义不一致 |
| FTC recovery | eligibility、Commander/failsafe、setpoint owner、fallback | L4 + L6 + L8 + L9 | 未仲裁的控制竞争 |
| logger topics | rate、可选 topic、存储和分析脚本 | L2 + L4 + L5 | 日志带宽与时间对齐 |

## 图分析的解释边界

- GitNexus 已确认 `publish_ftc_effectiveness_matrix` 由 `update_effectiveness_matrix_if_needed` 调用，后者上游影响集中在 allocator 内。
- `MavlinkReceiver::handle_message_command_both` 命中 `handle_message` 流程，但 `swarm_command` 之后是 uORB 边界，不能只看 CALLS 图。
- `EKF2::Run`、`SwarmNode::Run`、`MotorHealthMonitor::Run` 的上游影响实测为 `UNKNOWN`。这些方法由 PX4 module/work queue 调度，零 caller 绝不表示可随意修改。
- 本索引包含已检出的子模块源码，第三方和 NuttX 符号会增加查询噪声；路径过滤和源码复核是必要步骤。
