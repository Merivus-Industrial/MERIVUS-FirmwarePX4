# 后续重构候选

本页记录尚未完成的源码/配置改进。已完成的共享 FTC 开关归属调整见[参数索引](../reference/FTC_PARAMETER_INDEX.md)。

| 优先级 | 问题与范围 | GitNexus/源码影响 | 风险 | 建议方向 |
| --- | --- | --- | --- | --- |
| P0 | `FTC_CA_EN`、`FTC_REC_ACT` 主动软件路径已实现，需后续验证 | 参数、recovery/supervisor 状态、未来 allocator/Commander | HIGH | 当前 owner、仲裁、fallback 见 FTC_ARCHITECTURE；后续验证前保持默认关闭 |
| P0 | Swarm 协议常量在固件与 GroundStation 两仓分别维护 | `MavlinkReceiver`、`SwarmNode`、`GroundStation SwarmController`；uORB/MAVLink 跨图边界 | HIGH | 建立可版本化协议合同或生成源，覆盖版本、命令、session、mask 和 timeout；保持两端可审查差异 |
| P1 | 其余 FTC 监控与恢复参数仍集中在 `motor_health_monitor_params.c`，由多个模块使用 | 32 个参数、多个 `DEFINE_PARAMETERS`、参数元数据 | MEDIUM | 后续按长期责任拆分剩余参数，保持单一定义源并做板型元数据检查 |
| P1 | FTC hook 直接位于 `ControlAllocator`，未来主动能力可能继续扩大分叉 | allocator 4 个已解析上游符号 + uORB shadow consumers | HIGH | 主动控制设计确认后，优先形成单一、窄接口的 allocator extension；不要继续追加散落分支 |
| P1 | `MavlinkReceiver::handle_message_command_both` 同时承担通用命令与 MERIVUS swarm 解析 | GitNexus 命中 6 个 `handle_message` 流程；下游跨 `swarm_command` | HIGH | 评估抽取产品命令解析 helper，保留 ACK 和 target 检查的单一事实源；先补两端测试 |
| P2 | `swarm_node.cpp` 状态机、轨迹、命令发布和 timeout 集中在单文件 | `Run` caller 为 `UNKNOWN`，源码显示 20+ 方法和多状态责任 | MEDIUM | 在行为测试覆盖后按“事务/位置租约/输出端口”拆分，禁止先移动再补测试 |
| P2 | GitNexus 首次索引把已检出子模块全部纳入，C/C++ callback 候选集多次超过 32 | 28,172 files、830,568 nodes；部分 CALLS 未发出 | LOW | 评估项目级 include/exclude 配置，保留 PX4 主仓与必要第三方边界，减少 NuttX/MAVLink 噪声 |
| P2 | FMUv6C V6C02（Hardware 仓）与 V6C22（当前 Mini 产品）同时存在 | BSP、硬件合同、刷写身份和验收文档 | HIGH | 建立明确产品变体矩阵，不共享含糊的“Rev 2”结论；每个变体冻结 HW type、传感器和验证证据 |
| P3 | 历史产品文档分散在 `Documentation/merivus` 与 `docs` | 构建、硬件、RTK/4G、CI 链接 | LOW | 以 `docs/README.md` 为门户，逐步迁移；旧路径先做兼容入口，不直接删除历史资料 |
