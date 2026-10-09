# 代码来源地图

这里的 Ownership 表示来源和维护边界，不是 GitHub `CODEOWNERS`。仓库于 `b491015d6f` 以完整源码快照重新初始化；初始导入没有保留可验证的上游提交 SHA，因此不能只凭当前 Git 历史把所有旧文件认定为某个精确 PX4 版本。

## 分类规则

| 标签 | 含义 | 处理原则 |
| --- | --- | --- |
| `PX4_UPSTREAM` | 可按 PX4 v1.14 系列原生设计理解，当前产品提交未改动 | 优先跟随上游结构，避免无关分叉 |
| `MERIVUS_MODIFIED_PX4` | 在 PX4 原有扩展点或文件中有产品提交 | 保持改动集中，升级时单独复核 |
| `MERIVUS_CUSTOM` | MERIVUS 新增模块、消息或产品文档 | 由产品仓库维护 |
| `BOARD_SPECIFIC` | 与 FMUv6C/V6C22 总线、器件或默认参数绑定 | 修改须同时核对硬件合同 |
| `EXPERIMENTAL` | 未达到目标验证等级的研究能力 | 默认关闭，不作为生产能力宣称 |
| `GENERATED` | 构建、参数、uORB、GitNexus 等机器生成物 | 源文件入库，缓存/产物不入库 |
| `THIRD_PARTY` | Git submodule 或外部库 | 遵循各自版本与许可证 |
| `UNKNOWN` | 初始导入前来源无法精确证明 | 不猜测作者或精确基线 |

## 当前目录与接口

| 路径/接口 | 分类 | 依据与说明 |
| --- | --- | --- |
| `src/modules/ekf2`、`mc_att_control`、`commander`、`flight_mode_manager` | `PX4_UPSTREAM` | 产品冻结基线及 FTC 工作未修改这些路径 |
| `src/modules/mc_pos_control` | `PX4_UPSTREAM` + `MERIVUS_MODIFIED_PX4` + `EXPERIMENTAL` | 保留原生位置/速度控制；SITL 编译条件下增加默认关闭的悬停研究候选入口，尚无闭环验证 |
| `src/modules/mc_rate_control` | `PX4_UPSTREAM` + `MERIVUS_MODIFIED_PX4` | 原生 rate 控制器保留；FTC 在 setpoint 入口仲裁恢复候选，默认关闭实际接管 |
| `src/modules/control_allocator` | `PX4_UPSTREAM` + `MERIVUS_MODIFIED_PX4` | 保留唯一原生 allocator；发布名义矩阵，并在有效性、解锁/着陆等门控下应用 FTC 分配策略，默认关闭实际接管 |
| `src/modules/logger/logged_topics.cpp` | `MERIVUS_MODIFIED_PX4` | 记录 11 个 FTC topic，不改变 logger 架构 |
| `src/modules/simulation/simulator_mavlink` | `MERIVUS_MODIFIED_PX4` + `EXPERIMENTAL` | 追加 SITL 电机效能注入和 `ftc_simulation_status` |
| `src/modules/motor_health_monitor`、`src/modules/ftc_*` | `MERIVUS_CUSTOM` + `EXPERIMENTAL` | 2026 FTC 分支新增，版权头与提交历史均可追溯 |
| 11 个 FTC `.msg`（含 `MotorHealthStatus`） | `MERIVUS_CUSTOM` + `EXPERIMENTAL` | 由 FTC 分支新增，进入 uORB 生成流程 |
| `src/modules/swarm_node`、`msg/SwarmCommand.msg` | `MERIVUS_CUSTOM` + `UNKNOWN` | 初始导入已存在；后续 MERIVUS 提交有 v1.14 适配，导入前精确来源不可证 |
| `boards/px4/fmu-v6c` | `BOARD_SPECIFIC` + `MERIVUS_MODIFIED_PX4` | V6C22/BMI088、Hyper982、HyperLte、swarm/FTC 构建开关均有产品提交 |
| `src/modules/mavlink` | `PX4_UPSTREAM` + `MERIVUS_MODIFIED_PX4` | 产品提交涉及串口带宽语义、副 GNSS 精度和 v1.14 兼容；其余仍是上游主体 |
| `Documentation/merivus`、`docs/extreme_control`、本 `docs` 信息架构 | `MERIVUS_CUSTOM` | 产品合同、审计与实验设计文档 |
| `platforms/nuttx/NuttX`、MAVLink、Gazebo 等子模块 | `THIRD_PARTY` | Git 索引记录 17 个 gitlink；具体许可证和提交由父仓库记录 |
| `build/`、生成的参数/uORB 文件、`.gitnexus/` | `GENERATED` | 可重建；`.gitnexus/` 本机约 5.0 GiB，不进入 Git |

## 可追溯性边界

- `upstream/release/1.14` 与产品导入历史没有共同祖先，不能用 `merge-base` 推导精确官方基线。
- 现有内容哈希审计表明初始导入的 6,001 个 blob 中，5,725 个与本机 v1.14.4 同路径文件一致、116 个不同、160 个仅存在于导入快照；v1.14.4 另有 9 个文件。历史审计由 Git 和最终归档保留。
- 历史审计记录了相对 `3ec2f9f2c3` 的 64 个 FTC 变更路径；后续差异继续由 Git 提交追溯，当前验证范围见 [验证总结](../testing/FTC_VALIDATION_SUMMARY.md)。
