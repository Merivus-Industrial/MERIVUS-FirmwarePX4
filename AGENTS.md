# MERIVUS FirmwarePX4 工程约定

本仓库是基于 PX4 v1.14 源码快照维护的 MERIVUS 飞控产品仓库。主要目标为 `px4_fmu-v6c_default` 与 `px4_sitl_default`；产品能力包括 FMUv6C/V6C22 适配、Hyper982/HyperLte 配置、`swarm_node`，以及默认关闭的实验性 FTC 软件链。

## 开始工作前

- 先检查分支、HEAD、工作树和子模块；不得覆盖用户未提交内容。
- 先读 [docs/README.md](docs/README.md)、[系统地图](docs/architecture/SYSTEM_MAP.md) 和 [代码来源地图](docs/architecture/CODE_OWNERSHIP_MAP.md)。
- GitNexus 本地索引位于 `.gitnexus/`，不提交。索引缺失或过期时运行 `npx gitnexus@latest analyze`。
- 修改核心符号前用 GitNexus `impact` 查询上游影响；提交前用 `detect_changes` 检查当前差异。PX4 调度器和 uORB 属于运行时边界，图结果为 `UNKNOWN` 或无 caller 时还必须核对源码、订阅关系和启动脚本。

## 受保护边界

- `src/modules/ekf2`、`mc_att_control`、`mc_rate_control`、`mc_pos_control`、`commander`、`flight_mode_manager` 是 PX4 成熟控制与安全核心，不因整理任务改动。
- `src/modules/control_allocator`、`src/modules/mavlink`、`boards/px4/fmu-v6c`、`ROMFS/px4fmu_common/init.d` 的变更会跨控制、通信或硬件边界，必须执行针对性影响分析和相应级别验证。
- `src/modules/motor_health_monitor` 与 `src/modules/ftc_*` 为实验性 FTC 模块。包含观察、影子、候选及门控主动分配/恢复软件路径。ACTIVE 默认关闭，未完成后续仿真/硬件验证时只能标记 `IMPLEMENTED_UNVERIFIED`。
- `FTC_MON_EN`、`FTC_CA_SHADOW`、`FTC_IMPACT_EN`、`FTC_LOC_EN`、`FTC_REC_EN`、`FTC_SIM_EN` 等入口默认关闭；`FTC_CA_EN` 和 `FTC_REC_ACT` 已接入门控软件路径，默认必须为 0。

## 变更与验证

- 优先使用 PX4 既有扩展点；产品差异应集中、可追溯，避免无关格式化和大范围重排。
- 新增或修改参数、uORB、板级默认值、MAVLink 协议或日志 topic 时，同步更新 `docs/reference/` 与测试矩阵。
- Windows 用于编辑、Git 和 QGroundControl；可复现构建在 Ubuntu 22.04 或已有 PX4 v1.14 工具链的 Ubuntu 环境中完成。
- 默认先做静态与局部验证。构建、SITL、HITL、台架或飞行验证未执行时，必须明确记录，不能把 `IMPLEMENTED_UNVERIFIED` 写成已验证。
