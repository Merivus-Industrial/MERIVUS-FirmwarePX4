# FTC 恢复候选与仲裁

`ftc_extreme_state_monitor` 使用姿态/角速度跟踪误差、饱和、控制裕度、推进退化、加速度、jerk 和垂直运动判断 Impact、硬着陆与 LOC。`ftc_recovery` 在满足资格门时生成角速度抑制、推力方向、姿态恢复、垂直速度/高度稳定、重新交接或紧急下降候选。

恢复候选只通过 `FtcRateInput` 与正常 rate/thrust setpoint 仲裁，随后仍进入原生 `mc_rate_control` 和唯一 ControlAllocator。候选包含有限值、姿态、垂直状态、authority、时效和状态机门；正常输入出现非有限值时，FTC 混合权重立即释放，不制造替代正常指令。

历史验证记录中的 7 次正常 SITL 仿真飞行没有 Impact 或 `LOC_LOSS_OF_CONTROL` 误报；一次会话短暂出现 `LOC_DISTURBED`。这些结果对应 [历史验证总结](../testing/FTC_VALIDATION_SUMMARY.md) 的旧提交，不覆盖当前候选源码。主动恢复门控软件路径存在，但历史记录未开启 Active Recovery，整体状态仍为 `IMPLEMENTED_UNVERIFIED`，默认 `FTC_REC_ACT=0`。

## 入口与垂直参考

入口现在要求在消耗触发的同一周期满足完整资格条件，包括有效且足够的入口高度；`DISTURBANCE_DETECTED` 也受入口高度门约束。恢复过程中，位置时间倒退或 `z_reset_counter` / `vz_reset_counter` 改变会使候选进入 `ABORTED` 并清除垂直积分；不根据可能遗漏的 `delta_z` / `delta_vz` 猜测旧目标的新坐标。

此时 `fallback_reason` 置位 `1u << 4`。`FtcRateInput` 收到该原因后硬退出旧候选，不通过渐变释放继续持有重置前目标；跨任务检测/发布延迟仍存在。主动参数默认值未改变。

## 验证边界

入口高度门和立即退出原因判断由生产与测试共用函数实现，不改变参数或主动默认值。外部 v2 参考包报告 46 项独立 Controller / Arbiter / 共享门用例在普通 Host 和 ASan + UBSan 下均通过；当前仓库改动须重新验证，不能继承该通过状态。

这些 Host 用例覆盖资格/入口高度、重置计数与时间倒退、恢复各阶段退出、理想状态恢复/重新交接及仲裁硬退出；不覆盖完整 uORB 调度、真实发布延迟、动力学、碰撞或断桨效果。主动 FTC 默认关闭，整体未达到实机验收。当前状态见[实施记录](../development/FLIGHT_RELIABILITY_IMPLEMENTATION.md)和[测试矩阵](../testing/TEST_MATRIX.md)。
