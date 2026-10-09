# MERIVUS 术语表

| 英文/标识 | 统一中文 | 本项目含义 |
| --- | --- | --- |
| Control Allocation | 控制分配 | 将六轴 torque/thrust 需求映射为执行器指令 |
| Control Authority | 剩余控制能力 | 当前动态效能与执行器余量下，各控制轴还能提供的能力；不用“控制权限”表示这个量 |
| Control Ownership | 控制所有权 | 哪个模块是某类 setpoint 或 actuator 输出的唯一写入者 |
| Effectiveness | 执行器效能 | 执行器指令对力/力矩的实际贡献，FTC 中以 `lambda` 表示相对标称值 |
| Effectiveness Matrix | 效能矩阵 | 控制分配矩阵 `B`；`B_nominal` 为 allocator 名义矩阵，`B_dynamic` 为 shadow 动态矩阵 |
| Fault-Tolerant Control (FTC) | 容错控制 | 故障感知、控制能力评估、恢复与可能的控制重构总称；当前项目未实现主动接管 |
| Observe | 观察 | 只读输入并发布诊断，不改变控制 |
| Shadow Mode | 影子模式 | 与真实控制并行计算候选，用于比较和记录，不驱动执行器 |
| Candidate Setpoint | 候选设定值 | 尚未进入正常 setpoint owner/仲裁器的建议值 |
| Active Control | 主动控制 | 真实写入控制 setpoint、allocator 或执行器链；当前 FTC 不具备 |
| Intervention | 控制介入 | FTC 获得真实控制所有权并改变飞行行为；不能与 `intervention_enabled` 状态字段混淆 |
| Loss of Control (LOC) | 失控 | 姿态/rate 跟踪、饱和、权限和动力健康共同表明无法维持期望运动的状态 |
| Recovery | 恢复控制 | 从异常状态向可控状态返回的策略；包含候选与默认关闭的独占输入仲裁 |
| Control Reentry | 控制重入 | 恢复候选结束后将所有权安全交还正常控制器的阶段；尚未实现仲裁 |
| Hard Landing | 硬着陆 | 落地条件下的高冲击事件，与空中外部撞击分开分类 |
| Residual | 残差 | 模型预测、目标 wrench 或候选输出与观测/需求之间的差异 |
| Headroom | 执行器余量 | 当前输出到上下限之间可用的调节空间 |
| Saturation | 饱和 | 执行器达到约束，无法继续按需求增加或减少输出 |
| Supervisor | 监督器 | 聚合子系统事实和状态的模块；当前不拥有控制或 failsafe 权限 |
| Session Lease | 会话租约 | 编队 FOLLOW_TARGET 中绑定 source 和 session、且有新鲜度限制的数据合同 |
| Hold / Loiter | 悬停/盘旋保持 | 编队 ABORT 的退出目标；不等于已经自动降落 |
| RTK Float / Fixed | RTK 浮点解/固定解 | GNSS 载波相位解状态；不要把 `fix_type>=3` 直接写成 RTK Fixed |
| Source Ownership | 代码来源分类 | 本项目 `CODE_OWNERSHIP_MAP` 中的 upstream/custom/modified 等来源，不是维护人员名单 |
