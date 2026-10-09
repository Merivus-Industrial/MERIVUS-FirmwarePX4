# FTC 控制分配与 Shadow

`ftc_control_monitor` 从 PX4 ControlAllocator 获取名义矩阵快照，并按 `B_dynamic[:,i] = B_nominal[:,i] * lambda_i` 形成候选矩阵。Shadow 使用 PX4 Sequential Desaturation 计算候选电机输出、六轴残差、饱和和控制裕度，只发布诊断 topic，不写执行器。

Active Allocation 复用唯一的 PX4 ControlAllocator。只有模型、sigma、age、authority、机型、解锁和着陆门全部满足时才允许应用动态矩阵；软失效渐进回到名义矩阵，上锁、着陆或结构不支持时直接清除适配状态。故障诊断、Shadow 候选和使能参数本身都不能代表已经接管。

16 项分配、恢复及仲裁 Host 回归通过，包含时间戳、sigma、参数关闭、着陆、上锁、结构不支持与回退。Shadow 的数据链和隔离关系已验证；尚无有效故障模型下 candidate 相对 nominal 的定量优势证据。Active Allocation 从未开启，状态为 `IMPLEMENTED_UNVERIFIED`，默认 `FTC_CA_EN=0`。
