# MERIVUS 自主飞控算法研究系统：第一阶段

状态：`CANDIDATE_006_ROBUSTNESS_FAIL`。候选 006 在 Ubuntu PX4 SITL、Gazebo Classic iris 的两个发现种子与三个一次性保留种子中，通过了原 8 m/s 阵风悬停位移和恢复门槛；原始证据见[发现集](experiments/round-005/DISCOVERY_RESULT.md)与[保留集](experiments/round-005/HELDOUT_RESULT.md)。后续冻结参数的[26 项鲁棒性矩阵](experiments/round-006/ROBUSTNESS_RESULT.md)在 12 m/s、135° 阵风中触发候选安全硬门，[安全边界回归](experiments/round-006/SAFETY_BOUNDARY_RESULT.md)还发现 failsafe 和 GPS loss 后的非零修正，因此总体为 `FAIL`。此前候选 001–005 的失败和无效试验均保留；针对定位源、动力链与原生参数的悬停审计仍是实机诊断依据。

## 四个角色与边界

| 角色 | 输入与输出 | 当前实现 |
| --- | --- | --- |
| Researcher | 一手论文及 PX4 官方资料；每条结论附来源、适用工况、控制层级与可证伪假设 | `research.py` 自动抓取 11 个主题的 Crossref 元数据；全文归纳与证据审核尚未自动化 |
| Algorithm Designer | 从有量纲的算子组合候选图，限定状态量、幅值与计算复杂度 | `candidate.json`、`candidate.py`；当前候选只是有界残差补偿，不宣称新算法 |
| Engineer | 将生成候选接到 PX4 速度环输出与加速度到推力转换之间 | 仅 SITL 编译；默认关闭，`shadow` 只计算，`active` 只对悬停位置设定点施加 |
| Experimental Scientist | 构建、试验、ULog、硬门、同种子比较、保留或淘汰 | `cycle.py`、`run_sitl.py`、`evaluate.py`；已在 Ubuntu 20.04.4、Gazebo Classic 11.12.0 与 PX4 SITL 工具链实测 |

候选结构采用拓扑有序的类型图。输入为速度误差（m/s）和加速度残差代理（m/s²），输出为加速度修正（m/s²）。生成器检查单位、拓扑顺序、节点数、增益、滤波时间常数、门控阈值和末端限幅。残差代理包含执行器动态、控制延迟及估计噪声，不能直接称为真实外扰。候选 006 仅在已稳定定点悬停中启用，水平速度误差 0.08–0.25 m/s 连续门控；每轴水平修正不超过 0.35 m/s²，Z 轴增益为零，全局上限为 0.5 m/s²。输入、周期或数值无效时退回 PX4 原输出。

Researcher 可用 `python3 Tools/merivus/flight_control_research/research.py --output /absolute/results/literature.json` 批量抓取 11 个主题的 DOI 元数据。输出统一标记 `METADATA_ONLY`；要形成数学假设，仍须读取原文并记录控制层级、状态量、带宽、验证平台和反例。首轮一手资料锚点如下：

| 来源 | 本项目可借鉴的范围 | 不可直接外推的结论 |
| --- | --- | --- |
| [PX4 v1.14 控制模块说明](https://docs.px4.io/v1.14/en/modules/modules_controller) | 位置 P、速度 PID 的原生接口和控制层级 | 更换一个控制律就能改善定位误差 |
| [TU Delft 的 INDI-SMC/SMDO 四旋翼研究](https://research.tudelft.nl/en/publications/quadrotor-fault-tolerant-incremental-sliding-mode-control-driven-/) | 增量控制和扰动观测器的耦合假设 | 其故障与风场实验结果可直接复制到本机悬停 |
| [IEEE 的 DOB-MPC 四旋翼研究](https://ieeexplore.ieee.org/document/10778610) | 观察器与预测控制的组合结构 | 论文轨迹跟踪性能等于 PX4 悬停收益 |
| [UZH/ETH 相关的 GP-MPC 残差动力学研究](https://arxiv.org/pdf/2102.05773v2) | 残差学习和标称模型分离的思路 | 高速轨迹结果代表低速悬停表现 |

PX4 原位置环、姿态环、角速度环、EKF2、allocator 与 failsafe 保持原有职责。研究模式只在 `CONFIG_ARCH_BOARD_PX4_SITL` 构建中存在，还须同时具有 `PX4_SIM_MODEL` 与 `MERIVUS_AFCR_MODE=shadow|active`。Position/Loiter 模式下，定点位置和速度连续 2 秒满足[第四轮入口条件](experiments/round-004/PROTOCOL.md)后才启用；离开模式、着地或设定点移动即关闭并重置。真实固件无该入口。SITL 通过已有 `debug_vect` 的 `AFCR_DA` 记录候选修正，设置 `SDLOG_PROFILE=163` 包含该日志；没有新增参数或 uORB 类型。

## 每轮实验合同

1. AI 只修改 `Tools/merivus/flight_control_research/candidate.json`，由生成器生成 `ResearchCandidateSpec.hpp`。`cycle.py` 检查相对 HEAD 的更改路径；评价器、试验器与门限必须经单独审查。此路径检查是流程约束，不是恶意代码沙箱。
2. Linux 运行 `python3 Tools/merivus/flight_control_research/cycle.py --output /absolute/results/round-005 --dialect /absolute/generated/pymavlink/dialect.py --seeds 11 12`。输出目录须不存在；脚本运行单测及 PX4/Gazebo 构建，先跑阵风 `shadow` 检查有限值、样本数和限幅，再对每个种子运行原生 PX4 与候选 Active 的正常、固定风、阵风和较重载荷场景。保留集单独使用种子 21–23；MAVLink dialect 路径须指向适配本仓库的已生成 Python 文件。
3. 每次独立 Gazebo 会话保存场景、种子、源码提交、世界或模型、日志和 ULog。评分读取仿真真值、位置设定点、allocator、电机输出和飞行模式；真值必须是有限、靠近世界原点的本地坐标，否则整种子按[第五轮协议](experiments/round-005/PROTOCOL.md)重试，最多三次。窗口中必须保持已解锁的 Position/Loiter。载荷是起飞前固定 1.8 kg，阵风在仿真第 46–51 秒施加。
4. 真值相对设定点的 XY/Z 峰值、锚点位移、分配失败和飞行模式构成安全硬门；物理悬停位移以窗口前 2 秒真值为锚点。正常、固定风和固定载荷要求候选从启用到评分窗口结束全程零修正；阵风事件 XY 位移至少改善 0.080 m，停风后至少改善 0.050 m，Z 与电机输出按[第四轮控制判据](experiments/round-004/PROTOCOL.md)限制。阈值只用于 SITL 候选筛选，不能用作实机安全阈值。

## 下一阶段

- 为 Researcher 建立 IEEE、TU Delft、ETH、PX4 等一手来源的可追溯证据库；逐项登记 INDI、L1 adaptive、H∞、DOB/ESO、滑模、MPC、自适应、容错、在线辨识、控制分配、残差学习的控制层级、假设、采样率与失效模式。
- 先修复 failsafe 与 GPS loss 时研究入口未及时撤销的问题，补足 Position→Stabilized 期间的连续零诊断与 EKF reset 试验设施，再用独立种子重跑安全边界。Candidate 006 本轮冻结结果不得回写；后续若改变门控或算法，应有新的版本和预注册合同。
- 扩展真实 GPS 数据链路与本地状态时延注入、风型交互、估计器创新/重置及控制周期/CPU 门限，再设新的独立保留集。
- 调查 Gazebo HIL 初始经纬度偶发为零导致的真值投影参考错误，修复仿真发布源后重跑相关场景；继续保留原始异常 ULog 和质量门槛。
- 实机阶段需另立机体、定位、台架与飞行批准合同。
