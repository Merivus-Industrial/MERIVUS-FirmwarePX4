# AFCR 首轮 PX4 SITL 对照实验

## 版本与执行条件

- 实验源码：`1dccfad98df023c3dbde82b63692c69fb239aa42`，分支 `codex/autonomous-flight-control-research`。虚拟机使用隔离检出 `/home/cwkj/merivus_afcr_c2bb1815fd`；既有 PX4 工作树未修改。
- 环境：Ubuntu 20.04.4、Python 3.8.10、g++ 9.4.0、Gazebo Classic 11.12.0。依次通过 `make px4_sitl_default -j4` 和 `make px4_sitl_default sitl_gazebo-classic -j4`，然后通过 8 个 Python 单元测试。
- 执行：`PYTHONPATH=/home/cwkj/afcr-python-deps python3 Tools/merivus/flight_control_research/cycle.py --output /home/cwkj/afcr-cycle-1dcc-seeds1-2 --dialect /home/cwkj/afcr-python-dialect.py --seeds 1 2`。同一提交的脚本及候选 SHA-256 见 [manifest.json](manifest.json)。
- 每个种子对比 `off` 和 `active` 的正常、固定风、固定 1.8 kg 载荷场景，共 12 次独立 Gazebo 会话；`shadow` 正常场景另行跑通并确认 `AFCR_ACC` 记录存在。每次评分窗口约 15 秒、750 个仿真真值样本。
- 原始 [results.json](results.json)、[decision.json](decision.json) 随仓库保留；完整 ULog、场景、PX4/Gazebo 日志的本机归档为 `E:\MERIVUS\analysis\afcr-cycle-1dcc-seeds1-2.tar.gz`，SHA-256：`7488e6196dfaecee9c5a9e078d8a187b5bc3d92ae92051f090bd3bcc76a1e524`。虚拟机原目录仍为 `/home/cwkj/afcr-cycle-1dcc-seeds1-2`。

## 结果与判定

下表为位置真值相对 PX4 位置设定点的 RMSE，单位为米。箭头左侧是 `off`，右侧是 `active`。

| 种子 | 场景 | XY RMSE | Z RMSE | 判定 |
| --- | --- | ---: | ---: | --- |
| 1 | 正常 | 0.0433 → 0.0501 | 0.1389 → 0.1235 | XY 增加约 16%，超过 5% 回退门槛 |
| 1 | 固定风 | 0.0470 → 0.0387 | 0.0487 → 0.0769 | XY 改善，但 Z 退化 |
| 1 | 固定载荷 | 0.0377 → 0.0469 | 0.1057 → 0.1249 | XY、Z 均退化 |
| 2 | 正常 | 0.0483 → 0.0409 | 0.1525 → 0.2245 | Z 增加约 47%，超过 5% 回退门槛 |
| 2 | 固定风 | 0.0406 → 0.0447 | 0.0480 → 0.1358 | XY、Z 均退化 |
| 2 | 固定载荷 | 0.0422 → 0.0402 | 0.0706 → 0.1168 | XY 略改善，Z 退化 |

12 次试验的硬门均通过，分配失败比例与无效飞行模式比例均为零；候选的正常悬停回退门槛在两个种子中分别失败。因此 `retain_for_further_sitl=false`，本候选淘汰，`active` 不进入实机。该结论仅针对当前候选、两个种子与固定扰动场景；当前样本不足以估计泛化性能。

## 下一轮设计输入

1. 先把正常悬停作为硬约束，对 XY 与 Z 同时看误差、速度噪声和控制输出。当前残差代理量混有控制延迟及估计噪声；现有 `AFCR_ACC` 记录的是候选总加速度，下一轮需单独记录或从原始 topic 重建修正量，再分析其频谱、限幅触发和误差相关性。
2. 风场改为有开始和停止时刻的阵风，载荷改为可重复的空中变化；记录恢复时间、峰值误差与连续饱和时间。当前固定风和起飞前固定载荷只能评价稳态，不能证明抗扰恢复能力。
3. 为新候选建立可证伪的理论卡片，明确控制层级、需要的状态量、采样率、稳定性或有界性假设，以及与原生 PX4 控制器的耦合点。候选先跑 `shadow`，只在输出和安全约束成立后进入 `active`，并保留未用于设计的种子做最终检验。
