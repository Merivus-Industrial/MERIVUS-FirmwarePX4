# FTC 自动验证工具

本目录用于隔离 SITL、Host 测试和结果分析，不启动真实硬件。实际结果见 `docs/testing/FTC_VALIDATION_SUMMARY.md`；脚本存在不代表对应场景通过。

- `run_sitl.py`：创建独立 rootfs、参数、日志和 Gazebo 会话；默认关闭故障注入及两条 Active 路径。Baseline 门未通过时拒绝注入。正常机动和角运动机动可选；`--groundstation` 使用独立链路测试 FTC 停流及恢复。
- `estimator_sweep.cpp`：41 个理想模型场景，输出 CSV 数值。间歇故障场景未达到准确跟踪要求，不能把整个 sweep 写成全部通过。
- `SafetyContractTest.cpp`、`SystemPolicyTest.cpp`、`FaultEvidenceTest.cpp`：实际生产策略的 Host 回归。
- `test_iris_contract.py`：核对锁定 SDF 与 Iris 旋翼几何、力矩系数及推力线性化。
- `analyze_ulog.py`：11 个 FTC topic 的完整性、频率和字段统计。
- `analyze_alignment.py`：离线延迟与回归一致性分析。`--iris-model` 是对旧配置日志的假设变换，不应再用于已经修正机型配置的日志。
- `mavlink_bridge.py`：测试用 SSH 双向传输。Windows 把真实 SITL 数据送到地面站 14550 及独立后端探针 14655；不会修改防火墙。

Ubuntu Host 示例，从仓库根执行：

```sh
g++ -std=c++14 -O2 -Wall -Wextra -Werror -I . \
  Tools/merivus/ftc_validation/SafetyContractTest.cpp \
  src/modules/ftc_recovery/FtcControlContractTest.cpp \
  -lgtest_main -lgtest -pthread -o /tmp/ftc-safety
/tmp/ftc-safety

g++ -std=c++14 -O2 -Wall -Wextra -Werror -D__EXPORT= \
  -I . -I platforms/common -I build/px4_sitl_default \
  Tools/merivus/ftc_validation/SystemPolicyTest.cpp \
  Tools/merivus/ftc_validation/FaultEvidenceTest.cpp \
  -lgtest_main -lgtest -pthread -o /tmp/ftc-system
/tmp/ftc-system
```

第二条命令使用本次 SITL 构建生成的 uORB 头文件。完整 SITL 还需要锁定的 Gazebo 插件、生成的 MERIVUS Python MAVLink dialect；ULog 分析使用 `pyulog==0.9.0`、NumPy。会话目录必须是新路径，工具拒绝覆盖旧证据。每次保存源提交、runner SHA-256、参数、事件、stdout、MAVLink 数据及 ULog。

手工传入的 `--effectiveness` 是仿真输出命令比例，不能作为物理推力 λ 的真值。本工具没有自动启用 Active 的入口。
