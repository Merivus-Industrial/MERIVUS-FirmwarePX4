# 构建、产物与刷写

当前产品固件目标是 `px4_fmu-v6c_default`，主机仿真目标是 `px4_sitl_default`。构建、测试与飞行验收状态见[飞行可靠性实施记录](FLIGHT_RELIABILITY_IMPLEMENTATION.md)。本页的刷写步骤仅供具备条件的工程师执行。

## 环境分工

| 环境 | 职责 |
| --- | --- |
| Windows | 编辑与 Git、GitNexus、本地静态检查、Windows QGroundControl 刷写 |
| Ubuntu 22.04 / 已有 PX4 v1.14 工具链的 Ubuntu VM | 初始化子模块、运行 PX4/NuttX 构建、SITL |
| Git 远端 | 在环境间同步不可变提交；不能用整目录覆盖代替版本控制 |

普通 Git Bash 不带完整的 PX4 NuttX/ARM 工具链。VMware 共享目录适合传文件，不适合直接构建。

## 首次准备

```bash
mkdir -p ~/src
cd ~/src
git clone --recursive https://github.com/Ale-xl/MERIVUS-FirmwarePX4.git FirmwarePX4
cd FirmwarePX4
```

以上是普通源码初始化示例。复现某次发布时必须检出发布记录中的不可变提交及递归子模块版本。

已有 clone 缺子模块时：

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

检查已有工具链：

```bash
arm-none-eabi-gcc --version
cmake --version
ninja --version
python3 --version
```

缺少依赖时，仓库已有的安装入口是：

```bash
bash Tools/setup/ubuntu.sh --no-sim-tools
source ~/.profile
```

只有需要图形仿真时再安装相应 simulation tools。

当前 `Tools/setup/ubuntu.sh` 为 NuttX 固定选择 GNU Arm Embedded `9-2020-q2-update`（GCC 9.3.1）。历史构建记录中的 GCC 9.2.1 是另一环境，不能混称。安装脚本会修改系统包和 profile；隔离构建可将该固定工具链放入私有目录，仅在相应构建进程中设置路径。更换编译器时须重新配置隔离构建目录，避免沿用旧 CMake 编译器缓存。

## 构建前身份检查

```bash
git status --porcelain=v1
git rev-parse HEAD
git submodule status --recursive
arm-none-eabi-gcc --version | head -n 1
```

部署构建要求主仓和子模块无非预期修改。Windows 当前环境执行 `git submodule status` 曾出现 Git for Windows signal pipe error 5；这不改变索引中 17 个 gitlink，但发布构建必须在正常 Ubuntu 环境取得完整子模块 SHA。

## 构建目标

```bash
# Pixhawk 6C Mini / FMUv6C 产品配置
make px4_fmu-v6c_default

# SITL
make px4_sitl_default
```

主要固件产物：

```text
build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
build/px4_fmu-v6c_default/px4_fmu-v6c_default.elf
```

记录 SHA-256：

```bash
sha256sum build/px4_fmu-v6c_default/px4_fmu-v6c_default.{px4,elf}
```

发布记录至少保存主仓 SHA、递归子模块 SHA、构建目标、工具链版本、`.px4`/`.elf` SHA-256、设备和验证结果。MD5 不能作为唯一完整性校验。

产品 `default.px4board` 不编入板内 SIH；电脑仿真使用 SITL。v2 候选包曾因原 default 超出 Flash 1,272 字节而临时使用 `engineer` 配置，本仓已将相同的 SIH 取舍归入产品默认目标，避免两个真机配置长期分叉。当前源码的 Flash、CPU、堆与栈余量仍须重新构建和测量，不能沿用 v2 二进制的数值。硬件工程师还须核对机上 `SYS_HITL=0` 与启动配置。

测试配置须同时启用 `BUILD_TESTING=ON` 和 `CMAKE_TESTING=ON`；仅前者会构建测试二进制，但不会正确注册顶层 CTest。v2 候选包使用过 ARM GCC 9.3.1 和主机 GCC 11.5；当前源码须以新的构建日志记录实际工具链。

## 复制和刷写

固件可以通过 VMware 共享目录或 SCP 传回 Windows。复制前后都要复核 SHA-256。Windows 使用：

```powershell
Get-FileHash -Algorithm SHA256 E:\MERIVUS\FirmwareOutput\px4_fmu-v6c_default.px4
```

QGroundControl 中选择“Custom firmware file”，使用 `.px4`，不要把 `.elf`、普通 `.bin` 或 bootloader 文件当作日常自定义固件。刷写前断开电池和不必要外设，确认目标确实是 Pixhawk 6C Mini/FMUv6C；刷写后重新检查参数、传感器、遥控、输出和 failsafe。

首次硬件上电和输出检查必须拆桨或采取等效防护。编队按单机、双机、六机逐级验证；FTC 主动分配/恢复的门控软件路径已实现，但默认关闭，整体仍为 `IMPLEMENTED_UNVERIFIED`，不得作为已验证的实机安全能力使用。源码默认值不等于机上保存参数，后续获准进行硬件操作的工程师仍须核实实际配置。

CI 与产品产物合同见 `Documentation/merivus/CI_CONTRACT.md`；RTK/4G 参数和验收见 `Documentation/merivus/RTK_AND_4G_CONFIGURATION.md`。
