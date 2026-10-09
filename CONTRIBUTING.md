# MERIVUS FirmwarePX4 贡献指南

本仓库由 [Ale-xl](https://github.com/Ale-xl) 维护，基于 PX4 v1.14 系列源码快照。支持范围和验证限制见 [项目首页](README.md) 与 [测试矩阵](docs/testing/TEST_MATRIX.md)。上游、第三方许可证和版权声明必须保留。

## 历史与作者

产品历史从 `b491015d6f` 的源码快照开始；精确上游基线不能仅凭该导入提交证明，来源审计见 [代码来源地图](docs/architecture/CODE_OWNERSHIP_MAP.md)。迁移至 `Ale-xl/MERIVUS-FirmwarePX4` 保留既有提交 SHA、提交关系和原始记录，不把上游源码归为维护者原创。

`.mailmap` 将维护者曾用的 `Merivus-Industrial` 身份统一显示为 `Ale-xl`。它影响支持 mailmap 的日志与贡献统计，不修改历史提交对象；GitHub 的账户贡献归属仍以平台对原始邮箱的识别规则为准。维护者的新提交使用 `Ale-xl <239906286+Ale-xl@users.noreply.github.com>`，其他贡献者使用自己的真实身份。

## 开发与评审

1. 从最新 `main` 创建短生命周期分支，先阅读 [系统地图](docs/architecture/SYSTEM_MAP.md) 和相关配置契约。
2. 优先使用 PX4 扩展点，将板级、通信、控制和实验性 FTC 的职责放在各自层级。
3. 参数、uORB、MAVLink、板级默认值发生变化时，同步更新对应参考文档和验证方法。
4. 执行 `git diff --check` 及覆盖变更的局部测试；提交和 PR 默认使用中文，说明原因、影响与未验证项。
5. 推送功能分支并创建 PR。完成检查与评审后合并到 `main`，只删除已确认合并的分支，不改写已发布历史。

## 验证与产物

Windows 用于编辑和 Git；完整固件构建在项目约定的 Ubuntu/PX4 工具链中执行。常用轻量检查：

```sh
python3 Tools/merivus/verify_ftc_telemetry.py
python3 Tools/merivus/ftc_validation/test_iris_contract.py
git diff --check
```

Host、SITL、HITL 和实机验证须分别记录，不能以静态检查代替闭环验证。FTC 主动路径保持默认关闭，未验证项标记为 `IMPLEMENTED_UNVERIFIED`。本任务或 CI 不自动连接真实硬件、注入故障或执行真实飞行。

不得提交凭据、真实飞行日志、生产坐标、本机缓存和构建产物。发布产物须记录主仓/子模块 SHA、工具链、构建参数和 SHA-256；操作入口见 [构建与刷写](docs/development/BUILD_AND_FLASH.md)。
