# MERIVUS FirmwarePX4 持续集成与产物契约

本文规定产品仓库中必须长期成立的 CI、构建目标和产物追溯关系。上游 PX4 工作流可以作为参考，但不得依赖 MERIVUS 无法控制的上游发布凭据或对象存储。

## 支持范围

- 产品飞控目标：`px4_fmu-v6c_default`；
- 仿真与元数据目标：`px4_sitl_default`；
- 主分支：`main`；
- 上游基线：PX4 v1.14 源码快照。当前产品 Git 历史未保留可验证的上游提交 SHA，因此只能追溯到版本级基线；后续升级必须在提交或 ADR 中记录准确的上游提交。

其他上游板型源码继续保留以降低后续升级成本，但它们不属于 MERIVUS 产品 CI 的发布门槛。新增受支持硬件时，必须同时更新板级配置、构建矩阵、文档和验证记录。

## 工作流

| 工作流 | 触发条件 | 责任与产物 |
| --- | --- | --- |
| `compile_nuttx.yml` | 推送到 `main`、面向仓库的 PR | 只编译 `px4_fmu-v6c_default`，上传 `.px4` 和 `.elf` |
| `metadata.yml` | 推送到 `main`、面向 `main` 的 PR、手动触发 | 生成并上传机架、参数、事件和 uORB 图元数据 |
| `cflite_batch.yml` | 上游 ClusterFuzzLite 约定的定时或手动触发 | 批量模糊测试，不产生可部署固件 |

元数据作为 GitHub Actions artifact 保存，不上传 PX4 官方 S3；产品仓库不要求 `AWS_ACCESS_KEY_ID`、`AWS_SECRET_ACCESS_KEY` 或自定义 checkout token。

## 构建与部署同源

下载或复制固件后，发布记录至少应包含：

1. 主仓库 Git 提交 SHA 与递归子模块 SHA；
2. 构建目标 `px4_fmu-v6c_default`；
3. 编译器与工具链版本；
4. `.px4` 与 `.elf` 的 SHA-256；
5. 实际部署设备、时间和验证结果。

Linux 示例：

```bash
git rev-parse HEAD
git submodule status --recursive
arm-none-eabi-gcc --version | head -n 1
sha256sum build/px4_fmu-v6c_default/px4_fmu-v6c_default.{px4,elf}
```

不得在部署机或服务器直接修改已批准源码和产物；修复必须回到仓库并通过相同流程重新构建。
