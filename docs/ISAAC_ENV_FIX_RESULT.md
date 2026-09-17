# Isaac Environment Validation Result

STATUS: HISTORICAL
Current baseline: [docs/ISAAC_51_DEV_BASELINE.md](ISAAC_51_DEV_BASELINE.md)
Historical raw evidence was archived to:
`/data/shared/AstrEX_project_data/archives/isaac/2026-09-baseline-consolidation/`
See `MANIFEST.json` for original-path mapping (includes paths kept in place and paths deleted).

验证结束日期：2026-09-17（Asia/Shanghai）。

## Final recommendation

唯一推荐保留环境：`isaaclab60`（用户已验证的 WORKING_BASELINE）。

**不要用 `isaaclab60_6010test` 替换基线。** 新环境可以安装、运行 CUDA、启动 Sim 和 Cartpole，但 PhysX effort 验收连续两次失败。PPO 优化器能够更新，不代表物理控制有效。

本次没有修改原 Conda 环境、原 Isaac Lab 源码、驱动或系统 Python。没有执行 commit/push。

## Current working environment

- 用户历史验证：Sim、Lab、Cartpole、PPO、GUI、ROS 2 Bridge 曾正常运行。
- 本会话此前探测：基线 CUDA 运算通过，torchaudio 导入失败，Sim/Lab 启动未复现，出现 native crash。
- 上述历史验证与本次探测不是同一组证据。此报告不把基线写成本次完整验收通过。
- 实施阶段只读取基线的包清单，不再次安装、卸载或修复基线。
- 备份：[pip freeze](env/legacy/isaaclab60_final_freeze.txt)、[Conda export](env/legacy/isaaclab60_final_conda_export.yml)。它们记录当前状态，不是依赖无冲突的可重建 lockfile。

## Exact environment

| 项目 | 保留基线 | 新候选（实装） | 第二候选（仅 dry-run） |
|---|---|---|---|
| Conda | isaaclab60 | isaaclab60_6010test | 未切换环境 |
| Python | 3.12.13 | 3.12.14 | 同一 Python 3.12 解析目标 |
| Isaac Sim | 6.0.0.1 | 6.0.1.0 | 6.0.1.0 |
| torch | 2.10.0+cu128 | 2.11.0+cu128 | 2.10.0+cu128 |
| torchvision | 0.25.0+cu128 | 0.26.0+cu128 | 0.25.0+cu128 |
| torchaudio | 2.11.0（不匹配） | 2.11.0+cu128 | 2.10.0+cu128 |
| CUDA runtime | 12.8 | 12.8 | 未实装 |
| packaging | 23.2 | 23.2 | 未实装 |
| coverage | 7.6.1 | 7.6.1 | 未实装 |
| Pillow | 11.3.0 | 12.2.0 | 未实装 |
| websockets | 17.1 | 12.0 | 未实装 |
| RSL-RL | 5.0.1 | 5.0.1 | 未实装 |

两套已安装环境的 Lab 源码基准均为 `v3.0.0-beta2.patch1`，提交 `ffff603eafc6b74264a5261cc0183d6a65390d78`。Python distribution `isaaclab` 的版本是 `6.1.14`，不是 Git tag 的版本号。

GPU：NVIDIA RTX PRO 1000 Blackwell Generation Laptop GPU，8151 MiB。驱动：580.173.02。系统内核：6.17.0-1032-oem。

## Conflict root cause

Lab 根开发配置固定 Torch 2.10 三件套。实际核心/任务组件允许 torch >=2.10、torchvision >=0.25，因此本次安装运行组件，不安装根开发聚合配置。

Sim 6.0.1.0 精确要求 Torch 2.11 三件套。第二候选的 resolver 明确拒绝 `torchaudio==2.10.0+cu128`，因为 `isaacsim-core==6.0.1.0` 要求 `torchaudio==2.11.0`。没有强行降级，没有使用 `--no-deps`，没有修改依赖声明。6.0.0.0 不在本轮测试范围内。

只安装 Sim 时，`pip check` 通过。随后安装 Lab 运行组件，正常 resolver 接受安装，但最终出现以下 3 项已安装包声明冲突：

```text
isaacsim-kernel 6.0.1.0 has requirement coverage==7.4.4, but you have coverage 7.6.1.
wheel 0.47.0 has requirement packaging>=24.0, but you have packaging 23.2.
isaacsim-core 6.0.1.0 has requirement packaging==26.0, but you have packaging 23.2.
```

这些警告不等于 effort 失败的已知原因，也不等于环境功能通过。Lab 核心固定 coverage 7.6.1，Lab RL 固定 packaging <24。没有为消除这些警告继续更改核心包。

## Installation record

- 创建全新 `isaaclab60_6010test`，未克隆或修改基线环境。
- 独立工作目录：`/home/sssxy/Projects/isaac6010_validation_peSDykmW`。下文证据路径均相对此目录。
- Lab 在该目录的 `IsaacLab/` 副本中构建，并以 editable 方式安装 12 个运行组件。原源码目录保持不变。
- 每次安装前检查 dry-run。测试环境额外安装 `uv==0.12.15`，用于加快解析。
- 完整 Kit 缓存的流式下载多次中断。曾尝试不安装该可选预缓存，让 Kit 按需下载扩展。
- 按需获取失败：第一次 registry 访问失败，重试同步成功后仍缺 `isaacsim.anim.robot.schema`，两次启动退出码均为 55。
- 最终通过分段下载补齐官方缓存，未改变版本。整包 5,879,615,742 字节，SHA-256 与 NVIDIA 索引一致：`35b64cf35ce3d31875ef43025f243c8bd03e29cb28fe844b0cd06b76ba535714`。
- 最终安装全部 Sim 功能包与三个 extscache 包，均为 6.0.1.0。下载片段、wheel 和缓存保留，未自动清理。

## Functional test

| 验收项 | 新候选结果 | 证据 |
|---|---|---|
| CUDA / 三件套导入 | PASS | `gpu_import_result.json`，64×64 GPU 矩阵乘法输出 64 |
| Lab / tasks / RSL-RL 导入 | PASS | `candidate_import.log`，导入指向隔离 Lab 副本 |
| 最小 Sim / PhysX | PASS | `sim_physx_fullcache.log`，60 步，时间 0.516666694 s，正常关闭，退出 0 |
| Direct Cartpole，PhysX，16 环境 | 启动和步进 PASS | `cartpole_effort.log`、`effort_repeat/repeat.log` |
| Joint effort | **FAIL / BLOCKER** | 两次独立结果完全一致，见下节 |
| PPO，16 环境，10 次迭代 | 优化器诊断 PASS，整体验收 FAIL | effort 前置条件失败，不可据此开展有效控制训练 |
| GUI | PARTIAL，未完成视觉验收 | 两次窗口启动并正常关闭；重试运行 585 帧。截图仍被 IDE 遮挡，不能确认场景与关节运动可见 |
| ROS 2 Bridge /clock | PASS | DDS domain 63；系统 Jazzy 收到 116666666、133333333、150000000 ns 三个严格递增样本 |

基线没有在实施阶段重跑这些功能测试。第二候选因解析失败，上述功能全部未测试。

### Joint effort: reproducible blocker

同一任务、同一 PhysxCfg、16 个环境。初始关节位置和速度均为零。输入依次为 0、+0.5、-0.5，对应 0、+50、-50 N。每组最多 60 步，出现 reset 就停止该组。

比较共同的 reset 前第 18 步（0.3 s）。阈值为方向正确且位移 >1 mm、速度 >0.01 m/s。

| 输入 | 环境 0 相对零输入位移 | 环境 0 相对零输入速度 | 第 18 步 effort target / applied buffer | 达标环境数 |
|---|---|---|---|---|
| +0.5 | -1.7120e-7 m | +4.0085e-4 m/s | +50 / +49.99598 N | 1/16 |
| -0.5 | +1.7257e-7 m | -4.0085e-4 m/s | -50 / -49.99599 N | 0/16 |

正输入仅环境 11 出现显著运动，并在第 19 步触发 reset。比较不包含该 reset 帧。不是“所有环境完全不动”，而是不能一致、正确地响应控制。

effort buffer 非零，但多数实际位置/速度响应不达标。独立新进程重试得到相同数值。原始文件：`effort_trajectories.json`、`effort_result.json`，以及 `effort_repeat/` 下对应文件。

注意：日志包含测试断言失败，但 Kit 的关闭路径最终让进程返回 0。因此不能只检查 shell 退出码。本报告依据断言和测量 JSON 判为失败。

[上游 #7601](https://github.com/isaac-sim/IsaacLab/issues/7601) 报告的是 Windows 环境。本次 Ubuntu 的失败来自本机测量，不是直接套用上游结论，也尚未证明两者根因完全相同。

### PPO diagnostic

`ppo_repeat/ppo_result.json`：完成 10 次更新，actor 参数 L2 变化量 `1.4385592937469482`。参数与全部 loss 有限。关节位置最大方差 `0.719691276550293`，奖励最大方差 `1.1404120922088623`。

这些数值只证明优化器和采样链运行，不证明 action 对物理系统有效。effort 验收失败，所以整体 PPO 验收不通过。没有声称策略收敛。

第一次自建测量脚本遗漏官方 `handle_deprecated_rsl_rl_cfg()`，导致 `stochastic` 参数 TypeError。补上与上游训练入口相同的转换后重跑通过。只修正隔离测量脚本，没有修改 Lab 或 RSL-RL。

## AstrBotEX contamination

历史污染结论：**UNKNOWN**。没有可靠记录证明 AstrBotEX requirements 曾安装到基线。不能只凭 websockets 17.1 就判断来源。

当前边界：AstrBotEX `.venv` 使用系统 Python 3.12.3，禁用 system-site-packages，含 pyzmq 27.2.0、websockets 17.1，不含 Torch/Isaac。测试 Isaac 环境没有安装 AstrBotEX requirements。系统 Jazzy 保持 `/usr/bin/python3`。

## ROS 2 Bridge evidence

候选进程使用 Sim 自带 Jazzy 库，系统订阅进程使用 `/usr/bin/python3` 与 `/opt/ros/jazzy/setup.bash`。两者只在各自进程设置 `ROS_DOMAIN_ID=63`、`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。

发布图连接 OnTick、IsaacReadSimulationTime 和 ROS2PublishClock。系统订阅端实际收到三个递增样本；发布端完成 25 秒循环并正常进入关闭流程。证据：`ros_publisher.log`、`ros_subscriber.log`、`ros_clock_result.json`。此结果只验证 `/clock` 通路，不代表所有机器人消息或控制接口已验收。

## Preservation checks

- 最终重新读取基线 `pip freeze`，与开始时保存的文件 SHA256 完全一致：`a5e65288c7072de18278811eb495639ba2dad06f281267656ba2e4d8e51f5bd7`。
- 原 `/home/sssxy/Projects/IsaacLab` 的 Git 工作区保持干净，HEAD 仍为 `ffff603eafc6b74264a5261cc0183d6a65390d78`。构建、安装均使用隔离副本。
- 最终 `pip check` 仍是前述 3 项声明冲突，没有把它写成通过。
- 主工程既有三份 AstrBotEX Markdown 修改，以及未跟踪的 `docs/`、`ros2_ws/` 保留。没有暂存、提交或推送。
- 未执行针对基线、系统 Python、ROS 安装或驱动的安装/卸载。Git 与 freeze 检查不是对系统全部文件的哈希审计。

## Other warnings and limits

- GUI 数值记录显示倾斜摆杆在零 action 下的角度跨度约 1.2647 rad，但这只证明状态变化。两张重试截图未提供可见场景证据，不将其写成 GUI PASS；重力运动也不证明 effort 有效。

- 内核多次记录 `nvme nvme1: I/O ... timeout, completion polled`。项目/环境所在 ext4 文件系统是 `/dev/nvme1n1p6`。Sim 曾处于 `folio_wait_bit_common`。这是独立的系统稳定性风险，不能通过调整 Python pins 证明已解决。
- Sim 冷启动约 234 s。GUI 首次启动约 263 s。未修改内核、NVMe、电源、IOMMU 或驱动配置。
- 日志还包括重复注册 `grpc/health/v1/health.proto`、TGS external-force 配置、visualizers 缺少 extension.toml、MaterialX、USD/USDRT 与观察组默认映射警告。
- GUI 出现 Fabric 接口 v0.16/v0.14 以及渲染 transform 同步警告。尚未定位这些警告与 effort 问题的因果关系。
- [patch1 官方发布说明](https://github.com/isaac-sim/IsaacLab/releases/tag/v3.0.0-beta2.patch1) 声明支持 Sim 6.0.1。版本支持声明不能替代本机功能验收。

## Next action

继续使用原环境，不把候选环境接入 AstrEX 开发或训练：

```bash
conda activate isaaclab60
```

基线仍有 torchaudio 版本不匹配及本会话启动未复现的问题，需要单独处理。候选的 effort 问题需要后续上游修复或另行批准的版本方案。NVMe 超时需要独立的系统排查，本次没有越界修改。

复现候选失败时，可在新的输出目录中运行隔离工作目录下的 `candidate_python.sh --lab .../cartpole_effort.py`。不要把候选测试结果当作训练有效性的证明。
