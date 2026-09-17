# Isaac PhysX joint-effort A/B Result

STATUS: HISTORICAL
Current baseline: [docs/ISAAC_51_DEV_BASELINE.md](ISAAC_51_DEV_BASELINE.md)
Historical raw evidence was archived to:
`/data/shared/AstrEX_project_data/archives/isaac/2026-09-baseline-consolidation/`
See `MANIFEST.json` for original-path mapping (includes paths kept in place and paths deleted).

日期：2026-09-17，Asia/Shanghai。

## 结论

| 环境 | Sim | 本次结果 | 有效轨迹 | 正输入达标 | 负输入达标 |
|---|---|---|---|---|---|
| A：isaaclab60 | 6.0.0.1 | **FAIL** | 有 | 1/16 | 0/16 |
| B：isaaclab60_6010test | 6.0.1.0 | **FAIL** | 有 | 1/16 | 0/16 |

**未观察到本测试中的版本相关差异。** 两边原始轨迹 JSON 和测试结果 JSON 分别逐字节一致。

不标记 `SIM_6010_PHYSX_REGRESSION_SUSPECTED`。本次结果不支持只归因于 Sim 6.0.1.0，也不证明所有功能没有版本差异。

推荐保留 `isaaclab60` 为既有基线，两个环境均保留。此建议不是 effort 可用性背书。两边都未通过本项控制验收。

## 固定条件与运行

- 两份 Lab 均为 Beta2 patch1，提交 `ffff603eafc6b74264a5261cc0183d6a65390d78`。
- 两边运行同一份既有 `cartpole_effort.py`，没有编辑或复制修改该脚本。
- 任务：`Isaac-Cartpole-Direct-v0`。后端：显式 `PhysxCfg()`。设备：`cuda:0`。无 GUI。
- 16 个环境，seed 42，初始 pole angle 范围 `[0, 0]`。两边三组输入的初始关节位置和速度均为零，且完全一致。
- 关节：`slider_to_cart`，实测索引 0。物理步长 `1/120 s`，decimation 2，action 步长 `1/60 s`。
- 输入依次为 0、+0.5、-0.5，action scale 为 100，目标分别为 0、+50、-50 N。
- cart actuator：ImplicitActuator，stiffness 0，damping 10，effort limit 400 N。
- 每组重置后最多推进 60 个 action 步。任一环境 terminated 或 truncated 时停止该组。
- 任务 reset 条件保持原样：cart 超过 3 m、pole 角度超过 π/2，或达到 episode 时限。
- A 运行完成后才启动 B。两边使用相同启动脚本，清除 `VIRTUAL_ENV`、`PYTHONPATH`、`LD_LIBRARY_PATH`，统一线程变量。
- 每次运行上限 600 秒。A 用时 353.40 秒，B 用时 8.16 秒。本次不分析启动时长差异。

两边均完成有效轨迹采集，并触发 effort 验收失败断言。关闭路径返回 0，不能把该退出码当作 PASS。

两边不是只有 Sim 一个变量不同：A 使用 torch 2.10.0+cu128，B 使用 2.11.0+cu128。没有为了实验修改任何包。

## 配置指纹

下列 SHA256 在两边一致。完整配置清单见输出目录中的 `metadata.json`。

| 文件 | SHA256 |
|---|---|
| cartpole_effort.py | `974eaa4378daaab6a4e6c69b57af3a3ef07e041caa840c41c822d4bc41e3b0cb` |
| cartpole_env.py | `94f51c81be263b8b19254da47227d220c47b93d8d7d592e2c9e378d6dcbbd553` |
| cartpole_env_cfg.py | `88b5526fcd5cc72ed3001f658585e50b1cb7f31830356ec1a0a50a2d97497165` |
| robots/cartpole.py | `c8ce39af31ccd1f27c8f036085a92660572e073da5b67d612800d247bd14c3fb` |
| physx_manager_cfg.py | `a09ae907d6e46cf466e0b764be56753e6e1e554755aa1ea5fa3ac4863d70aed3` |
| apps/isaaclab.python.kit | `56bb61ddf2ca9644bdc9d7be6132dfca6d8be61e234865c1307b17590011fcb7` |

## 统一判据与逐环境结果

两边正输入都在第 19 步触发 reset。零输入与负输入均完成 60 步，无 reset。

统一比较三组输入、两套环境共同的最后一个 reset 前步：**第 18 步，0.3 s**。该步所有 terminated/truncated 均为 false。

Δposition 和 Δvelocity 分别是有输入轨迹减去本环境零输入轨迹。正负输入都必须方向正确。每个环境都必须满足位移 >0.001 m、速度 >0.01 m/s，且数值有限。

下表同时适用于 A、B，因为原始轨迹完全一致。数字显示经过舍入，判定使用未舍入数据。

| env | +0.5 Δposition (m) | +0.5 Δvelocity (m/s) | + 判定 | -0.5 Δposition (m) | -0.5 Δvelocity (m/s) | - 判定 |
|---|---|---|---|---|---|---|
| 0 | -1.71203e-7 | 4.00852e-4 | FAIL | 1.72572e-7 | -4.00846e-4 | FAIL |
| 1 | -2.86566e-9 | 1.35294e-6 | FAIL | 4.86084e-9 | -1.32360e-6 | FAIL |
| 2 | -6.26629e-10 | 1.32375e-6 | FAIL | -6.11379e-9 | -1.32393e-6 | FAIL |
| 3 | -1.71203e-7 | 4.00852e-4 | FAIL | 1.72572e-7 | -4.00846e-4 | FAIL |
| 4 | -1.71948e-7 | 4.00854e-4 | FAIL | 1.71185e-7 | -4.00851e-4 | FAIL |
| 5 | -2.87240e-9 | 1.35259e-6 | FAIL | 4.87236e-9 | -1.32303e-6 | FAIL |
| 6 | -6.41005e-10 | 1.32345e-6 | FAIL | -6.09064e-9 | -1.32299e-6 | FAIL |
| 7 | -1.71948e-7 | 4.00854e-4 | FAIL | 1.71185e-7 | -4.00851e-4 | FAIL |
| 8 | -1.68917e-7 | 4.00845e-4 | FAIL | 1.70979e-7 | -4.00850e-4 | FAIL |
| 9 | -2.87517e-9 | 1.35302e-6 | FAIL | 4.86578e-9 | -1.32366e-6 | FAIL |
| 10 | -6.26619e-10 | 1.32363e-6 | FAIL | -6.10627e-9 | -1.32332e-6 | FAIL |
| 11 | 0.858566 | 3.43462 | PASS | 3.66955e-7 | -4.01309e-4 | FAIL |
| 12 | -1.71203e-7 | 4.00852e-4 | FAIL | 1.72572e-7 | -4.00846e-4 | FAIL |
| 13 | -2.86566e-9 | 1.35294e-6 | FAIL | 4.86084e-9 | -1.32360e-6 | FAIL |
| 14 | -6.26629e-10 | 1.32375e-6 | FAIL | -6.11379e-9 | -1.32393e-6 | FAIL |
| 15 | -1.71203e-7 | 4.00852e-4 | FAIL | 1.72572e-7 | -4.00846e-4 | FAIL |

第 18 步正/负 target 均为 +50/-50 N。环境 0 的 applied effort buffer 为 +49.995975/-49.995987 N。正输入环境 11 为 15.275539 N，其余正输入环境接近 50 N。负输入全部接近 -50 N。

完整逐步数据包括两个 joint 的绝对位置、速度、target、applied effort buffer，以及每个环境的 reset 标志。step 0 只记录初始位置和速度，没有伪造尚未施加输入的 effort 记录。

## 两边失败后的只读实现检查

1. **Action 与 joint**：任务把 action 乘以 100，再通过 `_cart_dof_idx` 写入 effort target。日志与结果确认 cart joint 索引为 0。未发现脚本把 action 写入 pole 的证据。
2. **写入与步进**：`DirectRLEnv.step()` 依次调用 `_apply_action()`、`scene.write_data_to_sim()`、`sim.step()` 和 `scene.update()`。测试没有跳过正常任务步进链。
3. **Actuator**：ImplicitActuator 原样返回控制目标。它另行计算近似 effort，包含 damping 项。`applied_torque` 是模型估计缓冲区，不是独立测力结果。缓冲区非零不能证明 PhysX 实际施力。
4. **PhysX 写入**：标准 actuator 路径经过 `update_targets`，再调用 `set_dof_actuation_forces`。本轮未在该边界添加探针，不能确认底层收到的数组布局和环境映射正确。
5. **状态与 reset**：任务步进会先执行自动 reset，再返回状态。测试保存 reset 标志，并排除触发 reset 的第 19 步。JSON 使用 `.cpu().tolist()` 保存快照，不是保留可变张量引用。
6. **初态限制**：两边三组关节初始状态均相同，但这不证明 PhysX 所有内部 solver 状态都相同。固定顺序 0→正→负未改变，也未追加新进程分组实验。

当前只能确认共享测试链下的实际响应不满足判据。actuator→PhysX 写入边界、环境映射、状态读取与 reset 内部状态仍是后续检查候选，尚未定位根因。本轮未修改实现，也未追加变体实验。

## 证据与保护检查

输出根目录：`/home/sssxy/Projects/physx_ab_JPThAicl`。

- [A 原始轨迹](/home/sssxy/Projects/physx_ab_JPThAicl/isaaclab60/effort_trajectories.json)
- [B 原始轨迹](/home/sssxy/Projects/physx_ab_JPThAicl/isaaclab60_6010test/effort_trajectories.json)
- [逐环境比较](/home/sssxy/Projects/physx_ab_JPThAicl/comparison.json)
- [运行信息与指纹](/home/sssxy/Projects/physx_ab_JPThAicl/metadata.json)
- 各环境子目录保留 `run.log`、`effort_result.json`、`freeze_before.txt`。

两份轨迹 SHA256 均为 `73856ce9c12604a11228ca76c7f68716c5559e4a199b422d7d649a549303c3fa`。没有复制 A 的结果给 B。两边从不同解释器独立运行相同脚本。

两套环境运行前后的 pip freeze 完全一致。两份 Lab 工作区保持干净。原测试脚本指纹未变化。

没有安装或卸载包，没有修改 Lab、AstrBotEX 或系统配置。没有执行 NVMe/I/O/CUDA benchmark、PPO、commit 或 push。报告区分了实测结果、实现证据与未确认的原因。
