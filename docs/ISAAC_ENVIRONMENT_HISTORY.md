# Isaac 环境历史

本文件区分开发默认环境与历史验证环境。当前切换状态见 [开发基线](ISAAC_51_DEV_BASELINE.md)。

| 环境 | Sim | Lab | Torch | 状态与用途 |
|---|---|---|---|---|
| `isaaclab232_test` | 5.1.0.0 | v2.3.2 | 2.7.0+cu128 | **当前 AstrEX 开发默认基线**（BASELINE_CUTOVER = PASS） |
| `isaaclab60` | 6.0.0.1 | Beta2 patch1 | 2.10.0+cu128 | HISTORICAL；env removed after snapshot（见 [`docs/env/legacy/`](env/legacy/README.md)） |
| `isaaclab60_6010test` | 6.0.1.0 | Beta2 patch1 | 2.11.0+cu128 | HISTORICAL；env removed after snapshot（见 [`docs/env/legacy/`](env/legacy/README.md)） |

`isaaclab232_test` 的版本期望、两处最小功能修复（`zero_effort` 张量类型、`reset` 的 OmniGraph 编辑寻址）、图生命周期规则（图 = 进程生命周期，reset = episode 生命周期）、command gate 与 fault-latch 语义，以及严格 domain preflight 的定位限制，见 [开发基线](ISAAC_51_DEV_BASELINE.md)。该文件同时说明：严格 preflight 只作为 cutover 门禁，不得成为日常开发的永久阻塞条件。

历史验证报告（ROS/RL/GUI/环境修复）已标记为 HISTORICAL，不再代表当前基线；它们的结论未被修改。

当前基线环境的原始快照在 [`docs/env/current/`](env/current/)，两个 Isaac 6 环境的删除前 package snapshot 与说明在 [`docs/env/legacy/`](env/legacy/README.md)。历史验证的原始大数据已归档到 `/data/shared/AstrEX_project_data/archives/isaac/2026-09-baseline-consolidation/`，原路径与现路径的映射见该目录的 `MANIFEST.json`（含保留在原处与被删除的路径）。

两个 Isaac 6 环境的历史同条件 effort 测试均 FAIL。该结果没有证明 Sim 版本是原因。详见 [Isaac 历史验证（合并记录）](ISAAC_HISTORICAL.md#4-isaac-6-physx-joint-effort-ab)。

Isaac 5.1 的完整验收和限制保留在 [Isaac 历史验证（合并记录）](ISAAC_HISTORICAL.md#1-isaac-51--isaac-lab-232-基线验收2026-09-17)；四份早期报告的全文归档在 data archive 的 `historical_reports/`，长期稳定性结论不因开发用途切换而改写。

本次不删除、重命名或克隆任何环境。默认入口不使用 Isaac 6。后续迁移必须建立独立环境并完成同等功能验收，不在当前基线上原地升级。
