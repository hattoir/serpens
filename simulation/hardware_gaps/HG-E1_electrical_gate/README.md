# HG-E1 電気層（ELECTRICAL_SAFETY_GATE 8 項目）

**ELECTRICAL_BUDGET_MC。すべて prior（ASSUMPTION）。実測 0 件（HARDWARE_VERIFIED = 0）。GATE の状態は書き換えない。**

| ファイル | 中身 |
|---|---|
| `assumptions.yaml` | 電流・電源・配線・熱・停止時間の prior（出どころつき。C044 の電流は UNKNOWN） |
| `run.py` | 解析 + Monte Carlo（seed 固定）。`python simulation/hardware_gaps/HG-E1_electrical_gate/run.py` |
| `decision_boundary.md` | **自動生成**。項目ごとの判断境界（電源の全抵抗・保護の設定電流・幹線の実効電流・サーボの温度・停止時間） |
| `results/e1_electrical.json` | 全数値 |

項目ごとの「測る量・最小の実機試験・Human Action・取り込み」は `docs/electrical_gate_boundary.md`。実測の取り込みは `tools/ingest_measurements.py`。
実測が来たら `assumptions.yaml` を置き換えて `run.py` を回し直す（判断境界の位置は prior の幅に従属する）。
