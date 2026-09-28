# HG-H0 摩擦（PARAMETER GAP + MODEL GAP）

**何が実測不足か**: 腹面インサートと家の床（フローリング / ラグ / カーペット）の摩擦。前 / 後 / 左 / 右、静止 / 動、ばらつき。
さらに、異方性摩擦の**法則の形**そのもの（MODEL GAP）。

**なぜ効くか**: Pure Snake を続けるか Wheel Belly に戻すか、6 本目を Head Yaw と Body Yaw のどちらに使うかが、ここで決まる。

## モデル

`simulation/planar_friction.py`（PLANAR_FRICTION_SIM）。平面・準静的、方向別クーロン摩擦（Hu et al. 2009 の形 = decoupled と、最大散逸の楕円 = ellipse）。
形は指令どおり、慣性なし、各瞬間に 合力 0・モーメント 0。

検証（2026-09-29）:
- 刻み 24 / 32 / 48 / 96、接地点 4 / 8 で前進の差 < 0.4%
- 全方向の μ を 0.5〜4 倍しても前進は同じ、トルクは倍率に比例（厳密に）
- MuJoCo（elliptic cone）と ellipse の平面モデルは前進がよく一致（235 対 215、87 対 109、99 対 92 mm/s）。
  MuJoCo 既定の pyramidal cone は低い異方性で前進を多めに出す（等方で 40 mm/s、理論は 0）
- 等方の摩擦では前進しない（理論どおり）
- 釣り合いの残差は体重の 0.5% 以内で受け入れる。高い比（r > 7）では一部の瞬間に 0.2% 程度で止まる点がある。全 run の約 1.5% は収束せず、判定から除いている
- **既知の副作用**: ellipse で r > 20 の FW5 と FW7_YAW6 は、最速の歩容が収束せずに除かれ、`plots/speed_vs_ratio.png` で速さが落ちて見える。判定に効く r = 2〜6 の範囲の外

## ファイル

| ファイル | 中身 |
|---|---|
| `assumptions.yaml` | 構成・合否の基準・床ごとの事前分布（**探索用の幅**）・車輪・法則の事前確率 |
| `sweep_config.yaml` | S1（比の掃引）・S2（感度）・S3（Monte Carlo）の設定 |
| `run.py` | 掃引・判定書の生成・実測 CSV の取り込み（`--measured`）・保存結果から作り直し（`--from-results`） |
| `results/` | 生の run（CSV）と境界（JSON）。実測は `results/measured/`（生データの控えは `raw/`） |
| `plots/` | 前進 vs 比、床ごとの読み、感度 |
| `decision_boundary.md` | **自動生成**。比の境界・トルクの境界・Monte Carlo・不合格の理由・トルク上限の誤差への感度 |
| `hardware_test_plan.md` | 最小の実測（45° の滑り方向試験を追加、後ろ向きは 1 回に削減） |

## 実行

```
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H0_friction\run.py                 # 全部（14 並列で約 30 分）
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H0_friction\run.py --from-results  # 判定書と図だけ
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H0_friction\run.py --measured hardware\prototypes\H0_friction\h0_friction_YYYYMMDD.csv
```

実測の取り込み: 生データの保存 → 検証（必須列・範囲・回数・ばらつき・鱗の向き）→ 当てはめ（比の 95% ブートストラップ区間、45° 試験から法則を推定）
→ 5 構成 × 比の区間の下端 / 中央 / 上端 で再計算 → 床ごとの読み → `ai-outbox/handoffs/<date>_HG-H0_measured.md` に追記。

## 読み方の注意

- Monte Carlo の割合は**事前分布（探索用の幅）でほぼ決まる**。「フローリングで Wheel が 82%」は、クーポンの比が 1〜3 だろうという仮定の帰結で、床の予測ではない。
  判定を確定させるのは H0 の実測の比。
- 旋回半径の基準 600 mm、巡回の速さ 50 mm/s は ASSUMPTION（製品側の要件。User / Design が決める）。
- 摩擦の値の実測は、クーポン × 床 × 条件に限って HARDWARE_VERIFIED。前進の予測は PLANAR_FRICTION_SIM のまま。
