# HG-H0 摩擦 — 判定の境界（自動生成 2026-09-29 02:58。source = PLANAR_FRICTION_SIM。**実測ではない**）

`run.py` が書き出す。手で直さない（`hardware_test_plan.md` と `README.md` は手で書く）。

基準（ASSUMPTION）: 前進 ≥ 50 mm/s、旋回半径 ≤ 600 mm（振幅 + γ ≤ 50°）、直進のずれ ≤ 旋回の余力 × 0.5、トルク ≤ 0.45 N·m × 0.8。歩容は構成・摩擦ごとに選び直す。0.5 Hz。

## 1. 比 r = μ_横 / μ_前 の境界（トルク以外の基準をすべて満たす最小の r）

準静的クーロン摩擦では、動きは比だけで決まる（全方向の μ を同じ倍率にしても同じ。2026-09-29 に数値で確認）。

| 構成 | decoupled（全基準）| decoupled（速さだけ） | ellipse（全基準）| ellipse（速さだけ） |
|---|---|---|---|---|
| FW5 | 5.96 | 5.96 | 2.65 | 2.65 |
| FW6_HEADYAW | 5.96 | 5.96 | 2.65 | 2.65 |
| FW6_YAW5 | 3.66 | 3.66 | 2.25 | 2.25 |
| FW7_YAW5_HEADYAW | 3.66 | 3.66 | 2.25 | 2.25 |
| FW7_YAW6 | 2.25 | 2.25 | 1.63 | 1.63 |

- **decoupled**: r ≥ 5.96 なら Head Yaw（FW6_HEADYAW）で足りる。3.66 ≤ r < 5.96 は Body Yaw（FW6_YAW5）が要る。r < 3.66 は 6 モーターの Pure Snake では不足 → Wheel Belly（受動輪で r を大きくする）
- **ellipse**: r ≥ 2.65 なら Head Yaw（FW6_HEADYAW）で足りる。2.25 ≤ r < 2.65 は Body Yaw（FW6_YAW5）が要る。r < 2.25 は 6 モーターの Pure Snake では不足 → Wheel Belly（受動輪で r を大きくする）

## 2. トルクの境界（μ_前 の上限。トルクは μ に比例）

| 構成 | 法則 | r=2 | r=4 | r=8 |
|---|---|---|---|---|
| FW6_HEADYAW | decoupled | μ_前 ≤ 0.50 | μ_前 ≤ 0.30 | μ_前 ≤ 0.23 |
| FW6_YAW5 | decoupled | μ_前 ≤ 0.45 | μ_前 ≤ 0.31 | μ_前 ≤ 0.25 |
| FW6_HEADYAW | ellipse | μ_前 ≤ 0.52 | μ_前 ≤ 0.30 | μ_前 ≤ 0.20 |
| FW6_YAW5 | ellipse | μ_前 ≤ 0.47 | μ_前 ≤ 0.31 | μ_前 ≤ 0.20 |

## 3. 床ごとの Monte Carlo（事前分布は `assumptions.yaml`。**探索用の幅で、床の実測ではない**）

| 床 | 法則 | n | HEAD_YAW_OK | BODY_YAW_NEEDED | WHEEL_FALLBACK | NONE |
|---|---|---|---|---|---|---|
| WOOD | decoupled | 73 | 0% | 0% | 100% | 0% |
| WOOD | ellipse | 87 | 13% | 20% | 68% | 0% |
| WOOD | (両方) | 160 | 7% | 11% | 82% | 0% |
| RUG | decoupled | 83 | 0% | 0% | 70% | 30% |
| RUG | ellipse | 77 | 5% | 8% | 64% | 23% |
| RUG | (両方) | 160 | 2% | 4% | 67% | 27% |
| CARPET | decoupled | 81 | 0% | 0% | 21% | 79% |
| CARPET | ellipse | 79 | 0% | 1% | 25% | 73% |
| CARPET | (両方) | 160 | 0% | 1% | 23% | 76% |

### 不合格の理由（サンプル数。torque = どの歩容も上限超え / speed = 遅い / drift = 速い歩容はあるがずれを直せない / turn = 旋回半径）

| 床 | 構成 | 合格 | torque | speed | drift | turn |
|---|---|---|---|---|---|---|
| WOOD | FW5 | 13/160 | 0 | 147 | 0 | 25 |
| WOOD | FW6_HEADYAW | 11/160 | 0 | 149 | 0 | 6 |
| WOOD | FW6_YAW5 | 28/160 | 0 | 132 | 0 | 7 |
| WOOD | FW7_YAW5_HEADYAW | 20/160 | 0 | 139 | 1 | 9 |
| WOOD | FW7_YAW6 | 51/160 | 0 | 105 | 0 | 22 |
| WOOD | WHEEL_FW5 | 160/160 | 0 | 0 | 0 | 0 |
| RUG | FW5 | 12/160 | 15 | 132 | 1 | 40 |
| RUG | FW6_HEADYAW | 4/160 | 50 | 106 | 0 | 26 |
| RUG | FW6_YAW5 | 10/160 | 22 | 126 | 1 | 48 |
| RUG | FW7_YAW5_HEADYAW | 5/160 | 59 | 96 | 0 | 33 |
| RUG | FW7_YAW6 | 7/160 | 48 | 93 | 0 | 61 |
| RUG | WHEEL_FW5 | 113/160 | 0 | 47 | 0 | 0 |
| CARPET | FW5 | 4/160 | 46 | 109 | 1 | 34 |
| CARPET | FW6_HEADYAW | 0/160 | 100 | 60 | 0 | 21 |
| CARPET | FW6_YAW5 | 1/160 | 73 | 85 | 0 | 35 |
| CARPET | FW7_YAW5_HEADYAW | 0/160 | 113 | 46 | 0 | 18 |
| CARPET | FW7_YAW6 | 1/160 | 110 | 44 | 0 | 27 |
| CARPET | WHEEL_FW5 | 37/160 | 0 | 123 | 0 | 0 |

### トルク上限の誤差（HG-H1）への感度: 各床で「6 本目の読み」の割合

| 床 | 上限 × | HEAD_YAW_OK | BODY_YAW_NEEDED | WHEEL_FALLBACK | NONE |
|---|---|---|---|---|---|
| WOOD | 0.6 | 2% | 5% | 93% | 0% |
| RUG | 0.6 | 0% | 1% | 70% | 29% |
| CARPET | 0.6 | 0% | 0% | 23% | 77% |
| WOOD | 0.8 | 4% | 9% | 88% | 0% |
| RUG | 0.8 | 2% | 1% | 69% | 28% |
| CARPET | 0.8 | 0% | 0% | 23% | 77% |
| WOOD | 1 | 7% | 11% | 82% | 0% |
| RUG | 1 | 2% | 4% | 67% | 27% |
| CARPET | 1 | 0% | 1% | 23% | 76% |
| WOOD | 1.2 | 8% | 12% | 80% | 0% |
| RUG | 1.2 | 6% | 9% | 59% | 26% |
| CARPET | 1.2 | 1% | 3% | 22% | 74% |
| WOOD | 1.4 | 8% | 12% | 80% | 0% |
| RUG | 1.4 | 9% | 14% | 54% | 23% |
| CARPET | 1.4 | 2% | 7% | 21% | 70% |

## 4. 感度（r = 基準値のまわり。速さの変化）

| 構成 | 法則 | r | パラメータ | 値 → 前進 mm/s |
|---|---|---|---|---|
| FW6_HEADYAW | decoupled | 2 | fb_ratio | 0.5→8, 0.8→8, 1→8, 1.5→8, 3→9 |
| FW6_HEADYAW | decoupled | 2 | lr_asym | 0→8, 0.05→9, 0.1→10, 0.2→12, 0.3→15 |
| FW6_HEADYAW | decoupled | 2 | point_sigma | 0→8, 0.05→8, 0.15→9, 0.25→9 |
| FW6_HEADYAW | decoupled | 4 | fb_ratio | 0.5→26, 0.8→26, 1→26, 1.5→26, 3→26 |
| FW6_HEADYAW | decoupled | 4 | lr_asym | 0→26, 0.05→26, 0.1→27, 0.2→28, 0.3→30 |
| FW6_HEADYAW | decoupled | 4 | point_sigma | 0→26, 0.05→27, 0.15→27, 0.25→30 |
| FW6_HEADYAW | ellipse | 2 | fb_ratio | 0.5→20, 0.8→20, 1→20, 1.5→20, 3→20 |
| FW6_HEADYAW | ellipse | 2 | lr_asym | 0→20, 0.05→21, 0.1→21, 0.2→22, 0.3→24 |
| FW6_HEADYAW | ellipse | 2 | point_sigma | 0→20, 0.05→21, 0.15→22, 0.25→21 |
| FW6_HEADYAW | ellipse | 4 | fb_ratio | 0.5→99, 0.8→99, 1→99, 1.5→99, 3→99 |
| FW6_HEADYAW | ellipse | 4 | lr_asym | 0→99, 0.05→99, 0.1→97, 0.2→93, 0.3→85 |
| FW6_HEADYAW | ellipse | 4 | point_sigma | 0→99, 0.05→100, 0.15→100, 0.25→103 |
| FW6_YAW5 | decoupled | 2 | fb_ratio | 0.5→19, 0.8→19, 1→19, 1.5→19, 3→20 |
| FW6_YAW5 | decoupled | 2 | lr_asym | 0→19, 0.05→20, 0.1→22, 0.2→25, 0.3→28 |
| FW6_YAW5 | decoupled | 2 | point_sigma | 0→19, 0.05→19, 0.15→20, 0.25→20 |
| FW6_YAW5 | decoupled | 4 | fb_ratio | 0.5→54, 0.8→54, 1→54, 1.5→54, 3→54 |
| FW6_YAW5 | decoupled | 4 | lr_asym | 0→54, 0.05→55, 0.1→55, 0.2→55, 0.3→56 |
| FW6_YAW5 | decoupled | 4 | point_sigma | 0→54, 0.05→55, 0.15→57, 0.25→58 |
| FW6_YAW5 | ellipse | 2 | fb_ratio | 0.5→43, 0.8→43, 1→43, 1.5→43, 3→43 |
| FW6_YAW5 | ellipse | 2 | lr_asym | 0→43, 0.05→44, 0.1→44, 0.2→44, 0.3→43 |
| FW6_YAW5 | ellipse | 2 | point_sigma | 0→43, 0.05→44, 0.15→46, 0.25→44 |
| FW6_YAW5 | ellipse | 4 | fb_ratio | 0.5→154, 0.8→154, 1→154, 1.5→154, 3→154 |
| FW6_YAW5 | ellipse | 4 | lr_asym | 0→154, 0.05→154, 0.1→153, 0.2→147, 0.3→137 |
| FW6_YAW5 | ellipse | 4 | point_sigma | 0→154, 0.05→155, 0.15→158, 0.25→159 |

### 左右の差 → 直進のずれ（°/周期）と、使える γ での旋回の余力

| 構成 | 法則 | r | 左右差 a | ずれ | 余力 | 全基準 |
|---|---|---|---|---|---|---|
| FW6_HEADYAW | decoupled | 2 | 0 | -0.00 | 0.0 | NG |
| FW6_HEADYAW | decoupled | 2 | 0.05 | +1.84 | 0.1 | NG |
| FW6_HEADYAW | decoupled | 2 | 0.1 | +3.33 | 0.1 | NG |
| FW6_HEADYAW | decoupled | 2 | 0.2 | +6.54 | 0.2 | NG |
| FW6_HEADYAW | decoupled | 2 | 0.3 | +9.71 | 0.2 | NG |
| FW6_HEADYAW | decoupled | 4 | 0 | +0.00 | 0.3 | NG |
| FW6_HEADYAW | decoupled | 4 | 0.05 | +1.53 | 0.2 | NG |
| FW6_HEADYAW | decoupled | 4 | 0.1 | +2.97 | 0.2 | NG |
| FW6_HEADYAW | decoupled | 4 | 0.2 | +5.79 | 0.1 | NG |
| FW6_HEADYAW | decoupled | 4 | 0.3 | +8.85 | 0.1 | NG |
| FW6_YAW5 | decoupled | 2 | 0 | +0.00 | 1.1 | NG |
| FW6_YAW5 | decoupled | 2 | 0.05 | +1.10 | 2.3 | NG |
| FW6_YAW5 | decoupled | 2 | 0.1 | +2.72 | 0.7 | NG |
| FW6_YAW5 | decoupled | 2 | 0.2 | +4.99 | 1.4 | NG |
| FW6_YAW5 | decoupled | 2 | 0.3 | +7.69 | 1.0 | NG |
| FW6_YAW5 | decoupled | 4 | 0 | +0.00 | 3.2 | NG |
| FW6_YAW5 | decoupled | 4 | 0.05 | +0.89 | 3.1 | NG |
| FW6_YAW5 | decoupled | 4 | 0.1 | +1.64 | 4.2 | NG |
| FW6_YAW5 | decoupled | 4 | 0.2 | +3.70 | 2.7 | NG |
| FW6_YAW5 | decoupled | 4 | 0.3 | +5.80 | 2.6 | NG |
| FW6_HEADYAW | ellipse | 2 | 0 | +0.00 | 2.5 | NG |
| FW6_HEADYAW | ellipse | 2 | 0.05 | +0.25 | 1.2 | NG |
| FW6_HEADYAW | ellipse | 2 | 0.1 | +0.57 | 1.3 | NG |
| FW6_HEADYAW | ellipse | 2 | 0.2 | +6.50 | 2.2 | NG |
| FW6_HEADYAW | ellipse | 2 | 0.3 | +10.39 | 2.1 | NG |
| FW6_HEADYAW | ellipse | 4 | 0 | -0.00 | 11.5 | OK |
| FW6_HEADYAW | ellipse | 4 | 0.05 | +0.64 | 11.4 | OK |
| FW6_HEADYAW | ellipse | 4 | 0.1 | +1.25 | 11.3 | OK |
| FW6_HEADYAW | ellipse | 4 | 0.2 | +2.73 | 10.7 | OK |
| FW6_HEADYAW | ellipse | 4 | 0.3 | +4.39 | 9.9 | OK |
| FW6_YAW5 | ellipse | 2 | 0 | -0.00 | 5.7 | OK |
| FW6_YAW5 | ellipse | 2 | 0.05 | +0.97 | 5.7 | OK |
| FW6_YAW5 | ellipse | 2 | 0.1 | +2.11 | 5.4 | OK |
| FW6_YAW5 | ellipse | 2 | 0.2 | +4.21 | 5.2 | NG |
| FW6_YAW5 | ellipse | 2 | 0.3 | +7.20 | 4.7 | NG |
| FW6_YAW5 | ellipse | 4 | 0 | +0.00 | 19.3 | OK |
| FW6_YAW5 | ellipse | 4 | 0.05 | +0.53 | 19.0 | OK |
| FW6_YAW5 | ellipse | 4 | 0.1 | +0.96 | 18.8 | OK |
| FW6_YAW5 | ellipse | 4 | 0.2 | +1.61 | 18.1 | OK |
| FW6_YAW5 | ellipse | 4 | 0.3 | +2.18 | 17.5 | OK |

## 図

- `plots/speed_vs_ratio.png`
- `plots/mc_reading_per_floor.png`
- `plots/sensitivity.png`
