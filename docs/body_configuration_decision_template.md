# 展示機の身体構成 — 決定レポート（テンプレート。Pilot 後に埋める）

作成 2026-09-21。**未決定。** 6 / 8 / 10 Yaw と Limited Body Pitch 0 / 1 / 2 を、
Human Evaluation・Simulation・CAD・BOM・Safety で決める。Pilot 前にどれかを「最適」と書かない。

## 1. 比較表（Yaw 軸数）

| 観点 | 6 Yaw | 8 Yaw | 10 Yaw | 出どころ |
|---|---|---|---|---|
| Snake-likeness（median） | | | | VIDEO_HUMAN_EVALUATION（n = ） |
| Animacy | | | | 同上 |
| Approachability | | | | 同上 |
| Affection | | | | 同上 |
| Fear（低いほど良い） | | | | 同上 |
| Smoothness | | | | 同上 |
| 可視波数（2 波の Ω で） | 2.13（Ω=120°） | 2.21（Ω=90°） | 2.26（Ω=72°） | KINEMATIC_SIM（`output/body_compare.md`） |
| 前進 mm/周期（1 波 / 2 波） | 384 / 100 | 519 / 199 | 620 / 291 | KINEMATIC_SIM |
| 前進 mm/s SPEED_MATCHED で届くか | 基準 | | | KINEMATIC_SIM |
| 旋回 °/周期（γ0=20°、2 波） | 8.9 | 14.4 | 17.8 | KINEMATIC_SIM |
| MuJoCo（前進・飽和率・追従誤差） | | | | PHYSICS_SIM（未。yaw8/10 の MJCF は未生成） |
| 全長（同等リンク 95mm） | 850 | 1040 | 1230 | config |
| 全長（同等身体長） | 850 | 850（リンク 71mm） | 850（リンク 57mm） | config |
| サーボ数 | 9 | 11 | 13 | config |
| 推定質量 g（仮定） | 1000 | 1200 | 1400 | config（実測なし） |
| 推定コスト | | | | BOM（未） |
| CAD への影響 | 現行 R03 | 節の追加 / リンク短縮 | 同 | CAD 担当 |
| ソフトの複雑さ | 基準 | 関節数非依存で config のみ | 同 | SOFTWARE_VERIFIED |
| 推定負荷（関節速度 / 加速度） | | | | KINEMATIC_SIM（サーボ応答なし） |
| Safety（可動域・トルク上限・曲げ半径） | R03 の値 | 要再計算 | 要再計算 | HARDWARE_UNVERIFIED |

## 2. Limited Body Pitch（別実験。有力な Yaw 構成にだけ）

| | Pitch 0 | Pitch +1 | Pitch +2 |
|---|---|---|---|
| 目的 | 現行 | 体の部分的な持ち上げ・背腹方向の呼吸 | 同 + sidewinding の可能性 |
| 安全との切り分け | 巻き付き封印（曲げ半径 40mm / 上下波なし） | 要検討（ハード / 構想側へ差し戻し中） | 同 |
| ソフトの前提 | `gait.pitch_amplitude_deg` は 0 固定（テストが拒否） | 解禁しない（本 Phase の範囲外） | 同 |
| Human Evaluation | — | Yaw 決定後に別 Pilot | 同 |

## 3. 判断

（Pilot の結果を見てから書く。n が小さいので「傾向」と「未確定」を分けて書く。）

## 4. 動画では評価できていないこと

実際に近づいた時の恐怖・大きさ・音・振動・接触・距離感・見られている感覚 → `PHYSICAL_HUMAN_EVALUATION` で別に。
