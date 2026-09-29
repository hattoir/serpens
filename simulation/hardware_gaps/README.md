# simulation/hardware_gaps — 実機を待つあいだの近似環境

「ここから先は実機が要る」となったとき、止まらずに、その実機試験を近似する環境を作って
**感度・パラメータ探索・失敗条件の探索**を行い、**どの実測値を取れば判断が確定するか**まで絞る（User 指示 2026-09-29）。

**ここの結果はすべてシミュレーション。HARDWARE_VERIFIED に上げない。**
実測値そのもの（摩擦係数・サーボの電流など）は、取り込んだ時点で「その個体・その条件に限って」HARDWARE_VERIFIED。

| Gap | 分類 | 何が不足か | 状態 |
|---|---|---|---|
| [HG-H0_friction](HG-H0_friction/) | PARAMETER + MODEL | 腹面と床の方向別摩擦、異方性摩擦の法則の形 | 掃引・Monte Carlo・判定境界・試験計画・取り込み **済み** |
| [HG-H1_actuator](HG-H1_actuator/) | PARAMETER + SAFETY | C044 のトルク・電流・熱・上限の効き・衝撃 | 同上 **済み** |
| [HG-H2_sensor_head](HG-H2_sensor_head/) | SENSOR | 実カメラの FOV・ピント・歪み・ぶれ・露出・照明 | 1 変数ずつの感度 **済み**、実画像の取り込み口あり |
| [HG-S1_contact_safety](HG-S1_contact_safety/) | SAFETY | 巻き付き・圧迫・引っ張り・曲げ・踏みつけ・持ち上げの力 | worst-case の解析 **済み**（しきい値は User） |
| [HG-C1_cable_routing](HG-C1_cable_routing/) | MODEL + PARAMETER | 関節渡りのケーブルの長さ変化・たわみ・曲げ半径 | 幾何の掃引 **済み** |
| [HG-S2_yaw_sum_limit](HG-S2_yaw_sum_limit/) | SAFETY + MODEL | 角度合計の上限 145 / 150 / 160 / 170 / 180° | 掃引 **済み**。**145° を実装**（3 層）、上げる evidence は無い |
| [HG-S3_torque_limiter](HG-S3_torque_limiter/) | SAFETY + PARAMETER | 機械式トルクリミッターの滑りトルク 0.4〜1.5 N·m | 掃引 **済み**。窓 0.7〜1.0 N·m、config に設計値 |

各フォルダ（HG-S1・HG-S2・HG-S3・HG-C1 は解析式 / 小さな掃引なので sweep_config / plots を省略）: `README.md` / `assumptions.yaml` / `sweep_config.yaml` / `run.py` / `results/` / `plots/` /
`decision_boundary.md`（run.py が自動生成）/ `hardware_test_plan.md`。

## 共有データ

| ファイル | 中身 |
|---|---|
| `safety_thresholds.yaml` | 接触しきい値の**暫定値**（PROVISIONAL / SAFETY_UNVERIFIED）。候補・導出・未確認。baseline = 候補の最小。**緩める変更は User 承認** |
| `child_anthropometry.yaml` | 子どもの身体寸法（Snyder 1977 の生データから独立に再計算）。**2 歳未満は空白** |
| `../../docs/reports/2026-09-29_child_safety_sources.md` | 出典の全文（調査エージェントの報告。一部だけ独立に検証） |

## Gap どうしのつながり

- H0 の歩容負荷（関節トルク・角速度の時系列）を H1 が使う。**トルクは μ_横 に比例**するので、H0 の μ_横 と H1 の上限の効きの積で可否が決まる
- H0 の Monte Carlo は、トルク上限の誤差（H1 の e = ±20 / 40%）への感度も出す
- H2 はカメラ高さ・下向き角を振る。首 J1 の姿勢（H0 / H1 の外）と頭の設計に効く

## 実測の取り込み（共通の流れ）

生データを `results/measured/raw/` に控える → 検証 → 当てはめ → 再計算 → 判定の更新 → `ai-outbox/handoffs/<date>_HG-*_measured.md` に追記。
**安全に関わる設定（トルク制限レジスタ比など）は自動で書き換えない**（提案を出して User 承認）。

## 使っているモデル

| モデル | ファイル | 検証 |
|---|---|---|
| 平面・準静的の方向別クーロン摩擦 | `simulation/planar_friction.py` | 離散化の収束、比のスケール不変性、等方で前進 0、MuJoCo（elliptic cone）との一致 |
| MuJoCo（既存） | `simulation/mujoco/` | 慣性・転倒を見るとき。**異方性の法則は pyramidal cone だと低い比で前進を多めに出す** |
| サーボ・電源・熱の線形モデル | `HG-H1_actuator/run.py` | 式の単体テスト。係数はすべて prior |
| 合成画像 + 実カメラの劣化 | `serpens/floorwatch/synthetic.py` + `HG-H2_sensor_head/run.py` | 既存の検出器をそのまま使う |
