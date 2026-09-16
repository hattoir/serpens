# 安全の数値 — 出典・導出・達成度

出典: `../蛇ロボット_HomeAI_構想設計書.pdf` **16章「安全性 — 4重の防御」**（v0.1, 2026-09）
作成 2026-09-14。設定は `config/robot.yaml` の `safety_limits`、検査は `tests/test_safety_limits.py`。

構想設計書の第一原則:

> 安全は最下層で保証する。AI がどれだけ誤判断しても、モーターに危険な指令が届かない。

そのため**上の層ほど強い**。ソフトの都合で下の層を緩めない。

| 層 | 方式 | このリポジトリの担当範囲 |
|---|---|---|
| 1 | 本質安全（機構） | 検算のみ（可動域・質量・曲げ半径）。実体は CAD 側 |
| 2 | 電気的制限 | **未着手**（電流制限・INA226・安全 MCU による電源遮断） |
| 3 | ファームウェア L0 | `serpens/link/device.py` と `firmware/serpens_esp32/` |
| 4 | ソフトウェア L1〜L3 | `serpens/behavior/`・`serpens/safety.py` |

## 1. 数値と導出

| 項目 | 構想設計書 | 本機の設定 | 導出・根拠 |
|---|---|---|---|
| トルク上限 | 1.2 N·m | `software_torque_limit_nm: 0.450` | **STS3215-C044（7.4V / 1:191）の REFERENCE 値**から。定格 5.2kgf·cm = 0.510N·m を超えない値を選んだ。ストール 16kgf·cm = 1.569N·m は**連続安全トルクとして使わない**。レジスタ比 = 0.450/1.569 = **0.287** |
| 最小曲げ半径 | 40 mm | software_operational_limit **±50°**、リンク 95mm | R = L/(2·tan25°) = **101.9 mm ≥ 40**。自由な穴の直径 2×(101.9−25) = **154 mm ≥ 80**（16章の要求を満たす） |
| 総質量 | 1.7 kg 以下 | 質量収支 **1.00 kg** | `mass_budget_g` の合計（バッテリ無しのテザー機） |
| 人の近くの速度 | 半径1m で 8 cm/s | `near_speed_limit_mm_s: 80` | 状態によらず常時かかる層で実装済み |
| サーボ停止温度 | 80 ℃ | 機体 **60 ℃** / 行動 55 ℃ | 手前で二段に落とす（55℃ で休憩 → 60℃ で緊急停止ラッチ） |
| 通信断 → 安全姿勢 | 3 秒 | **DRIVE TTL 300ms / heartbeat 400ms** | 二段構え。実測は `docs/phase2_acceptance.md` |
| 接触 → 脱力 | 20 ms | **800 ms**（PC 側検知） | **未達。** §3 |

### トルク上限から出る力

腕（隣接リンク 95mm）の先端で挟んだときの押し付け力:

```
F = T / r = 0.450 N·m / 0.095 m ≒ 4.7 N
```

⚠ この 0.450 N·m は **CAD 資料の REFERENCE 値（STS3215-C044, 7.4V）からの設計値**であって、
実測ではない（`SIMULATED_FROM_REFERENCE`）。`measured_safe_torque_nm` は **UNKNOWN**。
**実測するまで「安全トルクは 0.45N·m」と確定しない。**

### トルクの4つを混ぜない（`safety_limits.torque`）

| 名前 | 値 | 意味 |
|---|---|---|
| `rated_torque_reference_nm` | 0.510 | 定格（連続で出せるとされる REFERENCE 値） |
| `stall_torque_reference_nm` | 1.569 | ストール（起動・拘束時）。**連続安全トルクとして使わない** |
| `software_torque_limit_nm` | 0.450 | ソフトが出してよい上限（定格以下に取る） |
| `measured_safe_torque_nm` | **null** | 実測値。**UNKNOWN**。埋まるまで `hardware_verified: false`

首を持ち上げるのに要るのは 0.20 N·m（質量収支から: 0.2kg × 9.81 × 0.104m）なので、
上限を掛けても**5.8 倍の余裕**がある。「安全のために動けない」にはならない。

> ISO/TS 15066（協働ロボットの接触限界）と突き合わせるべきだが、規格本文を参照できていない。
> **12.4 N が痛み閾値に対してどうかは未検証。** 実機で指を模した荷重計で測ること。

## 2. 危険源ごとの現在地

| 危険源 | 構想設計書の対策 | 本機の状態 |
|---|---|---|
| 人への圧迫・巻き付き | 最小曲げ半径 40mm / 螺旋歩容の削除 / 質量上限 | **歩容が動かすのは水平ヨーだけ**（`test_gait_cannot_wrap_around_anything`）。上下に波を伝える歩容は存在しない |
| 衝突 | 1m 以内 8cm/s / 頭部にバネ | 速度制限は実装済み。頭部のコンプライアンスは機構側（未） |
| モーター暴走 | 安全 MCU が独立して電源遮断 | **未着手。** いまは ESP32 が自分で保持へ入るだけ（電源は切れない） |
| 通信障害 | 3秒で coil_and_hold | **400ms で保持**（より速い）。ただし**とぐろへは移らない** → §4 の判断 |
| AI の誤判断 | L3 は検証済みスキルのみ、関節軌道を書けない | 行動は歩容パラメータ（`DRIVE`）しか出せない。角度列を送る経路は無い |
| センサ故障 | ToF 3個・IMU 2個の冗長 | **未着手**（ToF 1個、IMU 無し） |
| 持ち上げられた | IMU で浮上検知 → 全脱力 | **未着手**（IMU が無い） |
| 落下・段差 | 頭部 ToF で 60mm の落差検知 | ToF は頭部 I/O にあるが、**落差判定は未実装** |
| バッテリ異常 | PSE 適合セル / BMS | **該当なし**（いまは 12V テザー） |
| 緊急停止 | 頭部・尾部の物理スイッチ（3秒長押しで電源遮断） | **未着手。** いまは PC の `E` キーと機体の watchdog のみ |
| 誤飲・挟み込み | 部品 32mm 以上 / 隙間 8mm 以下か 25mm 以上 | 機構側（未検証） |

## 3. 未達の項目（隠さない）

1. **接触 → 脱力 20ms**（要求の穴と時間予算は [contact_release_requirements.md](contact_release_requirements.md)）。いまは PC 側が負荷率 0.75 を 800ms 観測してから脱力する。
   20ms は PC を経由する限り届かない（テレメトリ 10Hz + 往復）。**機体側で負荷を見て切る必要がある。**
   → `link.faults` に負荷のしきい値を足し、ファームの制御周期（100Hz = 10ms）で判定する設計にする。
2. **電気的制限（層2）が丸ごと無い。** 電流検出も、独立電源でサーボバスを切る安全 MCU も無い。
   ファームが暴走したら止める手段が電源プラグしかない。→ 電子系設計（`docs/product_status.md` §5 #3）。
3. **物理の緊急停止スイッチが無い。** 人が手で止められる経路が無いのは、展示会でも問題。
4. **IMU が無い**ので「持ち上げたら脱力」が作れない。落下検知も無い。
5. ~~輪の内径が 80mm に届かない~~ → **解消。** 2026-09-16 に CAD の最新提案へ合わせ、
   胴体ヨーを **software_operational_limit ±50°（CONDITIONAL）** にした結果、
   曲げ半径 101.9mm・穴の直径 154mm となった。
   **可動域は意味の違う3つを別々に持つ**（`config/robot.yaml` の `joint_limit_policy`）:
   干渉の始まり 64.8〜64.9°（**clamp に使わない**）/ mechanical ±55°（PROVISIONAL）/
   software ±50°（CONDITIONAL）。`verified_with_cable` と `verified_with_hardware` は **false**。
   **副作用**: 旧とぐろ（329°巻き・J1=83°）は入らないので `legacy_poses`
   （SIMULATION_LEGACY_ONLY）へ移し、R03 では緩い弧（`rest_arc`, 240°）を休憩姿勢にした。
   展示用のとぐろは R03 の範囲で別途再設計する。

## 4. ここでの判断

- **通信断で「とぐろへ移る」ことはしない。** 構想設計書は `coil_and_hold` を挙げているが、
  通信が切れた直後に胴体 6軸を大きく動かすのは、PC が状況を見られない状態での大きな動作になる。
  本機は**その場で保持**する（ホーム姿勢へも動かさない）。とぐろが要るのは充電ドッキングの文脈で、
  そちらは人の操作か機体の自己判断が確立してから足す。
- **トルク比は「安全上限に対する割合」**に統一した。脱力演出（`poses.relax.torque_ratio: 0.6`）は
  0.6 × 0.287 = **0.17**（ストール参照値の 17%）になる。演出の経路から全力へ戻せない。
- 機体側の温度上限（60℃）は構想設計書の 80℃ より手前。サーボの最高温度の初期値は 70℃
  （`docs/sts3215_registers.md`）なので、**サーボ自身が止まるより先にこちらが止める**。

## 5. ELECTRICAL_SAFETY_GATE（実機の前に必ず通す）

`config/robot.yaml` の `safety_limits.electrical_safety_gate`。**8 項目すべてが INCOMPLETE（2026-09-16）。**
1 項目でも未完了なら `autonomy_blockers()` が実機の自律走行を拒否する（`serpens/electrical_gate.py`、
`tests/test_electrical_gate.py`）。COMPLETE にするには**証拠**（日付・条件・測定器）が要り、
状態だけ書き換えても通らない。シミュレーションの結果では埋めない。

| 項目 | 確かめること | 状態 |
|---|---|---|
| real_current | 9軸の実電流（待機・歩容・ストール） | INCOMPLETE |
| power_capacity | 電源容量と電圧降下（全軸同時の突入を含む） | INCOMPLETE |
| overcurrent_protection | 過電流保護が実際に働くこと | INCOMPLETE |
| wiring_heat | 配線・コネクタの発熱（連続運転） | INCOMPLETE |
| independent_power_cut | ESP32 に依存しないサーボ電源の遮断経路 | INCOMPLETE（回路が無い） |
| physical_estop | 人が手で押せる物理の緊急停止 | INCOMPLETE（部品が無い） |
| servo_temperature | 実サーボの温度上昇と 60℃ 停止の実働 | INCOMPLETE |
| real_stop_time | 緊急停止・通信断から実際に止まるまでの時間 | INCOMPLETE |

接触 → 脱力（20ms）は要求そのものが未確定なので別文書: [contact_release_requirements.md](contact_release_requirements.md)。
