# STATE — serpens

<!-- 作業のたびに更新する。ここが次のセッションの出発点になる。 -->

_last updated: 2026-09-14_

## Current Goal

安全の絶対値を実装に落とし終えた（2026-09-14）。次は
**「行動 → DRIVE → 機体が歩容生成 → 世界が動く」**を End-to-End で繋ぐ（RobotInterface）。
いま行動は 9軸の角度を直接サーボへ書いていて、駆動リンクを通っていない。

## Current Architecture

| 層 | 責任 | 主なファイル |
|---|---|---|
| 知覚 | ArUco で自己位置、YOLO で人。**外部固定カメラ前提** | `serpens/perception/` |
| 行動 | 内部状態 → 効用 → 10 状態 → `DriveCommand`（歩容パラメータ）としぐさ | `serpens/behavior/` |
| モーション | 歩容式・キーフレーム・呼吸を合成して**9軸の角度**を作る | `serpens/motion/` |
| セッション | 上記を 50Hz で回し、停止を出力へ届ける | `serpens/sim/session.py` |
| 安全（PC 側） | RUN / HOLD / DISABLED / EMERGENCY の分離とラッチ | `serpens/safety.py` |
| 駆動リンク | PC ⇄ 機体の契約。**機体側が自分で止まる** | `serpens/link/`, `docs/link_protocol.md` |
| 機体（模擬） | 状態機械・watchdog・歩容生成・上限強制 | `serpens/link/device.py` |

**継ぎ目の問題**: `session.step()` は `anim.send(bus, angles)` で 9軸の角度を直接サーボへ書く。
駆動リンクは歩容**パラメータ**を送る設計なので、両者が繋がっていない。
`Brain._set_drive()` は既に `GaitParams + γ0` を作っているので、変換は要らず継ぎ目を1枚入れるだけでよい。

## Completed（確かめたもの）

- Phase 1: 停止の3分離・実機経路の配線・終了処理（`tests/test_phase1_*.py` 20 件）
- Phase 2: 駆動リンク（`tests/test_phase2_*.py` 48 件、`tools/link_check.py` で条件 1〜12 と時間の実測）
- 全 222 件が緑（`.venv/Scripts/python.exe -m pytest -q`、約 65 秒）
- 安全の絶対値（構想設計書 16章）を `config/robot.yaml` の `safety_limits` と
  `tests/test_safety_limits.py`（10件）に落とした。トルク上限 1.18N·m を**経路を問わず**強制
- 駆動リンクに BODY（胴体姿勢）と TORQUE（脱力）を追加。とぐろ・鎌首・脱力がリンク越しに出せる
- 胴体ヨーの本数と移動性能の関係を測定（`docs/product_status.md` §3）
- 関節数の決め打ちを除去（`device_motion.py`。3軸構成で `KeyError` だったのを修正）

## Current Problems

1. **行動と駆動リンクが未接続。** `serpens.app` はリンクを一切使わない（`--bus feetech` は生の角度書き込み）
2. **接触 → 脱力が 800ms**（構想設計書の目標 20ms）。PC を経由する限り届かない。機体側で負荷を見る必要がある
3. **電気的制限（層2）が無い**。電流検出も、独立して電源を切る安全 MCU も無い（`docs/safety_limits.md` §3）
4. **輪の内径 53.6mm** は構想設計書の「80mm 以上」に届かない。±72° まで狭めれば満たすが、
   可動域ととぐろの形は機構担当の領分なので変更していない（要判断）
5. ファームは未コンパイル・未書き込み、ESP32 とサーボバスの配線が未確定
6. 知覚が外部固定カメラ前提。室内を動き回る前提では成立しない
7. `tangential_drag_ratio = 0.02` は推定値。実機校正まで前進量の絶対値は信用しない

## Assumptions（覆るかもしれない仮定）

- **A1** 第一号機は「3軸ベンチ → 展示用 9軸」の順。室内ペット（バッテリ・搭載カメラ）はその後。
  理由: 実機が1台も無く、モータ制御と安全停止の実証が先。可逆（config を足すだけ）
- **A2** 電源は当面 12V テザー。バッテリ・BMS は電子系の設計と同時に検討する
- **A3** 床はフローリング等の平滑面。絨毯・段差は受動輪 14 個の前提を壊すので後で再検討
- **A4** 所有者認識は未着手。方式（顔 / 声 / タグ）は知覚の作り直しと一緒に決める
- **A5** 巻き付き動作は**実装しない**（憲章どおり）。とぐろは自分の体だけで巻く平面渦

## Blockers

- 実サーボ・ESP32 が無い（条件 15、ファームのビルド、電流・温度の実測）
- ESP32 ⇄ サーボバスの配線方式（`firmware/serpens_esp32/README.md` の「決まっていないこと」）
- CAD / PCB の担当（憲章はこのリポジトリの作業に含めているが、これまで機械設計は別担当）

## Next Best Actions

1. **RobotInterface の継ぎ目を入れる**（`DirectRobot` / `LinkRobot`）。行動 → DRIVE → 機体 → 世界を
   シミュレーションで End-to-End に繋ぐ。実機が来た日に配線だけで動く状態にする
2. **接触 → 脱力を機体側へ**（`link.faults` に負荷しきい値、100Hz で判定）。20ms 目標への唯一の道
3. 電子系のブロック図と BOM（MCU・ドライバ・配電・電流検出・IMU）。層2 が丸ごと無い
4. ファームのサーボ層を偽サーボで検証できる形にする
