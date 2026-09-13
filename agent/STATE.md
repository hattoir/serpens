# STATE — serpens

<!-- 作業のたびに更新する。ここが次のセッションの出発点になる。 -->

_last updated: 2026-09-14_

## Current Goal

PC の行動決定と機体の駆動リンクを1本に繋ぎ、**「行動 → DRIVE → 機体が歩容生成 → 世界が動く」**を
End-to-End で成立させる。いま両者は別々に存在していて、縦スライスが途中で切れている。

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
- 全 205 件が緑（`.venv/Scripts/python.exe -m pytest -q`、約 65 秒）
- 胴体ヨーの本数と移動性能の関係を測定（`docs/product_status.md` §3）
- 関節数の決め打ちを除去（`device_motion.py`。3軸構成で `KeyError` だったのを修正）

## Current Problems

1. **行動と駆動リンクが未接続。** `serpens.app` はリンクを一切使わない（`--bus feetech` は生の角度書き込み）
2. **安全の数値が無い。** 最大力・最大トルク・電流上限が決まっておらず、`link.faults.current_limit_ma: 0`（無効）
3. ファームは未コンパイル・未書き込み、ESP32 とサーボバスの配線が未確定
4. 知覚が外部固定カメラ前提。室内を動き回る前提では成立しない
5. `tangential_drag_ratio = 0.02` は推定値。実機校正まで前進量の絶対値は信用しない

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
2. **安全の数値を決める**（`docs/safety_limits.md`）。STS3215 の 30kgf·cm から挟み込み力を計算し、
   トルク制限・電流上限・関節速度を config に落とす
3. 電子系のブロック図と BOM（MCU・ドライバ・配電・電流検出・IMU）
4. ファームのサーボ層を偽サーボで検証できる形にする
