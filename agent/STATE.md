# STATE — serpens

<!-- 作業のたびに更新する。ここが次のセッションの出発点になる。 -->

_last updated: 2026-09-14_

## Current Goal

Phase 2/3 の**仮想実機**が揃った（2026-09-15）。SimulatedSnake（PC → 仮想ESP32 → 仮想サーボ →
シミュレータ）で通常走行と異常（通信断・再起動・緊急停止・サーボ故障）を再現できる。
次は**実機が来た日に前へ進むための準備**: 接触→脱力を機体側へ、電子系の BOM、ファームのサーボ層。

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
- 縦スライスの接続（`serpens/robot.py` / `serpens/link/robot.py`、`--robot link`）。
  **アプリ経路でも PC 強制終了・USB 抜去で機体が自分で止まる**ことを試験で確認
- Virtual ESP32（7状態）+ 仮想サーボバス + テレメトリ v2（source=SIMULATION）
- 故障注入 4層（経路 / 機体 / サーボ / 指令値）と `docs/verification_status.md` の4段区分
- 可動域を3段化し CAD R03（±64°）へ。輪の内径 114mm で構想設計書16章を満たした
- Belly（wheel/snake）× 摩擦プロファイル、歩容パラメータの掃引（`tools/gait_sweep.py`）
- GUI 下段に関節ペイン（指令角・実測角・安全・機体状態）
- 全 274 件が緑（`.venv/Scripts/python.exe -m pytest -q`、約 65 秒）
- 安全の絶対値（構想設計書 16章）を `config/robot.yaml` の `safety_limits` と
  `tests/test_safety_limits.py`（10件）に落とした。トルク上限 1.18N·m を**経路を問わず**強制
- 駆動リンクに BODY（胴体姿勢）と TORQUE（脱力）を追加。とぐろ・鎌首・脱力がリンク越しに出せる
- 胴体ヨーの本数と移動性能の関係を測定（`docs/product_status.md` §3）
- 関節数の決め打ちを除去（`device_motion.py`。3軸構成で `KeyError` だったのを修正）

## Current Problems

1. **実 ESP32 が無い。** リンク経路は模擬機体でしか動かしていない（`--robot link --link-port COMx` は未検証）
2. **接触 → 脱力が 800ms**（構想設計書の目標 20ms）。PC を経由する限り届かない。機体側で負荷を見る必要がある
3. **電気的制限（層2）が無い**。電流検出も、独立して電源を切る安全 MCU も無い（`docs/safety_limits.md` §3）
4. **輪の内径 53.6mm** は構想設計書の「80mm 以上」に届かない。±72° まで狭めれば満たすが、
   可動域ととぐろの形は機構担当の領分なので変更していない（要判断）
5. ファームは未コンパイル・未書き込み、ESP32 とサーボバスの配線が未確定
6. 知覚が外部固定カメラ前提。室内を動き回る前提では成立しない
7. `tangential_drag_ratio = 0.02` は推定値。実機校正まで前進量の絶対値は信用しない

## 直近で判断が要ること

- **胴体ヨーの operational limit ±60° は暫定**（CAD R03 の ±64° 由来）。とぐろの形がこれで決まる
- **STS3215 7.4V 1:191 のストールトルクが未確認** → 安全のトルク上限を出し直せない
- ESP32 ⇄ サーボバスの配線方式（ファームを書き込めない理由）

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

1. **接触 → 脱力を機体側へ**（`link.faults` に負荷しきい値、100Hz で判定）。20ms 目標への唯一の道
2. 電子系のブロック図と BOM（MCU・ドライバ・配電・電流検出・IMU）。層2 が丸ごと無い
3. ファームのサーボ層を偽サーボで検証できる形にする
4. 3軸ベンチの config プロファイル（サーボが届いた日に試せる状態にする）
