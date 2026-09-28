# Task / Event API（Home AI ↔ Serpens、v1）

作成 2026-09-25（Floor Watch フェーズ 1）。**Home AI と Serpens は別システム・別リポジトリ。接続はこの API だけ。**
MQTT はシステム間の境界にだけ使い、リアルタイムの関節制御には使わない（決定事項 3・4・14）。
スキーマ: `schemas/common.json` / `schemas/task.json` / `schemas/event.json`（JSON Schema 2020-12。
Serpens 側の検証器は `serpens/api/validate.py` の最小部分集合。Home AI 側は本物の検証器でそのまま読める）。

## 1. トピック

| トピック | 向き | QoS | retain | 内容 |
|---|---|---|---|---|
| `home/serpens/task` | Home AI → Serpens | 1 | なし | Task（5 種） |
| `home/serpens/event/task_status` | Serpens → Home AI | 1 | なし | Task の受理 / 進行 / 完了 / 失敗 |
| `home/serpens/event/floor_finding` | Serpens → Home AI | 1 | なし | 落とし物の候補・確認・解決 |
| `home/serpens/event/safety_state` | Serpens → Home AI | 1 | **あり** | 機体の安全状態（後から購読しても最後の状態が届く） |
| `home/serpens/event/battery` | Serpens → Home AI | 1 | なし | 電圧と推定残量 |

Serpens 内部の PC → 機体（歩容パラメータ・首角・停止）は既存の USB 駆動リンク（`docs/link_protocol.md`）のまま。

## 2. 共通ヘッダ

`v`（= 1。違う版は拒否）、`id`（UUID。同じ id の再送は冪等）、`t_ms`（送り手の Unix ms）、`source`（home_ai / serpens / mock）。
座標を持つメッセージは `frame_id: "home"` と `map_version`（床の AprilTag 配置 `tags.yaml` の版）を必ず持つ。
座標系 home: 原点 = ドックのタグ中心、x = ドックから部屋の方向、y = 左、z = 上、単位 m / rad、yaw は反時計回りが正。

## 3. Task

| task | 必須 | 意味 |
|---|---|---|
| `inspect_point` | frame_id, map_version, target{x_m,y_m,yaw_rad} | 地点へ行き、止まり、5 枚撮って（照明 3 条件 + 全消灯 + 動き確認）判定する |
| `patrol_route` | frame_id, map_version, waypoints[1..64] | 巡回（各点で inspect） |
| `highlight_point` | frame_id, map_version, target, (duration_s) | 発見した物の位置を身体で示す（物 → 人 → 物）。**CSAR: 子どもが近い・不明のあいだは `failed`（理由に CSAR）**。物を指す・照らすと子どもを物へ連れていくため（未実装、門だけ先に） |
| `return_dock` | — | ドックへ戻る（MVP では別機能。受理はするが実行は後） |
| `stop` | (reason) | **常に最優先。** 実行中の Task を中断。安全停止そのものは機体側が行う |

**Task に安全の設定（速度上限・角度・TTL・トルク）を変えるフィールドは無い。** 未知のフィールドは拒否。

### 受理の規則（`serpens/api/endpoint.py`。この順に判定）

0. **`task: stop` は何より先に判定し、版違い・スキーマ違反・map_version 違い・id 無しでも受理する**（止める方向は常に通す）。
   実行中と待ち行列の Task は `aborted`、以後は人の操作（`operator_resume`）まで新しい Task を受けない
1. retain 付きで届いた Task（後から購読したときブローカーの保存分が retain=1 で来る）→ `rejected`。**Task トピックに retain を付けない**
2. スキーマ違反 → `rejected("schema: …")`
3. 同じ id の再送 → 前回と同じ `task_status` を再送（実行はしない）
4. `t_ms` が最後の stop の `t_ms` 以前（QoS1 の再送・再接続で後から届いた古い Task）→ `rejected`
5. stop / FAULT_HOLD / EMERGENCY_LATCHED / TORQUE_DISABLED / OFFLINE のあと人の操作前 → `rejected`
6. `map_version` が受け側と違う → `rejected`
7. 実行できない理由がある（ELECTRICAL_SAFETY_GATE 未完了、自己位置なし、電池低下 …）→ `rejected` に理由を並べる
8. 実行中なら待ち行列へ（`accepted`, reason "待ち行列"）。前の Task が終わったら `running`

FAULT / EMERGENCY（`safety_state` の mode）でも待ち行列を破棄し、自動では再開しない。

状態遷移: `accepted → running → (paused) → done | failed | aborted`。`rejected` は開始前だけ。

## 4. Event

| event | 必須 | 備考 |
|---|---|---|
| `task_status` | task_id, status, (reason, progress) | |
| `floor_finding` | finding | 下記 |
| `safety_state` | mode（7 状態 + OFFLINE）, stop_reason, latched, resume_requires="operator" | **停止後は人の操作なしで再開しない**をそのまま外へ出す。retain。**変化が無くても周期的に出す**（`api.safety_state_period_ms` = 2 s < 受け側の失効 5 s、`Endpoint.tick`）。異常切断は LWT が `mode: OFFLINE` を retain で出す（keepalive の 1.5 倍後）。**正常終了は LWT が出ないので自分で OFFLINE（stop_reason OPERATOR）を出す**（`Endpoint.close`）。再接続では今の状態で retain を上書きする（`Endpoint.announce`） |
| `battery` | voltage_v, low, (percent_est) | percent は推定 |

すべての Event に `data_source`（HARDWARE / SIMULATION）。模擬の値を実測と混ぜない。

### floor_finding.finding

| 欄 | 内容 |
|---|---|
| finding_id, t_ms, frame_id, map_version | 識別と座標系 |
| pose | x_m, y_m, yaw_rad, **sigma_xy_m, sigma_yaw_rad**（姿勢推定の不確かさ）, pose_source（APRILTAG / ODOMETRY_IMU / GROUND_TRUTH_SIM / ARUCO_EXTERNAL / UNKNOWN） |
| photos[1..3] | kind（normal / raking / line）、**crop_path（候補の切り抜き。原画像は家の外へ出さない）**、w_px, h_px, t_ms, led |
| candidates[1..5] | kind（coin / button_cell / magnet / medicine / **metal_disc** / washer / bead / food_crumb / stain_or_pattern / unknown）, confidence。metal_disc = 分類器ができるまでの代用: 線の途切れ + 円形 + 直径 5〜25mm（ボタン電池の可能性）→ critical_kinds に入れて必ず通知 |
| size | diameter_mm ± sigma_mm、height_mm ± height_sigma_mm（線光）、**height_reason**（measured / specular_break / off_line / too_few_rows）、method。**測れない高さは 0 ではなく null**（0 は「平ら = 汚れ」と読まれる。鏡面 = 金属 = ボタン電池そのものが 0 になるのが最悪） |
| risk | ingestion, sharp, child_reachable, child_distance_m（不明は null = 近いとみなす）, **score（複合。サイズのしきい値 1 つで決めない）**, **mandatory_notify + critical_kinds（ボタン電池・磁石・薬は score と別枠。確信度が低くても通知）**, rationale[] |
| state / state_by | candidate → confirmed →（**人の操作だけ**: state_by=operator）resolved / dismissed。ロボットの再訪は `reobservations[{t_ms, seen, pose_sigma_xy_m}]` に記録するだけで状態を変えない |

## 5. 受け側（Home AI）の規則

- `safety_state` の鮮度は**受け側の時計**で「最後に生で届いてからの時間」で判定する（`api.safety_state_max_age_ms` = 5 s。
  送り側の `t_ms` は時計ずれがあるので使わない）。Serpens は 2 s 周期で出すので、5 s 届かなければ状態不明 = 停止扱い。
  購読時に retain で届いた値は「最後に知られた状態」で、周期送信が届くまで鮮度は保証しない（`HomeAiMock.safety_fresh`）。
- **`operator_resume` は MQTT / Home AI から呼べない。** トピックに無く、Task に見せかけた `operator_resume` / `resume` /
  `clear_fault` / `arm`（フィールドでも）は `rejected`。再開は機体のボタンか運用者の端末（人の操作）だけ。
- `floor_finding` は `risk.score ≥ 0.5` **または** `risk.mandatory_notify` で通知する。

## 5'. エラー時の扱い

- 壊れた JSON は捨てる（何もしない = 安全側）。スキーマ違反は `rejected` で理由を返す。
- ブローカー断: Serpens は Task を受けられないだけで、機体の安全は既存の Heartbeat / TTL（USB）が守る。この API は安全経路ではない。
- 版を上げるときは `v` を上げ、旧版は拒否する（黙って解釈しない）。

### 5''. stop と緊急停止（いまの割り切りと将来）

いまは**どの stop も `operator_resume` が要る**（大人が見ている MVP ではこれでよい）。将来は分ける:
「stop = 取り消し（実行中と待ち行列を捨てる。新しい Task は受ける）」と「緊急停止 = ラッチ（人の操作まで受けない）」。
分けるときも安全側の既定（分からなければラッチ）は変えない。

## 6. 通知

Serpens は Event を出すだけ。誰にどう通知するかは Home AI が決める（決定事項 12）。
MVP では `tests/mocks/home_ai_mock.py` が「受け取ってログに出す Webhook」の代わり。

## 7. 本物のブローカーで確かめるもの

retain・LWT・QoS1 の再送はループバックでは確かめられない。`tests/test_mqtt_live.py` が確かめる:
retain が後からの購読に届く / **LWT が keepalive の約 1.5 倍で出る**（MQTT 3.1.1 §3.1.2.10。テストは keepalive 2 s で
2〜5 s を許容。ソケットが閉じた場合（プロセス落ち）は即座に出る）/ 正常終了の OFFLINE が即時に出る / 再接続で retain が今の状態に置き換わる / QoS1 + clean_session=False の再送。
**Mosquitto は常駐サービスにしない**: テストが 127.0.0.1 限定・一時ポート・一時設定（匿名・永続化なし）でサブプロセス起動し、
終わったら止める。paho-mqtt は開発用の任意依存（`pip install paho-mqtt`。requirements には入れない）。
mosquitto 実行ファイルは PATH / 環境変数 `SERPENS_MOSQUITTO` / `C:\Program Files\mosquitto` / `%LOCALAPPDATA%\Programs\mosquitto` から探し、無ければ skip。

## CSAR（Child-Safe Attention Rules）と Task の結果（2026-09-29）

User 採用（USER-DEC-SERPENS-DESIGN-0001 #1）。実行条件は Serpens 側（`serpens/floorwatch/csar.py`、config `floor_watch.csar`）。

- 子どもが近いか（NEAR / FAR / UNKNOWN）は **Serpens が自分の観測で決める**。人は誰でも「子どもかもしれない」（区別できない）。
  「近くに人がいない」は最後に確かめてから `far_trust_s`（3 秒）だけ信じ、切れたら UNKNOWN = NEAR 扱い。Home AI の情報は「近い」だけ受け取る:
  Task に **`child_near: true`**（任意）を付けると `home_hint_s`（30 秒）NEAR として扱う。`false` は無視（Serpens の判定を「遠い」へ動かせない）
- `inspect_point`: 撮影（閃光）は FAR のときだけ。NEAR / UNKNOWN なら物を照らさずに待ち、`capture_wait_s` を過ぎたら
  **`failed`（理由に「CSAR … 候補は未確認」）**。Home AI はこれを「未確認の候補がある」として**保護者へ知らせる**（R4）。再試行は Home AI の判断
- 判定が確定したら `floor_finding` を**すぐ**出し（離れるのを待たない）、それから物から `retreat_mm` 離れて `done`（R3）
- 子どもが近いあいだは、頭を人の方へ向ける（上限 `look_at_person_max_deg`。近づく動きは入れない、R5）
