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
| `inspect_point` | frame_id, map_version, target{x_m,y_m,yaw_rad} | 地点へ行き、止まり、3 枚撮って判定する |
| `patrol_route` | frame_id, map_version, waypoints[1..64] | 巡回（各点で inspect） |
| `highlight_point` | frame_id, map_version, target, (duration_s) | 発見した物の位置を身体で示す（物 → 人 → 物） |
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
| `safety_state` | mode（7 状態 + OFFLINE）, stop_reason, latched, resume_requires="operator" | **停止後は人の操作なしで再開しない**をそのまま外へ出す。retain。接続断は LWT が `mode: OFFLINE` を retain で出す |
| `battery` | voltage_v, low, (percent_est) | percent は推定 |

すべての Event に `data_source`（HARDWARE / SIMULATION）。模擬の値を実測と混ぜない。

### floor_finding.finding

| 欄 | 内容 |
|---|---|
| finding_id, t_ms, frame_id, map_version | 識別と座標系 |
| pose | x_m, y_m, yaw_rad, **sigma_xy_m, sigma_yaw_rad**（姿勢推定の不確かさ）, pose_source（APRILTAG / ODOMETRY_IMU / GROUND_TRUTH_SIM / ARUCO_EXTERNAL / UNKNOWN） |
| photos[1..3] | kind（normal / raking / line）、**crop_path（候補の切り抜き。原画像は家の外へ出さない）**、w_px, h_px, t_ms, led |
| candidates[1..5] | kind（coin / button_cell / washer / bead / food_crumb / stain_or_pattern / unknown）, confidence |
| size | diameter_mm ± sigma_mm、height_mm ± height_sigma_mm（線光。高さ 0 = 汚れ・模様）、method |
| risk | ingestion, sharp, child_reachable, child_distance_m（不明は null = 近いとみなす）, **score（複合。サイズのしきい値 1 つで決めない）**, **mandatory_notify + critical_kinds（ボタン電池・磁石・薬は score と別枠。確信度が低くても通知）**, rationale[] |
| state / state_by | candidate → confirmed →（**人の操作だけ**: state_by=operator）resolved / dismissed。ロボットの再訪は `reobservations[{t_ms, seen, pose_sigma_xy_m}]` に記録するだけで状態を変えない |

## 5. 受け側（Home AI）の規則

- `safety_state` は `t_ms` が古ければ無効（モックは 5 秒）。retain で残った古い状態を今の状態と取り違えない。
- `floor_finding` は `risk.score ≥ 0.5` **または** `risk.mandatory_notify` で通知する。

## 5'. エラー時の扱い

- 壊れた JSON は捨てる（何もしない = 安全側）。スキーマ違反は `rejected` で理由を返す。
- ブローカー断: Serpens は Task を受けられないだけで、機体の安全は既存の Heartbeat / TTL（USB）が守る。この API は安全経路ではない。
- 版を上げるときは `v` を上げ、旧版は拒否する（黙って解釈しない）。

## 6. 通知

Serpens は Event を出すだけ。誰にどう通知するかは Home AI が決める（決定事項 12）。
MVP では `tests/mocks/home_ai_mock.py` が「受け取ってログに出す Webhook」の代わり。

## 7. 本物のブローカーで確かめるもの

retain・LWT・QoS1 の再送はループバックでは確かめられない。`tests/test_mqtt_live.py`（paho-mqtt と Mosquitto が
localhost:1883 にあれば動く。無ければ skip）。ハード不要なのでフェーズ 5 を待たず PC 上で行う。
