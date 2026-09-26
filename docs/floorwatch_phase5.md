# Floor Watch フェーズ 5 — 縦一本（模擬）: inspect_point → 移動 → 撮影 → 判定 → floor_finding

作成 2026-09-26。**Home AI モック → Loopback MQTT → Serpens → KINEMATIC_SIM の世界 → 合成画像 → floor_finding → モックが通知**、
の往復が 1 本つながった段階。**実機の値は 1 つも無い。** 実写・実サーボ・実タグ・実 IMU はすべて未。

## 1. 流れ（`serpens/floorwatch/executor.py`、`mission.py`、`scene.py`）

| 段 | 何が起きるか | 場所 |
|---|---|---|
| Task 受理 | Endpoint が inspect_point を受け、Executor の `blockers()` を見る: 機体が停止中 / 自己位置の確認待ち / **σ と IMU・オドメトリの健全性**（フェーズ 3 `start_blockers`）。塞がっていれば `rejected` に理由 | `FloorWatchExecutor.blockers` |
| 目標 | home（m）→ 世界（mm）。**カメラの視野中心をその地点に置く**ため、首マーカの目標はその手前（首→頭先端 210 mm + 視野中心 64 mm） | `FloorWatchExecutor.start` |
| 移動 | `InspectMission` GOTO: 展示の行動（brain）は使わず、`Controller.drive_to` の安全（人との距離・マット端の柔らかい柵）だけ通す。歩容は止めても振幅が消えるまで進む（KINEMATIC_SIM で約 107 mm、**歩容の位相で散る**）ので `arrive_mm + 惰行` の手前で止める | `mission.py` |
| 位置合わせ | SETTLE → ADJUST: 頭先端から地点までの (前方, 左) を測り、前方が `view_target_mm` ± `view_tol_mm`（床カメラの見える範囲 38〜147 mm の中ほど）に入るまで短い前後の微調整（`creep`、振幅を即 0 にして惰行させない）。横は**頭ヨーで地点を線の上に置く**（回すと (頭リンク長 + 前方距離) × sin θ 横に動く）。蛇行で頭は数 cm 横にずれるので、これが無いと地点が視野（±23 mm）から外れる | 同上 |
| 静止・撮影 | CAPTURE（`capture_s`。通常→斜め→線光→全消灯→通常）→ JUDGE。動いていれば撮り直し（`max_retakes`）。線から外れた候補があれば頭ヨーで線を向けて撮り直す（`max_aims`） | 同上 |
| 判定 | フェーズ 2 の `detect` → `is_object` の候補ごとに `risk.assess` | `_judge`, `_report` |
| 位置 | floor_finding の位置 = **推定した自己位置**（フェーズ 3）+ 頭からの相対位置。σ は推定器の値。真値ではない | `_report` |
| 写真 | 候補の切り抜きだけ `floor_watch.findings_dir` に保存（原画像は保存しない = 家の外に出さない） | `_save_crops` |
| 完了 | `task_status done（候補 n 件）`。候補が無ければ floor_finding は出ない | `tick` |
| 停止 | Task stop → mission を捨て、`session.request_stop`（StopSupervisor HOLD）。**再開は API の operator_resume と機体側の開始操作の両方が人**。safety_state に ARMED_HOLD / OPERATOR が写る | `stop`, `_publish_safety` |
| 待機 | Task が無いあいだは `IdleHold` が歩容を止めておく（展示の巡回に戻らない。Floor Watch は Task 以外で動かない） | `mission.IdleHold` |

模擬の部分（実機では差し替える）: タグ観測（`SimTagObserver`、真の姿勢から合成）、IMU（真の向き + 磁北差 + 雑音）、
撮影（`capture_synthetic`: home に置いた物を頭カメラの床座標へ移して `Renderer`）。歩容オドメトリは本物の歩容位相
（`GaitEngine.phase_rad`）× `advance_per_cycle_mm`（**要実機校正**）。

## 2. 確かめたこと（`tests/test_floorwatch_slice.py`、3 件）

- タグを見る前の inspect_point は `rejected`（未初期化）。正面の壁のタグで初期化した後は受理され、20 cm 先の M10 座金（鏡面）へ
  行って止まり、`floor_finding` が届く: `frame_id home`、`map_version`、`task_id`、`height_mm null + specular_break`、
  `metal_disc` → `mandatory_notify`、位置は推定 σ の 3 倍以内、切り抜き 3 枚。モックが通知する。safety_state は DRIVING で新しい。
- 汚れだけの地点は `done（候補 0 件）`、floor_finding なし。
- 移動中の stop → `aborted`、機体は HOLD（safety_state ARMED_HOLD / OPERATOR）、次の Task は `rejected`。`operator_resume` だけでは
  まだ「機体が停止中」で rejected、機体側の開始操作の後に受理。return_dock は未実装として `failed`。
- 数字（KINEMATIC_SIM、seed 3）: 目標 20 cm 先の座金に 11 s で到着・撮影・報告。位置の誤差 1 cm（直前にタグを見ているので σ 4 mm）。
  停止位置の散り（惰行 33〜188 mm）と蛇行による横ずれ（±5 cm）は位置合わせで吸収した。

## 3. まだ無いもの

- patrol_route（各点で inspect）、highlight_point（物 → 人 → 物の身体表現）、return_dock。
- 実機: 頭カメラの撮影、実タグ観測、実 IMU、滑りの校正、照明の制御（頭の XIAO）。
- 目標が届かない（マット外・家具）ときの計画。いまは `goto_timeout_s` で `failed`。
- 位置合わせの (前方, 左) は模擬では真値から取っている。実機では推定姿勢 + 地点（σ 数 cm）になるので、
  通常照明の画像で候補を見つけてから線を向ける（JUDGE 後の AIM）が主になる。
- 人が近いときの inspect の扱い（drive_to の速度制限と停止はかかるが、撮影の可否は決めていない）。
