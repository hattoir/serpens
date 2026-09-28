# EXP-ENG-0001 — Floor Watch 身体構成・腹面・トルク上限の感度（MuJoCo）

- Date: 2026-09-29
- Agent: Engineering Agent（Claude Code）
- Branch: agent/engineering-floor-watch
- Label: **PHYSICS_SIM**（MUJOCO_SIM）。摩擦・質量・サーボ応答はすべて未実測。**構成を決めるための数値ではない**
- Reproduce: `.venv/Scripts/python.exe tools/fw_body_study.py` → `simulation/results/fw_body_study.{md,csv}`（66 run、決定的。seed を変えても結果は変わらなかった）

## Goal

「次に何を実物で測れば、6 モーター構成と腹面を決められるか」を絞る。

## Model

- 既存の `simulation/mujoco`（config から MJCF を生成する方式）をそのまま使い、overlay を 3 つ追加した。
  - `config/robot_fw5.yaml` — FW03 の CAD 記録値（J1 pitch ＋ yaw ×4、全長 573、節 95）。**CAD 記録を写しただけで、Fusion 実体の再照合はしていない**
  - `config/robot_fw6_yaw5.yaml` — 案 A: yaw ×5 ＋ pitch（尾側に 95mm の節を 1 つ追加、全長 668）。CONCEPT
  - `config/robot_fw6_headyaw.yaml` — 案 B: yaw ×4 ＋ pitch ＋ 頭 yaw（頭を 50mm 延長、全長 623）。CONCEPT
- 質量は ASSUMPTION（0.865 / 0.980 / 0.945 kg）。サーボ 55 g は REFERENCE。電池などは `tail_payload` 130 g として尾に置いた。
- 接触は径 92mm のカプセルで近似した。中立でもカプセルどうしが重なるので、FW overlay に限り自己接触を切った（`sim.mujoco_self_collision: false`）。

## Results（MUJOCO_SIM）

| # | 問い | 結果 | 読み方 |
|---|---|---|---|
| E1 | 構成 × 腹面（歩容は構成ごとに選び直し） | SNAKE 異方性 6.7: FW5 170 / FW6_YAW5 227 / FW6_HEADYAW 180 mm/s。等方: 23〜40 mm/s | 前進は**腹面の異方性で決まる**。6 本目を胴 yaw に使うと約 +30%。頭 yaw に使うと FW5 とほぼ同じ |
| E1' | 1 つの歩容（振幅 30°・1 波）で比べた初回 | FW5 58 / FW6_YAW5 153 mm/s | **歩容の相性だけで 3 倍の差が出る。** 構成比較は歩容を選び直してから行う（初回の結論は撤回） |
| E2 | 摩擦の掃引（前後 0.05〜0.3 × 横 0.3〜0.8） | 異方性 1.0 → 23〜41 mm/s、1.5〜1.7 → 55〜96、≥2.5 → 98〜278 | **しきい値は横/前後の比でおよそ 1.5〜2.5。** 50 mm/s（巡回の目安、ASSUMPTION）はほぼ比 ≥1.7 で超えた |
| E3 | トルク上限 0.30 / 0.45 / 0.90 / 1.50 N·m | 前進はほぼ変わらない（±8%）。0.30 でトルク上限に張り付く割合が 6〜18% | 平床の蛇行では**現行の安全側上限 0.45 N·m が推進を縛らない**（この摩擦の範囲で） |
| E4 | 旋回 γ0 ±20° | 直進との差: FW5 +145/−128°、FW6_YAW5 +206/−211°、FW6_HEADYAW +108/−107°（8 秒） | 3 案とも旋回できる。直進でも 8 秒で −53〜−59° 向きがずれる → **IMU による向きの閉ループが必須** |
| E5 | 首 J1 の静的保持（解析 τ = m g d） | 60〜200 g × 25〜75 mm → 0.015〜0.147 N·m | 頭を持ち上げるだけなら上限 0.45 に対して余裕がある。**J1 がボトルネックになるのは頭スキッドで体を支えるとき**（未モデル化） |

## Found bug（修正済み、SOFTWARE_VERIFIED）

MuJoCo モデルの pitch 軸が逆向きだった（`simulation/mujoco/model.py` の `AXIS_VECTOR`）。
`serpens/motion/kinematics.py` は「+ で頭が上がる」なのに、MuJoCo では + で頭が床へ潜っていた。
その結果、EX-1 の home（J7 = +8°）は頭を床へ押し込んでいた。影響:

- 旧モデルでは γ = +20°（左旋回の指令）で**右へ** 43° 回っていた。テストは絶対値で比べていたので通っていた。
- 「サーボゲインで前進量が ±45% 変わる」（docs/product_status.md）は、符号の誤りを含んだ値だった。修正後は soft / stiff の前進比が約 1.16。
- 修正: 軸を `0 -1 0` に変えた。テスト `test_positive_pitch_raises_the_head` を追加した。`test_turning_uses_gamma_only` は直進との**差**で判定するように直した（γ の向きまで検査するので、以前より厳しい）。`test_servo_gain_changes_the_answer` の閾値は 1.2 → 1.1 に下げ、理由を docstring に書いた。

## Limits（この実験で言えないこと）

- 摩擦の絶対値。床材（フローリング・ラグ・カーペット）ごとの値。鱗やインサートで実際に得られる異方性
- カプセル近似の接触（接地は 1 本の線。実際の腹面シェル・インサートの面接触ではない）
- 騒音、段差、敷居、家具下の走行
- 電気的な消費電力（energy は機械仕事）
- 子どもとの接触力（別の実験が要る）

## Decision

- **supported**: 推進の成否を決める一番の物理量は、実際の腹面インサートと実際の床の **横/前後 摩擦比**。そのしきい値はおよそ 1.5〜2.5
- **supported**: 平床の蛇行では、トルク上限 0.45 N·m は律速ではない（sim の範囲で）
- **inconclusive**: 6 本目を胴 yaw に使うか頭 yaw に使うか。速さは胴 yaw がやや有利。Floor Watch の「止まって候補に光を向ける」操作は頭 yaw が有利。**実床の摩擦比と、頭の撮影試験の結果を待って決める**
- **rejected**: 初回の「FW5 は遅すぎる」という結論（歩容の選び方による見かけの差だった）
