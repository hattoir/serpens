# DEC-USER-0001 — Hardware Gate の順番と、C044・6 本目・push の扱い

- Date: 2026-09-29 / Decided by: **User**（chat で指示）/ Recorded by: Engineering Agent
- Responds to: PROP-ENG-0001

## Decision

1. **H0 を最優先。** フローリング / ラグ / カーペットの 3 床で、前後と横の摩擦を測り、横/前後の比を比べる。Pure Snake を続けるか Wheel Belly に戻すかの判断材料にする
2. 次に **H1 = C044 の 1 関節**（実トルク・電流・温度・telemetry・停止挙動）。電源・ヒューズ・電池は H1 の実測まで確定しない
3. その次に **H2**（実カメラの FOV、ピント、斜め照明、ライン光、小物の検出性能を実床で）
4. **C044 の仕様**: 現行の Seeed / Feetech 資料の値（7.4V / 1:191 / ストール 27.4 kg·cm / 定格 9 kg·cm）を使う。リポジトリの 16 kgf·cm は **stale / source-unverified**。元出典を確かめるまでは削除せず、矛盾として残す。最終的な判断には H1 の実測値を使う
5. **GitHub**: main を先に push してよい。その後 `agent/engineering-floor-watch` を push する。main への merge はまだしない
6. **ai-engineering-os**: `../ai-engineering-os/global/OPERATING_RULES.md` はまだ無い。missing dependency として記録し、`.ai/LOCAL_RULES.md`・`.ai/BOOTSTRAP.md`・現在の Agent 指示を優先する。存在しない OS ルールを推測で作らない
7. **MQTT retain test**: Floor Watch の変更とは別問題。race / concurrency の Open Question または独立した Task として扱い、blocking にしない
8. **6 本目のモーター**はまだ固定しない。H0 の結果を見て A（Body Yaw）と B（Head Yaw）を比べる。推進が不足するなら Body Yaw 寄り、Floor Watch の照準性能が律速なら Head Yaw 寄り
9. 現在の Simulation 結果は PHYSICS_SIM のまま。HARDWARE_VERIFIED に上げない
10. 次の Checkpoint: **H0 結果 → locomotion 再評価 → 6 本目の用途を仮決定 → H1 → H2**

## Engineering 側の対応（2026-09-29）

- H0: `tools/h0_coupons.py`（STL 3 種）、`hardware/prototypes/H0_friction/`（手順・CSV テンプレート）、`tools/h0_friction.py`（実測 → μ → MuJoCo 予測 → 床ごとの読み）
- H1: `hardware/prototypes/H1_joint/`（計画と CSV テンプレート）。**トルク制限レジスタ比の見直し（0.287 → 0.167）を承認待ちで提案**
- C044: `config/robot.yaml` にコメントで「stale / source-unverified」と矛盾を追記（値は変えていない）
