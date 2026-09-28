# EXP-ENG-VIS-0003 — 床の線を姿勢センサにも使う（カメラの高さ・pitch のずれを推定）

- Date: 2026-09-29 / Branch: `agent/engineering-vision-sim` / Source: **SYNTHETIC_VISION_SIM**
- Code: `simulation/h2_pose_from_line.py`、`tests/test_h2_vision.py::test_the_floor_line_recovers_camera_height_and_pitch_errors`

## Hypothesis
光の面は頭に固定なので、床の線が画像のどこに写るかは床に対するカメラの高さ・pitch で決まる。画像の線は 2 つの量（位置・傾き）を持つので、
Δ高さと Δpitch の 2 つを解ける。解ければ VIS-0002 の「姿勢のずれ → 位置の誤差 11〜30mm → 線が外れて metal_disc が落ちる」を断てる。

## Configuration
Δ高さ U(−8, 8) mm、Δpitch U(−4, 4)° を 8 通り。1 円玉を 75mm 先に置き、検出の `trace_line` の行ごとの線の列に、候補の姿勢で予測した床の線を
最小二乗で合わせる（3 段の格子、物で持ち上がった行は残差で外す）。

## Result
- 推定の誤差: 高さ おおむね ±0.6mm（最大 1.0mm）、pitch おおむね ±0.4°（線の残差 0.4〜4.4px）
- 1 円玉の手前の縁（真値 65.0mm）: 名目のカメラでは 51〜81mm（最大 15mm ずれ）→ 推定した姿勢で **64.7〜65.8mm**
- 1 件は、別の候補（遠い端 145mm）を拾った（試験の書き方の問題。推定そのものは正しかった）

## Conclusion
**床の線だけで、姿勢のずれによる位置の誤差をほぼ消せる**（SYNTHETIC）。これで線の狙い ±2mm（VIS-0002）が現実的になる。
追加の部品は要らない（線光と頭カメラだけ）。ラグで機体が沈む・首が垂れる、のどちらにも効く。

## Limits / Next
roll と画角のずれは扱っていない。床が平らで線の大半が床に乗っている前提。1 回 約 20 秒（格子探索。遅い）→ 検出に入れる前に
ガウス–ニュートンで速くする。実機では J1 の角度（サーボの読み値）を事前分布にできる。検出（`detect.py`）への組み込みは、眉か頬かが決まってから（線の向きで式が変わる）
