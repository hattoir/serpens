# EXP-ENG-VIS-0001 — ライン光の置き方: 頬（横基線）と眉（縦基線）

- Date: 2026-09-29 / Branch: `agent/engineering-vision-sim` / Source: **GEOMETRY_SIM**（実写・実測ではない）
- Responds to: integration-log ENTRY-0006(a)、OPEN-SERPENS-DESIGN-001
- Code: `simulation/h2_line_light.py`、`tools/h2_line_light.py` → `simulation/results/h2_line_light.{md,csv}`、`tests/test_h2_line_light.py`

## Hypothesis
眉の段（縦基線 34〜40 mm）でも三角測量は成り立つ。頬（横基線 30 mm・45° 内向き、現行）との差は感度より「線を候補へ当てる手段」に出る。

## Configuration
カメラ floor_still（1600×1200、f 1256 px、高さ 30 mm、pitch 25°。すべて ASSUMED）。
頬: 投光部 (±15/30/45, 0, 30)、床の線は x=0 を前後。眉: 投光部 (0, 0, 64/70)、床の線は y = 60/71/90 mm を左右。
線の重心の誤差 σc = 0.2 px（仮定）。誤較正は光の面を 0.5° 回した場合（tilt = 床の線まわり、yaw = 鉛直軸まわり）。
首の回転軸からカメラまで 40/60/80/104 mm（ASSUMPTION、CAD で要確認）。

## Result（`simulation/results/h2_line_light.md`）
| | 頬（現行） | 眉 z64・y71 |
|---|---|---|
| 感度 px/mm（最小/中央） | 9.0 / 13.5 | 8.1（線の上で一定） |
| σ_H（σc 0.2 px） | 0.022 mm | 0.025 mm |
| yaw 0.5° 誤較正 → 5 mm の物の誤差 | 0.205 mm | 0.057 mm |
| 線の長さ（視野 38〜147 mm の中） | 109 mm（前後） | 98 mm（左右） |
| 線を当てる手段 | 頭 yaw（2.3〜3.4 mm/°） | **J1 首 pitch（既存）** 3.3〜4.6 mm/° |

## Conclusion
- どちらも出っ張りの閾値 0.5 mm より 1 桁以上細かく測れる（GEOMETRY_SIM）。**眉でも三角測量は成り立つ。**
- 眉の方が向きの誤較正に約 3.5 倍強く、感度が線の上で一定（閾値を決めやすい）。
- **決定に効く差は狙いの手段。** 頬の線は前後に走るので、横にある候補へ当てるには頭 yaw（6 本目 = Head Yaw）か胴の旋回が要る。
  眉の線は視野の横幅いっぱいを左右に走るので、前後だけ合わせればよく、それは既存の J1 首 pitch で ±5〜9°（視野中ほど ±30 mm）。
  → **眉なら、線の狙いのために Head Yaw は要らない。** 6 本目を Head Yaw にする理由のうち「線を向ける」は消え、
  CSAR の「目をそらす」（ENTRY-0009）が残る。
- 眉の費用: `detect.trace_line` は「行ごとに 1 本（線が前後に走る）」を前提にしている → 列ごとに追う版が要る（ソフト）。
  鼻先が光路をさえぎらないこと（Design の条件 X ≥ −230）。

## Not covered（次の実験）
線の太さ・ボケ・カーペットでの散乱、鏡面での途切れ、物自身の影（投光部が低い頬ほど長い）→ VIS-0002 の合成画像で。

## Next hypothesis
眉の線（列ごとの追跡）を合成画像で描き、既存の検出と同じ候補・高さが出るか。ボケ・床の模様・環境光に対して、どちらが先に崩れるか。
