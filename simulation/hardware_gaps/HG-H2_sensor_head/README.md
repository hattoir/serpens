# HG-H2 センサーヘッド（SENSOR GAP）

**何が実測不足か**: 頭カメラ（XIAO ESP32S3 Sense、OV2640 / OV3660 のどちらかも未確認）の FOV・ピント・歪み・雑音・露出、
あごの斜め照明で実際に影が出るか、線光の幅、部屋の明るさ。

**なぜ効くか**: Floor Watch の「発見 → 確認」の入口。合成画像での検出率 98〜100%（既存の評価）は理想的な光学を仮定した楽観値。

## モデル（SYNTHETIC_SENSOR_SIM）

既存の `serpens/floorwatch/synthetic.py`（Renderer）と `detect.py` をそのまま使い、その間に**実カメラの劣化**を挟む（`run.py` の `Degrader`）:

- ピント: 薄肉レンズの錯乱円 c = 口径 · f_px · |1/d_focus − 1/d|（画素ごとの距離 d で。σ ≈ c / 2.5 の段で近似）
- 放射歪み k1（検出器はピンホールを仮定したまま = 未較正の歪み）
- 横ぶれ（水平の箱形フィルタ）、露出ゲインと 255 での飽和、ショット雑音 + 読み出し雑音
- カメラの画角・解像度・高さ・下向き角、あごの LED の高さ、影の暗さ、線光の幅、環境光

小物の代わり: 1 円玉、ボタン電池（CR2032 / LR44、鏡面）、ビーズ、白い錠剤、磁石（ネオジム = 鏡面 / フェライト = 黒）、食べかす。
陰性: 暗い汚れ、明るい汚れ、床の溝。**色は明るさに縮めている**（色の違いはモデル化していない = 限界）。

## ファイル

| ファイル | 中身 |
|---|---|
| `assumptions.yaml` | nominal（config の ASSUMED 値）、小物と陰性、合否の基準 |
| `sweep_config.yaml` | 1 変数ずつの掃引（15 パラメータ）と最悪側の組み合わせ |
| `run.py` | 掃引、判定書、FOV・ピント・実画像データセットの取り込み |
| `results/oat.csv` | 条件ごとの検出率・誤報率・範囲外の偽物・大きさ誤差・ぼけ |
| `plots/oat_sensitivity.png` | パラメータごとの検出率・誤報率 |
| `decision_boundary.md` | **自動生成** |
| `hardware_test_plan.md` | T1〜T5 |

```
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py            # 14 並列で約 15 分
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py --measured-fov fov.csv
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py --measured-focus focus.csv
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H2_sensor_head\run.py --real-dataset data\floorwatch\head_2026-10
```

## 読み方の注意

- 1 条件は 陽性 16 件・陰性 6 件。**1 件で割合が 6〜17% 動く**。傾向（どのパラメータが効くか）を読み、割合の細かい差は読まない
- 誤報は頭から `floor_watch.mission.reach_mm` 以内だけを数え、それより遠い偽物は別の列にした。（2026-10-01 の統合 branch `f8f2229` で回し直した結果: **範囲外の偽物が出る条件は 26/44 → 0/44**（nominal 41% → 0%）。vision-sim の検出の修正（`d259f59` ほか）が既に除外している。farfield-roi の `line_max_range_mm` は統合していない。**SYNTHETIC_SENSOR_SIM。実カメラでは未確認**）。
- **2026-10-01 の再実行（統合 `f8f2229`、旧は eng-floor-watch 時代の `oat.csv`）の変化（LB-E-009）**: 主な指標のうち良くなったセル 39、**悪くなったセル 2**（`exposure_gain=2.5` の停止時検出 1.00 → 0.88、危険物の停止時検出 1.00 → 0.80。露出過多の条件。原因は未調査 = A の候補）。範囲外の偽物 26/44 → 0/44。farfield-roi の遠方シナリオは `tests/test_hardware_gap_h2.py` に統合 base 用の回帰として追加（新しい設定キー無し）。**合成画像の値は楽観値**。
