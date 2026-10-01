# HG-H1 アクチュエータ（PARAMETER GAP + SAFETY GAP）

**何が実測不足か**: ST3215-C044（7.4V / 1:191）の実トルク・無負荷速度・電流（待機・歩容・ストール）・温度上昇・トルク制限レジスタの効き・
脱力時のバックドライブ・反射慣性。電池の内部抵抗と電圧降下。

**なぜ効くか**: 電源・ヒューズ・電池容量・トルク上限（子どもの安全）・歩容の速さの上限が、全部これ待ち（DEC-USER-0001: H1 の実測まで電源は確定しない）。

## モデル（ACTUATOR_MODEL_SIM）

`run.py` の冒頭に式。直流モータ＋減速機の線形モデル、電流はトルクに比例、電池の電圧降下で使えるトルクが下がる、一次遅れの熱、
トルク上限 = 0.167 × τ_stall(V) × (1 + e)。**レジスタが「電圧ごとの最大に対する割合」だという仮定**も未確認。
歩容の負荷（関節トルクと角速度の時系列）は HG-H0 と同じ平面摩擦モデルから取る → H0 と H1 の結果はつながっている。

安全（危険側に倒す）:
- 静的な挟み込み: 上限 / 腕の長さ（関節のすき間 20 mm 〜 隣の節の先 95 mm）
- 上限が効かない場合（レジスタの解釈違い・設定漏れ）: ストール全開 / 腕
- 衝撃: F ≈ ω √(J_reflected · k_contact)。**トルク上限では抑えられない過渡**（減速比 1:191 の反射慣性）
- しきい値は未決定（OQ-0006）なので合否は出さない

## ファイル

| ファイル | 中身 |
|---|---|
| `assumptions.yaml` | prior（資料値が中心、UNKNOWN は広め）、トルク上限の誤差 e = 0 / ±20 / ±40%、電源・安全の幅 |
| `run.py` | Monte Carlo 3000 点 × 歩容負荷 72 通り、感度（Spearman）、安全の分布、実測 CSV の取り込み（`--measured`） |
| `results/` | `scenarios.csv`（負荷ごとの追従確率・熱・電流・稼働時間）、`sensitivity.csv`、`safety.json` |
| `plots/` | 追従確率 vs μ_横、安全の力の分布 |
| `decision_boundary.md` | **自動生成** |
| `hardware_test_plan.md` | 最小の実測と、それで確定すること |

```
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H1_actuator\run.py                       # 約 20 秒
.\.venv\Scripts\python.exe simulation\hardware_gaps\HG-H1_actuator\run.py --measured hardware\prototypes\H1_joint\h1_joint_YYYYMMDD.csv
```

実測の取り込み: 生データの控え → トルク-電流の傾き・ストール・熱時定数・上限の効きを当てはめ → レジスタ比の推奨値を出す
（**config は自動で書き換えない**。安全に関わるので User 承認）→ Handoff に追記。
