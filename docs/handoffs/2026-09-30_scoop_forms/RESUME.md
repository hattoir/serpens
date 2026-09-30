# 再開のコマンド

プロジェクトの Python は `C:\2026\Serpens_Home AI\serpens\.venv\Scripts\python.exe`。Windows の文字化けを避けるため `PYTHONIOENCODING=utf-8` を付ける。
リポジトリ: `https://github.com/hattoir/sesrpens.git`（origin）。**main にはマージしない。force push・履歴の書き換えはしない。**

## worktree を作り直す（一時フォルダが消えたとき）

```bash
cd "C:/2026/Serpens_Home AI/serpens"
git fetch origin
git worktree add ../serpens-scoop-wt agent/engineering-scoop
git worktree add ../serpens-eng-wt agent/engineering-floor-watch
```

## テスト

```bash
cd ../serpens-scoop-wt
PYTHONIOENCODING=utf-8 "C:/2026/Serpens_Home AI/serpens/.venv/Scripts/python.exe" -m pytest -q tests/test_scoop_model.py tests/test_scoop_forms.py
```
全体は約 13 分（482 passed / 1 skipped）。

## 掃引・報告書（スコップの形と機構）

```bash
PY="C:/2026/Serpens_Home AI/serpens/.venv/Scripts/python.exe"
PYTHONIOENCODING=utf-8 "$PY" tools/scoop_forms_sweep.py baseline     # 受け身のフードの基準（N=30）。約 30 分
PYTHONIOENCODING=utf-8 "$PY" tools/scoop_forms_report.py             # simulation/results/scoop_forms_2026-09-30.md を作り直す
```
1 エピソード 1 行の CSV は `output/`（git 管理外）。集計 CSV は `simulation/results/scoop_forms_*_designs.csv`（コミット済み）。
長い掃引は、**必ずバックグラウンドで直接実行する**（`&` でつなぐとランチャーが終わったときに落ちる）。

## 次にやること（順）

1. `simulation/scoop/forms/passive.py` の変種（垂れ布・スカート・凹凸・ゲート力）を小さく動作確認する（`gate="curtain"` は `curtain_f` = 0.01 / 0.1 / 1.0）
2. `tools/scoop_forms_sweep.py` に段階を足す:
   - `tolerance`: `clearance_mm` {0, 0.3, 1.0} × `plate` {0, 0.1, 0.2, 0.5}（漏斗つき・ゲートあり・10 mm/s・N=30）
   - `curtain`: `gate="curtain"`、`curtain_f` {0.01, 0.03, 0.1, 0.3, 1.0} × `retreat` {0, 30}
   - `skirt`: `skirt_mm` {0, 3, 6} × `skirt_k` {2e-4, 2e-3} × `bump_mm` {0, 0.5}
   - `gateforce`: `gate_force_n` {5, 2.8, 2, 1, 0.5, 0.25} × `retreat` 30
   - 内訳の CSV は `cell_table(rows)`（今は `baseline` 専用の名前。段階ごとの名前に直す）
3. `tools/scoop_forms_report.py` に節を足し、**推奨を更新**（結果が悪くてもそのまま書く）。カップの位置合わせ精度のカメラからの逆算を 1 行
4. `agent/DECISIONS.md`、`docs/verification_status.md` §4.14 を更新 → テスト → commit → push（ブランチ `agent/engineering-scoop`）
5. 終わったら、できたこと / できなかったこと / 次に人が決めること を各 3 行以内で報告

## 安全側（`agent/engineering-floor-watch`）

```bash
cd ../serpens-eng-wt
PYTHONIOENCODING=utf-8 "$PY" simulation/hardware_gaps/HG-S3_torque_limiter/flank_v.py
PYTHONIOENCODING=utf-8 "$PY" simulation/hardware_gaps/HG-S3_torque_limiter/j1_head.py
PYTHONIOENCODING=utf-8 "$PY" -m pytest -q tests/test_flank_v.py tests/test_safety_layers.py tests/test_yaw_sum_limit.py
```
Design から接触点の s（法線力の腕）などが届いたら、`j1_head.py` / `flank_v.py` の入力を更新して回し直す。
