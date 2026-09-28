# H2 視覚: Monte Carlo（SYNTHETIC_VISION_SIM。条件 300、試行 900、3.3 分、UXGA×0.5）

条件は家の中の見込み（`tools/h2_vision.py` の `_sample_condition`、すべて ASSUMPTION）から同時にずらした。

| 指標 | 値 |
|---|---|
| n_pos | 600 |
| n_neg | 300 |
| patrol_recall | 0.90 |
| inspect_recall | 0.94 |
| critical_inspect_recall | 0.95 |
| specular_critical_flagged | 0.54 |
| false_alarm_patrol | 0.21 |
| false_alarm_inspect | 0.34 |
| height_abs_err_median_mm | 3.46 |
| diameter_rel_err_median | 0.18 |
| loc_err_median_mm | 11.02 |

## critical の inspect 検出 × 因子（数値因子は 3 分位、カテゴリは値ごと）

| 因子 | 区分 | n | 検出 |
|---|---|---|---|
| floor | carpet | 76 | 0.91 |
| floor | pattern | 69 | 0.97 |
| floor | rug | 62 | 0.95 |
| floor | tile | 90 | 0.96 |
| floor | wood | 139 | 0.95 |
| floor_height_sigma_mm | ≤0.00 | 229 | 0.95 |
| floor_height_sigma_mm | 0.00〜0.10 | 63 | 0.97 |
| floor_height_sigma_mm | >0.10 | 144 | 0.93 |
| blur_sigma_px | ≤0.00 | 174 | 0.95 |
| blur_sigma_px | 0.00〜2.00 | 178 | 0.94 |
| blur_sigma_px | >2.00 | 84 | 0.96 |
| ambient_lux | ≤8.00 | 149 | 0.98 |
| ambient_lux | 8.00〜50.00 | 151 | 0.94 |
| ambient_lux | >50.00 | 136 | 0.92 |
| shot_noise_k | ≤0.34 | 147 | 0.95 |
| shot_noise_k | 0.34〜0.61 | 144 | 0.94 |
| shot_noise_k | >0.61 | 145 | 0.94 |
| line_scatter_mm | ≤0.00 | 298 | 0.96 |
| line_scatter_mm | 0.00〜0.00 | 0 | — |
| line_scatter_mm | >0.00 | 138 | 0.93 |
| cam_height_err_mm | ≤-2.23 | 146 | 0.93 |
| cam_height_err_mm | -2.23〜0.55 | 145 | 0.95 |
| cam_height_err_mm | >0.55 | 145 | 0.96 |
| cam_pitch_err_deg | ≤-0.94 | 146 | 0.94 |
| cam_pitch_err_deg | -0.94〜0.72 | 145 | 0.97 |
| cam_pitch_err_deg | >0.72 | 145 | 0.93 |
| fov_err_deg | ≤-1.51 | 146 | 0.97 |
| fov_err_deg | -1.51〜1.12 | 146 | 0.96 |
| fov_err_deg | >1.12 | 144 | 0.92 |
| aim_err_mm | ≤0.94 | 146 | 0.92 |
| aim_err_mm | 0.94〜2.06 | 145 | 0.96 |
| aim_err_mm | >2.06 | 145 | 0.97 |
| clutter | False | 373 | 0.95 |
| clutter | True | 63 | 0.90 |
