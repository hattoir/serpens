# H2 視覚: Monte Carlo（SYNTHETIC_VISION_SIM。条件 300、試行 900、5.3 分、UXGA×0.5）

条件は家の中の見込み（`tools/h2_vision.py` の `_sample_condition`、すべて ASSUMPTION）から同時にずらした。

| 指標 | 値 |
|---|---|
| n_pos | 600 |
| n_neg | 300 |
| patrol_recall | 0.64 |
| inspect_recall | 0.75 |
| critical_inspect_recall | 0.78 |
| specular_critical_flagged | 0.47 |
| false_alarm_patrol | 0.28 |
| false_alarm_inspect | 0.38 |
| height_abs_err_median_mm | 2.84 |
| diameter_rel_err_median | 0.19 |
| loc_err_median_mm | 6.62 |

## critical の inspect 検出 × 因子（数値因子は 3 分位、カテゴリは値ごと）

| 因子 | 区分 | n | 検出 |
|---|---|---|---|
| floor | carpet | 76 | 0.61 |
| floor | pattern | 69 | 0.91 |
| floor | rug | 62 | 0.84 |
| floor | tile | 90 | 0.77 |
| floor | wood | 139 | 0.78 |
| floor_height_sigma_mm | ≤0.00 | 229 | 0.78 |
| floor_height_sigma_mm | 0.00〜0.10 | 63 | 0.90 |
| floor_height_sigma_mm | >0.10 | 144 | 0.72 |
| blur_sigma_px | ≤0.00 | 174 | 0.84 |
| blur_sigma_px | 0.00〜2.00 | 178 | 0.72 |
| blur_sigma_px | >2.00 | 84 | 0.75 |
| ambient_lux | ≤8.00 | 149 | 0.85 |
| ambient_lux | 8.00〜50.00 | 151 | 0.80 |
| ambient_lux | >50.00 | 136 | 0.68 |
| shot_noise_k | ≤0.34 | 147 | 0.84 |
| shot_noise_k | 0.34〜0.61 | 144 | 0.76 |
| shot_noise_k | >0.61 | 145 | 0.73 |
| line_scatter_mm | ≤0.00 | 298 | 0.81 |
| line_scatter_mm | 0.00〜0.00 | 0 | — |
| line_scatter_mm | >0.00 | 138 | 0.71 |
| cam_height_err_mm | ≤-2.23 | 146 | 0.64 |
| cam_height_err_mm | -2.23〜0.55 | 145 | 0.79 |
| cam_height_err_mm | >0.55 | 145 | 0.91 |
| cam_pitch_err_deg | ≤-0.94 | 146 | 0.89 |
| cam_pitch_err_deg | -0.94〜0.72 | 145 | 0.82 |
| cam_pitch_err_deg | >0.72 | 145 | 0.62 |
| fov_err_deg | ≤-1.51 | 146 | 0.75 |
| fov_err_deg | -1.51〜1.12 | 146 | 0.77 |
| fov_err_deg | >1.12 | 144 | 0.81 |
| aim_err_mm | ≤0.94 | 146 | 0.79 |
| aim_err_mm | 0.94〜2.06 | 145 | 0.74 |
| aim_err_mm | >2.06 | 145 | 0.81 |
| clutter | False | 373 | 0.77 |
| clutter | True | 63 | 0.81 |
