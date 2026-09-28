# H2 視覚: Monte Carlo（SYNTHETIC_VISION_SIM。条件 300、試行 900、4.5 分、UXGA×0.5）

条件は家の中の見込み（`tools/h2_vision.py` の `_sample_condition`、すべて ASSUMPTION）から同時にずらした。

| 指標 | 値 |
|---|---|
| n_pos | 600 |
| n_neg | 300 |
| patrol_recall | 0.99 |
| inspect_recall | 0.99 |
| critical_inspect_recall | 1.00 |
| specular_critical_flagged | 0.82 |
| false_alarm_patrol | 0.09 |
| false_alarm_inspect | 0.18 |
| height_abs_err_median_mm | 3.39 |
| diameter_rel_err_median | 0.13 |
| loc_err_median_mm | 5.92 |

## critical の inspect 検出 × 因子（数値因子は 3 分位、カテゴリは値ごと）

| 因子 | 区分 | n | 検出 |
|---|---|---|---|
| floor | carpet | 76 | 1.00 |
| floor | pattern | 69 | 1.00 |
| floor | rug | 62 | 1.00 |
| floor | tile | 90 | 0.99 |
| floor | wood | 139 | 1.00 |
| floor_height_sigma_mm | ≤0.00 | 229 | 1.00 |
| floor_height_sigma_mm | 0.00〜0.10 | 63 | 1.00 |
| floor_height_sigma_mm | >0.10 | 144 | 1.00 |
| blur_sigma_px | ≤0.00 | 174 | 1.00 |
| blur_sigma_px | 0.00〜2.00 | 178 | 1.00 |
| blur_sigma_px | >2.00 | 84 | 0.99 |
| ambient_lux | ≤8.00 | 149 | 1.00 |
| ambient_lux | 8.00〜50.00 | 151 | 0.99 |
| ambient_lux | >50.00 | 136 | 1.00 |
| shot_noise_k | ≤0.34 | 147 | 1.00 |
| shot_noise_k | 0.34〜0.61 | 144 | 1.00 |
| shot_noise_k | >0.61 | 145 | 0.99 |
| line_scatter_mm | ≤0.00 | 298 | 1.00 |
| line_scatter_mm | 0.00〜0.00 | 0 | — |
| line_scatter_mm | >0.00 | 138 | 1.00 |
| cam_height_err_mm | ≤-2.23 | 146 | 1.00 |
| cam_height_err_mm | -2.23〜0.55 | 145 | 0.99 |
| cam_height_err_mm | >0.55 | 145 | 1.00 |
| cam_pitch_err_deg | ≤-0.94 | 146 | 0.99 |
| cam_pitch_err_deg | -0.94〜0.72 | 145 | 1.00 |
| cam_pitch_err_deg | >0.72 | 145 | 1.00 |
| fov_err_deg | ≤-1.51 | 146 | 1.00 |
| fov_err_deg | -1.51〜1.12 | 146 | 0.99 |
| fov_err_deg | >1.12 | 144 | 1.00 |
| aim_err_mm | ≤0.94 | 146 | 1.00 |
| aim_err_mm | 0.94〜2.06 | 145 | 0.99 |
| aim_err_mm | >2.06 | 145 | 1.00 |
| clutter | False | 373 | 1.00 |
| clutter | True | 63 | 1.00 |
