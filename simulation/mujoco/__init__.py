"""Serpens の MuJoCo モデルと橋渡し。

  model.py   … config/robot.yaml から MJCF（9軸の簡略モデル）を作る
  belly.py   … WHEEL / SNAKE_ISOTROPIC / SNAKE_ANISOTROPIC の接触設定
  runner.py  … 歩容を与えて走らせ、前進量・トルクなどを測る
  record.py  … run の記録（seed / config hash / model version / 結果）

**すべて SIMULATED。** 質量も摩擦も実測ではない（docs/verification_status.md）。
"""
