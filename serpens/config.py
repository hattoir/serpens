"""設定ファイル config/robot.yaml の読み込み。

全モジュールはこの関数で得た dict からパラメータを取り出す。
コード中に寸法・しきい値・ゲインを直書きしないこと。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

# リポジトリ直下の config/robot.yaml
DEFAULT_CONFIG_PATH: Path = Path(__file__).resolve().parent.parent / "config" / "robot.yaml"


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    """YAML 設定を読み込んで dict で返す。"""
    p = Path(path) if path is not None else DEFAULT_CONFIG_PATH
    with p.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"設定ファイルの形式が不正です: {p}")
    check_joint_limits(data)
    return data


def check_joint_limits(cfg: dict[str, Any]) -> None:
    """可動域の包含関係を確かめる（意味の違う3つを取り違えないため）。

        software_operational_limit ⊆ mechanical_design_limit ⊆ geometry_collision_onset

    出典は CAD `Serpens_BELLY_R03_TWO_LINK_REVIEW` / `Serpens_R03_COUPON_CABLE_PREP`。
    **geometry_collision_onset（干渉が始まる境界）を指令のクランプ値として使わない。**
    クランプに使うのは `min_deg` / `max_deg`（software_operational_limit）だけ。
    onset が null の関節（頭部）は CAD の干渉検査をしていない = UNKNOWN。
    """
    policy = cfg.get("joint_limit_policy", {})
    if policy.get("clamp_source", "software_operational_limit") != "software_operational_limit":
        raise ValueError("clamp_source は software_operational_limit でなければならない"
                         f"（現在: {policy.get('clamp_source')}）")
    for j in cfg.get("joints", []):
        mech = (float(j["mechanical_min_deg"]), float(j["mechanical_max_deg"]))
        oper = (float(j["min_deg"]), float(j["max_deg"]))
        if not mech[0] <= oper[0] < oper[1] <= mech[1]:
            raise ValueError(f"{j['name']}: software_operational_limit {oper} が "
                             f"mechanical_design_limit {mech} の外")
        onset = j.get("onset_deg")
        if onset is None:
            continue                      # 頭部は CAD 未検証（UNKNOWN）
        onset = float(onset)
        if max(abs(mech[0]), abs(mech[1])) > onset:
            raise ValueError(f"{j['name']}: mechanical_design_limit {mech} が "
                             f"干渉の始まる角度 ±{onset}° を超えている")
