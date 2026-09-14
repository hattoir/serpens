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
    """可動域の包含関係 geometry ⊇ mechanical ⊇ operational を確かめる。

    **ソフトの都合で operational を機構の外へ広げられないようにする。**
    出典: CAD `Serpens_BELLY_R03_TWO_LINK_REVIEW`（±64° 干渉なし / ±65° 干渉）。
    """
    for j in cfg.get("joints", []):
        g = (float(j["geometry_min_deg"]), float(j["geometry_max_deg"]))
        m = (float(j["mechanical_min_deg"]), float(j["mechanical_max_deg"]))
        o = (float(j["min_deg"]), float(j["max_deg"]))
        if not g[0] <= m[0] < m[1] <= g[1]:
            raise ValueError(f"{j['name']}: mechanical {m} が geometry {g} の外")
        if not m[0] <= o[0] < o[1] <= m[1]:
            raise ValueError(f"{j['name']}: operational {o} が mechanical {m} の外"
                             "（CAD の干渉検査を超えた角度をソフトから出そうとしている）")


def joint_names(cfg: dict[str, Any]) -> list[str]:
    """関節名の一覧（J1〜J9）を返す。"""
    return [j["name"] for j in cfg["joints"]]


def servo_ids(cfg: dict[str, Any]) -> list[int]:
    """サーボ ID の一覧を関節順で返す。"""
    return [int(j["servo_id"]) for j in cfg["joints"]]
