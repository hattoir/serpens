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
    return data


def joint_names(cfg: dict[str, Any]) -> list[str]:
    """関節名の一覧（J1〜J9）を返す。"""
    return [j["name"] for j in cfg["joints"]]


def servo_ids(cfg: dict[str, Any]) -> list[int]:
    """サーボ ID の一覧を関節順で返す。"""
    return [int(j["servo_id"]) for j in cfg["joints"]]
