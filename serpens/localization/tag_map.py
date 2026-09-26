"""AprilTag の地図（`config/tags.yaml`）。座標系 home の定義そのもの（決定 Q11）。

Task / Event の `frame_id` と `map_version` はここから取る。地図を変えたら map_version を上げる
（受け側は版違いの Task を拒否する）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class Tag:
    id: int
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float               # タグの面が向いている方向（壁 → 部屋の内側）
    note: str = ""

    def facing(self) -> tuple[float, float]:
        return math.cos(self.yaw_rad), math.sin(self.yaw_rad)


@dataclass(frozen=True)
class TagMap:
    frame_id: str
    map_version: str
    family: str
    tag_size_m: float
    tags: dict[int, Tag]

    @classmethod
    def load(cls, path: str | Path) -> "TagMap":
        with Path(path).open(encoding="utf-8") as fp:
            d = yaml.safe_load(fp)
        return cls.from_dict(d)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "TagMap":
        tags = {int(t["id"]): Tag(int(t["id"]), float(t["x_m"]), float(t["y_m"]), float(t["z_m"]), float(t["yaw_rad"]),
                                  str(t.get("note", ""))) for t in d["tags"]}
        if len(tags) != len(d["tags"]):
            raise ValueError("tags.yaml: id が重複している")
        if str(d["frame_id"]) != "home":
            raise ValueError(f"tags.yaml: frame_id は home 固定（{d['frame_id']}）")
        return cls(str(d["frame_id"]), str(d["map_version"]), str(d["family"]), float(d["tag_size_m"]), tags)

    @classmethod
    def from_cfg(cls, cfg: dict[str, Any]) -> "TagMap":
        return cls.load(cfg["localization"]["tag_map_file"])

    def get(self, tag_id: int) -> Tag | None:
        return self.tags.get(int(tag_id))
