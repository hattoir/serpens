"""シミュレーション run の記録（Record/Replay lite）。

**同じ run を後からもう一度作れるだけの情報**を残す。Phase 8 の本実装ではない。

残すもの: 時刻 / source / config のハッシュ / モデルの版と指紋 / belly / 摩擦 /
歩容パラメータ / seed / 各ステップの指令角・実角・機体の姿勢・安全状態・故障イベント。

**`source` を必ず入れる**（KINEMATIC_SIM / MUJOCO_SIM / HARDWARE）。
記録を後から見たときに、模擬と実測を取り違えないため。
"""
from __future__ import annotations

import hashlib
import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

FORMAT_VERSION = "serpens-record-1"


def config_hash(cfg: dict[str, Any]) -> str:
    """設定の指紋。config が変われば記録も別物になる。"""
    dump = json.dumps(cfg, sort_keys=True, default=str, ensure_ascii=False)
    return hashlib.sha256(dump.encode("utf-8")).hexdigest()[:16]


@dataclass
class RunMeta:
    """run を再現するために要るもの。"""

    source: str                       # KINEMATIC_SIM / MUJOCO_SIM / HARDWARE
    config_hash: str
    seed: int
    started_at: str = field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    format_version: str = FORMAT_VERSION
    model_version: str = ""
    model_digest: str = ""
    belly: str = ""
    friction: dict[str, float] = field(default_factory=dict)
    gait: dict[str, float] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


class Recorder:
    """1 run ぶんの時系列を貯めて JSON で書き出す。"""

    def __init__(self, meta: RunMeta) -> None:
        self.meta = meta
        self.rows: list[dict[str, Any]] = []
        self.events: list[dict[str, Any]] = []

    def add(self, t: float, **fields: Any) -> None:
        """1ステップぶん（指令角・実角・姿勢・安全状態など）。"""
        self.rows.append({"t": round(float(t), 4), **fields})

    def event(self, t: float, kind: str, detail: str = "") -> None:
        """故障・停止・状態遷移などの出来事。"""
        self.events.append({"t": round(float(t), 4), "kind": kind, "detail": detail})

    def save(self, path: str | Path) -> Path:
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps({"meta": asdict(self.meta), "rows": self.rows,
                                 "events": self.events}, ensure_ascii=False, indent=1),
                     encoding="utf-8")
        return p

    @property
    def duration_s(self) -> float:
        return self.rows[-1]["t"] - self.rows[0]["t"] if len(self.rows) > 1 else 0.0


@dataclass(frozen=True)
class LoadedRun:
    meta: dict[str, Any]
    rows: list[dict[str, Any]]
    events: list[dict[str, Any]]

    @property
    def source(self) -> str:
        return str(self.meta.get("source", "UNKNOWN"))

    def series(self, key: str) -> list[Any]:
        return [r.get(key) for r in self.rows]


def load(path: str | Path) -> LoadedRun:
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    return LoadedRun(meta=d["meta"], rows=d["rows"], events=d.get("events", []))


def same_setup(a: LoadedRun, b: LoadedRun) -> bool:
    """2つの記録が「同じ条件」か（再現できるか）を判定する。"""
    keys = ("source", "config_hash", "seed", "model_digest", "belly", "gait")
    return all(a.meta.get(k) == b.meta.get(k) for k in keys)
