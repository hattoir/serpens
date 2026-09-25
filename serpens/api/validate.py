"""JSON Schema の**最小部分集合**の検証器（標準ライブラリだけ。`jsonschema` は依存に入れていない）。

対応: type / const / enum / required / properties / additionalProperties(false) / items / minItems / maxItems /
      minimum / maximum / minLength / maxLength / allOf / $ref（同一ファイル "#/$defs/x" と "common.json#/$defs/x"）
      / type の配列（["number","null"]）
加えて Serpens 固有の `x-serpens-rules`（task / event の種類ごとの required）を検査する。
スキーマは `schemas/*.json`。Home AI 側は本物の JSON Schema 検証器でそのまま読める。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

SCHEMA_DIR = Path(__file__).resolve().parent.parent.parent / "schemas"
_TYPES = {"object": dict, "array": list, "string": str, "integer": int, "number": (int, float), "boolean": bool,
          "null": type(None)}


class SchemaError(ValueError):
    """スキーマ違反。`path` は違反した場所。"""

    def __init__(self, path: str, message: str) -> None:
        super().__init__(f"{path}: {message}")
        self.path, self.message = path, message


class Validator:
    """1 つのスキーマファイルと、それが参照するファイルを読む。"""

    def __init__(self, name: str, schema_dir: Path = SCHEMA_DIR) -> None:
        self.dir = schema_dir
        self.docs: dict[str, dict[str, Any]] = {}
        self.root = self._load(name)

    def _load(self, name: str) -> dict[str, Any]:
        if name not in self.docs:
            self.docs[name] = json.loads((self.dir / name).read_text(encoding="utf-8"))
        return self.docs[name]

    def _resolve(self, ref: str, doc: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        file_part, _, ptr = ref.partition("#")
        d = self._load(file_part) if file_part else doc
        node: Any = d
        for key in [k for k in ptr.split("/") if k]:
            node = node[key]
        return node, d

    # ---- 検証 -------------------------------------------------------------------
    def validate(self, data: Any) -> None:
        """違反があれば SchemaError。"""
        self._check(data, self.root, self.root, "$")
        self._check_kind_rules(data, self.root)

    def errors(self, data: Any) -> list[str]:
        try:
            self.validate(data)
        except SchemaError as e:
            return [str(e)]
        return []

    def _check(self, v: Any, s: dict[str, Any], doc: dict[str, Any], path: str) -> None:
        if "$ref" in s:
            target, tdoc = self._resolve(s["$ref"], doc)
            self._check(v, target, tdoc, path)
        for sub in s.get("allOf", []):
            self._check(v, sub, doc, path)
        if "type" in s:
            types = s["type"] if isinstance(s["type"], list) else [s["type"]]
            ok = any(isinstance(v, _TYPES[t]) and not (t in ("integer", "number") and isinstance(v, bool)) for t in types)
            if not ok:
                raise SchemaError(path, f"型が {types} でない: {type(v).__name__}")
        if "const" in s and v != s["const"]:
            raise SchemaError(path, f"{v!r} ≠ {s['const']!r}")
        if "enum" in s and v not in s["enum"]:
            raise SchemaError(path, f"{v!r} は {s['enum']} に無い")
        if isinstance(v, (int, float)) and not isinstance(v, bool):
            if "minimum" in s and v < s["minimum"]:
                raise SchemaError(path, f"{v} < {s['minimum']}")
            if "maximum" in s and v > s["maximum"]:
                raise SchemaError(path, f"{v} > {s['maximum']}")
        if isinstance(v, str):
            if "minLength" in s and len(v) < s["minLength"]:
                raise SchemaError(path, "短すぎる")
            if "maxLength" in s and len(v) > s["maxLength"]:
                raise SchemaError(path, "長すぎる")
        if isinstance(v, dict):
            for k in s.get("required", []):
                if k not in v:
                    raise SchemaError(path, f"必須 {k!r} が無い")
            props = s.get("properties", {})
            for k, sub in props.items():
                if k in v and sub:
                    self._check(v[k], sub, doc, f"{path}.{k}")
            if s.get("additionalProperties") is False:
                extra = set(v) - set(props)
                if extra:
                    raise SchemaError(path, f"未知のフィールド {sorted(extra)}")
        if isinstance(v, list):
            if "minItems" in s and len(v) < s["minItems"]:
                raise SchemaError(path, "要素が少ない")
            if "maxItems" in s and len(v) > s["maxItems"]:
                raise SchemaError(path, "要素が多い")
            if "items" in s:
                for i, item in enumerate(v):
                    self._check(item, s["items"], doc, f"{path}[{i}]")

    def _check_kind_rules(self, data: Any, s: dict[str, Any]) -> None:
        """x-serpens-rules: 種類（task / event）ごとの必須フィールド。"""
        rules = s.get("x-serpens-rules")
        if not rules or not isinstance(data, dict):
            return
        key = "task" if "task" in rules.get("stop", {"required": []}) or "task" in data else "event"
        kind = data.get(key)
        for k in rules.get(kind, {}).get("required", []):
            if k not in data:
                raise SchemaError(f"$.{k}", f"{key}={kind!r} には {k!r} が必須")


_cache: dict[str, Validator] = {}


def validator(name: str) -> Validator:
    if name not in _cache:
        _cache[name] = Validator(name)
    return _cache[name]


def validate_task(msg: Any) -> list[str]:
    return validator("task.json").errors(msg)


def validate_event(msg: Any) -> list[str]:
    return validator("event.json").errors(msg)
