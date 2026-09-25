"""1 地点の 4 枚（全消灯 / 通常 / 斜め照明 / 線光）から「床から何か出っ張っているか」を判定する。

  1. 各照明画像から全消灯を引く（環境光を落とす）
  2. 通常画像と基準床（いつもの床）の差分 → 前景の候補（模様・汚れも入る）
  3. 斜め照明 / 通常 の**比**（床の模様・汚れの濃さが打ち消される）→ 局所的な影 = 出っ張りの証拠
  4. 線光: 行ごとに線の位置を検出し、床上の線からの横ずれ → 較正した光の面との交点で高さ。
     線の途切れ（鏡面で線が乗らない）も出っ張りの証拠として扱う。**高さが測れないことを安全の根拠にしない**
  5. 候補ごとに 大きさ（画素 → mm。床上の局所スケール）・高さ・証拠 をまとめ、汚れ（高さなし・影なし）と物体を分ける

位置合わせ（撮影中の微小なずれ）は ECC は使わず、位相相関で平行移動だけ補正する（`align`）。
数値はすべて config の floor_watch.detect（DESIGN 値。実写で調整）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from serpens.floorwatch.geometry import Camera, LightPlane


@dataclass
class LineTrace:
    """線光の行ごとの検出。"""

    rows: np.ndarray                 # 行番号
    u_line: np.ndarray               # 検出した線の列（nan = 線が無い）
    u_floor: np.ndarray              # 床上の線の期待列
    height_mm: np.ndarray            # 交点の高さ（nan = 線が無い）
    width_px: float                  # 自動推定した線幅


@dataclass
class Candidate:
    """検出した 1 つの塊。"""

    bbox_px: tuple[int, int, int, int]        # x, y, w, h
    centroid_px: tuple[float, float]
    floor_xy_mm: tuple[float, float]           # 足元の床座標（カメラ直下原点）
    diameter_mm: float
    diameter_sigma_mm: float
    height_mm: float                           # 線光から（測れなければ 0）
    height_sigma_mm: float
    height_measured: bool
    shadow: bool                               # 斜め照明で影が付いた
    line_dropout: bool                         # 線が途切れた（鏡面の疑い）
    is_object: bool                            # 出っ張りの証拠（影 / 高さ / 途切れ）がある
    shape: str                                 # blob / line（床の段差は線状）
    rationale: list[str] = field(default_factory=list)


def align(ref: np.ndarray, img: np.ndarray, min_response: float = 0.2, max_shift_px: float = 12.0) -> np.ndarray:
    """位相相関で img を ref に合わせる（平行移動だけ。回転・スケールは無い前提）。

    同じ照明で撮った画像同士にだけ使う（線光の画像を通常画像に合わせようとすると内容が違って暴れる）。
    応答が弱い・ずれが大きすぎるときは合わせない（静止して撮る前提なので、ずれは小さいはず）。
    """
    a = np.float32(ref)
    b = np.float32(img)
    (dx, dy), response = cv2.phaseCorrelate(a, b)
    if response < min_response or abs(dx) > max_shift_px or abs(dy) > max_shift_px or (abs(dx) < 0.3 and abs(dy) < 0.3):
        return img
    M = np.float32([[1, 0, -dx], [0, 1, -dy]])
    return cv2.warpAffine(img, M, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)


def _sub(img: np.ndarray, dark: np.ndarray) -> np.ndarray:
    return np.clip(np.float32(img) - np.float32(dark), 0, None)


def estimate_line_width_px(line_on: np.ndarray, line_off: np.ndarray, u_center: float, search_px: int) -> float:
    """点灯 − 消灯 の床画像から線幅（半値幅）を自動推定する。式には入れない。"""
    d = _sub(line_on, line_off)
    lo, hi = int(max(0, u_center - search_px)), int(min(d.shape[1], u_center + search_px))
    prof = np.median(d[:, lo:hi], axis=0)
    if prof.max() <= 0:
        return float("nan")
    return float(np.sum(prof >= prof.max() / 2))


def _run_extent(mask: np.ndarray, i: int) -> int:
    """mask[i] を含む連続 True 区間の、i からの最大距離。"""
    lo = i
    while lo - 1 >= 0 and mask[lo - 1]:
        lo -= 1
    hi = i
    while hi + 1 < len(mask) and mask[hi + 1]:
        hi += 1
    return max(i - lo, hi - i)


def trace_line(line_sub: np.ndarray, cam: Camera, plane: LightPlane, det: dict[str, Any]) -> LineTrace:
    """行ごとに線の位置（輝度重心）を求め、光の面との交点で高さにする。"""
    H, W = line_sub.shape
    search, min_i = int(det["line_search_px"]), float(det["line_min_intensity"])
    rows = np.arange(H)
    u_floor = np.array([plane.line_u_on_floor(cam, float(v)) or np.nan for v in rows])
    u_line = np.full(H, np.nan)
    height = np.full(H, np.nan)
    widths: list[float] = []
    for v in rows:
        if np.isnan(u_floor[v]):
            continue
        lo, hi = int(max(0, u_floor[v] - search)), int(min(W, u_floor[v] + search))
        prof = line_sub[v, lo:hi]
        if prof.size == 0 or prof.max() < min_i:
            continue
        w = prof >= prof.max() * 0.5
        us = np.arange(lo, hi)[w]
        # 半値以上の連続区間のうち、最も明るい画素を含むもの（床の線と上面の線が両方写る行では明るい方）
        peak = lo + int(np.argmax(prof))
        run = np.abs(us - peak) <= _run_extent(w, peak - lo)
        us, pw = us[run], prof[w][run]
        widths.append(float(len(us)))
        u = float((us * pw).sum() / pw.sum())
        h = plane.height_at(cam, u, float(v))
        if h is None:
            continue
        u_line[v], height[v] = u, h
    width = float(np.nanmedian(widths)) if widths else float("nan")
    return LineTrace(rows, u_line, u_floor, height, width)


def detect(frames: dict[str, np.ndarray], reference: np.ndarray, cam: Camera, plane: LightPlane,
           cfg: dict[str, Any]) -> tuple[list[Candidate], LineTrace, np.ndarray]:
    """frames: dark / normal / raking / line。reference: 同じ地点の「いつもの床」（通常照明、全消灯を引いた物）。"""
    det = cfg["floor_watch"]["detect"]
    dark = np.float32(frames["dark"])
    normal = _sub(frames["normal"], dark)
    raking = _sub(align(frames["normal"], frames["raking"]), dark)     # 同じ床が写る同士だけ位置合わせ
    line = _sub(frames["line"], dark)                                    # 線光は内容が違うので合わせない
    ref = np.float32(align(normal, reference)) if reference.shape == normal.shape else np.float32(reference)
    # 2. 基準床との差分（比で照明の違いを吸収 → 差）
    gain = float(np.median(normal) / max(np.median(ref), 1e-6))
    diff = normal - ref * gain
    sigma = max(float(np.median(np.abs(diff - np.median(diff))) / 0.6745), float(det["noise_sigma_min"]))
    fg = (np.abs(diff) > float(det["diff_z"]) * sigma).astype(np.uint8)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # 3. 斜め / 通常 の比 → 影
    ratio = raking / np.maximum(normal, 1.0)
    ratio /= max(float(np.median(ratio)), 1e-6)
    shadow = (ratio < float(det["shadow_ratio_max"])).astype(np.uint8)
    shadow = cv2.morphologyEx(shadow, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    # 4. 線光
    tr = trace_line(line, cam, plane, det)
    # 5. 候補
    n, labels, stats, cents = cv2.connectedComponentsWithStats(fg, connectivity=8)
    out: list[Candidate] = []
    for i in range(1, n):
        x, y, w, h, area = (int(v) for v in stats[i])
        if area < int(det["min_blob_px"]):
            continue
        out.append(_candidate(i, labels, (x, y, w, h), cents[i], shadow, tr, cam, det))
    return out, tr, fg


def _candidate(i: int, labels: np.ndarray, bbox: tuple[int, int, int, int], cent: np.ndarray, shadow: np.ndarray,
               tr: LineTrace, cam: Camera, det: dict[str, Any]) -> Candidate:
    x, y, w, h = bbox
    cu, cv_ = float(cent[0]), float(cent[1])
    mask = labels == i
    # 大きさ: 足元の局所スケール（画素 → mm）
    foot = cam.floor_point(cu, y + h)
    scale = cam.mm_per_px_at(cu, y + h)
    diameter = float(max(w, h)) * scale if np.isfinite(scale) else float("nan")
    # 影: 塊の右隣（LED が左）に影があるか。影は床に落ちるので、上面より下の行（足元〜さらに h 下）を見る
    r0, r1 = y + h // 2, min(shadow.shape[0], y + 3 * h)
    band = shadow[r0:r1, x + w:min(shadow.shape[1], x + 2 * w + 4)]
    has_shadow = band.size > 0 and band.mean() > 0.15
    # 線光: 床の線（u_floor）が塊の列範囲を通る行だけ評価できる（線から外れた物は線の証拠を持たない）
    rows = np.arange(y, y + h)
    margin = tr.width_px if np.isfinite(tr.width_px) else 0.0
    crossing = [v for v in rows if not np.isnan(tr.u_floor[v]) and x - margin <= tr.u_floor[v] <= x + w + margin]
    hs = np.array([tr.height_mm[v] for v in crossing if not np.isnan(tr.height_mm[v])])
    missing = sum(1 for v in crossing if np.isnan(tr.u_line[v]))
    dropout = missing >= int(det["dropout_rows_min"])
    height = float(np.nanmedian(hs)) if hs.size else 0.0
    h_sigma = float(np.nanstd(hs)) if hs.size > 1 else float("nan")
    measured = hs.size >= 3 and height >= float(det["height_object_min_mm"])
    rationale = []
    if has_shadow:
        rationale.append("斜め照明で影")
    if measured:
        rationale.append(f"線光の高さ {height:.1f}mm")
    if dropout:
        rationale.append(f"線が {missing} 行途切れ（鏡面の疑い）")
    is_object = has_shadow or measured or dropout
    if not is_object:
        rationale.append("高さなし・影なし → 模様/汚れ")
    shape = "line" if max(w, h) > 4 * max(min(w, h), 1) else "blob"
    return Candidate((x, y, w, h), (cu, cv_), (float(foot[0]), float(foot[1])) if foot is not None else (np.nan, np.nan),
                     diameter, diameter * 0.15 if np.isfinite(diameter) else float("nan"),
                     height if measured else 0.0, h_sigma if measured else float("nan"), measured, bool(has_shadow),
                     bool(dropout), bool(is_object), shape, rationale)
