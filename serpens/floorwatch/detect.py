"""1 地点の撮影（通常 → 斜め照明 → 線光 → 全消灯 → 通常）から「床から何か出っ張っているか」を判定する。

  0. 最初と最後の通常画像を比べ、motion_max_px 以上動いていたら撮り直し（照明の違う画像同士の位置合わせはしない）
  1. 各照明画像から全消灯を引く
  2. **基準床は使わない**（実機では同じ視点の「物の無い床」は手に入らない）。通常画像に低次の多項式の面を頑健に
     当てはめて（外れ値 = 物や汚れを除きながら）背景にし、その差で前景の候補（模様・汚れも入る）
  3. 斜め照明 / 通常 の比（床の模様の濃さが打ち消される）を全体の中央値で正規化 → 影。影は物の**奥**（画像の上側）。
     明るさの差が無い物（床と同じ色）は影だけ、線の異常だけからも候補を作る（patrol は影だけ、inspect は線も）
  4. 線光: 行ごとに線の位置。基準は較正した光の面が予測する床の線と、物の前後の床上の線（局所の中央値）。
     横ずれ → 光の面との交点で高さ。線が途切れたら「測れない」（**height=None + 理由**。0 とは書かない）
  5. 候補: 大きさ・高さ（None なら理由）・影・途切れ・形。「線の途切れ + 円形 + 直径（metal_disc_diameter_mm）」は metal_disc
     （ボタン電池の可能性）として危険物側へ回す。汚れ・模様（影なし・高さなし）と線状の継ぎ目は物ではない

数値はすべて config の floor_watch.detect（DESIGN 値。実写で調整）。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np

from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.pose import estimate_pose, posed_camera

HEIGHT_REASONS = ("measured", "specular_break", "off_line", "too_few_rows")


class MotionError(RuntimeError):
    """撮影中に動いた。撮り直す。"""


@dataclass
class LineTrace:
    rows: np.ndarray
    u_line: np.ndarray               # 検出した線の列（nan = 線が無い）
    u_floor: np.ndarray              # 較正した面が予測する床の線の列
    height_mm: np.ndarray            # 交点の高さ（nan = 線が無い）
    width_px: float                  # 自動推定した線幅
    pose: tuple[float, float, float] | None = None   # 床の線から推定したカメラのずれ (Δ高さ mm, Δpitch °, 残差 px)。使わなければ None


@dataclass
class Candidate:
    bbox_px: tuple[int, int, int, int]
    centroid_px: tuple[float, float]
    floor_xy_mm: tuple[float, float]
    diameter_mm: float
    diameter_sigma_mm: float
    height_mm: float | None           # None = 測れない（height_reason に理由）
    height_sigma_mm: float | None
    height_reason: str
    shadow: bool
    line_dropout: bool
    on_line: bool                     # 床の線が塊を通る（線の証拠を評価できる）
    is_object: bool
    shape: str                        # blob / line
    kinds: list[dict[str, Any]]       # [{kind, confidence}]（分類器ができるまでは形と証拠だけ）
    rationale: list[str] = field(default_factory=list)


def motion_px(a: np.ndarray, b: np.ndarray) -> float:
    (dx, dy), _resp = cv2.phaseCorrelate(np.float32(a), np.float32(b))
    return float(np.hypot(dx, dy))


def _sub(img: np.ndarray, dark: np.ndarray) -> np.ndarray:
    return np.clip(np.float32(img) - np.float32(dark), 0, None)


def scaled_thresholds(cfg: dict[str, Any], cam: Camera) -> dict[str, Any]:
    """画素単位のしきい値を、使うモードの f / reference_f_px に比例させる（面積は 2 乗）。設計値は f=1000 で決めた。"""
    det = dict(cfg["floor_watch"]["detect"])
    k = cam.f_px / float(det["reference_f_px"])
    for key in det["scale_with_f"]:
        det[key] = int(round(det[key] * k)) if isinstance(det[key], int) else det[key] * k
    for key in det["scale_with_f2"]:
        det[key] = int(round(det[key] * k * k)) if isinstance(det[key], int) else det[key] * k * k
    return det


def robust_background(img: np.ndarray, degree: int, z: float, iters: int = 3, step: int = 4) -> np.ndarray:
    """同じ画像の床に低次の多項式面を頑健に当てはめる（残差 z·σ 超えを外して繰り返す）。基準床の代わり。"""
    H, W = img.shape
    ys, xs = np.mgrid[0:H:step, 0:W:step]
    terms = [(i, j) for i in range(degree + 1) for j in range(degree + 1 - i)]

    def design(xx: np.ndarray, yy: np.ndarray) -> np.ndarray:
        nx, ny = xx / W * 2 - 1, yy / H * 2 - 1
        return np.stack([nx ** i * ny ** j for i, j in terms], axis=-1)
    A, b = design(xs.ravel(), ys.ravel()), img[::step, ::step].ravel().astype(np.float64)
    keep = np.ones(len(b), bool)
    coef = np.zeros(len(terms))
    for _ in range(iters):
        coef = np.linalg.lstsq(A[keep], b[keep], rcond=None)[0]
        res = b - A @ coef
        sigma = max(float(np.median(np.abs(res[keep] - np.median(res[keep]))) / 0.6745), 1e-3)
        keep = np.abs(res) < z * sigma
    yf, xf = np.mgrid[0:H, 0:W]
    return (design(xf.ravel(), yf.ravel()) @ coef).reshape(H, W).astype(np.float32)


def _run_extent(mask: np.ndarray, i: int) -> int:
    lo = i
    while lo - 1 >= 0 and mask[lo - 1]:
        lo -= 1
    hi = i
    while hi + 1 < len(mask) and mask[hi + 1]:
        hi += 1
    return max(i - lo, hi - i)


def _floor_line_columns(plane: LightPlane, cam: Camera, rows: np.ndarray) -> np.ndarray:
    """行ごとの、床の上の線が写る列。床の線は画像でも直線なので式で出す（行ごとの二分法より速い。値は 1e-6 px まで一致）。
    床に届かない行（水平線より上）は nan。"""
    img = plane.floor_line_image(cam)
    if img is None or abs(img[1][1]) < 1e-9:
        return np.array([plane.line_u_on_floor(cam, float(v)) or np.nan for v in rows])
    (a0, a1), (d0, d1) = img
    u = a0 + (rows.astype(float) - a1) * d0 / d1
    ok = np.array([cam.floor_point(float(uu), float(v)) is not None for uu, v in zip(u, rows)])
    return np.where(ok & (u > -cam.width_px) & (u < 2 * cam.width_px), u, np.nan)


def trace_line(line_sub: np.ndarray, cam: Camera, plane: LightPlane, det: dict[str, Any]) -> LineTrace:
    """行ごとに線の位置（半値以上の連続区間の輝度重心）→ 光の面との交点で高さ。"""
    H, W = line_sub.shape
    search, min_i = int(det["line_search_px"]), float(det["line_min_intensity"])
    rows = np.arange(H)
    u_floor = _floor_line_columns(plane, cam, rows)
    u_line, height = np.full(H, np.nan), np.full(H, np.nan)
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
        peak = lo + int(np.argmax(prof))
        run = np.abs(us - peak) <= _run_extent(w, peak - lo)
        us, pw = us[run], prof[w][run]
        widths.append(float(len(us)))
        u = float((us * pw).sum() / pw.sum())
        h = plane.height_at(cam, u, float(v))
        if h is not None:
            u_line[v], height[v] = u, h
    return LineTrace(rows, u_line, u_floor, height, float(np.nanmedian(widths)) if widths else float("nan"))


def detect(frames: dict[str, np.ndarray], cam: Camera, plane: LightPlane, cfg: dict[str, Any],
           with_line: bool = True) -> tuple[list[Candidate], LineTrace | None, np.ndarray]:
    """frames: normal / raking / line / dark / normal2。with_line=False は巡回中の発見（線なし）。"""
    det = scaled_thresholds(cfg, cam)
    if "normal2" in frames:
        m = motion_px(frames["normal"], frames["normal2"])
        if m >= float(det["motion_max_px"]):
            raise MotionError(f"撮影中に {m:.1f}px 動いた（上限 {det['motion_max_px']}px）。撮り直し")
    dark = np.float32(frames["dark"])
    normal = _sub(frames["normal"], dark)
    raking = _sub(frames["raking"], dark)
    blur = int(det["background_blur_px"]) | 1
    smooth = cv2.GaussianBlur(normal, (blur, blur), 0)                  # 木目の細かい縞を落とす
    diff = smooth - robust_background(smooth, int(det["background_poly_degree"]), float(det["diff_z"]))
    sigma = max(float(np.median(np.abs(diff - np.median(diff))) / 0.6745), float(det["noise_sigma_min"]))
    fg = (np.abs(diff) > float(det["diff_z"]) * sigma).astype(np.uint8)
    close = int(det["blob_close_px"]) | 1
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((close, close), np.uint8))
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    ratio = raking / np.maximum(normal, 1.0)
    ratio = ratio / max(float(np.median(ratio)), 1e-3)                  # 全体で正規化（露出差）
    # 通常画像で暗すぎる画素（黒い繊維・暗い床）は、明るさの落ち込みを見分けられない → 影としない（比が雑音になる。VIS-0005）
    shadow = ((ratio < float(det["shadow_ratio_max"])) & (normal >= float(det["shadow_min_signal"]))).astype(np.uint8)
    shadow = cv2.morphologyEx(shadow, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    line_sub = _sub(frames["line"], dark) if with_line and "line" in frames else None
    tr = trace_line(line_sub, cam, plane, det) if line_sub is not None else None
    tr, cam_floor = _pose_corrected(line_sub, tr, cam, plane, det)
    n, _labels, stats, _cents = cv2.connectedComponentsWithStats(fg, connectivity=8)
    boxes = [tuple(int(a) for a in stats[i][:4]) for i in range(1, n) if stats[i][4] >= int(det["min_blob_px"])]
    # 影・線だけの候補を「もう候補がある」として捨ててよいのは、**物と判定された候補**が覆っているときだけ。
    # 物にならなかった前景（模様・汚れ・継ぎ目と判定された塊）が覆っていても捨てない（H2 VIS-0002）:
    #   - 床と同じ色の薄い硬貨（10 円玉）は、手前の縁の細い帯だけが前景になり線状（継ぎ目）と判定される
    #   - 床に近い色の物は、上面が床に紛れ、左右の側面だけが小さな 2 つの塊になる（どちらも上に影が無い）
    #   どちらも、奥の影から作る候補までその塊が消していた
    out = [_candidate(b, shadow, tr, cam_floor, det) for b in boxes]
    objs = [c.bbox_px for c in out if c.is_object]
    extra = [_candidate(b, shadow, tr, cam_floor, det) for b in _shadow_only_boxes(shadow, objs, det)]
    out += extra
    objs += [c.bbox_px for c in extra if c.is_object]
    out += [_candidate(b, shadow, tr, cam_floor, det) for b in _line_only_boxes(tr, objs, det)]
    return out, tr, fg


def _pose_corrected(line_sub: np.ndarray | None, tr: LineTrace | None, cam: Camera, plane: LightPlane,
                   det: dict[str, Any]) -> tuple[LineTrace | None, Camera]:
    """線があるとき、床の線からカメラの高さ・pitch のずれを推定し（H2 VIS-0003/0006）、
      1. 直した姿勢の床の線を中心に線を探し直す（名目で探すと、ずれの大きい側で探す窓から線が外れていた）
      2. 候補の床の位置・大きさを、直した姿勢のカメラで出す（名目で床へ戻すと 10〜30mm ずれ、線の狙い直しが外れていた）
    推定が信用できない（点が少ない・残差が大きい・範囲の端）ときは名目のまま。"""
    if tr is None or line_sub is None or not det.get("pose_from_line", False):
        return tr, cam
    hmax, pmax, rmax = float(det["pose_max_height_err_mm"]), float(det["pose_max_pitch_err_deg"]), float(det["pose_rms_max_px"])

    def points(t: LineTrace) -> np.ndarray:
        ok = ~np.isnan(t.u_line)
        return np.stack([t.u_line[ok], t.rows[ok].astype(float)], axis=1)

    def plausible(dh: float, dp: float) -> bool:
        return bool(np.isfinite(dh) and abs(dh) < 0.95 * hmax and abs(dp) < 0.95 * pmax)
    dh, dp, _ = estimate_pose(points(tr), cam, plane, hmax, pmax, trim=0.5)
    if not plausible(dh, dp):
        return tr, cam
    cam1 = posed_camera(cam, dh, dp)
    tr1 = trace_line(line_sub, cam1, plane.fixed_to_head(cam, cam1), det)
    dh, dp, rms = estimate_pose(points(tr1), cam, plane, hmax, pmax, trim=0.9)
    if not (plausible(dh, dp) and rms <= rmax):
        return tr, cam
    cam2 = posed_camera(cam, dh, dp)
    tr2 = trace_line(line_sub, cam2, plane.fixed_to_head(cam, cam2), det)
    tr2.pose = (dh, dp, rms)
    return tr2, cam2

def _is_line_shape(box: tuple[int, int, int, int], det: dict[str, Any]) -> bool:
    _x, _y, w, h = box
    return max(w, h) > float(det["line_aspect_min"]) * max(min(w, h), 1)


def _covered(box: tuple[int, int, int, int], boxes: list[tuple[int, int, int, int]], margin: int) -> bool:
    x, y, w, h = box
    for bx, by, bw, bh in boxes:
        if bx - margin < x + w and x < bx + bw + margin and by - margin < y + h and y < by + bh + margin:
            return True
    return False


def _shadow_only_boxes(shadow: np.ndarray, boxes: list[tuple[int, int, int, int]],
                       det: dict[str, Any]) -> list[tuple[int, int, int, int]]:
    """明るさの差が無い物: 影の塊の手前（画像の下）に物があるとみなして候補を作る（patrol の主な経路）。"""
    H = shadow.shape[0]
    n, _l, stats, _c = cv2.connectedComponentsWithStats(shadow, connectivity=8)
    out = []
    for i in range(1, n):
        sx, sy, sw, sh, area = (int(a) for a in stats[i])
        if area < int(det["min_blob_px"]) or sw > float(det["line_aspect_min"]) * sh:      # 横に長い影 = 段差
            continue
        h = min(max(3, int(sw * float(det["shadow_object_rows_ratio"]))), H - (sy + sh))
        box = (sx, sy + sh, sw, h)
        if h * sw >= int(det["min_blob_px"]) and not _covered(box, boxes, int(det["blob_close_px"])):
            out.append(box)
    return out


def _line_only_boxes(tr: LineTrace | None, boxes: list[tuple[int, int, int, int]],
                     det: dict[str, Any]) -> list[tuple[int, int, int, int]]:
    """線の異常（途切れ・持ち上がり）が続く行から候補を作る（床と同じ色の物、透明な物）。"""
    if tr is None or not np.isfinite(tr.width_px):
        return []
    valid = ~np.isnan(tr.u_floor)
    present = ~np.isnan(tr.u_line)
    base = _running_baseline(tr.height_mm, int(det["line_baseline_rows"]))
    resid = tr.height_mm - base
    # 床そのものの凹凸（カーペットの毛足など）が閾値を超えると、何も無い床で線だけの候補が出る（VIS-0005: 毛足 σ0.8mm で誤報 0.78）。
    # 閾値は床の線の揺れ（頑健な σ）の line_rough_z 倍と height_object_min_mm の大きい方にする（平らな床では従来どおり）
    r = resid[present & ~np.isnan(resid)]
    rough = float(np.median(np.abs(r - np.median(r))) / 0.6745) if r.size else 0.0
    thr = max(float(det["height_object_min_mm"]), float(det["line_rough_z"]) * rough)
    raised = present & (np.abs(np.nan_to_num(resid)) >= thr)
    anomaly = valid & (~present | raised)
    rows_present = np.flatnonzero(present)
    ctx = int(det["line_context_rows"])
    out, start = [], None
    margin = int(tr.width_px * 1.5)
    for v in range(len(anomaly) + 1):
        on = v < len(anomaly) and anomaly[v]
        if on and start is None:
            start = v
        elif not on and start is not None:
            # 物が線を途切れさせる・持ち上げるなら、その前後には床の線が見えている。線の端（遠くで細って消える・
            # 画像の縁）で始まる／終わる異常は、物の証拠にしない（H2 VIS-0002: ぼけ・画角のずれで遠くの端が消えて誤報）
            flanked = bool(((rows_present < start) & (rows_present >= start - ctx)).any()
                           and ((rows_present >= v) & (rows_present < v + ctx)).any())
            if v - start >= int(det["dropout_rows_min"]) and flanked:
                u = float(np.nanmedian(tr.u_floor[start:v]))
                box = (max(0, int(u - margin)), start, 2 * margin, v - start)
                if not _covered(box, boxes, int(det["blob_close_px"])):
                    out.append(box)
            start = None
    return out


def _running_baseline(height: np.ndarray, half_rows: int) -> np.ndarray:
    """線の上の床の高さの基準を、行の近くの中央値で取る（全体の中央値 1 つにしない）。カメラの高さ・pitch が少しずれると、
    床の高さは線に沿って傾いた坂（遠くほど大きく外れる）になり、全体の中央値との差で近い側が「出っ張り」に見えていた
    （VIS-0002: pitch −2° で近い側が +1.3mm）。窓は物より十分広い（物は窓の数分の 1 なので中央値に効かない）。"""
    h = np.asarray(height, float)
    out = np.full(h.shape, np.nan)
    idx = np.flatnonzero(~np.isnan(h))
    if idx.size == 0:
        return out
    vals = h[idx]
    lo = np.searchsorted(idx, idx - half_rows)
    hi = np.searchsorted(idx, idx + half_rows, side="right")
    for k, (a, b) in enumerate(zip(lo, hi)):
        out[idx[k]] = float(np.median(vals[a:b]))
    return out


def _line_evidence(bbox: tuple[int, int, int, int], tr: LineTrace | None,
                   det: dict[str, Any]) -> tuple[bool, list[float], int]:
    """塊を通る線の行の高さと、途切れた行数。高さは物の前後の床上の線を基準に補正する。"""
    if tr is None:
        return False, [], 0
    x, y, w, h = bbox
    margin = tr.width_px if np.isfinite(tr.width_px) else 0.0
    rows = [v for v in range(y, y + h) if not np.isnan(tr.u_floor[v]) and x - margin <= tr.u_floor[v] <= x + w + margin]
    if not rows:
        return False, [], 0
    ctx = int(det["line_context_rows"])
    before = [tr.height_mm[v] for v in range(max(0, y - ctx), y) if not np.isnan(tr.height_mm[v])]
    after = [tr.height_mm[v] for v in range(y + h, min(len(tr.rows), y + h + ctx)) if not np.isnan(tr.height_mm[v])]
    base = float(np.median(before + after)) if before + after else 0.0          # 物の前後の床上の線（局所基準）
    hs = [float(tr.height_mm[v] - base) for v in rows if not np.isnan(tr.height_mm[v])]
    missing = sum(1 for v in rows if np.isnan(tr.u_line[v]))
    return True, hs, missing


def _roundish(dx_mm: float, foot: np.ndarray | None, top: np.ndarray | None, cam: Camera, max_h_mm: float) -> bool:
    """横幅 dx と前後の長さが円に見合うか。**前後の長さは物の高さで変わる**: 上面が高さ H にあると、塊の上辺の視線が
    床ではなく z=H で交わるので、床へ投影した奥の縁は (1 − H/h_cam) 倍に縮む。H ∈ [0, max_h_mm] のどれかで
    0.5 ≤ dx/dy ≤ 2 になれば円形とする（高さを線で測れない鏡面の物でも使える）。横に長い塊（段差・継ぎ目）は通さない。"""
    if foot is None or top is None or not np.isfinite(dx_mm):
        return False
    near, far0 = float(foot[1]), float(top[1])
    far_min = far0 * (1.0 - min(max_h_mm, cam.height_mm * 0.95) / cam.height_mm)
    dy_hi, dy_lo = far0 - near, far_min - near                    # H = 0 / H = max_h_mm のときの前後の長さ
    lo, hi = dx_mm / 2.0, dx_mm * 2.0                              # 円形とみなす dy の範囲
    return dy_hi > 0 and max(dy_lo, 1e-3) <= hi and dy_hi >= lo


def _candidate(bbox: tuple[int, int, int, int], shadow: np.ndarray, tr: LineTrace | None, cam: Camera,
               det: dict[str, Any]) -> Candidate:
    x, y, w, h = bbox
    cu, cv_ = x + w / 2.0, y + h / 2.0
    foot, top = cam.floor_point(cu, y + h), cam.floor_point(cu, y)
    scale = cam.mm_per_px_at(cu, y + h)
    diameter = float(max(w, h)) * scale if np.isfinite(scale) else float("nan")
    dx_mm = w * scale if np.isfinite(scale) else float("nan")
    dy_mm = float(top[1] - foot[1]) if foot is not None and top is not None else float("inf")   # 床上の前後の長さ
    roundish = _roundish(dx_mm, foot, top, cam, float(det.get("metal_disc_max_height_mm", 0.0)))
    band = shadow[max(0, y - int(h * float(det["shadow_band_rows_ratio"]))):y, x:x + w]   # 影は物の奥 = 塊の上辺から上へ
    has_shadow = band.size > 0 and band.mean() > float(det["shadow_band_min"])
    on_line, hs, missing = _line_evidence((x, y, w, h), tr, det)
    dropout = on_line and missing >= int(det["dropout_rows_min"])
    height: float | None = None
    h_sigma: float | None = None
    min_h = float(det["height_object_min_mm"])
    enough = len(hs) >= int(det["line_min_rows"])
    if not on_line:
        reason = "off_line"
    elif enough and (not dropout or float(np.median(hs)) >= min_h):     # 途切れがあっても持ち上がった行が十分なら測定
        height, h_sigma, reason = float(np.median(hs)), float(np.std(hs)), "measured"
    elif dropout:
        reason = "specular_break"                                        # 鏡面（金属）か黒い物: 線が乗らない
    else:
        reason = "too_few_rows"
    raised = height is not None and height >= min_h
    shape = "line" if _is_line_shape(bbox, det) else "blob"
    lo, hi = (float(v) for v in det["metal_disc_diameter_mm"])
    metal_disc = dropout and shape == "blob" and roundish and lo <= diameter <= hi
    big_enough = np.isfinite(diameter) and diameter >= float(det["object_min_diameter_mm"])
    is_object = (has_shadow or raised or dropout) and shape == "blob" and big_enough
    rationale = []
    if has_shadow:
        rationale.append("斜め照明で影（物の奥）")
    if raised:
        rationale.append(f"線光の高さ {height:.1f}mm")
    if dropout:
        rationale.append(f"線が {missing} 行途切れ（鏡面の疑い）。高さは測れない（None）")
    if metal_disc:
        rationale.append(f"途切れ + 円形 + 直径 {lo:g}〜{hi:g}mm → metal_disc（ボタン電池・磁石の可能性）")
    if shape == "line":
        rationale.append("線状 → 床の継ぎ目・段差の可能性（物ではない）")
    if shape == "blob" and not big_enough:
        rationale.append(f"直径 {diameter:.1f}mm は小さすぎる（雑音・木目のかけら）")
    elif not is_object and shape == "blob":
        rationale.append("高さなし・影なし → 模様/汚れ")
    kinds: list[dict[str, Any]] = []
    if metal_disc:
        kinds.append({"kind": "metal_disc", "confidence": float(det["metal_disc_confidence"])})
    if is_object:
        kinds.append({"kind": "unknown", "confidence": 0.5})
    elif shape == "blob":
        kinds.append({"kind": "stain_or_pattern", "confidence": 0.6})
    return Candidate((x, y, w, h), (cu, cv_), (float(foot[0]), float(foot[1])) if foot is not None else (np.nan, np.nan),
                     diameter, diameter * 0.15 if np.isfinite(diameter) else float("nan"), height, h_sigma, reason,
                     bool(has_shadow), bool(dropout), on_line, bool(is_object), shape, kinds, rationale)
