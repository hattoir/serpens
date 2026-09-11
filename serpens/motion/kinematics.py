"""胴体の順運動学（尾端基準のボディ座標）。

ボディ座標: 尾端 = 原点、胴体をまっすぐにしたとき +x が頭方向、+z が上。
関節角の符号:
  yaw   (+) = 上から見て反時計回り（左へ曲がる）
  pitch (+) = 頭側を持ち上げる
  roll  (+) = 頭方向を向いて右に傾ける（右耳が下がる）
ポーズの自己干渉チェック（とぐろ）とシミュレータの両方で使う。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

AXIS_INDEX = {"roll": 0, "pitch": 1, "yaw": 2}


def _rot(axis: str, deg: float) -> np.ndarray:
    """ローカル軸まわりの回転行列。pitch は +で頭が上がるよう y 軸まわり負回転。"""
    a = np.radians(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "yaw":
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    if axis == "pitch":
        return np.array([[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]])
    if axis == "roll":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, s], [0.0, -s, c]])
    raise ValueError(f"未知の軸: {axis}")


@dataclass(frozen=True)
class Chain:
    """関節の並びとリンク長。"""

    names: list[str]
    axes: list[str]
    link_mm: list[float]    # link_mm[0] = 尾端→J1, link_mm[k] = J_k→J_{k+1}, 最後 = J9→頭先端

    @staticmethod
    def from_cfg(cfg: dict[str, Any]) -> "Chain":
        """config の body / joints から作る。"""
        js = cfg["joints"]
        xs = [float(cfg["body"]["tail_x_mm"])] + [float(j["x_mm"]) for j in js]
        xs.append(float(cfg["body"]["head_tip_x_mm"]))
        links = [b - a for a, b in zip(xs[:-1], xs[1:])]
        return Chain([j["name"] for j in js], [j["axis"] for j in js], links)


def forward(chain: Chain, angles: dict[str, float]) -> tuple[np.ndarray, list[np.ndarray]]:
    """関節角 → 各点の3次元位置。

    戻り値: (points, frames)
      points: shape (N+2, 3)。尾端, J1…J9, 頭先端 の位置 [mm]
      frames: 各点での姿勢行列（列ベクトル = ローカル x,y,z 軸）
    """
    R = np.eye(3)
    p = np.zeros(3)
    pts = [p.copy()]
    frames = [R.copy()]
    for k, (name, axis) in enumerate(zip(chain.names, chain.axes)):
        p = p + R @ np.array([chain.link_mm[k], 0.0, 0.0])
        R = R @ _rot(axis, angles.get(name, 0.0))
        pts.append(p.copy())
        frames.append(R.copy())
    p = p + R @ np.array([chain.link_mm[-1], 0.0, 0.0])
    pts.append(p.copy())
    frames.append(R.copy())
    return np.array(pts), frames


def sample_centerline(points: np.ndarray, step_mm: float) -> tuple[np.ndarray, np.ndarray]:
    """折れ線の中心線を step_mm 間隔でサンプリングする。

    戻り値: (samples, arc)  samples: shape (M, dim)、arc: 尾端からの道のり [mm]
    """
    samples, arc = [points[0]], [0.0]
    s0 = 0.0
    for a, b in zip(points[:-1], points[1:]):
        length = float(np.linalg.norm(b - a))
        n = max(int(np.ceil(length / step_mm)), 1)
        for k in range(1, n + 1):
            samples.append(a + (b - a) * k / n)
            arc.append(s0 + length * k / n)
        s0 += length
    return np.array(samples), np.array(arc)


def min_self_clearance(points: np.ndarray, arc_skip_mm: float, step_mm: float) -> float:
    """胴体の中心線どうしの最短距離 [mm]。

    胴体に沿った道のりが arc_skip_mm 以上離れた点どうしだけを比べる
    （近くの点は関節でつながっているので当然近い）。
    この値が胴体の直径以上なら、セグメント同士はぶつかっていない。
    """
    pts, arc = sample_centerline(points, step_mm)
    d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=-1)
    far = np.abs(arc[:, None] - arc[None, :]) >= arc_skip_mm
    return float(d[far].min()) if far.any() else float("inf")
