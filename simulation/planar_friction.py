"""平面・準静的の蛇行モデル（方向別クーロン摩擦）。**source は PLANAR_FRICTION_SIM（PHYSICS_SIM の一種）。**

MuJoCo の接触摩擦では表せない次の差を扱うために作った（H0 摩擦の Hardware Gap 用）:

  - 前 / 後ろの差（鱗の向き）      μ_f ≠ μ_b
  - 左 / 右の差                    μ_left ≠ μ_right
  - 静止摩擦 / 動摩擦の差          方向ごとに違う倍率として扱う（下の「静止摩擦」）
  - 節ごとのばらつき（床のむら・摩耗）

モデル（Hu, Nirody, Scott, Shelley, PNAS 2009 と同じ形）:

  各接地点の摩擦力  F = −N · [ μ_t (v̂·t̂) t̂ + μ_n (v̂·n̂) n̂ ]（law="decoupled"。law="ellipse" も選べる）
    t̂ = 節の向き（頭方向）、n̂ = 左向きの法線、v̂ = v / √(|v|² + ε²)（原点付近をなめらかにする）
    μ_t = μ_f（前へ動く）/ μ_b（後ろへ動く）、μ_n = μ_left / μ_right（切り替えは ε の幅の tanh でなめらかに）
  体の形は指令角どおり（サーボが完全に追従する）と仮定し、慣性を無視する（準静的。0.1 m/s 程度の蛇行では
  慣性力は摩擦力より十分小さい、という仮定）。各瞬間に 合力 = 0・モーメント = 0 を満たす体の並進・回転を解く。

静止摩擦: 速くなるほど摩擦が下がる（Stribeck 型）性質を入れると、準静的な釣り合いの解が一意でなくなる
（スティックスリップは動力学が要る = MODEL GAP）。一方、準静的なクーロン摩擦では **全方向の μ を同じ倍率にしても
動きは変わらない**（変わるのはトルクと仕事だけ）。そこで静止 / 動の差は「方向ごとに違う倍率」= 比の不確かさとして、
呼び出し側（simulation/hardware_gaps/HG-H0_friction）で扱う。

限界（**このモデルで言えないこと**）:
  サーボが追従できない場合の形の崩れ（トルク上限を超えた時点で「不成立」と判定するだけ）、転倒・跳ね、
  床の空間的なむら（節ごとの固定倍率で近似）、毛足への沈み込み、段差。
"""
from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any

import numpy as np

from serpens.motion.gait import GaitParams, angles_at_phase, body_joint_names
from simulation.mujoco.model import _masses

SOURCE = "PLANAR_FRICTION_SIM"
# 異方性摩擦の法則の形。前後・横だけを測ると両者は同じ値になり、**斜めに滑らせたときだけ差が出る**。
#   decoupled … F = −N (μ_t v_t t̂ + μ_n v_n n̂) / |v|（Hu et al. 2009。蛇の腹の実測に合わせた形）
#   ellipse   … F = −N M² v / |M v|（最大散逸の楕円。MuJoCo の elliptic cone に近い）
LAWS = ("decoupled", "ellipse")
G = 9.81


@dataclass(frozen=True)
class Friction:
    """方向別の動摩擦係数と、静止摩擦の倍率。**値はすべて探索用（実測ではない）。**"""

    mu_f: float                 # 前へ滑るとき（節の頭方向）
    mu_b: float                 # 後ろへ滑るとき
    mu_left: float              # 左へ滑るとき
    mu_right: float             # 右へ滑るとき
    eps_m_s: float = 0.002      # v̂ のなめらかさ（数値のためだけ）
    law: str = "decoupled"      # 異方性摩擦の法則の形（下の LAWS）。**どちらが実物に近いかは未確認 = MODEL GAP**
    point_scale: tuple[float, ...] | None = None   # 接地点ごとの倍率（床のむら）。None で 1

    @staticmethod
    def simple(mu_along: float, mu_across: float, **kw: Any) -> "Friction":
        return Friction(mu_along, mu_along, mu_across, mu_across, **kw)

    @property
    def ratio(self) -> float:
        """横 / 前（推進に効く比の代表値）。"""
        return 0.5 * (self.mu_left + self.mu_right) / self.mu_f


@dataclass(frozen=True)
class Body:
    """平面モデルの体。節の長さ・質量・どの関節が歩容で動くか。"""

    name: str
    seg_len_m: np.ndarray        # 尾 → 頭
    seg_mass_kg: np.ndarray
    gait_joint: list[int]        # 各関節（節の境目）が歩容の何番目の胴体 yaw か（-1 = 固定）
    joint_names: list[str]       # 歩容で動く胴体 yaw の名前（尾 → 頭）
    turn_profile: str

    @property
    def length_m(self) -> float:
        return float(self.seg_len_m.sum())


def body_from_cfg(cfg: dict[str, Any], name: str = "") -> Body:
    """config（overlay 込み）から平面モデルの体を作る。pitch と、首より頭側の yaw は 0° で固定。"""
    joints = cfg["joints"]
    xs = [float(cfg["body"]["tail_x_mm"])] + [float(j["x_mm"]) for j in joints] + [float(cfg["body"]["head_tip_x_mm"])]
    seg_len = np.diff(np.array(xs)) / 1000.0
    masses = np.array(_masses(cfg, len(seg_len)))
    names = body_joint_names(cfg)
    gait_joint = [names.index(j["name"]) if j["name"] in names else -1 for j in joints]
    return Body(name or "cfg", seg_len, masses, gait_joint, names, str(cfg["gait"]["turn_profile"]))


@dataclass
class CycleResult:
    source: str
    body: str
    amplitude_deg: float
    waves: float
    freq_hz: float
    gamma_deg: float
    speed_mm_s: float            # 1 周期の重心の移動（初めの向きに沿った成分）× 周波数
    lateral_mm_s: float          # 横へのずれ × 周波数
    heading_deg_per_cycle: float  # 1 周期で向きが変わる量（直進指令なら「ずれ」）
    turn_radius_mm: float | None
    energy_j_per_m: float | None  # 正の機械仕事 / 移動距離
    peak_torque_nm: float
    peak_joint_speed_dps: float
    slip: float                  # 接地点の速さのうち横向き成分の割合（荷重重み付き平均）
    converged: bool
    notes: list[str] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


class PlanarSnake:
    """1 つの体 × 1 つの摩擦で、歩容 1 周期を解く。"""

    def __init__(self, body: Body, friction: Friction, points_per_segment: int = 4) -> None:
        self.body = body
        self.fr = friction
        k = points_per_segment
        # 接地点: 各節を k 等分した中点。荷重は節の重さを均等に配る（平らな床にすべて接地）
        self.seg_of_point = np.repeat(np.arange(len(body.seg_len_m)), k)
        frac = (np.arange(k) + 0.5) / k
        self.s_of_point = np.concatenate([frac * L for L in body.seg_len_m])
        self.normal = np.repeat(body.seg_mass_kg / k, k) * G
        scale = np.ones(len(self.normal)) if friction.point_scale is None else np.asarray(friction.point_scale)
        if len(scale) != len(self.normal):
            raise ValueError(f"point_scale は {len(self.normal)} 個必要（{len(scale)} 個）")
        self.scale = scale

    # ---- 形（体の座標系 = 尾の節。原点は尾端） ----------------------------------------------------
    def shape(self, q_body_deg: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """胴体 yaw の角 → 接地点の位置 (P,2)、節の向き (P,)、関節の位置 (J,2)。"""
        seg_theta = np.zeros(len(self.body.seg_len_m))
        starts = np.zeros((len(self.body.seg_len_m), 2))
        th, pos = 0.0, np.zeros(2)
        joints = []
        for k, L in enumerate(self.body.seg_len_m):
            if k > 0:
                gj = self.body.gait_joint[k - 1]
                th += math.radians(q_body_deg[gj]) if gj >= 0 else 0.0
                joints.append(pos.copy())
            seg_theta[k] = th
            starts[k] = pos
            pos = pos + L * np.array([math.cos(th), math.sin(th)])
        th_p = seg_theta[self.seg_of_point]
        p = starts[self.seg_of_point] + self.s_of_point[:, None] * np.stack([np.cos(th_p), np.sin(th_p)], 1)
        return p, th_p, np.array(joints)

    # ---- 摩擦力 ------------------------------------------------------------------------------------
    def forces(self, twist: np.ndarray, p: np.ndarray, th: np.ndarray, pdot: np.ndarray) -> np.ndarray:
        u, v, w = twist
        vel = np.stack([u - w * p[:, 1], v + w * p[:, 0]], 1) + pdot
        t = np.stack([np.cos(th), np.sin(th)], 1)
        n = np.stack([-np.sin(th), np.cos(th)], 1)
        vt = (vel * t).sum(1)
        vn = (vel * n).sum(1)
        speed = np.hypot(vel[:, 0], vel[:, 1])
        rho = np.sqrt(speed ** 2 + self.fr.eps_m_s ** 2)
        # 前 / 後ろ・左 / 右の切り替えは ε の幅でなめらかにする（段差があると、滑りが 0 の点で釣り合いの解が無くなる）
        st = 0.5 * (1.0 + np.tanh(vt / self.fr.eps_m_s))
        sn = 0.5 * (1.0 + np.tanh(vn / self.fr.eps_m_s))
        mu_t = self.fr.mu_b + (self.fr.mu_f - self.fr.mu_b) * st
        mu_n = self.fr.mu_right + (self.fr.mu_left - self.fr.mu_right) * sn
        if self.fr.law == "decoupled":
            c = -(self.normal * self.scale / rho)
            return c[:, None] * ((mu_t * vt)[:, None] * t + (mu_n * vn)[:, None] * n)
        if self.fr.law == "ellipse":
            # 最大散逸の楕円形: F = −N M² v / |M v|（M = diag(μ_t, μ_n)）
            mv = np.sqrt((mu_t * vt) ** 2 + (mu_n * vn) ** 2 + (0.5 * (mu_t + mu_n) * self.fr.eps_m_s) ** 2)
            c = -(self.normal * self.scale / mv)
            return c[:, None] * ((mu_t ** 2 * vt)[:, None] * t + (mu_n ** 2 * vn)[:, None] * n)
        raise ValueError(f"未知の摩擦法則: {self.fr.law}（{LAWS}）")

    def _residual(self, twist: np.ndarray, p: np.ndarray, th: np.ndarray, pdot: np.ndarray,
                  L: float) -> np.ndarray:
        f = self.forces(twist, p, th, pdot)
        m = (p[:, 0] * f[:, 1] - p[:, 1] * f[:, 0]).sum()
        return np.array([f[:, 0].sum(), f[:, 1].sum(), m / L])

    def viscous_twist(self, p: np.ndarray, pdot: np.ndarray) -> np.ndarray:
        """初期値: 等方の粘性抵抗（F = −N·V）なら釣り合いは線形で解ける。"""
        w = self.normal
        # V_i = (u − ω y_i, v + ω x_i) + ṗ_i、Σ w_i V_i = 0 と Σ w_i (x_i V_iy − y_i V_ix) = 0
        A = np.array([[w.sum(), 0.0, -(w * p[:, 1]).sum()],
                      [0.0, w.sum(), (w * p[:, 0]).sum()],
                      [-(w * p[:, 1]).sum(), (w * p[:, 0]).sum(), (w * (p ** 2).sum(1)).sum()]])
        rhs = -np.array([(w * pdot[:, 0]).sum(), (w * pdot[:, 1]).sum(),
                         (w * (p[:, 0] * pdot[:, 1] - p[:, 1] * pdot[:, 0])).sum()])
        return np.linalg.solve(A, rhs)

    def solve_twist(self, p: np.ndarray, th: np.ndarray, pdot: np.ndarray, guess: np.ndarray) -> tuple[np.ndarray, bool]:
        """合力 0・モーメント 0 になる体の並進・回転（信頼領域つき Levenberg–Marquardt、数値ヤコビアン）。

        摩擦は速さで飽和するので、解から遠いところでは残差が平らになり、解から離れても残差がわずかに減り続ける。
        体の速さは形の変化の速さ（max |ṗ|）の数倍を超えないので、解の範囲そのものをそこで抑える。
        前の時刻の解から始めて失敗したら、粘性近似の解から始め直す。
        """
        for start in (guess, self.viscous_twist(p, pdot)):
            x, ok = self._lm(p, th, pdot, start)
            if ok:
                return x, True
        return x, False

    def _lm(self, p: np.ndarray, th: np.ndarray, pdot: np.ndarray, x0: np.ndarray) -> tuple[np.ndarray, bool]:
        L = self.body.length_m
        vmax = max(float(np.abs(pdot).max()), self.fr.eps_m_s)
        scale = np.array([1.0, 1.0, L])                        # 並進 [m/s] と 回転×L をそろえる
        bound = 3.0 * vmax

        def clip(x: np.ndarray) -> np.ndarray:
            n = float(np.linalg.norm(x * scale))
            return x * (bound / n) if n > bound else x

        x = clip(np.asarray(x0, dtype=float).copy())
        tol = 1e-9 * float(self.normal.sum())
        r = self._residual(x, p, th, pdot, L)
        radius = vmax
        lam = 1e-6
        for _ in range(100):
            if np.max(np.abs(r)) < tol:
                return x, True
            J = np.empty((3, 3))
            for j in range(3):
                dx = np.zeros(3)
                dx[j] = 1e-6 * vmax / scale[j]
                J[:, j] = (self._residual(x + dx, p, th, pdot, L) - r) / dx[j]
            Js = J / scale
            A = Js.T @ Js
            g = Js.T @ r
            step_s = np.linalg.solve(A + lam * (np.trace(A) / 3.0 + 1e-30) * np.eye(3), -g)
            n = float(np.linalg.norm(step_s))
            if n > radius:
                step_s *= radius / n
            xn = clip(x + step_s / scale)
            rn = self._residual(xn, p, th, pdot, L)
            if np.linalg.norm(rn) < np.linalg.norm(r):
                x, r = xn, rn
                lam = max(lam * 0.3, 1e-9)
                radius = min(radius * 2.0, bound)
            else:
                lam *= 10.0
                radius *= 0.5
                if radius < 1e-9 * vmax:
                    break
        return x, bool(np.max(np.abs(r)) < 1e-6 * float(self.normal.sum()))

    # ---- 1 周期 ------------------------------------------------------------------------------------
    def run_cycle(self, amplitude_deg: float, waves: float, freq_hz: float = 0.5, gamma_deg: float = 0.0,
                  steps: int = 48, cycles: int = 1, record: dict[str, list] | None = None) -> CycleResult:
        """1 周期（cycles 周期）を解く。record に dict を渡すと、関節トルク [N·m]・関節角速度 [deg/s] の時系列を入れる。"""
        n_body = len(self.body.joint_names)
        gait = GaitParams(amplitude_deg, 360.0 * waves / n_body, freq_hz)
        period = 1.0 / freq_hz
        dt = period / steps
        h = dt * 1e-3

        def q_at(t: float) -> np.ndarray:
            a = angles_at_phase(gait, 2.0 * math.pi * freq_hz * t, self.body.joint_names, gamma_deg,
                                self.body.turn_profile)
            return np.array([a[nm] for nm in self.body.joint_names])

        com_w = self.normal / self.normal.sum()
        pose = np.zeros(3)                                      # 世界での尾の位置と向き
        twist = np.zeros(3)
        ok = True
        work = 0.0
        peak_tau = 0.0
        peak_qd = 0.0
        slip_num = slip_den = 0.0

        def com_world(pose: np.ndarray, p: np.ndarray) -> np.ndarray:
            c, s = math.cos(pose[2]), math.sin(pose[2])
            pc = (p * com_w[:, None]).sum(0)
            return pose[:2] + np.array([c * pc[0] - s * pc[1], s * pc[0] + c * pc[1]])

        p0, th0, _ = self.shape(q_at(0.0))
        c_start = com_world(pose, p0)
        heading0 = pose[2] + math.atan2((np.sin(th0) * com_w).sum(), (np.cos(th0) * com_w).sum())
        total_steps = steps * cycles
        for k in range(total_steps):
            t = (k + 0.5) * dt                                  # 中点で解く
            q = q_at(t)
            p, th, jp = self.shape(q)
            p2, _, _ = self.shape(q_at(t + h))
            p1, _, _ = self.shape(q_at(t - h))
            pdot = (p2 - p1) / (2 * h)
            qdot = (q_at(t + h) - q_at(t - h)) / (2 * h)       # deg/s
            twist, conv = self.solve_twist(p, th, pdot, twist)
            ok &= conv
            f = self.forces(twist, p, th, pdot)
            # 関節トルク: 関節より頭側の摩擦力のモーメント（準静的なので尾側から見ても同じ大きさ）
            taus = np.zeros(n_body)
            for j_idx, gj in enumerate(self.body.gait_joint):
                if gj < 0:
                    continue
                distal = self.seg_of_point > j_idx
                r = p[distal] - jp[j_idx]
                taus[gj] = (r[:, 0] * f[distal, 1] - r[:, 1] * f[distal, 0]).sum()
            # 関節が頭側の節へ出すトルクは −τ_distal。正の仕事だけを数える
            power = -(taus * np.radians(qdot))
            work += float(np.clip(power, 0, None).sum()) * dt
            peak_tau = max(peak_tau, float(np.abs(taus).max()))
            if record is not None:
                record.setdefault("t_s", []).append(t)
                record.setdefault("torque_nm", []).append(taus.copy())
                record.setdefault("qdot_dps", []).append(qdot.copy())
            peak_qd = max(peak_qd, float(np.abs(qdot).max()))
            vel = np.stack([twist[0] - twist[2] * p[:, 1], twist[1] + twist[2] * p[:, 0]], 1) + pdot
            vn = np.abs(vel[:, 0] * -np.sin(th) + vel[:, 1] * np.cos(th))
            sp = np.hypot(vel[:, 0], vel[:, 1])
            slip_num += float((self.normal * vn).sum())
            slip_den += float((self.normal * sp).sum())
            # 姿勢の更新（中点の向きで並進を回す）
            phi_mid = pose[2] + 0.5 * twist[2] * dt
            c, s = math.cos(phi_mid), math.sin(phi_mid)
            pose = pose + np.array([(c * twist[0] - s * twist[1]) * dt, (s * twist[0] + c * twist[1]) * dt,
                                    twist[2] * dt])
        pe, _, _ = self.shape(q_at(total_steps * dt))
        d = com_world(pose, pe) - c_start
        hdir = np.array([math.cos(heading0), math.sin(heading0)])
        ndir = np.array([-hdir[1], hdir[0]])
        dphi = pose[2] / cycles
        dist = float(np.hypot(*d))
        radius = None
        if gamma_deg and abs(dphi) > 1e-3:
            chord = dist / cycles
            radius = chord / (2.0 * abs(math.sin(dphi / 2.0))) * 1000.0
        return CycleResult(
            source=SOURCE, body=self.body.name, amplitude_deg=amplitude_deg, waves=waves, freq_hz=freq_hz,
            gamma_deg=gamma_deg,
            speed_mm_s=float(d @ hdir) * 1000.0 * freq_hz / cycles,
            lateral_mm_s=float(d @ ndir) * 1000.0 * freq_hz / cycles,
            heading_deg_per_cycle=math.degrees(dphi),
            turn_radius_mm=radius,
            energy_j_per_m=(work / dist) if dist > 1e-3 else None,
            peak_torque_nm=peak_tau, peak_joint_speed_dps=peak_qd,
            slip=slip_num / slip_den if slip_den else 0.0,
            converged=ok,
        )
