"""`config/robot.yaml` から MuJoCo の MJCF（9軸の簡略モデル）を作る。

**外装は再現しない。** 入れるのは、動きに効くものだけ:

  9関節 / リンク長 / 質量 / 関節軸 / 可動域（software_operational_limit）/ 腹の接触

値の確かさは3段で持つ（`VALUE_SOURCES`）:

  CONFIRMED       … 実測または一次資料で確認した値（**現在 0 件**）
  REFERENCE_ONLY  … 設計値・CAD 資料・質量収支（実測ではない）
  UNKNOWN         … 決まっていない（摩擦・慣性の分布・外装の影響）

MJCF を手で書くと config とズレるので、**必ずここから生成する**
（`tests/test_mujoco_model.py` が config との一致を検査する）。
"""
from __future__ import annotations

import hashlib
from dataclasses import dataclass
from typing import Any

from simulation.mujoco.belly import BellyProfile, profile

MODEL_VERSION = "serpens-mjcf-0.1"      # 生成器の版（記録に残す）
MM = 0.001                              # config は mm、MuJoCo は m

# この値がどこから来たか（**実測は1つも無い**）
VALUE_SOURCES = {
    "link_length": "REFERENCE_ONLY",     # config の joints[].x_mm（CAD 由来の設計値）
    "joint_range": "REFERENCE_ONLY",     # software_operational_limit（CAD_PROVISIONAL）
    "mass": "REFERENCE_ONLY",            # mass_budget_g（収支であって計量値ではない）
    "inertia": "UNKNOWN",                # 形状から自動計算させる（分布は未確認）
    "friction": "UNKNOWN",               # belly プロファイルは仮値
    "torque_limit": "REFERENCE_ONLY",    # STS3215-C044 の REFERENCE 値から
    "servo_dynamics": "UNKNOWN",         # 応答（kp/damping）は未同定
}

# pitch は **+ で頭側が上がる**（serpens/motion/kinematics.py と同じ規約）。体は +x へ伸びるので y 軸まわり負回転。
# 2026-09-29 まで "0 1 0" で逆向きだった（+ で頭が床へ潜る。EX-1 の home J7=+8° は頭を床へ押していた）
AXIS_VECTOR = {"yaw": "0 0 1", "pitch": "0 -1 0", "roll": "1 0 0"}


@dataclass(frozen=True)
class ModelSpec:
    """生成した MJCF と、その元になった値。"""

    xml: str
    belly: str
    joint_names: list[str]
    link_lengths_m: list[float]
    total_mass_kg: float
    version: str = MODEL_VERSION

    @property
    def digest(self) -> str:
        """同じ run を再現できるよう、モデルの指紋を記録に残す。"""
        return hashlib.sha256(self.xml.encode("utf-8")).hexdigest()[:16]


def _masses(cfg: dict[str, Any], n_segments: int) -> list[float]:
    """各セグメントの質量 [kg]。**質量収支からの配分で、計量値ではない。**

    合計は `mass_budget_g` と一致させる（サーボ 9 個・フレーム 8 個は収支どおりの個数で置く）。
    どのセグメントに何が載るかは設計が決まっていないので、以下は**仮の配分**:
      サーボ … その関節の先のセグメント（seg1〜seg9）
      フレーム … 胴体側のセグメント（seg0〜seg7）
      受動輪・外皮・配線 … 全セグメントへ均等
      頭部一式 … 最後のセグメント
      尾の積荷（tail_payload。電池など、任意）… 最初のセグメント
    """
    b = cfg["mass_budget_g"]
    servo = float(b["servo_each"]) / 1000.0
    frame = float(b["segment_frame_each"]) / 1000.0
    extra = (float(b["passive_wheels_total"]) + float(b["skin_and_wiring"])) / 1000.0
    out = [extra / n_segments] * n_segments
    for k in range(min(int(b["servo_count"]), n_segments - 1)):
        out[k + 1] += servo                          # 関節の先へ載せる
    for k in range(min(int(b["segment_frame_count"]), n_segments)):
        out[k] += frame
    out[-1] += float(b["head_total"]) / 1000.0
    out[0] += float(b.get("tail_payload", 0.0)) / 1000.0
    return out


def build_mjcf(cfg: dict[str, Any], belly: str = "WHEEL", timestep: float = 0.002,
               kp_scale: float = 1.0) -> ModelSpec:
    """config から MJCF を組み立てる。`kp_scale` は位置サーボのゲイン（**未同定**）の倍率。"""
    prof = profile(belly)
    joints = cfg["joints"]
    radius = float(cfg["body"]["diameter_mm"]) / 2.0 * MM
    tail_x = float(cfg["body"]["tail_x_mm"]) * MM
    head_x = float(cfg["body"]["head_tip_x_mm"]) * MM
    xs = [tail_x] + [float(j["x_mm"]) * MM for j in joints] + [head_x]
    lengths = [xs[i + 1] - xs[i] for i in range(len(xs) - 1)]
    masses = _masses(cfg, len(lengths))
    torque = float(cfg["safety_limits"]["torque"]["software_torque_limit_nm"])
    # 太い胴（FW 系 overlay）では、近似カプセルどうしが中立でも重なる（実外装は節ごとの卵型で重ならない）。
    # その場合だけ自己接触を切る（床との接触は下の <pair> で明示するので残る）。既定は従来どおり有効
    self_collide = bool(cfg.get("sim", {}).get("mujoco_self_collision", True))
    no_self = "" if self_collide else ' contype="0" conaffinity="0"'

    body_xml, closing = [], []
    for k, seg_len in enumerate(lengths):
        indent = "      " + "  " * k
        if k == 0:
            body_xml.append(f'{indent}<body name="seg0" pos="0 0 {radius:.4f}">')
            body_xml.append(f'{indent}  <freejoint name="root"/>')
        else:
            j = joints[k - 1]
            lo = float(j["min_deg"]) * 3.141592653589793 / 180.0
            hi = float(j["max_deg"]) * 3.141592653589793 / 180.0
            body_xml.append(f'{indent}<body name="seg{k}" pos="{lengths[k - 1]:.4f} 0 0">')
            body_xml.append(
                f'{indent}  <joint name="{j["name"]}" type="hinge" axis="{AXIS_VECTOR[j["axis"]]}" '
                f'range="{lo:.4f} {hi:.4f}"/>')
        body_xml.append(
            f'{indent}  <geom name="seg{k}_geom" type="capsule" '
            f'fromto="0 0 0 {seg_len:.4f} 0 0" size="{radius:.4f}" mass="{masses[k]:.4f}"{no_self}/>')
        closing.append(f"{indent}</body>")
    body_xml += list(reversed(closing))

    names = [j["name"] for j in joints]
    actuators = "\n".join(
        f'    <position name="{n}_pos" joint="{n}" kp="{_kp(cfg, n) * kp_scale:.2f}" '
        f'forcerange="{-torque:.3f} {torque:.3f}"/>' for n in names)
    sensors = "\n".join(
        f'    <jointpos name="{n}_q" joint="{n}"/>\n'
        f'    <jointvel name="{n}_dq" joint="{n}"/>\n'
        f'    <jointactuatorfrc name="{n}_trq" joint="{n}"/>' for n in names)
    pairs = "\n".join(
        f'    <pair geom1="floor" geom2="seg{k}_geom" '
        f'friction="{" ".join(f"{v:g}" for v in prof.friction)}"/>' for k in range(len(lengths)))

    xml = f"""<mujoco model="serpens_ex1">
  <!-- 生成物。手で編集しない（simulation/mujoco/model.py が config/robot.yaml から作る） -->
  <!-- belly={prof.name} 摩擦は**未実測**: 前後 {prof.slide_along} / 横 {prof.slide_across} -->
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{timestep}" gravity="0 0 -9.81" integrator="implicitfast"/>
  <default>
    <joint damping="0.05" armature="0.002" limited="true"/>
    <geom rgba="0.35 0.82 0.47 1" condim="6"/>
  </default>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.77 0.70 0.55 1" condim="6"/>
{chr(10).join(body_xml)}
  </worldbody>
  <contact>
{pairs}
  </contact>
  <actuator>
{actuators}
  </actuator>
  <sensor>
{sensors}
  </sensor>
</mujoco>
"""
    return ModelSpec(xml=xml, belly=prof.name, joint_names=names, link_lengths_m=lengths,
                     total_mass_kg=sum(masses))


def _kp(cfg: dict[str, Any], name: str) -> float:
    """位置サーボのゲイン。**未同定（UNKNOWN）**なので、トルク上限から控えめに決める。

    実機の応答は `mock_servo.position_tau_s` の仮値しか無い。実機が来たらここを同定する。
    """
    torque = float(cfg["safety_limits"]["torque"]["software_torque_limit_nm"])
    return torque / 0.12          # 約 7° の偏差で上限トルクに達するゲイン


def write_mjcf(cfg: dict[str, Any], path: str, belly: str = "WHEEL") -> ModelSpec:
    """MJCF をファイルへ書き出す（人が MuJoCo viewer で開けるように）。"""
    spec = build_mjcf(cfg, belly)
    with open(path, "w", encoding="utf-8") as fp:
        fp.write(spec.xml)
    return spec


def belly_profile(name: str) -> BellyProfile:
    return profile(name)
