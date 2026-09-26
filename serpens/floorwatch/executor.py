"""フェーズ 5 の縦一本（**模擬**）: Task（inspect_point）→ 移動 → 撮影 → 判定 → floor_finding → task_status。

`FloorWatchExecutor` は Endpoint の Executor 契約を満たし、SimSession（KINEMATIC_SIM の世界）と自己位置の推定器
（AprilTag の合成観測 + IMU の模擬 + 歩容の位相）と Floor Watch の画像処理（合成画像）を繋ぐ。
  - 開始できない理由 = 自己位置の σ と局所センサーの健全性（localization.start_blockers）+ 位置の確認待ち（session）
  - 停止は Endpoint が先に判定し、ここでは mission を捨てて歩容を止める。安全停止そのものは StopSupervisor / 機体側
  - floor_finding の位置 = 推定した自己位置 + 頭からの相対位置。σ は推定器の値。写真は候補の切り抜きだけ保存
実機の値は 1 つも無い（HARDWARE_VERIFIED = 0）。実機では capture / observe / imu を差し替える。
"""
from __future__ import annotations

import math
import uuid
from pathlib import Path
from typing import Any

import numpy as np

from serpens.api.endpoint import Endpoint, Executor
from serpens.floorwatch.dataset import imwrite
from serpens.floorwatch.detect import Candidate, detect
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.mission import IdleHold, InspectMission
from serpens.floorwatch.risk import assess
from serpens.floorwatch.scene import FloorScene, HomeFrame, camera_to_home, capture_synthetic
from serpens.floorwatch.synthetic import Renderer, default_lighting
from serpens.localization.estimator import PoseEstimator, start_blockers, wrap
from serpens.localization.sim_observer import SimTagObserver
from serpens.localization.tag_map import TagMap
from serpens.motion.poses import HEAD_YAW
from serpens.safety import DriveState

SAFETY_MODE = {DriveState.RUN: "DRIVING", DriveState.HOLD: "ARMED_HOLD", DriveState.DISABLED: "TORQUE_DISABLED",
               DriveState.EMERGENCY: "EMERGENCY_LATCHED"}


class FloorWatchExecutor(Executor):
    def __init__(self, cfg: dict[str, Any], session: Any, scene: FloorScene, tag_map: TagMap,
                 rng: np.random.Generator, findings_dir: Path | None = None) -> None:
        self.cfg, self.session, self.scene, self.tag_map, self.rng = cfg, session, scene, tag_map, rng
        self.endpoint: Endpoint | None = None
        self.est = PoseEstimator(cfg, tag_map)
        self.obs = SimTagObserver(cfg, tag_map, rng)
        self.frame = HomeFrame(cfg, session.brain.ctrl.center())
        self.cam, self.plane = Camera.from_cfg(cfg), LightPlane.design(cfg)
        self.renderer = Renderer(self.cam, self.plane, default_lighting(cfg))
        sim = cfg["localization"]["sim"]
        self.imu_offset = float(sim["imu_offset_rad"])
        self.advance_m = float(cfg["behavior"]["controller"]["advance_per_cycle_mm"]) / 1000.0
        self.findings_dir = findings_dir if findings_dir is not None else Path(cfg["floor_watch"]["findings_dir"])
        self.mission: InspectMission | None = None
        self.idle = IdleHold(session.brain.loco)
        session.mission = self.idle                      # Task が無いあいだは止まっている（展示の行動へ戻らない）
        self.task: dict[str, Any] | None = None
        self.findings: list[dict[str, Any]] = []
        self._last_phase: float | None = None
        self._last_safety: tuple[str, bool] | None = None

    def attach(self, endpoint: Endpoint) -> None:
        self.endpoint = endpoint
        self._publish_safety()

    def _publish_safety(self) -> None:
        """機体の停止状態（StopSupervisor）を safety_state に写す。変化したときだけ（周期送信は Endpoint.tick）。"""
        sup = self.session.stop
        mode = "EMERGENCY_LATCHED" if sup.latched else SAFETY_MODE[sup.state]
        key = (mode, bool(sup.latched))
        if self.endpoint is not None and key != self._last_safety:
            self._last_safety = key
            self.endpoint.safety_state(mode, "NONE" if mode == "DRIVING" else "OPERATOR", latched=bool(sup.latched))

    # ---- Executor 契約 --------------------------------------------------------------
    def blockers(self) -> list[str]:
        t = self.session.t
        held = [] if self.session.stop.moving_allowed else ["機体が停止中（人の開始操作が要る。MQTT からは再開できない）"]
        return held + self.session.pose_blockers() + start_blockers(self.cfg, self.est.estimate(t), self.est.health(t))

    def start(self, task: dict[str, Any]) -> None:
        self.task = task
        if task["task"] != "inspect_point":
            self._finish("failed", f"{task['task']} はこの段階では未実装（模擬の縦一本は inspect_point だけ）")
            return
        tgt = self.frame.to_world(np.array([float(task["target"]["x_m"]), float(task["target"]["y_m"])]))
        # 目標 = カメラの視野中心をその地点に置く。機体（首マーカ）はその手前 = 首→頭先端 + 視野中心の距離だけ手前に止まる
        neck, tip = self.session.world.marker_xy("neck"), self.session.world.head_tip()[:2]
        d = tgt - neck
        direction = d / max(float(np.linalg.norm(d)), 1e-9)
        offset = float(np.linalg.norm(tip - neck)) + float(self.cam.floor_point(self.cam.cx, self.cam.cy)[1])
        pts = self.session.world.world_points()
        self.mission = InspectMission(self.cfg, self.session.brain.loco, tgt - direction * offset, self._capture, self._judge,
                                      self.session.t, aim=self._aim, measure=lambda: self._offset_to(tgt),
                                      head_link_mm=float(np.linalg.norm(pts[-1][:2] - pts[-2][:2])))
        self.session.mission = self.mission

    def stop(self, reason: str) -> None:
        """Task の stop: mission を捨て、機体を停止（HOLD）させる。再開は人の開始操作（session.request_start）だけ。"""
        if self.mission is not None:
            self.mission.abort(reason)
        self.session.brain.loco.stop(reason)
        self.session.request_stop(reason, source="Home AI")
        self.session.mission, self.mission = self.idle, None
        self.task = None

    # ---- 1 周期 -------------------------------------------------------------------
    def tick(self) -> None:
        self.session.step()
        t = self.session.t
        self._localize(t)
        if self.mission is not None and not self.mission.active and self.task is not None:
            m = self.mission
            if m.phase == "DONE":
                for c in m.result:
                    self._report(c, t)
                self._finish("done", f"候補 {len(m.result)} 件")
            elif m.phase == "FAILED":
                self._finish("failed", m.reason)
            self.session.mission, self.mission = self.idle, None
        self._publish_safety()
        if self.endpoint is not None:
            self.endpoint.tick()

    def _finish(self, status: str, reason: str) -> None:
        self.task = None
        if self.endpoint is not None:
            self.endpoint.task_finished(status, reason)

    def _true_pose_home(self) -> tuple[float, float, float]:
        x, y, th = self.session.world.snake_pose()
        hx, hy = self.frame.to_home(np.array([x, y]))
        return float(hx), float(hy), float(th)

    def _localize(self, t: float) -> None:
        """歩容の位相 → 前進量、IMU の模擬、タグの合成観測 → 推定器。"""
        phase = self.session.anim.gait.phase_rad
        dphase = 0.0 if self._last_phase is None else phase - self._last_phase
        self._last_phase = phase
        x, y, yaw = self._true_pose_home()
        imu = yaw + self.imu_offset + self.rng.normal(0, float(self.cfg["localization"]["imu"]["yaw_sigma_rad"]))
        d = None if self.est.last_imu_yaw is None else wrap(imu - self.est.last_imu_yaw)
        self.est.predict(t, self.advance_m * dphase / (2 * math.pi), d if d is not None else 0.0, d is not None)
        self.est.imu_yaw(t, imu)
        self.est.correct(self.obs.observe(t, x, y, yaw, blind=self.scene.blind(x, y)))

    # ---- 撮影と判定（模擬） ------------------------------------------------------------
    def _head(self) -> tuple[np.ndarray, float]:
        """真の頭先端の home 座標と頭リンク（カメラの光軸）の向き（合成画像を作るため。推定ではない）。"""
        pts = self.session.world.world_points()
        d = pts[-1][:2] - pts[-2][:2]
        return self.frame.to_home(pts[-1][:2]), math.atan2(float(d[1]), float(d[0]))

    def _capture(self) -> dict[str, np.ndarray]:
        head_xy, head_yaw = self._head()
        return capture_synthetic(self.renderer, self.scene, head_xy, head_yaw, float(self.cfg["floor_watch"]["mission"]["reach_mm"]))

    def _offset_to(self, target_mm: np.ndarray) -> tuple[float, float]:
        """頭先端から地点までの (頭リンク方向の前方, 左) [mm]（模擬では真値。実機では推定姿勢 + 地点から）。"""
        pts = self.session.world.world_points()
        tip, prev = pts[-1][:2], pts[-2][:2]
        h = tip - prev
        h = h / max(float(np.linalg.norm(h)), 1e-9)
        d = target_mm - tip
        return float(d @ h), float(h[0] * d[1] - h[1] * d[0])

    def _aim(self, t: float, delta_deg: float) -> None:
        """頭ヨー（J8 相当）を増分だけ回して線を候補へ向ける（正 = 左）。"""
        current = float(self.session.anim.base.get(HEAD_YAW, 0.0))
        self.session.brain.expr.look_at(t, current + delta_deg, force=True)

    def _judge(self, frames: dict[str, np.ndarray]) -> list[Candidate]:
        cands, _tr, _fg = detect(frames, self.cam, self.plane, self.cfg)
        self._frames = frames
        return [c for c in cands if c.is_object]

    def _report(self, c: Candidate, t: float) -> None:
        """候補 1 つ → floor_finding。位置 = 推定した自己位置 + 頭からの相対位置（真値ではない）。"""
        est = self.est.estimate(t)
        head_xy_true, head_yaw_true = self._head()
        obj_true = camera_to_home(c.floor_xy_mm, head_xy_true, head_yaw_true)
        x, y, yaw = self._true_pose_home()
        rel = obj_true - np.array([x, y])                                   # 真の機体からの相対（機体は自分の相対位置は知っている）
        rc, rs = math.cos(est.yaw_rad - yaw), math.sin(est.yaw_rad - yaw)   # 推定した向きで置き直す
        pos = np.array([est.x_m, est.y_m]) + np.array([rc * rel[0] - rs * rel[1], rs * rel[0] + rc * rel[1]])
        fid = f"f-{uuid.uuid4().hex[:12]}"
        photos = self._save_crops(fid, c)
        kinds = c.kinds or [{"kind": "unknown", "confidence": 0.5}]
        risk = assess(kinds, c.diameter_mm, c.height_mm, c.height_reason, None, True, self.cfg)
        finding = {"finding_id": fid, "t_ms": int(t * 1000), "frame_id": self.tag_map.frame_id,
                   "map_version": self.tag_map.map_version,
                   "pose": {**est.finding_pose(), "x_m": round(float(pos[0]), 4), "y_m": round(float(pos[1]), 4)},
                   "photos": photos, "candidates": kinds[:5],
                   "size": {"diameter_mm": round(c.diameter_mm, 2), "sigma_mm": round(c.diameter_sigma_mm, 2),
                            "height_mm": None if c.height_mm is None else round(c.height_mm, 2),
                            "height_sigma_mm": None if c.height_sigma_mm is None else round(c.height_sigma_mm, 2),
                            "height_reason": c.height_reason, "method": "line_light"},
                   "risk": risk, "state": "candidate", "state_by": "robot"}
        if self.task is not None:
            finding["task_id"] = self.task["id"]
        self.findings.append(finding)
        if self.endpoint is not None:
            self.endpoint.floor_finding(finding)

    def _save_crops(self, fid: str, c: Candidate) -> list[dict[str, Any]]:
        """候補の切り抜きだけを保存する（原画像は家の外へ出さない）。"""
        x, y, w, h = c.bbox_px
        pad = int(self.cfg["floor_watch"]["mission"]["crop_pad_px"])
        d = self.findings_dir / fid
        d.mkdir(parents=True, exist_ok=True)
        out = []
        for kind in ("normal", "raking", "line"):
            img = self._frames[kind]
            crop = img[max(0, y - pad):y + h + pad, max(0, x - pad):x + w + pad]
            path = d / f"{kind}.png"
            imwrite(path, crop)
            out.append({"kind": kind, "crop_path": str(path).replace("\\", "/"), "w_px": int(crop.shape[1]),
                        "h_px": int(crop.shape[0]), "t_ms": int(self.session.t * 1000)})
        return out
