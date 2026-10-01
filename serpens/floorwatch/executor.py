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
from typing import Any, Callable

import numpy as np

from serpens.api.endpoint import Endpoint, Executor
from serpens.floorwatch.dataset import imwrite
from serpens.floorwatch.csar import NEAR, ChildProximity
from serpens.floorwatch.detect import Candidate, detect
from serpens.floorwatch.geometry import Camera, LightPlane
from serpens.floorwatch.highlight import DONE as HL_DONE
from serpens.floorwatch.highlight import HighlightMission
from serpens.floorwatch.hood import STUCK, HoodMonitor
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
        self.csar = ChildProximity.from_cfg(cfg)             # 子どもが近いか（CSAR）。Serpens 側で決める
        self._reported = False
        # start() の中で即座に終える Task は、次の周期で終える。start() の中で task_finished を呼ぶと、Endpoint がその後で
        # 「accepted」を出し、Home AI には受理のまま残っていた（最後の状態が accepted になる）
        self._pending_finish: tuple[str, str] | None = None
        self._last_phase: float | None = None
        self._last_safety: tuple[str, bool] | None = None
        self._route: list[tuple[float, float]] | None = None   # patrol_route: 各点で inspect（SE-E7）
        self._route_i = 0
        self._route_log: list[tuple[str, str, int]] = []
        self.highlight: HighlightMission | None = None          # highlight_point: 物 → 人 → 物（頭ヨーだけ）
        self.hood: HoodMonitor | None = None                  # フードの端の 1 ビット（R-023）。既定は未接続（フードの機構・頭のファームがまだ無い）
        self._hood_read: Callable[[float], tuple[bool | None, bool | None]] | None = None

    def attach_hood(self, monitor: HoodMonitor, read: Callable[[float], tuple[bool | None, bool | None]]) -> None:
        """フードの端のビット（下, 上）の読み出しと、0.6 s のタイムアウトの監視を繋ぐ。**模擬の MockHood.read か、将来の頭 XIAO のビット。**"""
        self.hood, self._hood_read = monitor, read

    def _check_hood(self, t: float) -> None:
        """端のビットを 1 回読む。固着（STUCK）ならラッチして、いまの mission を止め、Task を failed にする（前進停止 + 記録）。人の確認（acknowledge）まで新しい Task は受けない。"""
        if self.hood is None:
            return
        was = self.hood.state
        bits = self._hood_read(t) if self._hood_read is not None else (None, None)
        self.hood.update(bits[0], bits[1], t)
        if self.hood.forward_inhibit and was != STUCK:
            reason = f"フードの端のビットが立たない: {self.hood.stuck_reason}"
            self.session.brain.loco.stop(reason)
            if self.mission is not None and self.mission.active:
                self.mission.abort(reason)
            self.session.mission, self.mission = self.idle, None          # Task が無いときと同じ止まり方（IdleHold が毎周期止める）
            if self.task is not None:
                self._pending_finish = ("failed", reason)

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
        if self.hood is not None and self.hood.forward_inhibit:
            held.append("フードの端のビットが立たなかった（人の確認が要る。MQTT からは解除できない）")
        return held + self.session.pose_blockers() + start_blockers(self.cfg, self.est.estimate(t), self.est.health(t))

    def start(self, task: dict[str, Any]) -> None:
        self.task = task
        self._reported = False
        if task.get("child_near") is True:                   # Home AI の「子どもが近い」は安全側にだけ効く（false は無視）
            self.csar.hint_near(self.session.t, float(self.cfg["floor_watch"]["csar"]["home_hint_s"]))
        self._route, self.highlight = None, None
        if task["task"] == "highlight_point":                # 物を指す・照らす = 子どもを物へ連れていく（CSAR R1・R2）。子どもが FAR のときだけ
            if not self.csar.allows_attention_to_object(self.session.t):
                self._pending_finish = ("failed", "CSAR: 子どもが近いかもしれない（または確かめられない）ので、物を指し示さない")
                return
            tgt = self.frame.to_world(np.array([float(task["target"]["x_m"]), float(task["target"]["y_m"])]))
            self.highlight = HighlightMission(tgt, float(task.get("duration_s", 4.5)), self.csar.allows_attention_to_object)
            return                                            # 胴は動かさない（IdleHold のまま。頭ヨーだけで示す）
        if task["task"] == "patrol_route":                   # 各点で inspect（順番に。1 点が後回し / 失敗でも次へ）
            self._route = [(float(w["x_m"]), float(w["y_m"])) for w in task["waypoints"]]
            self._route_i, self._route_log = 0, []
            self._begin_inspect(*self._route[0])
            return
        if task["task"] != "inspect_point":
            self._pending_finish = ("failed", f"{task['task']} はこの段階では未実装（模擬の縦一本は inspect_point / patrol_route / highlight_point）")
            return
        self._begin_inspect(float(task["target"]["x_m"]), float(task["target"]["y_m"]))

    def _begin_inspect(self, x_m: float, y_m: float) -> None:
        """1 地点の inspect の mission を始める（inspect_point と patrol_route の各点で共通）。"""
        self._reported = False
        tgt = self.frame.to_world(np.array([x_m, y_m]))
        # 目標 = カメラの視野中心をその地点に置く。機体（首マーカ）はその手前 = 首→頭先端 + 視野中心の距離だけ手前に止まる
        neck, tip = self.session.world.marker_xy("neck"), self.session.world.head_tip()[:2]
        d = tgt - neck
        direction = d / max(float(np.linalg.norm(d)), 1e-9)
        offset = float(np.linalg.norm(tip - neck)) + float(self.cam.floor_point(self.cam.cx, self.cam.cy)[1])
        pts = self.session.world.world_points()
        self.mission = InspectMission(self.cfg, self.session.brain.loco, tgt - direction * offset, self._capture, self._judge,
                                      self.session.t, aim=self._aim, measure=lambda: self._offset_to(tgt),
                                      head_link_mm=float(np.linalg.norm(pts[-1][:2] - pts[-2][:2])),
                                      attention_ok=self.csar.allows_attention_to_object, object_mm=tgt)
        self.session.mission = self.mission

    def stop(self, reason: str) -> None:
        """Task の stop: mission を捨て、機体を停止（HOLD）させる。再開は人の開始操作（session.request_start）だけ。"""
        if self.mission is not None:
            self.mission.abort(reason)
        self.session.brain.loco.stop(reason)
        self.session.request_stop(reason, source="Home AI")
        self.session.mission, self.mission = self.idle, None
        self.highlight, self._route = None, None
        self.task = None

    # ---- 1 周期 -------------------------------------------------------------------
    def tick(self) -> None:
        self.session.step()
        t = self.session.t
        self._localize(t)
        self._check_hood(t)
        self._observe_people(t)
        if self._pending_finish is not None and self.task is not None:
            status, why = self._pending_finish
            self._pending_finish = None
            self._finish(status, why)
        if self.highlight is not None and self.task is not None:
            self._tick_highlight(t)
        m = self.mission
        if m is not None and self.task is not None:
            if m.phase in ("RETREAT", "DONE") and not self._reported and m.result is not None:
                self._reported = True                               # 判定が確定したらすぐ知らせる（離れるのを待たない。CSAR R4）
                for c in m.result:
                    self._report(c, t)
            if m.phase in ("WAIT_CHILD", "RETREAT") and self.csar.state(t) == NEAR:
                self._look_at_person(t)                             # 子どもが来たら、物ではなく子どもの方を見る（CSAR R5）
        if m is not None and not m.active and self.task is not None:
            if self._route is not None:
                self._route_log.append((m.phase, m.reason, len(m.result) if m.result else 0))
                self.session.mission, self.mission = self.idle, None
                if self._route_i + 1 < len(self._route):
                    self._route_i += 1
                    self._begin_inspect(*self._route[self._route_i])         # 次の地点へ（1 点が後回し / 失敗でも続ける）
                else:
                    self._finish_route()
            else:
                if m.phase == "DONE":
                    self._finish("done", f"候補 {len(m.result)} 件")
                elif m.phase in ("FAILED", "DEFERRED"):
                    self._finish("failed", m.reason)
                self.session.mission, self.mission = self.idle, None
        self._publish_safety()
        if self.endpoint is not None:
            self.endpoint.tick()

    def _finish_route(self) -> None:
        log, self._route = self._route_log, None
        ok = [x for x in log if x[0] == "DONE"]
        n_find = sum(x[2] for x in log)
        notes = [f"{x[0]}: {x[1]}" for x in log if x[0] != "DONE"]
        if ok:
            self._finish("done", f"{len(ok)}/{len(log)} 点を確認、候補 {n_find} 件" + (f"（未確認 {len(notes)} 点: {'; '.join(notes)}）" if notes else ""))
        else:
            self._finish("failed", "どの地点も確認できなかった: " + "; ".join(notes))

    def _tick_highlight(self, t: float) -> None:
        h = self.highlight
        person = getattr(self.session, "target", None)
        pm = None if person is None else np.asarray(person.floor_mm, float)
        h.tick(t, lambda xy: self._look_toward(t, xy), pm)
        if not h.active:
            self.highlight = None
            if h.phase != HL_DONE and self.csar.state(t) == NEAR:
                self._look_at_person(t)                             # 子どもが来たら、物ではなく子どもの方を見る（CSAR R5）
            self._finish("done" if h.phase == HL_DONE else "failed", h.reason)

    def _look_toward(self, t: float, xy_mm: np.ndarray) -> bool:
        """頭ヨーをその点へ。上限（csar.look_at_person_max_deg）内なら向けて True、外なら向けずに False（無理に回さない）。"""
        pts = self.session.world.world_points()
        tip, prev = pts[-1][:2], pts[-2][:2]
        heading = math.atan2(float(tip[1] - prev[1]), float(tip[0] - prev[0]))
        d = np.asarray(xy_mm, float) - tip
        rel = math.degrees(wrap(math.atan2(float(d[1]), float(d[0])) - heading))
        current = float(self.session.anim.base.get(HEAD_YAW, 0.0))
        lim = float(self.cfg["floor_watch"]["csar"]["look_at_person_max_deg"])
        goal = current + rel
        if abs(goal) > lim:
            return False
        self.session.brain.expr.look_at(t, goal, force=True)
        return True

    def _observe_people(self, t: float) -> None:
        """人の観測 → 子どもが近いか。子どもと大人は区別できないので、人は誰でも「子どもかもしれない」。
        **模擬**: 人の追跡は展示のカメラ（上から見る）なので、いつでも見られる（sensing=True）。実機の頭カメラは床を向いている間
        人を見られない → sensing=False にして、撮影の前後に頭を上げて周りを見る（Discovery Loop の 3）。"""
        person = getattr(self.session, "target", None)
        if person is None:
            self.csar.observe(t, None, sensing=True)
            return
        tip = self.session.world.head_tip()[:2]
        self.csar.observe(t, float(np.linalg.norm(np.asarray(person.floor_mm, float) - tip)) / 1000.0, sensing=True)

    def _look_at_person(self, t: float) -> None:
        """頭（J8 相当）を人の方へ。上限つき。近づく動きは入れない（R5 の範囲は見るだけ）。"""
        person = getattr(self.session, "target", None)
        if person is None:
            return
        pts = self.session.world.world_points()
        tip, prev = pts[-1][:2], pts[-2][:2]
        heading = math.atan2(float(tip[1] - prev[1]), float(tip[0] - prev[0]))
        to_p = np.asarray(person.floor_mm, float) - tip
        rel = math.degrees(wrap(math.atan2(float(to_p[1]), float(to_p[0])) - heading))   # 今の頭の向きからの角度
        current = float(self.session.anim.base.get(HEAD_YAW, 0.0))
        lim = float(self.cfg["floor_watch"]["csar"]["look_at_person_max_deg"])
        self.session.brain.expr.look_at(t, max(-lim, min(lim, current + rel)), force=True)

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
