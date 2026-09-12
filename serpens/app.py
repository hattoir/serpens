"""Serpens EX-1 アプリ本体。

  python -m serpens.app --sim                      シミュレータだけ（実機なし）
  python -m serpens.app --sim --camera 0           実カメラで人を検出し、ヘビはシミュレータ
  python -m serpens.app --bus feetech --port COM5  実機のサーボへ（実機テストは未実施）

制御は 50Hz の専用スレッド（serpens/runner.py）、GUI の描画は 10fps。
ネットワークには一切つながない（YOLO の自動ダウンロード・自動更新は無効化してある）。
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np

from serpens.config import load_config
from serpens.runner import ControlLoop
from serpens.sim.session import SimSession
from serpens.sim.virtual_camera import SimPerson, VirtualCamera
from serpens.sim.world import BodyPose

# ultralytics がネットワークへ出ないようにする（import より前に設定する）
os.environ.setdefault("YOLO_AUTOINSTALL", "false")
os.environ.setdefault("YOLO_OFFLINE", "true")
os.environ.setdefault("YOLO_HUB_OFFLINE", "true")

PERCEPTION_HZ = 10.0
FRAME_HZ = 5.0          # 表示用の映像を作る頻度（重いので制御より遅くする）
DISPLAY_WIDTH_PX = 960


class SimCamera(threading.Thread):
    """--sim のときの映像: 仮想カメラでシミュレータの世界を描く（表示用に縮小）。

    描画は重いので専用スレッドで FRAME_HZ 回だけ作る。GUI スレッドは出来た画像を貼るだけにして、
    制御ループ（50Hz）の周期を乱さないようにする。
    """

    def __init__(self, cfg: dict[str, Any], session: SimSession) -> None:
        super().__init__(name="sim-camera", daemon=True)
        small = {**cfg, "virtual_camera": dict(cfg["virtual_camera"])}
        scale = DISPLAY_WIDTH_PX / float(cfg["virtual_camera"]["width_px"])
        small["virtual_camera"]["width_px"] = int(cfg["virtual_camera"]["width_px"] * scale)
        small["virtual_camera"]["height_px"] = int(cfg["virtual_camera"]["height_px"] * scale)
        small["virtual_camera"]["mat_corners_px"] = [[x * scale, y * scale]
                                                     for x, y in cfg["virtual_camera"]["mat_corners_px"]]
        self.cam = VirtualCamera(small)
        self.session = session
        self.frame: np.ndarray | None = None
        self._stop_evt = threading.Event()   # Thread._stop と名前が衝突しないように

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        period = 1.0 / FRAME_HZ
        while not self._stop_evt.is_set():
            t0 = time.perf_counter()
            self.frame = self.cam.render(self.session.world, self.session.people)
            time.sleep(max(period - (time.perf_counter() - t0), 0.0))

    def latest(self) -> np.ndarray | None:
        return self.frame


class RealCamera(threading.Thread):
    """実カメラを別スレッドで読み、ArUco と人を検出する。

    ホモグラフィ（床の校正）や YOLO の重みが無くても落ちない。映像だけ表示して先へ進む。
    """

    def __init__(self, cfg: dict[str, Any], source: str, session: SimSession | None) -> None:
        super().__init__(name="perception", daemon=True)
        from serpens.perception.camera import open_source

        self.cfg, self.session = cfg, session
        self.src = open_source(cfg, source)
        self.frame: np.ndarray | None = None
        self.notes: list[str] = []
        self.locator = self.people = None
        self.tracker = None
        hpath = Path(cfg["homography"]["file"])
        if hpath.exists():
            from serpens.perception.aruco_locator import ArucoLocator
            from serpens.perception.homography import FloorHomography

            h = FloorHomography.load(hpath)
            self.locator = ArucoLocator(cfg, h)
            try:
                from serpens.perception.person_detector import PersonTracker, YoloPersonDetector

                self.people = YoloPersonDetector(cfg, h)
                self.tracker = PersonTracker(cfg)
            except FileNotFoundError as e:
                self.notes.append(f"人物検出なし: {e}")
        else:
            self.notes.append(f"床の校正がありません（{hpath}）。映像のみ表示します")
        self._stop_evt = threading.Event()   # Thread._stop と名前が衝突しないように

    def stop(self) -> None:
        self._stop_evt.set()

    def run(self) -> None:
        period = 1.0 / PERCEPTION_HZ
        while not self._stop_evt.is_set():
            t0 = time.perf_counter()
            ok, frame = self.src.read()
            if ok and frame is not None:
                self.frame = frame
                if self.locator is not None:
                    self._process(frame)
            time.sleep(max(period - (time.perf_counter() - t0), 0.0))

    def _process(self, frame: np.ndarray) -> None:
        import cv2

        obs = self.locator.observe(frame)
        for c in obs.corners_px.values():
            cv2.polylines(frame, [c.astype(np.int32)], True, (0, 0, 255), 3)
        if self.people is None or self.tracker is None or self.session is None:
            return
        dets = self.people.detect(frame)
        target = self.tracker.update(time.monotonic(), dets, None)
        for d in dets:
            x1, y1, x2, y2 = (int(v) for v in d.bbox_px)
            hit = target is not None and target.detection is d
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 220, 255) if hit else (200, 200, 200), 4 if hit else 2)
        # 検出した人を、シミュレータのヘビが反応できるよう世界へ渡す
        self.session.people = [SimPerson(float(d.floor_mm[0]), float(d.floor_mm[1])) for d in dets]

    def latest(self) -> np.ndarray | None:
        return self.frame


def _demo_script(loop: ControlLoop) -> None:
    """展示のリハーサル: 人が来て、近づいて、撫でて、去る。"""
    steps = [(4.0, lambda: loop.add_person(600.0, -500.0)),
             (16.0, lambda: loop.add_person(600.0, -350.0)),
             (24.0, lambda: loop.touch(True)),
             (26.0, lambda: loop.touch(False)),
             (34.0, lambda: loop.clear_people())]
    t0 = time.monotonic()
    for when, fn in steps:
        time.sleep(max(t0 + when - time.monotonic(), 0.0))
        fn()


def build(args: argparse.Namespace) -> tuple[dict[str, Any], SimSession, ControlLoop, Any]:
    """設定・セッション・制御ループ・映像ソースを作る。"""
    cfg = load_config()
    start = BodyPose(float(cfg["sim"]["start_tail_x_mm"]), float(cfg["sim"]["start_tail_y_mm"]), 0.0)
    session = SimSession(cfg, start, seed=args.seed)
    if args.bus == "feetech":
        from serpens.hw.servo_bus import make_bus

        bus = make_bus("feetech", cfg, args.port)
        bus.connect()
        for sid in bus.ids:
            bus.set_torque(sid, True)
        session.bus = bus                      # 実機へ送る（シミュレータの世界はそのまま動かす）
        print(f"実機のサーボへ接続しました: {args.port}")
    loop = ControlLoop(session, realtime=True)
    if args.camera is not None:
        cam: Any = RealCamera(cfg, args.camera, session)
        cam.start()
        for n in cam.notes:
            print(f"[注意] {n}")
        source = cam.latest
    else:
        cam = SimCamera(cfg, session)
        cam.start()
        source = cam.latest
    return cfg, session, loop, (source, cam)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="シミュレータで動かす（既定）")
    ap.add_argument("--sim-robot", action="store_true", help="--sim と同じ（ヘビはシミュレータ）")
    ap.add_argument("--camera", default=None, help="カメラ番号 / 動画 / 画像")
    ap.add_argument("--bus", choices=["mock", "feetech"], default="mock", help="サーボバス")
    ap.add_argument("--port", default=None, help="実機のCOMポート（例: COM5）")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--seconds", type=float, default=None, help="この秒数で自動終了（動作確認用）")
    ap.add_argument("--record", default=None, help="GUI を GIF に記録する（--seconds と併用）")
    ap.add_argument("--no-gui", action="store_true", help="GUI を出さずに動かす（動作確認用）")
    ap.add_argument("--demo", action="store_true", help="人の出入りを自動で再現する（展示のリハーサル用）")
    args = ap.parse_args(argv)

    cfg, session, loop, (source, cam) = build(args)
    loop.start()
    if args.demo:
        threading.Thread(target=_demo_script, args=(loop,), daemon=True).start()
    try:
        if args.no_gui:
            end = time.monotonic() + (args.seconds or 5.0)
            while time.monotonic() < end:
                time.sleep(0.2)
                st = loop.latest().status
                if st is not None:
                    print(f"t={st.t:6.2f}s [{st.state_ja}] {st.thought}")
        else:
            from serpens.gui.run import run_gui

            run_gui(loop, cfg, source, seconds=args.seconds, record=args.record)
    finally:
        loop.stop()
        if cam is not None:
            cam.stop()
    s = loop.stats
    print(f"制御周期: 目標 {1000 / s.target_hz:.1f}ms / 実測 平均 {s.mean_ms:.2f}ms 最悪 {s.worst_ms:.2f}ms "
          f"（{s.count} 回、20%以上の遅延 {s.late_count} 回）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
