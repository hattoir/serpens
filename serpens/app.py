"""Serpens EX-1 アプリ本体。

  python -m serpens.app --sim                      シミュレータだけ（実機なし）
  python -m serpens.app --sim --camera 0           実カメラで人を検出し、ヘビはシミュレータ
  python -m serpens.app --bus feetech --port COM5  実機のサーボへ（実機テストは未実施）

制御は 50Hz の専用スレッド（serpens/runner.py）、GUI の描画は 10fps。
ネットワークには一切つながない（YOLO の自動ダウンロード・自動更新は無効化してある）。

実機経路（--bus feetech）では、サーボバスと頭部 I/O を**セッションより先に作って注入**する。
監視・指令・トルク操作が同じ接続先を参照することを、ここで確定させる。
実機は待機（停止）から始まり、開始条件（実観測の自己位置・校正・駆動リンク）が揃うまで走らない。
"""
from __future__ import annotations

import argparse
import math
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

    def release(self) -> None:
        """スレッドを止める（仮想カメラは解放するものが無い）。"""
        self.stop()
        self.join(timeout=1.0)


class RealCamera(threading.Thread):
    """実カメラを別スレッドで読み、ArUco と人を検出する。

    ホモグラフィ（床の校正）や YOLO の重みが無くても落ちない。映像だけ表示して先へ進む。
    """

    def __init__(self, cfg: dict[str, Any], source: str, session: SimSession | None) -> None:
        super().__init__(name="perception", daemon=True)
        from serpens.perception.camera import open_source

        self.cfg, self.session = cfg, session
        self.hz = float(cfg["person"]["detect_hz"])
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
        period = 1.0 / self.hz
        hold_s = float(self.cfg["person"]["hold_s"])
        last_ok = time.monotonic()
        while not self._stop_evt.is_set():
            t0 = time.perf_counter()
            ok, frame = self.src.read()
            if ok and frame is not None:
                self.frame = frame
                last_ok = time.monotonic()
                if self.locator is not None:
                    self._process(frame)
            elif time.monotonic() - last_ok > hold_s and self.session is not None:
                # カメラが止まったら、古い人物座標を新しい検出として使わせない
                self.session.people = []
                self.frame = None
            time.sleep(max(period - (time.perf_counter() - t0), 0.0))

    def release(self) -> None:
        """スレッドを止めてカメラを解放する。"""
        self.stop()
        self.join(timeout=1.0)
        try:
            self.src.release()
        except Exception as e:                    # noqa: BLE001 - 解放失敗も伝える
            print(f"[注意] カメラの解放に失敗: {e}")

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


def _open_real_bus(cfg: dict[str, Any], port: str | None) -> Any:
    """実機のサーボバスを開く（失敗したら原因の候補を出して終了）。"""
    from serpens.hw.servo_bus import make_bus

    bus = make_bus("feetech", cfg, port)
    try:
        bus.connect()
    except Exception as e:                     # noqa: BLE001 - ポートが無い・使用中など何でも
        raise SystemExit(
            f"サーボバスに接続できません（{port}）: {e}\n"
            "  1. USB が挿さっているか（ポート一覧: python tools/servo_setup.py → メニュー 1）\n"
            "  2. 他のソフトが COM を掴んでいないか\n"
            "  3. 12V 電源が入っているか\n"
            "  実機が無いときは --sim で動かしてください") from e
    for sid in bus.ids:
        bus.set_torque(sid, True)
    print(f"実機のサーボへ接続しました: {port}")
    return bus


def build(args: argparse.Namespace) -> tuple[dict[str, Any], SimSession, ControlLoop, Any]:
    """設定・出力先・セッション・制御ループ・映像ソースを作る。

    実機経路では**セッションより先に**バスと頭部を作り、注入する（参照を1つにそろえる）。
    途中で失敗したら、そこまでに開いたものを閉じてから投げ直す。
    """
    cfg = load_config()
    start = BodyPose(float(cfg["sim"]["start_tail_x_mm"]), float(cfg["sim"]["start_tail_y_mm"]),
                     math.radians(float(cfg["sim"]["start_theta_deg"])))
    real = args.bus == "feetech"
    bus = _open_real_bus(cfg, args.port) if real else None
    head: Any = None
    try:
        if args.head_port:
            from serpens.hw.head_io import SerialHeadIO

            head = SerialHeadIO(cfg, args.head_port)
            print(f"頭部 I/O へ接続します: {args.head_port}")
        elif real:
            print("[注意] 頭部 I/O 未接続（--head-port 未指定）。ToF・タッチ・目は使えません。"
                  "モックを実センサーとしては使いません")
        session = SimSession(cfg, start, seed=args.seed, bus=bus, head=head, robot_is_real=real)
        loop = ControlLoop(session, realtime=True)
        cam: Any = RealCamera(cfg, args.camera, session) if args.camera is not None else SimCamera(cfg, session)
        cam.start()
        for n in getattr(cam, "notes", []):
            print(f"[注意] {n}")
    except BaseException:
        if head is not None:
            head.close()
        if bus is not None:
            bus.disconnect(torque_off=False)
        raise
    if real:
        print(f"駆動状態: {session.stop.status_text()}")
        for b in session.blockers():
            print(f"[開始条件] {b}")
        print("  ※ 上が解消されるまで実機の自律走行は開始できません（G を押しても動きません）")
    return cfg, session, loop, (cam.latest, cam)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="シミュレータで動かす（既定）")
    ap.add_argument("--sim-robot", action="store_true", help="--sim と同じ（ヘビはシミュレータ）")
    ap.add_argument("--camera", default=None, help="カメラ番号 / 動画 / 画像")
    ap.add_argument("--bus", choices=["mock", "feetech"], default="mock", help="サーボバス")
    ap.add_argument("--port", default=None, help="実機のCOMポート（例: COM5）")
    ap.add_argument("--head-port", default=None, help="頭部 XIAO ESP32S3 のCOMポート（例: COM6）")
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
        errors = loop.shutdown()               # 停止を出力へ → join → 切断
        if cam is not None:
            cam.release()                      # カメラ解放（スレッド join 込み）
        for msg in errors:
            print(f"[終了時の問題] {msg}")
    s = loop.stats
    print(f"制御周期: 目標 {1000 / s.target_hz:.1f}ms / 実測 平均 {s.mean_ms:.2f}ms 最悪 {s.worst_ms:.2f}ms "
          f"（{s.count} 回、20%以上の遅延 {s.late_count} 回）")
    print(f"終了時の駆動状態: {session.stop.status_text()}")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
