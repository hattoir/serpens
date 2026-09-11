"""床のホモグラフィ校正: カメラ画像でマットの四隅を4回クリックして JSON に保存する。

クリック順: 左手前 (0,0) → 右手前 (W,0) → 右奥 (W,D) → 左奥 (0,D)
  （世界座標の原点 = マット左手前、X = 横、Y = 奥行き）
操作: 左クリック = 点を置く / Backspace = 1つ戻す / Enter = 保存 / Esc = やめる
保存後、マット上に 200mm 格子を重ねて表示するので、ずれていないか目で確認すること。

使い方:
  python tools/calibrate_floor.py --source 0             # Webカメラ 0 番
  python tools/calibrate_floor.py --source mat.jpg       # 静止画
  python tools/calibrate_floor.py --source 0 --points "240,690 1040,690 900,250 380,250"   # クリックなし
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from serpens.config import load_config  # noqa: E402
from serpens.perception.camera import open_source  # noqa: E402
from serpens.perception.homography import FloorHomography  # noqa: E402

WINDOW = "calibrate_floor"
GRID_MM = 200.0
KEY_ENTER, KEY_ESC, KEY_BACKSPACE = 13, 27, 8
POINT_BGR, TEXT_BGR, GRID_BGR = (0, 0, 255), (0, 255, 255), (0, 255, 0)
FONT_SCALE = 0.8


def parse_points(s: str) -> list[list[float]]:
    pts = [[float(v) for v in p.split(",")] for p in s.split()]
    if len(pts) != 4:
        raise SystemExit("--points は 4 点（'x,y x,y x,y x,y'）")
    return pts


def draw_grid(img: np.ndarray, h: FloorHomography, w_mm: float, d_mm: float) -> np.ndarray:
    """ホモグラフィで床の格子を重ねる（確認用）。"""
    out = img.copy()
    for x in np.arange(0.0, w_mm + 1, GRID_MM):
        a, b = h.floor_to_image(np.array([[x, 0.0], [x, d_mm]]))
        cv2.line(out, tuple(a.astype(int)), tuple(b.astype(int)), GRID_BGR, 1)
    for y in np.arange(0.0, d_mm + 1, GRID_MM):
        a, b = h.floor_to_image(np.array([[0.0, y], [w_mm, y]]))
        cv2.line(out, tuple(a.astype(int)), tuple(b.astype(int)), GRID_BGR, 1)
    return out


def click_corners(frame: np.ndarray, names: list[str]) -> list[list[float]] | None:
    """画像を表示してクリックで4点を集める。"""
    pts: list[list[float]] = []

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append([float(x), float(y)])

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW, on_mouse)
    while True:
        view = frame.copy()
        for i, p in enumerate(pts):
            cv2.circle(view, (int(p[0]), int(p[1])), 6, POINT_BGR, -1)
            cv2.putText(view, str(i + 1), (int(p[0]) + 8, int(p[1]) - 8), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, POINT_BGR, 2)
        msg = f"click {len(pts) + 1}/4: {['left-front', 'right-front', 'right-back', 'left-back'][len(pts)]}" \
            if len(pts) < 4 else "Enter = save / Backspace = undo"
        cv2.putText(view, msg, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, FONT_SCALE, TEXT_BGR, 2)
        cv2.imshow(WINDOW, view)
        k = cv2.waitKey(30) & 0xFF
        if k == KEY_ESC:
            return None
        if k == KEY_BACKSPACE and pts:
            pts.pop()
        if k == KEY_ENTER and len(pts) == 4:
            return pts


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="カメラ番号 / 画像 / 動画")
    ap.add_argument("--points", default=None, help="クリックの代わりに4点を与える")
    ap.add_argument("--out", default=None, help="保存先（既定は config の homography.file）")
    ap.add_argument("--no-show", action="store_true", help="確認表示をしない")
    args = ap.parse_args()
    cfg = load_config()
    src = open_source(cfg, args.source)
    ok, frame = src.read()
    src.release()
    if not ok or frame is None:
        raise SystemExit("画像を取得できません")
    print("クリック順:", " → ".join(cfg["homography"]["corner_order"]))
    pts = parse_points(args.points) if args.points else click_corners(frame, cfg["homography"]["corner_order"])
    if pts is None:
        print("中止しました")
        return
    h = FloorHomography.from_clicks(cfg, pts, (frame.shape[1], frame.shape[0]))
    out = args.out or cfg["homography"]["file"]
    h.save(out)
    print(f"saved {out}")
    if not args.no_show:
        cv2.imshow(WINDOW, draw_grid(frame, h, cfg["mat"]["width_mm"], cfg["mat"]["depth_mm"]))
        print("格子がマットに重なっているか確認して、何かキーを押すと閉じます")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
