"""印刷用 ArUco マーカ（DICT_4X4_50, ID0 = 尾 / ID1 = 首, 40mm）を作る。

A4（300dpi）1枚に、2枚のマーカ・白い余白・切り取り線・実寸表記・100mm の検尺バーを並べる。
印刷するときは「実際のサイズ」「拡大縮小なし」を選び、検尺バーが 100mm あるか定規で確かめること。

使い方: python tools/make_aruco.py [--out output/aruco_markers_A4.png] [--spares 2]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

from serpens.config import load_config  # noqa: E402

MM_PER_INCH = 25.4
A4_MM = (210.0, 297.0)
PAGE_MARGIN_MM = 15.0
SCALE_BAR_MM = 100.0
LABEL_PT_MM = 4.0          # 文字の高さ
CUT_LINE_GRAY = 160
FONT_CANDIDATES = ("C:/Windows/Fonts/YuGothM.ttc", "C:/Windows/Fonts/meiryo.ttc", "C:/Windows/Fonts/msgothic.ttc")


def mm2px(mm: float, dpi: int) -> int:
    return int(round(mm * dpi / MM_PER_INCH))


def load_font(size_px: int) -> ImageFont.ImageFont:
    for f in FONT_CANDIDATES:
        if Path(f).exists():
            return ImageFont.truetype(f, size_px)
    return ImageFont.load_default()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="output/aruco_markers_A4.png")
    ap.add_argument("--spares", type=int, default=2, help="予備を何組並べるか（貼り直し用）")
    args = ap.parse_args()
    cfg = load_config()
    m, a = cfg["markers"], cfg["aruco"]
    dpi = int(a["print_dpi"])
    size_px = mm2px(float(m["size_mm"]), dpi)
    margin_px = mm2px(float(a["print_margin_mm"]), dpi)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, m["dictionary"]))
    page = Image.new("L", (mm2px(A4_MM[0], dpi), mm2px(A4_MM[1], dpi)), 255)
    draw = ImageDraw.Draw(page)
    font = load_font(mm2px(LABEL_PT_MM, dpi))
    x0 = y = mm2px(PAGE_MARGIN_MM, dpi)
    draw.text((x0, y), f"Serpens EX-1 ArUco {m['dictionary']}  黒い部分 = {m['size_mm']:.0f}mm 角"
                       f"（白い余白 {a['print_margin_mm']:.0f}mm も切らずに残す）", fill=0, font=font)
    y += mm2px(LABEL_PT_MM * 2.5, dpi)
    cell = size_px + 2 * margin_px
    gap = mm2px(PAGE_MARGIN_MM, dpi)
    labels = [(int(m["tail_id"]), "尾（尾端-J1 上面）"), (int(m["neck_id"]), "首（J7 の手前・上面）")]
    for row in range(1 + max(args.spares, 0)):
        x = x0
        for mid, text in labels:
            img = cv2.aruco.generateImageMarker(d, mid, size_px)
            page.paste(Image.fromarray(img), (x + margin_px, y + margin_px))
            draw.rectangle([x, y, x + cell, y + cell], outline=CUT_LINE_GRAY, width=2)   # 切り取り線
            tag = f"ID {mid}  {text}" + ("" if row == 0 else "  予備")
            draw.text((x, y + cell + mm2px(1.5, dpi)), tag, fill=0, font=font)
            x += cell + gap * 4
        y += cell + mm2px(LABEL_PT_MM * 3, dpi)
    y += gap
    bar = mm2px(SCALE_BAR_MM, dpi)
    draw.rectangle([x0, y, x0 + bar, y + mm2px(3, dpi)], fill=0)
    for k in range(int(SCALE_BAR_MM / 10) + 1):
        xx = x0 + mm2px(10 * k, dpi)
        draw.line([xx, y - mm2px(2, dpi), xx, y], fill=0, width=2)
    draw.text((x0, y + mm2px(5, dpi)), f"検尺: この黒い棒が {SCALE_BAR_MM:.0f}mm になっていること（{dpi}dpi, 拡大縮小なしで印刷）",
              fill=0, font=font)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    page.save(out, dpi=(dpi, dpi))
    print(f"saved {out}  ({page.size[0]}x{page.size[1]}px, {dpi}dpi, マーカ {size_px}px = {m['size_mm']}mm)")


if __name__ == "__main__":
    main()
