"""ToF 小基板（12 x 18 x 3.2、長さは ASSUMED）の載せ位置 2 案を上から見た図。先頭の注意: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍のすき間が残る。背板は未解決。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。CAD_CONCEPT"""
import sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
plt.rcParams["font.family"] = ["Yu Gothic", "Meiryo", "sans-serif"]
XM = -236.0
def hood(ax):
    pts = [(0, 34.0), (30, 19.4), (60, 19.4), (60, -19.4), (30, -19.4), (0, -34.0)]
    ax.add_patch(Polygon([(x + XM, y) for x, y in pts], closed=True, fc="#f3e6c8", ec="k", lw=1)); ax.text(XM + 30, 0, "フード\n(口 60→30)", ha="center", va="center", fontsize=9)
def head(ax):
    ax.add_patch(Rectangle((-236, -50), 74, 100, fc="none", ec="#7a9a6a", ls="--")); ax.text(-198, 47, "頭の最大幅 ±50", fontsize=8, color="#5a7a4a")
fig, axs = plt.subplots(1, 3, figsize=(20, 7))
for ax in axs:
    ax.set_xlim(-245, -160); ax.set_ylim(-62, 62); ax.set_aspect("equal"); ax.set_xlabel("x [mm]（左が前）"); ax.set_ylabel("y [mm]"); hood(ax); head(ax)
# 現状（横スキッド 8 幅）
ax = axs[0]; ax.set_title("いまの配置（横スキッド |y| 36〜44、ToF は後ろ）", fontsize=12)
for sg in (1, -1):
    ax.add_patch(Rectangle((-226, min(sg * 36, sg * 44)), 48, 8, fc="#a9cfa0", ec="k")); ax.add_patch(Circle((-226, sg * 40), 2, fc="#f294a5", ec="k"))
ax.add_patch(Rectangle((-172, -7), 7, 14, fc="#5b7fc9", ec="k")); ax.text(-168, -12, "ToF(後ろ)\nE-0008: 不可", fontsize=8, ha="center")
# S1
ax = axs[1]; ax.set_title("案 S1: 外付け（スキッド外面に板 2 mm、基板 12×18 が外へ）", fontsize=12)
for sg in (1, -1):
    ax.add_patch(Rectangle((-226, min(sg * 36, sg * 44)), 48, 8, fc="#a9cfa0", ec="k")); ax.add_patch(Circle((-226, sg * 40), 2, fc="#f294a5", ec="k"))
    ax.add_patch(Rectangle((-226, min(sg * 44, sg * 46)), 18, 2, fc="#999", ec="k")); ax.add_patch(Rectangle((-226, min(sg * 46, sg * 58)), 18, 12, fc="#5b7fc9", ec="k"))
ax.text(-217, 60, "|y| 58（頭の最大 ±50 を超える）", fontsize=8, ha="center")
# S2
ax = axs[2]; ax.set_title("案 S2: スキッドの前端を広げる（丸い「肉球」|y| 36〜51.6）", fontsize=12)
for sg in (1, -1):
    yc = sg * 43.8; ax.add_patch(Rectangle((-222, min(sg * 36, sg * 51.6)), 12, 15.6, fc="#a9cfa0", ec="k")); ax.add_patch(Circle((-222, yc), 7.8, fc="#a9cfa0", ec="k")); ax.add_patch(Circle((-210, yc), 7.8, fc="#a9cfa0", ec="k"))
    ax.add_patch(Rectangle((-178 - 24, min(sg * 36, sg * 44)), 24, 8, fc="#a9cfa0", ec="k")); ax.add_patch(Rectangle((-223.5, min(sg * 37.8, sg * 49.8)), 16, 12, fc="#5b7fc9", ec="k")); ax.add_patch(Circle((-228, sg * 40), 2, fc="#f294a5", ec="k"))
ax.text(-217, 56, "肉球の幅 51.6（頭の最大 ±50 に近い）", fontsize=8, ha="center")
fig.suptitle("ToF 小基板の載せ位置（青 = 基板、緑 = スキッド、桃 = 頬のレンズ）— 先頭の注意: KNUCKLE DRUM 後も、まっすぐ比で約 20 倍のすき間が残る。背板は未解決。5.7 N・0.25 N·m は暫定（SAFETY_UNVERIFIED）。安全・合格の語は使わない。CAD_CONCEPT", fontsize=10)
fig.tight_layout(); fig.savefig(sys.argv[1], dpi=60)
