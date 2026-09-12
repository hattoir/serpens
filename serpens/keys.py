"""キーボード割り当て（GUI とヘッドレスで共通）。

  R … ホーム復帰（状態を PATROL に戻し、ホーム姿勢へ）
  D … デモ開始（人を来場者側に置く。もう一度押すと人を消す）
  S … 全停止（歩容もキーフレームも止める。もう一度押すと解除）
  H … ホーム姿勢
  C … カメラ再校正（tools/calibrate_floor.py の案内を出す。--sim では無効）
  T … タッチ（押している間だけ。GUI では押し下げ/離しで ON/OFF）
  Q / Esc … 終了
"""
from __future__ import annotations

from typing import Any

KEY_HELP = [
    ("R", "ホーム復帰"),
    ("D", "デモ開始 / 人を消す"),
    ("S", "全停止 / 解除"),
    ("H", "ホーム姿勢"),
    ("C", "カメラ再校正"),
    ("T", "タッチ"),
    ("Q", "終了"),
]

DEMO_PERSON_MM = (600.0, -400.0)     # D で置く人の位置（来場者側）


def handle(runner: Any, key: str) -> str:
    """キー1文字を処理して、画面に出すメッセージを返す。"""
    k = key.upper()
    s = runner.session
    if k == "R":
        s.brain.fsm.force(s.t, "PATROL", "キー操作: ホーム復帰")
        s.anim.set_pose_now(s.brain.poses.home())
        s.anim.gait.stop(immediate=True)
        return "ホーム復帰"
    if k == "H":
        s.anim.play_home = True
        s.anim.set_pose_now(s.brain.poses.home())
        return "ホーム姿勢"
    if k == "D":
        if s.people:
            runner.clear_people()
            return "人を消した"
        runner.add_person(*DEMO_PERSON_MM)
        return "デモ開始: 来場者側に人を置いた"
    if k == "S":
        runner.paused = not runner.paused
        if runner.paused:
            s.anim.gait.stop(immediate=True)
        return "全停止" if runner.paused else "全停止を解除"
    if k == "C":
        return "カメラ再校正: tools/calibrate_floor.py を実行してください（--sim では不要）"
    if k == "T":
        runner.touch(True)
        return "タッチ ON"
    return ""
