"""キーボード割り当て（GUI とヘッドレスで共通）。

停止は3つに分ける（`serpens/safety.py`）。「一時停止」と「機体の停止」を同じキーにしない。

  G … 開始（待機 → 走行）。実機は開始条件を満たさないと動かない
  S … **通常停止**（歩容を即時停止し、現在姿勢を保持。出力へ保持指令を送り続ける）
  E … **緊急停止**（ラッチ。通常操作や予約済みの演出では解除されない）
  U … 緊急停止の解除（→ 待機。走行は再開しない。再開は G）
  L … 駆動無効化（脱力）。**停止中の明示操作のみ**
  P … シミュレーションの一時停止（実機では使えない）
  R … ホーム復帰（走行中のみ）      H … ホーム姿勢（走行中のみ）
  D … デモの人を出す / 消す         C … カメラ再校正の案内
  T … タッチ（モックのみ）          Q / Esc … 終了
"""
from __future__ import annotations

from typing import Any

KEY_HELP = [
    ("G", "開始"),
    ("S", "停止(保持)"),
    ("E", "緊急停止"),
    ("U", "緊急解除"),
    ("L", "脱力"),
    ("P", "一時停止(sim)"),
    ("R", "ホーム復帰"),
    ("H", "ホーム姿勢"),
    ("D", "デモの人"),
    ("T", "タッチ"),
    ("Q", "終了"),
]

DEMO_PERSON_MM = (600.0, -400.0)     # D で置く人の位置（来場者側）


def handle(runner: Any, key: str) -> str:
    """キー1文字を処理して、画面に出すメッセージを返す。"""
    k = key.upper()
    s = runner.session
    # ---- 停止・開始 ----
    if k == "S":
        runner.request_stop("操作: 通常停止", "操作")
        return "通常停止（姿勢を保持）"
    if k == "E":
        runner.request_emergency("操作: 緊急停止", "操作")
        return "緊急停止（ラッチ）。解除は U"
    if k == "U":
        if not s.stop.latched:
            return "緊急停止していません"
        s.clear_emergency("操作")
        s.enforce_stop_output()
        return "緊急停止を解除 → 待機（走行するには G）"
    if k == "G":
        ok, why = s.request_start("操作")
        return "開始（走行）" if ok else "開始できません: " + " / ".join(why)
    if k == "L":
        if s.robot_is_real and s.stop.state.value == "RUN":
            return "脱力は停止してから（S で停止 → L）"
        return "駆動無効化（脱力）" if s.request_disable_torque("操作: 駆動無効化", "操作") \
            else "脱力できません（緊急停止中、または走行中）"
    if k == "P":
        if s.robot_is_real:
            return "実機では一時停止を使いません（停止は S / 緊急は E）"
        runner.paused = not runner.paused
        return "シミュレーション一時停止" if runner.paused else "一時停止を解除"
    # ---- 姿勢（走行中のみ。停止中に勝手に動かさない） ----
    if k in ("R", "H"):
        if not s.stop.moving_allowed:
            return f"停止中は姿勢を変えません（{s.stop.status_text()}）。開始は G"
        if k == "R":
            s.brain.fsm.force(s.t, "PATROL", "キー操作: ホーム復帰")
            s.anim.set_pose_now(s.brain.poses.home())
            s.anim.gait.stop(immediate=True)
            return "ホーム復帰"
        s.anim.set_pose_now(s.brain.poses.home())
        return "ホーム姿勢"
    # ---- その他 ----
    if k == "D":
        if s.people:
            runner.clear_people()
            return "人を消した"
        runner.add_person(*DEMO_PERSON_MM)
        return "デモ開始: 来場者側に人を置いた"
    if k == "C":
        return "カメラ再校正: tools/calibrate_floor.py を実行してください（--sim では不要）"
    if k == "T":
        runner.touch(True)
        return "タッチ ON"
    return ""
