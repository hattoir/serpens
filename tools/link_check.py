"""Phase 2 の完了条件を1本で確認し、時間を実測して表にする。

    .\\.venv\\Scripts\\python.exe tools\\link_check.py            # 模擬機体（偽時計）で全条件
    .\\.venv\\Scripts\\python.exe tools\\link_check.py --out docs\\phase2_measured.md

**模擬での値は「仕様どおりに作ればこうなる」という設計値**であって、実機の実測値ではない。
実機（ESP32 + サーボ）での測定は --port を使い、結果を docs/phase2_acceptance.md に転記する。
"""
from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path
from typing import Callable

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config
from serpens.link import messages as m
from serpens.link.harness import LinkHarness
from serpens.link.protocol import Cmd, FrameReader, Nack, State, StopReason, encode

Check = Callable[[dict], tuple[bool, str]]
TRIALS = 20            # 実測のばらつきを見るための回数


def _driving(cfg: dict, **kw) -> LinkHarness:
    h = LinkHarness(cfg)
    assert h.start_driving(**kw), "走行を開始できなかった"
    return h

# ---- 条件ごとの確認 -------------------------------------------------------------------
def c1_gait_on_device(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.advance(1.0)
    spread = max(h.device.goals[n] for n in h.device.mo.body) - min(
        h.device.goals[n] for n in h.device.mo.body)
    sent = len(m.Drive(300, 30.0, 60.0, 0.5, 0.0).pack())
    return h.device.driving and spread > 10.0, f"PC が送るのは {sent} バイトの DRIVE だけ／機体が6軸を生成"


def c2_pc_killed(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.tr.kill_pc()
    took = h.run_until(lambda: not h.device.driving, 2.0)
    h.advance(cfg["link"]["heartbeat_timeout_ms"] / 1000.0)
    moved = h.moved_deg(1.0)
    ok = took is not None and h.device.stop_reason is StopReason.HEARTBEAT_LOST and moved < 1e-9
    return ok, f"模擬: {took * 1000:.0f}ms で保持 → heartbeat 途絶で待機／停止後の移動 {moved:.4f}°"


def c3_usb_unplug(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.tr.unplug()
    took = h.run_until(lambda: not h.device.driving, 2.0)
    h.advance(cfg["link"]["heartbeat_timeout_ms"] / 1000.0)
    ok = took is not None and h.device.stop_reason is StopReason.HEARTBEAT_LOST
    return ok, f"模擬: {took * 1000:.0f}ms で保持（PC 側は再接続しても再開しない）"


def c4_drive_stop(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.client.clear_drive()
    last = h.device._drive_at
    h.run_until(lambda: not h.device.driving, 2.0)
    dt_ms = (h.now - last) * 1000.0
    ok = h.device.stop_reason is StopReason.DRIVE_TTL and h.device.state is State.ARMED
    return ok, f"最後の DRIVE から {dt_ms:.0f}ms（TTL {cfg['link']['drive_ttl_ms']}ms）で保持"


def c5_heartbeat_stop(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.client.hb_dt = 1e9
    took = h.run_until(lambda: not h.device.driving, 2.0)
    ok = took is not None and h.device.stop_reason is StopReason.HEARTBEAT_LOST
    return ok, f"DRIVE は届き続けているが {took * 1000:.0f}ms で停止"


def c6_reboot(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    boot0 = h.device.boot_id
    h.tr.reboot_device(h.now)
    h.advance(3.0)
    ok = not h.device.driving and h.device.state is State.DISARMED and h.client.rebooted
    return ok, f"boot_id {boot0} → {h.device.boot_id}／3 秒待っても走り出さない"


def c7_stale_frames(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    captured = encode(Cmd.DRIVE, h.client._seq, m.Drive(300, 30.0, 60.0, 0.5, 0.0).pack())
    h.client.stop(h.now)
    h.advance(0.2)
    rd = FrameReader()
    reason = m.unpack_nack(rd.feed(h.device.feed(captured, h.now))[0].payload)[2]
    h.tr.duplicate_next = 5                      # 重複も試す
    h.advance(0.5)
    ok = reason == Nack.STALE_SEQ and not h.device.driving
    return ok, f"再送は {Nack(reason).name} で拒否／重複 5 回も走行を再開させない"


def c8_emergency_latch(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.client.emergency(h.now)
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.5)
    ok = h.device.state is State.EMERGENCY and not h.device.driving
    return ok, f"ARM/DRIVE を送っても {h.device.state.name} のまま（NACK: LATCHED）"


def c9_reconnect_keeps_latch(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.client.emergency(h.now)
    h.advance(0.2)
    h.tr.unplug()
    h.advance(1.0)
    h.tr.plug()
    h.client = type(h.client)(h.tr, cfg, now=h.now)      # PC 側を作り直す
    h.advance(1.0)
    return h.device.state is State.EMERGENCY, "USB 抜き差し + PC 再起動でも解除されない"


def c10_clear_fault(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    h.client.emergency(h.now)
    h.advance(0.2)
    h.client.clear_fault(h.now)
    h.advance(0.2)
    disarmed = h.device.state is State.DISARMED
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.5)
    no_move = not h.device.driving
    h.client.arm(h.now)
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.4)
    return disarmed and no_move and h.device.driving, "解除 → 待機。DRIVE だけでは動かず、ARM + DRIVE で再開"


def c11_range_reject(cfg: dict) -> tuple[bool, str]:
    h = LinkHarness(cfg)
    h.advance(0.2)
    h.client.arm(h.now)
    h.advance(0.1)
    bad = [m.Drive(300, 80.0, 60.0, 0.5, 0.0), m.Drive(300, 30.0, 60.0, 3.0, 0.0),
           m.Drive(300, 30.0, 60.0, 0.5, 45.0), m.Drive(0, 30.0, 60.0, 0.5, 0.0)]
    rd, reasons = FrameReader(), []
    for k, d in enumerate(bad):
        rep = rd.feed(h.device.feed(encode(Cmd.DRIVE, h.client._seq + 1 + k, d.pack()), h.now))[0]
        reasons.append(m.unpack_nack(rep.payload)[2])
    head = m.Head(300, 120.0, 0.0, 0.0, 60.0)
    rep = rd.feed(h.device.feed(encode(Cmd.HEAD, h.client._seq + 9, head.pack()), h.now))[0]
    reasons.append(m.unpack_nack(rep.payload)[2])
    ok = all(r == Nack.OUT_OF_RANGE for r in reasons) and not h.device.driving
    return ok, f"振幅・周波数・旋回・TTL・頭部角の {len(reasons)} 件すべて OUT_OF_RANGE で拒否"


def c12_reasons_visible(cfg: dict) -> tuple[bool, str]:
    h = _driving(cfg)
    seen: dict[StopReason, str] = {}

    def snap() -> None:
        if h.client.telemetry:
            seen[h.client.telemetry.stop_reason] = h.client.telemetry.reason_ja

    h.client.stop(h.now)
    h.advance(0.3)
    snap()
    h.client.arm(h.now)
    h.advance(0.1)
    h.client.set_drive(30.0, 60.0, 0.5)
    h.advance(0.4)
    h.client.clear_drive()
    h.advance(0.6)
    snap()
    h.client.hb_dt = 1e9
    h.advance(cfg["link"]["heartbeat_timeout_ms"] / 1000.0 + 0.4)
    snap()
    h.client.hb_dt = 1.0 / cfg["link"]["heartbeat_hz"]
    h.advance(0.3)
    h.client.emergency(h.now)
    h.advance(0.3)
    snap()
    h.client.clear_fault(h.now)
    h.advance(0.3)
    h.device.inject_axis("J3", temp_c=cfg["link"]["faults"]["temp_limit_c"] + 5)
    h.advance(0.3)
    snap()
    want = {StopReason.OPERATOR_STOP, StopReason.DRIVE_TTL, StopReason.HEARTBEAT_LOST,
            StopReason.EMERGENCY_CMD, StopReason.OVERHEAT}
    return want <= set(seen), "／".join(seen[r] for r in want if r in seen)


CHECKS: list[tuple[int, str, Check]] = [
    (1, "PC は DRIVE のみ、歩容は機体が生成", c1_gait_on_device),
    (2, "PC 強制終了 → 機体だけで保持", c2_pc_killed),
    (3, "USB 抜去 → 保持", c3_usb_unplug),
    (4, "DRIVE だけ止める（heartbeat 継続）→ 停止", c4_drive_stop),
    (5, "heartbeat だけ止める → 停止", c5_heartbeat_stop),
    (6, "機体の再起動で自動再開しない", c6_reboot),
    (7, "古い・重複パケットで再開しない", c7_stale_frames),
    (8, "緊急停止を機体側でラッチ", c8_emergency_latch),
    (9, "PC 再接続で解除されない", c9_reconnect_keeps_latch),
    (10, "CLEAR_FAULT は待機へ戻すだけ", c10_clear_fault),
    (11, "上限外の値を機体が拒否", c11_range_reject),
    (12, "停止理由が PC から読める", c12_reasons_visible),
]

# ---- 時間の実測（条件 13 / 14） ---------------------------------------------------------
def measure_link_loss(cfg: dict) -> list[float]:
    """通信断 → 保持までの時間 [ms]。切断の瞬間を1制御周期ずつずらして測る。"""
    out = []
    for k in range(TRIALS):
        h = _driving(cfg, settle_s=0.5 + k * (1.0 / float(cfg["link"]["control_hz"])))
        h.tr.unplug()
        t0 = h.now
        took = h.run_until(lambda: not h.device.driving, 2.0)
        if took is not None:
            out.append((h.now - t0) * 1000.0)
    return out


def measure_heartbeat_only(cfg: dict) -> list[float]:
    """heartbeat だけ途絶（DRIVE は届き続ける）→ 停止までの時間 [ms]。"""
    out = []
    for k in range(TRIALS):
        h = _driving(cfg, settle_s=0.5 + k * (1.0 / float(cfg["link"]["control_hz"])))
        h.client.hb_dt = 1e9
        took = h.run_until(lambda: not h.device.driving, 2.0)
        if took is not None:
            out.append(took * 1000.0)
    return out


def measure_estop(cfg: dict) -> list[float]:
    """緊急停止の指令 → 出力が止まるまでの時間 [ms]（PC が送った時刻から数える）。

    模擬なので中身は「経路1回 + 制御1周期」。**実機ではここにシリアルの往復と
    9軸同期書き込みが乗る。要実測。**
    """
    out = []
    for k in range(TRIALS):
        h = _driving(cfg, settle_s=0.5 + k * (1.0 / float(cfg["link"]["control_hz"])))
        h.client.emergency(h.now)                    # PC が指令を出した時刻
        took = h.run_until(lambda: not h.device.driving, 1.0)
        if took is not None:
            out.append(took * 1000.0)
    return out

def summarize(name: str, values: list[float]) -> str:
    if not values:
        return f"| {name} | 測定できず | | |"
    return (f"| {name} | {statistics.mean(values):.1f} ms | {min(values):.1f} ms | "
            f"{max(values):.1f} ms |")

def main() -> None:
    ap = argparse.ArgumentParser(description="Phase 2 完了条件の確認と時間の実測")
    ap.add_argument("--out", type=Path, default=None, help="結果を Markdown で書き出す")
    ap.add_argument("--port", default=None, help="実機の COM ポート（未検証）")
    args = ap.parse_args()
    if args.port:  # 実機の口はまだ開けない
        raise SystemExit("実機での測定は未実施です。ESP32 と実サーボが揃ってから "
                         "docs/phase2_acceptance.md の手順で行ってください。")

    cfg = load_config()
    lines = ["# Phase 2 リンク確認（模擬機体）", "",
             "偽時計 + 偽経路 + シミュレート ESP32 による確認。**実機の実測値ではない。**", "",
             "| 条件 | 内容 | 結果 | 備考 |", "|---|---|---|---|"]
    all_ok = True
    for num, title, fn in CHECKS:
        ok, note = fn(cfg)
        all_ok &= ok
        lines.append(f"| {num} | {title} | {'OK' if ok else '**NG**'} | {note} |")

    lines += ["", "## 時間の実測（模擬。条件 13 / 14）", "",
              f"制御周期 {cfg['link']['control_hz']:.0f}Hz、heartbeat {cfg['link']['heartbeat_hz']:.0f}Hz、"
              f"TTL {cfg['link']['drive_ttl_ms']}ms、タイムアウト {cfg['link']['heartbeat_timeout_ms']}ms。"
              f"{TRIALS} 回。", "", "| 測定 | 平均 | 最小 | 最大 |", "|---|---|---|---|",
              summarize("13 通信断（USB 抜去）→ 保持", measure_link_loss(cfg)),
              summarize("13b heartbeat のみ途絶 → 停止", measure_heartbeat_only(cfg)),
              summarize("14 緊急停止の指令 → 出力停止", measure_estop(cfg)), "",
              "13 は DRIVE の TTL が先に効くので TTL 相当。13b は heartbeat タイムアウト相当。", ""]
    lines += [
              "実機ではこれにシリアルの往復と9軸同期書き込みの時間が乗る。**要実測。**", "",
              "## 条件 15（実サーボ）", "", "**未実施。** 実機が無い。記録欄は docs/phase2_acceptance.md。"]
    text = "\n".join(lines)
    print(text)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
        print(f"\n{args.out} に書きました。")
    raise SystemExit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
