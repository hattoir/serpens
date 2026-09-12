"""実機が届いた日に最初に使う、対話式のサーボ設定ツール。

  python tools/servo_setup.py                      モックで練習（実機なしでも全メニューが動く）
  python tools/servo_setup.py --bus feetech --port COM5

メニュー:
  1 ポートの一覧と自動検出          5 温度・電圧・負荷のライブ表示（1Hz, Ctrl+C で戻る）
  2 サーボのスキャン                6 現在負荷の符号ビットの検証（仮説 bit10 の確認）
  3 入力電圧の設定（最優先）        7 SYNC READ に対応しているかの判定
  4 ID の一括設定（1→9）            8 中立位置での保持（ホーン取付の補助）

**サーボが PING に応答するのに動かないときは、まず 3 番を見てください。**
資料の初期値は最高入力電圧 8.0V です。12V 版に 12V を入れても、この値が 8.0V のままだと
過電圧保護が働いて一切動かない可能性があります（docs/sts3215_registers.md §6）。
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from serpens.config import load_config  # noqa: E402
from serpens.hw import registers as reg  # noqa: E402
from serpens.hw.servo_bus import ServoBus, ServoCommError, make_bus  # noqa: E402

SIGN_TEST_DEG = 25.0          # 符号ビットの検証で動かす角度
SIGN_TEST_WAIT_S = 1.2
LIVE_PERIOD_S = 1.0
MENU = """
  1) ポートの一覧と自動検出        5) 温度・電圧・負荷のライブ表示
  2) サーボのスキャン              6) 現在負荷の符号ビットの検証
  3) 入力電圧の設定（最優先）      7) SYNC READ の対応判定
  4) ID の一括設定                 8) 中立位置での保持（ホーン取付）
  0) 終了
"""


def ask(prompt: str, default: str = "") -> str:
    """入力を1行もらう（Ctrl+C / EOF で空文字）。"""
    try:
        s = input(prompt).strip()
    except (EOFError, KeyboardInterrupt):
        return ""
    return s or default


def list_ports() -> None:
    """COM ポートを並べる。"""
    from serial.tools import list_ports as lp

    ports = list(lp.comports())
    if not ports:
        print("  COM ポートが見つかりません（USB を挿してから再実行）")
        return
    for p in ports:
        print(f"  {p.device:10} {p.description}　（{p.hwid}）")
    print("  サーボ側と頭部（XIAO ESP32S3）で2本あります。1本ずつ抜き差しして確かめてください。")


def scan(bus: ServoBus, cfg: dict) -> list[int]:
    """応答する ID を探して表示する。"""
    text = ask("  探す ID の範囲（既定 1-20、全部なら 1-253）: ", "1-20")
    try:
        lo, hi = (int(v) for v in text.split("-"))
    except ValueError:
        print("  範囲の書き方: 1-20")
        return []
    print(f"  ID {lo}〜{hi} を {cfg['servo']['baudrate']}bps で探します…")
    found = bus.scan(list(range(lo, hi + 1)))
    print(f"  応答: {found if found else 'なし'}")
    if not found:
        print("  見つからないとき: 電源（12V）、配線、ボーレート、他のソフトが COM を掴んでいないか を確認")
    for sid in found:
        model = bus.read_register(sid, reg.MODEL.addr, reg.MODEL.size)
        print(f"    ID {sid:3}  型番 {model}")
    return found


def show_voltage_settings(bus: ServoBus, ids: list[int], supply_v: float) -> None:
    """EEPROM の電圧・温度・トルク設定を表示し、危ないものを指摘する。"""
    print(f"  電源電圧の想定: {supply_v:.1f}V")
    print(f"  {'ID':>4} " + " ".join(f"{r.name:>10}" for r in reg.STARTUP_CHECK) + "   状態")
    for sid in ids:
        vals = [bus.read_register(sid, r.addr, r.size) for r in reg.STARTUP_CHECK]
        cells = " ".join(f"{(r.describe(v) if v is not None else '—'):>10}" for r, v in zip(reg.STARTUP_CHECK, vals))
        status = bus.read_register(sid, reg.STATUS.addr, reg.STATUS.size)
        print(f"  {sid:>4} {cells}   {reg.status_text(status or 0)}")
        vmax = vals[reg.STARTUP_CHECK.index(reg.MAX_VOLTAGE)]
        if vmax is not None and vmax * 0.1 < supply_v:
            print(f"       ⚠ 最高入力電圧 {vmax * 0.1:.1f}V < 電源 {supply_v:.1f}V。"
                  "過電圧保護で動かない可能性があります（このメニューで書き換えてください）")


def set_voltage_limits(bus: ServoBus, ids: list[int], cfg: dict) -> None:
    """最高・最低入力電圧を書き換える（EEPROM）。"""
    supply = float(cfg["servo"]["supply_voltage_v"])
    show_voltage_settings(bus, ids, supply)
    print("  ※ PING に応答するのに動かない場合は、まずここを疑ってください")
    new_max = ask(f"  新しい最高入力電圧 [V]（空欄で変更しない。推奨 {supply + 2:.1f}）: ")
    new_min = ask("  新しい最低入力電圧 [V]（空欄で変更しない）: ")
    for label, text, register in (("最高", new_max, reg.MAX_VOLTAGE), ("最低", new_min, reg.MIN_VOLTAGE)):
        if not text:
            continue
        raw = int(round(float(text) * 10))
        for sid in ids:
            ok = bus.write_register(sid, register.addr, register.size, raw, eeprom=True)
            got = bus.read_register(sid, register.addr, register.size)
            mark = "OK" if ok and got == raw else "失敗"
            print(f"    ID {sid:3} {label}入力電圧 → {register.describe(got or 0)}　{mark}")
    print("  書き込み後は一度電源を入れ直して、値が残っているか確認してください")


def assign_ids(bus: ServoBus, cfg: dict) -> None:
    """1個ずつつないで ID を振る（同じ ID が2個あるとバスが壊れるため）。"""
    targets = [int(j["servo_id"]) for j in cfg["joints"]]
    print(f"  {targets[0]}〜{targets[-1]} を順に振ります。**サーボは1個だけつないでください**")
    for target in targets:
        ans = ask(f"  ID {target} にするサーボを1個だけ繋いで Enter（s で飛ばす、q で中断）: ").lower()
        if ans == "q":
            print("    中断しました（ID は書き換えていません）")
            return
        if ans == "s":
            print(f"    ID {target} は飛ばしました")
            continue
        if ans != "":
            print("    Enter / s / q のどれかを入れてください")
            continue
        found = bus.scan(list(range(1, 21)))
        if len(found) != 1:
            print(f"    応答 {found}。1個だけになるようにしてください")
            continue
        old = found[0]
        if old == target:
            print(f"    すでに ID {target} です")
            continue
        print(f"    ID {old} → {target}: {'OK' if bus.set_servo_id(old, target) else '失敗'}")


def live_view(bus: ServoBus, ids: list[int], limit_c: float) -> None:
    """温度・電圧・負荷を 1Hz で表示する。"""
    print("  Ctrl+C で戻ります")
    try:
        while True:
            states = bus.sync_read_states(ids)
            line = []
            for sid in ids:
                st = states.get(sid)
                if st is None:
                    line.append(f"ID{sid}:—")
                    continue
                hot = "!" if st.temp_c >= limit_c * 0.9 else " "
                line.append(f"ID{sid}:{st.temp_c:4.1f}℃{hot}{st.volt:5.1f}V {st.load:+5.2f}")
            print("  " + "  ".join(line))
            time.sleep(LIVE_PERIOD_S)
    except KeyboardInterrupt:
        print("\n  戻ります")


def verify_load_sign(bus: ServoBus, sid: int, cfg: dict) -> None:
    """現在負荷の符号ビット（仮説 bit10）を実機で確かめる。"""
    bit = cfg["servo"]["load_sign_bit"]
    print(f"  ID {sid} を ±{SIGN_TEST_DEG:.0f}° 動かして、負荷の生値を見ます（仮説: 下位{bit}bit=大きさ, bit{bit}=方向）")
    print("  ※ 動いている間に軽く手で押さえると、はっきり出ます")
    bus.set_torque(sid, True)
    raw = {}
    for label, deg in (("＋方向", SIGN_TEST_DEG), ("−方向", -SIGN_TEST_DEG)):
        bus.set_goal(sid, deg, 60.0, 500.0)
        time.sleep(SIGN_TEST_WAIT_S)
        value = bus.read_register(sid, reg.PRESENT_LOAD.addr, reg.PRESENT_LOAD.size) or 0
        raw[label] = value
        print(f"    {label}: 生値 {value:5d} (0x{value:04X})  bit{bit}={'1' if value & (1 << bit) else '0'}  "
              f"大きさ {value & ((1 << bit) - 1)}")
    bus.set_goal(sid, 0.0, 60.0, 500.0)
    time.sleep(SIGN_TEST_WAIT_S)
    plus, minus = raw["＋方向"], raw["−方向"]
    same_bit = bool(plus & (1 << bit)) == bool(minus & (1 << bit))
    if same_bit:
        print("  → bit が変わりませんでした。仮説は誤りか、負荷が小さすぎます。"
              "手で押さえて再実行し、それでも変わらなければ config の load_sign_bit を null にしてください")
    else:
        cw = "＋方向" if not (plus & (1 << bit)) else "−方向"
        print(f"  → 仮説どおり bit{bit} が方向を表しています（bit=0 は {cw}）。"
              "この結果を docs/sts3215_registers.md §5 に追記してください")


def hold_center(bus: ServoBus, ids: list[int], cfg: dict) -> None:
    """全軸を中立（2047 step = 関節角 0°）で保持して、ホーンを付けられるようにする。"""
    print(f"  全軸を中立（{cfg['servo']['center_step']} step）で保持します。ホーンを付けてから Enter を押してください")
    for sid in ids:
        bus.set_torque(sid, True)
        bus.set_goal(sid, 0.0, 30.0, 300.0)
    ask("  終わったら Enter（トルクを切ります）: ")
    for sid in ids:
        bus.set_torque(sid, False)
    print("  トルクを切りました")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bus", choices=["mock", "feetech"], default="mock")
    ap.add_argument("--port", default=None, help="実機の COM ポート（例: COM5）")
    args = ap.parse_args()
    cfg = load_config()
    bus = make_bus(args.bus, cfg, args.port)
    bus.connect()
    ids = bus.ids
    print(f"接続しました（{args.bus}{'' if args.port is None else ' ' + args.port}）")
    if args.bus == "mock":
        print("※ モックです。実機と同じ手順を練習できます（電圧の初期値 8.0V も再現しています）")
    try:
        while True:
            print(MENU)
            choice = ask("  番号: ")
            if choice in ("0", "q", ""):
                break
            try:
                if choice == "1":
                    list_ports()
                elif choice == "2":
                    ids = scan(bus, cfg) or ids
                elif choice == "3":
                    set_voltage_limits(bus, ids, cfg)
                elif choice == "4":
                    assign_ids(bus, cfg)
                elif choice == "5":
                    live_view(bus, ids, float(cfg["servo"]["temperature_limit_c"]))
                elif choice == "6":
                    verify_load_sign(bus, int(ask(f"  試す ID（既定 {ids[0]}）: ", str(ids[0]))), cfg)
                elif choice == "7":
                    ok = bus.supports_sync_read()
                    print(f"  SYNC READ: {'対応しています' if ok else '応答がありません'}")
                    print("  " + ("そのまま使います（位置は 50Hz で読めます）" if ok else
                                  "個別 READ に自動で切り替わります。位置の読み出しは 10Hz、温度は 1Hz に落ちます"))
                elif choice == "8":
                    hold_center(bus, ids, cfg)
                else:
                    print("  1〜8 か 0 を入れてください")
            except ServoCommError as e:
                print(f"  通信エラー: {e}")
    finally:
        bus.disconnect()
        print("切断しました")
    return 0


if __name__ == "__main__":
    sys.exit(main())
