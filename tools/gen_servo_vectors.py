"""ファームのサーボ層（`firmware/serpens_esp32/servo_bus.h`）の**正解のバイト列**を、ベンダーの SDK（`scservo_sdk` = ftservo-python-sdk。PC 側の `serpens/hw/feetech_bus.py` が使うもの）から取り出して、
`firmware/serpens_esp32/servo_bus_vectors.h` に書く。C++ は `sbSelfTest()` がこの正解と自分の出力を比べる（**実機・別の C++ コンパイラ環境で走らせる**。この PC には C++ コンパイラが無く、ESP32 向けのコンパイルだけができる）。

    python tools/gen_servo_vectors.py            # ヘッダを書く
    python tools/gen_servo_vectors.py --check    # ヘッダが SDK の出力と一致するかだけ調べる（違えば終了コード 1）

出典: scservo_sdk（`sms_sts.py` / `protocol_packet_handler.py` / `group_sync_write.py`）。**SDK の送信バイト列が正解**（STS3215 のレジスタは `docs/sts3215_registers.md`）。
この道具は実機に接続しない（偽のポートに書かせて、書かれたバイトを拾うだけ）。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "firmware" / "serpens_esp32" / "servo_bus_vectors.h"

# ---- ベクトルの入力（ファームの sbSelfTest と同じ値を、ヘッダに書き出す）----
WRITE1 = {"id": 3, "addr": 40, "val": 1}                                    # トルクスイッチ ON
WRITE2 = {"id": 7, "addr": 48, "val": 167}                                  # トルク制限（0.167 × 1000）
READ_STATE = {"id": 5, "addr": 56, "len": 8}                                # 位置・速度・負荷・電圧・温度
SYNC = {"ids": [1, 2, 3], "pos": [2047, 1500, 3000], "speed": [3000, 1200, 500], "acc": 254}
STATUS_ID = 5
STATUS_DATA = [0xFF, 0x07, 0x00, 0x00, 0x10, 0x00, 74, 31]                 # 位置 2047、速度 0、負荷 16、電圧 7.4 V、温度 31 ℃（SDK の読み出しで同じ値が戻ることを試験）


class FakePort:
    """scservo_sdk の PortHandler の代わり。書かれたバイトを集め、決めた応答を返す。実機には何も出さない。"""

    def __init__(self, reply: bytes = b"") -> None:
        self.is_using = False
        self.sent: list[list[int]] = []
        self.reply = list(reply)
        self.baudrate = 1000000
        self.packet_timeout = 0

    def clearPort(self) -> None:
        pass

    def writePort(self, pkt: list[int]) -> int:
        self.sent.append(list(pkt))
        return len(pkt)

    def readPort(self, n: int) -> list[int]:
        out, self.reply = self.reply[:n], self.reply[n:]
        return out

    def setPacketTimeout(self, n: int) -> None:
        pass

    def isPacketTimeout(self) -> bool:
        return not self.reply

    def getBaudRate(self) -> int:
        return self.baudrate

    def setPacketTimeoutMillis(self, ms: float) -> None:
        pass


def status_packet(servo_id: int, data: list[int], err: int = 0) -> list[int]:
    """サーボの応答（FF FF ID LEN ERR data... CHK。LEN = len(data) + 2。チェックサム = ~(ID + LEN + ERR + data) & 0xFF）。"""
    body = [servo_id, len(data) + 2, err] + data
    return [0xFF, 0xFF] + body + [(~sum(body)) & 0xFF]


def sdk_vectors() -> dict[str, list[int]]:
    from scservo_sdk import sms_sts  # type: ignore[import-untyped]
    v: dict[str, list[int]] = {}
    p = FakePort()
    ph = sms_sts(p)
    ph.write1ByteTxRx(WRITE1["id"], WRITE1["addr"], WRITE1["val"])
    v["WRITE1"] = p.sent[-1]
    ph.write2ByteTxRx(WRITE2["id"], WRITE2["addr"], WRITE2["val"])
    v["WRITE2"] = p.sent[-1]
    ph.readTx(READ_STATE["id"], READ_STATE["addr"], READ_STATE["len"])
    v["READ_STATE"] = p.sent[-1]
    p.is_using = False                                  # readTx は応答を待たないので、SDK の「ポート使用中」が残る。解いてから送る
    n_before = len(p.sent)
    for sid, pos, spd in zip(SYNC["ids"], SYNC["pos"], SYNC["speed"]):
        ph.SyncWritePosEx(sid, pos, spd, SYNC["acc"])
    ph.groupSyncWrite.txPacket()
    if len(p.sent) != n_before + 1:
        raise SystemExit("SYNC_WRITE が SDK から送信されなかった")
    ph.groupSyncWrite.clearParam()
    v["SYNC_WRITE"] = p.sent[-1]
    v["STATUS"] = status_packet(STATUS_ID, STATUS_DATA)
    return v


def check_status_with_sdk() -> list[int]:
    """SDK に応答を読ませ、返ってくる data が STATUS_DATA と同じことを確かめる（正解の応答の形の裏づけ）。"""
    from scservo_sdk import sms_sts  # type: ignore[import-untyped]
    p = FakePort(bytes(status_packet(STATUS_ID, STATUS_DATA)))
    ph = sms_sts(p)
    data, result, err = ph.readTxRx(STATUS_ID, READ_STATE["addr"], READ_STATE["len"])
    if result != 0 or err != 0:
        raise SystemExit(f"SDK が応答を読めない: result={result} err={err}")
    return list(data)


def render(v: dict[str, list[int]]) -> str:
    def arr(name: str, b: list[int]) -> str:
        return f"static const uint8_t SBV_{name}[] = {{" + ", ".join(f"0x{x:02X}" for x in b) + "};\n" + f"static const size_t  SBV_{name}_LEN = {len(b)};\n"
    L = ["// **自動生成**: tools/gen_servo_vectors.py（ベンダー SDK scservo_sdk の送信バイト列が正解）。手で書き換えない。",
         "// 実機・別の C++ 環境で `sbSelfTest()`（servo_bus.h）がこの正解と自分の出力を比べる。HARDWARE_UNVERIFIED。",
         "#pragma once", "#include <stdint.h>", "#include <stddef.h>", "",
         f"// WRITE1: id={WRITE1['id']} addr={WRITE1['addr']} val={WRITE1['val']}（トルクスイッチ ON）",
         f"#define SBV_W1_ID {WRITE1['id']}", f"#define SBV_W1_ADDR {WRITE1['addr']}", f"#define SBV_W1_VAL {WRITE1['val']}", arr("WRITE1", v["WRITE1"]),
         f"// WRITE2: id={WRITE2['id']} addr={WRITE2['addr']} val={WRITE2['val']}（トルク制限 = 0.167 × 1000）",
         f"#define SBV_W2_ID {WRITE2['id']}", f"#define SBV_W2_ADDR {WRITE2['addr']}", f"#define SBV_W2_VAL {WRITE2['val']}", arr("WRITE2", v["WRITE2"]),
         f"// READ_STATE: id={READ_STATE['id']} addr={READ_STATE['addr']} len={READ_STATE['len']}",
         f"#define SBV_R_ID {READ_STATE['id']}", f"#define SBV_R_ADDR {READ_STATE['addr']}", f"#define SBV_R_LEN {READ_STATE['len']}", arr("READ_STATE", v["READ_STATE"]),
         "// SYNC_WRITE（位置・速度・加速度。3 軸）",
         f"#define SBV_SYNC_N {len(SYNC['ids'])}", f"#define SBV_SYNC_ACC {SYNC['acc']}",
         "static const uint8_t  SBV_SYNC_IDS[]   = {" + ", ".join(str(x) for x in SYNC["ids"]) + "};",
         "static const uint16_t SBV_SYNC_POS[]   = {" + ", ".join(str(x) for x in SYNC["pos"]) + "};",
         "static const uint16_t SBV_SYNC_SPEED[] = {" + ", ".join(str(x) for x in SYNC["speed"]) + "};",
         arr("SYNC_WRITE", v["SYNC_WRITE"]),
         f"// 応答（status）: id={STATUS_ID}、data = 位置 2047・速度 0・負荷 16・電圧 7.4 V・温度 31 ℃（SDK の readTxRx が同じ data を返すことを tests/test_servo_vectors.py で確認）",
         f"#define SBV_STATUS_ID {STATUS_ID}",
         arr("STATUS", v["STATUS"]),
         "static const uint8_t SBV_STATUS_DATA[] = {" + ", ".join(f"0x{x:02X}" for x in STATUS_DATA) + "};", ""]
    return "\n".join(L)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    v = sdk_vectors()
    got = check_status_with_sdk()
    if got != STATUS_DATA:
        print("SDK が読んだ応答の data が STATUS_DATA と違う", got, STATUS_DATA)
        return 1
    text = render(v)
    if a.check:
        cur = OUT.read_text(encoding="utf-8").replace("\r\n", "\n") if OUT.exists() else ""
        ok = cur == text
        print("一致" if ok else "古い（python tools/gen_servo_vectors.py で更新）")
        return 0 if ok else 1
    OUT.write_text(text, encoding="utf-8", newline="\n")
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
