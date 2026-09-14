"""故障注入。**シミュレーション専用**で、実機のコードには存在しない。

実機が無い今だからこそ、確かめたいのは「正常時に動く」ことではなく
**「異常時に勝手に走り続けない」**こと。注入できるのは4層:

| 層 | 注入するもの | どこで |
|---|---|---|
| 経路 | 切断 / PC 停止 / packet loss / delay / 順序入れ替え / 重複 / CRC 破損 / 分割 | `FaultInjector`（この file） |
| 機体 | 再起動 / 制御周期の超過 | `LoopbackTransport.reboot_device` / `SimulatedDevice` |
| サーボ | 応答なし / 過熱 / 過負荷 / fault ビット | `SimulatedDevice.inject_axis` |
| 指令値 | 範囲外の角度・速度 / NaN / Inf | PC 側（`LinkClient` が送る前に弾く） |

乱数は種を固定できる（テストが再現する）。
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


@dataclass
class FaultInjector:
    """経路に起こす異常。`LoopbackTransport` が書き込みのたびに通す。"""

    drop_next: int = 0              # 次の n フレームを落とす
    drop_ratio: float = 0.0         # 各フレームをこの確率で落とす（0〜1）
    duplicate_next: int = 0         # 次の n フレームを二重に届ける
    corrupt_next: int = 0           # 次の n フレームの末尾 1 バイトを壊す（CRC 不一致になる）
    delay_s: float = 0.0            # 届くまでの遅れ
    reorder_next: int = 0           # 次の n フレームを、その次のフレームより後に届ける
    rng: random.Random = field(default_factory=lambda: random.Random(0))
    dropped: int = 0                # 実際に落とした数（確認用）
    duplicated: int = 0
    corrupted: int = 0
    reordered: int = 0

    def on_write(self, data: bytes, now: float) -> list[tuple[float, bytes]]:
        """1回の書き込みを、(届く時刻, バイト列) の列に変える。空なら消えたということ。"""
        if self.drop_next > 0:
            self.drop_next -= 1
            self.dropped += 1
            return []
        if self.drop_ratio > 0.0 and self.rng.random() < self.drop_ratio:
            self.dropped += 1
            return []
        if self.corrupt_next > 0 and data:
            self.corrupt_next -= 1
            self.corrupted += 1
            data = data[:-1] + bytes([data[-1] ^ 0xFF])
        at = now + self.delay_s
        if self.reorder_next > 0:
            self.reorder_next -= 1
            self.reordered += 1
            at += max(self.delay_s, 0.0) + REORDER_GAP_S    # 次のフレームより後に届く
        out = [(at, data)]
        if self.duplicate_next > 0:
            self.duplicate_next -= 1
            self.duplicated += 1
            out.append((at, data))
        return out

    def clear(self) -> None:
        """注入をすべて解除する（数えた結果は残す）。"""
        self.drop_next = self.duplicate_next = self.corrupt_next = self.reorder_next = 0
        self.drop_ratio = 0.0
        self.delay_s = 0.0


REORDER_GAP_S = 0.15        # 順序入れ替えで後ろへ回す量（PC の送信間隔 100ms より長くする）


def finite(*values: float) -> bool:
    """NaN / Inf が混ざっていないか。**通信へ出す前に必ず通す。**

    `struct.pack` へ NaN を渡すと `round()` が例外になり、通信スレッドごと落ちる。
    そうなる前に、指令そのものを拒否する。
    """
    return all(isinstance(v, (int, float)) and math.isfinite(float(v)) for v in values)
