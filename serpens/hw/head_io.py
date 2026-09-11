"""頭部 I/O（XIAO ESP32S3: ToF・タッチ×2・目の LED）との通信。

プロトコル（USB CDC, 115200bps, 行指向 ASCII, LF 終端）:
  ESP32 → PC (10Hz):  S,<uptime_ms>,<tof_mm>,<touch_head>,<touch_back>,<seq>
  PC → ESP32:         E,<r>,<g>,<b>,<brightness>   /  M,<mode>  /  P
  ping 応答:          P,ok,<fw_version>

規約:
  - 不正な行は黙って捨てる（例外を投げない）
  - PC 側は S 行が link_timeout_s 来なければ「頭部I/O断」とするが、動作は止めない
  - ESP32-S3 はリセットでポートが消えるので SerialHeadIO は自動再接続する
  - MockHeadIO も format_sensor_line() で同じ行を作り、同じパーサに通す
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

log = logging.getLogger(__name__)
Clock = Callable[[], float]

TOUCH_VALUES = ("0", "1")
BYTE_MAX = 255


@dataclass(frozen=True)
class SensorFrame:
    """S 行1本分。"""

    uptime_ms: int
    tof_mm: int          # 0 = 測定不能
    touch_head: bool
    touch_back: bool
    seq: int


# =============================================================================
# 行のパース・生成（純粋関数）
# =============================================================================
def parse_sensor_line(line: str, tof_max_mm: int, seq_modulo: int) -> SensorFrame | None:
    """S 行をパースする。不正なら None（例外は投げない）。"""
    parts = line.strip().split(",")
    if len(parts) != 6 or parts[0] != "S":
        return None
    _, up, tof, th, tb, seq = parts
    if th not in TOUCH_VALUES or tb not in TOUCH_VALUES:
        return None
    try:
        up_i, tof_i, seq_i = int(up), int(tof), int(seq)
    except ValueError:
        return None
    if up_i < 0 or not 0 <= tof_i <= tof_max_mm or not 0 <= seq_i < seq_modulo:
        return None
    return SensorFrame(up_i, tof_i, th == "1", tb == "1", seq_i)


def format_sensor_line(f: SensorFrame) -> str:
    """S 行を作る（MockHeadIO とテスト用。ESP32 ファームと同じ形式）。"""
    return f"S,{f.uptime_ms},{f.tof_mm},{int(f.touch_head)},{int(f.touch_back)},{f.seq}\n"


def parse_ping_reply(line: str) -> str | None:
    """'P,ok,<fw>' なら fw_version を返す。"""
    parts = line.strip().split(",", 2)
    if len(parts) == 3 and parts[0] == "P" and parts[1] == "ok" and parts[2]:
        return parts[2]
    return None


def _byte(v: int) -> int:
    return min(max(int(v), 0), BYTE_MAX)


def format_eye(r: int, g: int, b: int, brightness: int) -> str:
    """E 行（目の色と明るさ）。"""
    return f"E,{_byte(r)},{_byte(g)},{_byte(b)},{_byte(brightness)}\n"


def format_mode(mode: int) -> str:
    """M 行（0=消灯 1=点灯 2=呼吸 3=まばたき）。"""
    return f"M,{int(mode)}\n"


def format_ping() -> str:
    """P 行。"""
    return "P\n"


# =============================================================================
# 共通部分
# =============================================================================
class HeadIO:
    """頭部 I/O の共通処理。受信行の解釈とリンク状態の管理を行う。"""

    def __init__(self, cfg: dict[str, Any], clock: Clock | None = None) -> None:
        h = cfg["head_io"]
        self._h = h
        self._clock: Clock = clock or time.monotonic
        self._tof_max = int(h["tof_max_mm"])
        self._seq_mod = int(h["seq_modulo"])
        self._timeout = float(h["link_timeout_s"])
        self._lock = threading.Lock()
        self.latest: SensorFrame | None = None
        self.fw_version: str | None = None
        self._last_rx_t: float | None = None
        self.dropped_lines = 0

    def _feed_line(self, line: str) -> None:
        """受信した1行を解釈する。不正な行は数えて捨てる。"""
        frame = parse_sensor_line(line, self._tof_max, self._seq_mod)
        with self._lock:
            if frame is not None:
                self.latest = frame
                self._last_rx_t = self._clock()
                return
            fw = parse_ping_reply(line)
            if fw is not None:
                self.fw_version = fw
            else:
                self.dropped_lines += 1

    def link_ok(self) -> bool:
        """最後の S 行から link_timeout_s 以内なら True。"""
        self.poll()
        with self._lock:
            return self._last_rx_t is not None and self._clock() - self._last_rx_t <= self._timeout

    def poll(self) -> None:
        """受信処理を進める（スレッドで受信する実装では何もしない）。"""

    # ---- 送信 -----------------------------------------------------------------
    def set_eye(self, r: int, g: int, b: int, brightness: int) -> None:
        """目の色と明るさ。"""
        self._send(format_eye(r, g, b, brightness))

    def set_mode(self, mode: int) -> None:
        """目のモード。"""
        self._send(format_mode(mode))

    def ping(self) -> None:
        """ping を送る。応答は fw_version に入る。"""
        self._send(format_ping())

    def _send(self, line: str) -> None:
        raise NotImplementedError

    def close(self) -> None:
        """終了処理。"""


# =============================================================================
# 実機
# =============================================================================
class SerialHeadIO(HeadIO):
    """pyserial で XIAO ESP32S3 と通信する。ポートが消えても自動再接続する。"""

    def __init__(self, cfg: dict[str, Any], port: str, clock: Clock | None = None) -> None:
        super().__init__(cfg, clock)
        self._port_name = port
        self._ser: Any = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="head-io", daemon=True)
        self._thread.start()

    def _open(self) -> bool:
        import serial  # 遅延 import（モックだけ使うときは不要）

        try:
            self._ser = serial.Serial(self._port_name, int(self._h["baudrate"]),
                                      timeout=float(self._h["read_timeout_s"]))
            log.info("head I/O connected: %s", self._port_name)
            return True
        except (serial.SerialException, OSError) as e:
            log.debug("head I/O open failed: %s", e)
            self._ser = None
            return False

    def _run(self) -> None:
        """受信スレッド。例外で落ちず、切断されたら再接続を繰り返す。"""
        buf = b""
        while not self._stop.is_set():
            if self._ser is None and not self._open():
                self._stop.wait(float(self._h["reconnect_interval_s"]))
                continue
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
            except Exception as e:  # noqa: BLE001 - USB 抜けは何が来ても再接続
                log.warning("head I/O lost: %s", e)
                self._close_port()
                continue
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                self._feed_line(raw.decode("ascii", errors="replace"))

    def _send(self, line: str) -> None:
        ser = self._ser
        if ser is None:
            return  # 未接続なら捨てる（動作は継続）
        try:
            ser.write(line.encode("ascii"))
        except Exception as e:  # noqa: BLE001
            log.warning("head I/O write failed: %s", e)
            self._close_port()

    def _close_port(self) -> None:
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001
                pass

    def close(self) -> None:
        """受信スレッドを止めてポートを閉じる。"""
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._close_port()


# =============================================================================
# モック（ESP32 ファームの振る舞いを模擬）
# =============================================================================
class MockHeadIO(HeadIO):
    """ESP32 側の振る舞いを模擬する。poll() のたびに経過時間ぶんの S 行を生成する。"""

    def __init__(self, cfg: dict[str, Any], clock: Clock | None = None) -> None:
        super().__init__(cfg, clock)
        m = self._h["mock"]
        self._fw = str(m["fw_version"])
        self._period = 1.0 / float(self._h["sensor_rate_hz"])
        self._failsafe_s = float(self._h["esp32_failsafe_s"])
        self._modes: dict[str, int] = dict(self._h["eye_modes"])
        self._t0 = self._clock()
        self._next_tx = self._t0
        self._seq = 0
        self._last_cmd_t = self._t0
        self.tof_mm = int(m["tof_idle_mm"])
        self.touch_head = False
        self.touch_back = False
        self.connected = True           # False にすると S 行が止まる（断線テスト用）
        self.eye_rgb_brightness = (0, 0, 0, 0)
        self.eye_mode = self._modes["breath"]
        self.tx_log: list[str] = []     # PC → ESP32 に送られた行

    def poll(self) -> None:
        """経過時間ぶんの S 行を生成し、ESP32 側フェイルセーフも評価する。"""
        now = self._clock()
        while self._next_tx <= now:
            if self.connected:
                uptime = int((self._next_tx - self._t0) * 1000)
                f = SensorFrame(uptime, self.tof_mm, self.touch_head, self.touch_back, self._seq)
                self._seq = (self._seq + 1) % self._seq_mod
                self._feed_line(format_sensor_line(f))
            self._next_tx += self._period
        if now - self._last_cmd_t > self._failsafe_s:
            self.eye_mode = self._modes["breath"]

    def _send(self, line: str) -> None:
        """ESP32 が受け取ったものとして解釈する。"""
        self.poll()
        self.tx_log.append(line)
        if not self.connected:
            return
        self._last_cmd_t = self._clock()
        parts = line.strip().split(",")
        if parts[0] == "E" and len(parts) == 5:
            r, g, b, br = (int(x) for x in parts[1:])
            self.eye_rgb_brightness = (r, g, b, br)
        elif parts[0] == "M" and len(parts) == 2:
            self.eye_mode = int(parts[1])
        elif parts == ["P"]:
            self._feed_line(f"P,ok,{self._fw}\n")
