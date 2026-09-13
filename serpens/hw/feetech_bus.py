"""実機用: Feetech STS3215 を ftservo-python-sdk（scservo_sdk）で操作する。

レジスタ番地・単位・バイト順の根拠はすべて docs/sts3215_registers.md。
**この段階では実機テスト未実施。** 実機が届いたら tools/servo_setup.py で1軸ずつ確認すること。
"""
from __future__ import annotations

import logging
import math
import threading
from typing import Any, Callable

from scservo_sdk import (  # type: ignore[import-untyped]
    COMM_SUCCESS,
    SMS_STS_PRESENT_POSITION_L,
    SMS_STS_TORQUE_ENABLE,
    GroupSyncRead,
    PortHandler,
    sms_sts,
)

from serpens.hw.servo_bus import Goal, JointSpec, ServoBus, ServoCommError, ServoState

log = logging.getLogger(__name__)

# --- レジスタ（docs/sts3215_registers.md §4）。SDK に定数が無いものだけ自前で定義 ---
ADDR_TORQUE_LIMIT = 48          # トルク制限 2byte, 0〜1000
ADDR_LOCK = 55                  # EEPROM 書き込みロック（0 = 保存する / 1 = 保存しない）
ADDR_ID = 5
# 状態の一括読み出し: 56〜63 = 位置2, 速度2, 負荷2, 電圧1, 温度1
READ_BLOCK_START = SMS_STS_PRESENT_POSITION_L
READ_BLOCK_LEN = 8
POSITION_LEN = 2
OFS_POS, OFS_SPEED, OFS_LOAD, OFS_VOLT, OFS_TEMP = 0, 2, 4, 6, 7
# 現在位置の符号ビット（SDK の ReadPos と同じ扱い。docs §5）
POSITION_SIGN_BIT = 15


# =============================================================================
# 単位変換（純粋関数。実機なしでテストできる）
# =============================================================================
def deg_to_step(deg: float, joint: JointSpec, s: dict[str, Any]) -> int:
    """関節角 [deg] → 目標位置 [step]。servo角 = direction × 関節角 + ホーン取付角オフセット。"""
    servo_deg = joint.direction * deg + joint.horn_offset_deg
    step = int(s["center_step"]) + round(servo_deg * int(s["steps_per_rev"]) / 360.0)
    return min(max(step, int(s["step_min"])), int(s["step_max"]))


def step_to_deg(step: int, joint: JointSpec, s: dict[str, Any]) -> float:
    """現在位置 [step] → 関節角 [deg]。deg_to_step の逆変換。"""
    servo_deg = (step - int(s["center_step"])) * 360.0 / int(s["steps_per_rev"])
    return (servo_deg - joint.horn_offset_deg) / joint.direction


def dps_to_step_s(speed_dps: float, s: dict[str, Any]) -> int:
    """速度 [deg/s] → 運転速度レジスタ [step/s]。0 は書かない（docs §6-3）。"""
    v = round(abs(speed_dps) * int(s["steps_per_rev"]) / 360.0)
    return min(max(v, int(s["speed_min_step_s"])), int(s["speed_max_step_s"]))


def dps2_to_accel_reg(accel_dps2: float, s: dict[str, Any]) -> int:
    """加速度 [deg/s²] → 加速度レジスタ（1 = 100 step/s²）。無限大は最大値。"""
    if math.isinf(accel_dps2):
        return int(s["accel_reg_max"])
    steps = abs(accel_dps2) * int(s["steps_per_rev"]) / 360.0
    reg = round(steps / float(s["accel_unit_step_s2"]))
    return min(max(reg, int(s["accel_reg_min"])), int(s["accel_reg_max"]))


def decode_sign_magnitude(raw: int, sign_bit: int) -> int:
    """符号ビット＋絶対値の表現を int に戻す（SDK の scs_tohost と同じ）。"""
    mask = 1 << sign_bit
    return -(raw & ~mask) if raw & mask else raw


def decode_load(raw: int, s: dict[str, Any]) -> float:
    """現在負荷 → サーボ座標での負荷率（符号付き）。

    【仮説・実機で要検証】公式資料に符号の記載はない。SCS/STS 系で一般的な
    「下位 load_sign_bit ビット = 大きさ、bit load_sign_bit = 方向（0=CW / 1=CCW）」と仮定する。
    CW/CCW と正負の対応も未確認。config の load_sign_bit が null なら生値をそのまま使う。
    """
    bit = s.get("load_sign_bit")
    if bit is None:
        return raw * float(s["load_unit"])
    mag = raw & ((1 << int(bit)) - 1)
    value = -mag if raw & (1 << int(bit)) else mag
    return value * float(s["load_unit"])


def ratio_to_torque_limit(ratio: float, s: dict[str, Any]) -> int:
    """トルク上限 0〜1 → トルク制限レジスタ（1000 = 100%）。"""
    full = int(s["torque_limit_full"])
    return min(max(round(ratio * full), 0), full)


def le_word(lo: int, hi: int) -> int:
    """リトルエンディアン 2 バイト → 整数（docs §2）。"""
    return (lo & 0xFF) | ((hi & 0xFF) << 8)


def decode_state_block(block: list[int], joint: JointSpec, s: dict[str, Any]) -> ServoState:
    """56 番地から読んだ 8 バイトを ServoState にする。"""
    pos = decode_sign_magnitude(le_word(block[OFS_POS], block[OFS_POS + 1]), POSITION_SIGN_BIT)
    load = joint.direction * decode_load(le_word(block[OFS_LOAD], block[OFS_LOAD + 1]), s)
    volt = block[OFS_VOLT] * float(s["voltage_unit_v"])
    return ServoState(step_to_deg(pos, joint, s), load, volt, float(block[OFS_TEMP]))


# =============================================================================
# バス本体
# =============================================================================
class FeetechServoBus(ServoBus):
    """STS3215 デイジーチェーン（Waveshare Bus Servo Adapter (A) 経由）。"""

    def __init__(self, cfg: dict[str, Any], port: str,
                 port_factory: Callable[[str], Any] = PortHandler) -> None:
        """port_factory はテストで偽のポートを差し込むためのもの。"""
        super().__init__(cfg)
        self._s = cfg["servo"]
        self._port_name = port
        self._port_factory = port_factory
        self._port: Any = None
        self._ph: Any = None
        self._lock = threading.Lock()
        self._use_sync_read = bool(self._s["sync_read"])

    # ---- 接続 -----------------------------------------------------------------
    def connect(self) -> None:
        """COM ポートを開いて 1Mbps に設定する。"""
        port = self._port_factory(self._port_name)
        if not port.openPort():
            raise ServoCommError(f"{self._port_name} を開けません（他のソフトが使用中？）")
        if not port.setBaudRate(int(self._s["baudrate"])):
            port.closePort()
            raise ServoCommError(f"{self._port_name} のボーレート設定に失敗")
        self._port, self._ph = port, sms_sts(port)
        log.info("connected %s", self._port_name)

    def disconnect(self, torque_off: bool = True) -> None:
        """ポートを閉じる。torque_off=True のときだけ全軸のトルクを切る。"""
        if self._port is None:
            return
        for sid in (self.ids if torque_off else []):
            try:
                self.set_torque(sid, False)
            except ServoCommError as e:
                log.warning("トルク OFF 失敗: %s", e)
        self._port.closePort()
        self._port = self._ph = None

    def ping(self, servo_id: int) -> bool:
        """PING に応答すれば True。"""
        with self._lock:
            _model, result, _err = self._handler().ping(servo_id)
        return result == COMM_SUCCESS

    # ---- 指令 -----------------------------------------------------------------
    def set_torque(self, servo_id: int, on: bool) -> None:
        """トルクスイッチ（40 番地）に 1 / 0 を書く。"""
        with self._lock:
            r = self._handler().write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 1 if on else 0)
        self._check(r[0], f"トルク設定 ID{servo_id}")

    def _set_torque_limit(self, servo_id: int, ratio: float) -> None:
        """トルク制限（48 番地・SRAM）を書く。ratio はストールトルクに対する割合。

        **実機未検証。** 48 番地の単位は資料間で食い違いがある（docs/sts3215_registers.md §注意 2）。
        """
        val = ratio_to_torque_limit(ratio, self._s)
        with self._lock:
            r = self._handler().write2ByteTxRx(servo_id, ADDR_TORQUE_LIMIT, val)
        self._check(r[0], f"トルク制限 ID{servo_id}")

    def _set_goals(self, goals: dict[int, Goal]) -> None:
        """1軸なら WritePosEx、複数軸なら SyncWritePosEx でまとめて送る。"""
        packets = {
            sid: (deg_to_step(g.deg, self.joints[sid], self._s),
                  dps_to_step_s(g.speed_dps, self._s),
                  dps2_to_accel_reg(g.accel_dps2, self._s))
            for sid, g in goals.items()
        }
        with self._lock:
            ph = self._handler()
            if len(packets) == 1:
                (sid, (pos, spd, acc)), = packets.items()
                result, _err = ph.WritePosEx(sid, pos, spd, acc)
            else:
                for sid, (pos, spd, acc) in packets.items():
                    ph.SyncWritePosEx(sid, pos, spd, acc)
                result = ph.groupSyncWrite.txPacket()
                ph.groupSyncWrite.clearParam()
        self._check(result, f"目標送信 {list(packets)}")

    # ---- 読み出し -------------------------------------------------------------
    def _read_states(self, ids: list[int]) -> dict[int, ServoState]:
        blocks: dict[int, list[int]] = {}
        with self._lock:
            if self._use_sync_read and len(ids) > 1:
                blocks = self._sync_read_blocks(ids)
                if not blocks:
                    log.warning("SYNC READ が応答しないため個別 READ に切り替えます")
                    self._use_sync_read = False
            for sid in ids:
                if sid not in blocks:
                    data, result, _err = self._handler().readTxRx(sid, READ_BLOCK_START, READ_BLOCK_LEN)
                    if result == COMM_SUCCESS:
                        blocks[sid] = list(data)
        return {sid: decode_state_block(b, self.joints[sid], self._s) for sid, b in blocks.items()}

    def read_positions(self, ids: list[int] | None = None) -> dict[int, float]:
        """現在位置（56 番地 2 バイト）だけを読む。"""
        ids = list(ids) if ids is not None else self.ids
        raw: dict[int, list[int]] = {}
        with self._lock:
            if self._use_sync_read and len(ids) > 1:
                raw = self._sync_read_blocks(ids, POSITION_LEN)
            for sid in ids:
                if sid not in raw:
                    data, result, _err = self._handler().readTxRx(sid, READ_BLOCK_START, POSITION_LEN)
                    if result == COMM_SUCCESS:
                        raw[sid] = list(data)
        return {sid: step_to_deg(decode_sign_magnitude(le_word(b[0], b[1]), POSITION_SIGN_BIT),
                                 self.joints[sid], self._s) for sid, b in raw.items()}

    @property
    def fast_reads(self) -> bool:
        """SYNC READ が使えていれば True。"""
        return self._use_sync_read

    def _sync_read_blocks(self, ids: list[int], length: int = READ_BLOCK_LEN) -> dict[int, list[int]]:
        """SYNC READ で 56 番地から length バイトを一括取得する。取れた軸だけ返す。"""
        g = GroupSyncRead(self._handler(), READ_BLOCK_START, length)
        for sid in ids:
            g.addParam(sid)
        g.txRxPacket()
        out: dict[int, list[int]] = {}
        for sid in ids:
            ok, _err = g.isAvailable(sid, READ_BLOCK_START, length)
            if ok:
                # data_dict[sid] は [ERROR, データ…]（SDK group_sync_read.readRx）
                out[sid] = list(g.data_dict[sid][1:1 + length])
        return out

    # ---- レジスタ単位（tools/servo_setup.py 用） --------------------------------
    def read_register(self, servo_id: int, addr: int, size: int) -> int | None:
        """レジスタを読む（1 / 2 バイト。リトルエンディアン）。"""
        with self._lock:
            data, result, _err = self._handler().readTxRx(servo_id, addr, size)
        if result != COMM_SUCCESS:
            return None
        return data[0] if size == 1 else le_word(data[0], data[1])

    def write_register(self, servo_id: int, addr: int, size: int, value: int, eeprom: bool = False) -> bool:
        """レジスタを書く。

        EEPROM は書き込みロック（55番地）を 0 にしてから書き、書いたら 1 に戻す。
        機種によっては EEPROM 書き込み時にトルク OFF が必要なので、念のため先にトルクを切る
        （docs/sts3215_registers.md §6 の未確認事項。実機で確認すること）。
        """
        ph = self._handler()
        with self._lock:
            if eeprom:
                ph.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 0)
                ph.write1ByteTxRx(servo_id, ADDR_LOCK, 0)
            if size == 1:
                result, _ = ph.write1ByteTxRx(servo_id, addr, int(value))
            else:
                result, _ = ph.write2ByteTxRx(servo_id, addr, int(value))
            if eeprom:
                ph.write1ByteTxRx(servo_id, ADDR_LOCK, 1)
        return result == COMM_SUCCESS

    def set_servo_id(self, old_id: int, new_id: int) -> bool:
        """ID を変更する（EEPROM）。変更後は joints の対応も付け替える。"""
        if not self.write_register(old_id, ADDR_ID, 1, new_id, eeprom=True):
            return False
        if old_id in self.joints:
            self.joints[new_id] = self.joints.pop(old_id)
        return True

    def scan(self, ids: list[int] | None = None) -> list[int]:
        """応答する ID を探す（既定は 1〜253）。"""
        return [sid for sid in (ids if ids is not None else range(1, 254)) if self.ping(sid)]

    # ---- 内部 -----------------------------------------------------------------
    def _handler(self) -> Any:
        if self._ph is None:
            raise ServoCommError("FeetechServoBus: connect() 前に操作しました")
        return self._ph

    def _check(self, result: int, what: str) -> None:
        if result != COMM_SUCCESS:
            msg = self._handler().getTxRxResult(result)
            raise ServoCommError(f"{what} 失敗: {msg}")
