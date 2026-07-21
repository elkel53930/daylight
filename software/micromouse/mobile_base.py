"""mob (ESP32-S3) シリアルドライバ。

プロトコル (software/mob/mob.ino):
    PC → mob : "FWD,<speed>,<accel>,<dist>\\n" などの CSV 行
    mob → PC : "DONE\\n" / "QSTPDONE,<残距離>\\n" / "SEN,..." / "#<デバッグ>"

Twilight の mobile_base_threaded.py を同期版として再実装したもの。
スレッド化はせず、DONE 待ちループに abort_check コールバックを挿して
「待機中のボタン中断 → QSTP」を実現する(判断点間の処理は全て同期で
足りるため。理由は README「Twilight からの移植」参照)。

注意: Daylight の mob に引数なしの STOP コマンドは存在しない
("STOP," のみ)。無条件のモータ停止は MOT,0,0 を使う。
"""

from __future__ import annotations

import time
from typing import Callable, List, Optional

import serial

from errors import AbortRequested, MobileBaseError
from wall_detector import SensorFrame, parse_sen_line


class MobileBase:
    """mob との同期シリアル通信。

    abort_check: 各待機ループで呼ばれるコールバック。True を返すと
    QSTP でモータを止めて AbortRequested を送出する。
    """

    def __init__(
        self,
        port: str,
        baud: int = 3000000,
        *,
        timeout_s: float = 10.0,
        abort_check: Optional[Callable[[], bool]] = None,
        raw_log_fn: Optional[Callable[[str], None]] = None,
    ):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self.timeout_s = timeout_s
        self.abort_check = abort_check
        # mob から読んだ行を(SEN/DONE 含め)そのまま受け取るフック。
        # #V,... デバッグテレメトリは SEN でも DONE でもないため、これが
        # 無いと _wait_for() の tail バッファ(直近5行、タイムアウト時のみ
        # 表示)以外には一切残らず捨てられる。
        self.raw_log_fn = raw_log_fn
        self.last_frame: Optional[SensorFrame] = None
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    # ---- 低レベル ----

    def _send(self, line: str) -> None:
        try:
            self.ser.reset_input_buffer()
        except serial.SerialException:
            pass
        self.ser.write(line.encode("ascii", errors="ignore"))

    def _read_line(self) -> Optional[str]:
        """1 行読む。空・制御文字は除去し、無ければ None。"""
        try:
            raw = self.ser.readline()
        except serial.SerialException as e:
            raise MobileBaseError(f"serial read failed: {e}") from e
        if not raw:
            return None
        line = raw.decode("ascii", errors="replace")
        line = "".join(ch for ch in line if ch >= " " or ch == "\t").strip()
        if line and self.raw_log_fn is not None:
            self.raw_log_fn(line)
        return line or None

    def _check_abort(self) -> None:
        if self.abort_check is not None and self.abort_check():
            self.emergency_stop()
            raise AbortRequested("aborted by user")

    def _wait_for(self, terminators: tuple, timeout_s: Optional[float]) -> str:
        """terminators のいずれかで始まる行を待つ。SEN 行は随時取り込む。"""
        deadline = time.monotonic() + (timeout_s or self.timeout_s)
        tail: List[str] = []
        while time.monotonic() < deadline:
            self._check_abort()
            line = self._read_line()
            if line is None:
                continue
            if line.startswith("SEN,"):
                frame = parse_sen_line(line)
                if frame is not None:
                    self.last_frame = frame
                continue
            if line.startswith(terminators):
                return line
            tail.append(line)
        raise MobileBaseError(
            "timeout waiting for %s (last: %s)" % (terminators, tail[-5:])
        )

    def _command_and_wait_done(self, line: str, timeout_s: Optional[float] = None) -> None:
        self._send(line)
        self._wait_for(("DONE",), timeout_s)

    # ---- 走行コマンド ----

    def forward(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """加減速つき前進。距離到達で DONE(停止はしない)。"""
        self._command_and_wait_done(
            f"FWD,{speed_mmps:.1f},{accel_mmps2:.1f},{distance_mm:.1f}\n"
        )

    def stop_at(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """指定距離で減速して停止。"""
        self._command_and_wait_done(
            f"STOP,{speed_mmps:.1f},{accel_mmps2:.1f},{distance_mm:.1f}\n"
        )

    def turn(self, angle_rad: float) -> None:
        """その場旋回。正: 左(CCW)。"""
        self._command_and_wait_done(f"TURN,{angle_rad:.6f}\n")

    def quick_stop(self) -> float:
        """最大減速度で停止し、元目標までの残距離 [mm] を返す。"""
        self._send("QSTP\n")
        line = self._wait_for(("QSTPDONE",), timeout_s=5.0)
        try:
            return float(line.split(",")[1])
        except (IndexError, ValueError):
            return 0.0

    def motors_off(self) -> None:
        """即時にモータ指令をゼロにする(応答は待たない)。"""
        try:
            self.ser.write(b"MOT,0,0\n")
        except serial.SerialException:
            pass

    def emergency_stop(self) -> None:
        """安全停止: QSTP で減速停止を試み、失敗時も必ず MOT,0,0。"""
        try:
            self._send("QSTP\n")
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                line = self._read_line()
                if line is not None and line.startswith("QSTPDONE"):
                    break
        except Exception:
            pass
        finally:
            self.motors_off()

    # ---- キャリブレーション・リセット ----

    def gyro_calibrate(self, timeout_s: float = 10.0) -> None:
        """ジャイロオフセット校正。静止状態で呼ぶこと。"""
        self._command_and_wait_done("GCAL\n", timeout_s)

    def reset_distance(self) -> None:
        self._command_and_wait_done("RDST\n", timeout_s=3.0)

    def reset_angle(self) -> None:
        self._command_and_wait_done("RANG\n", timeout_s=3.0)

    def wall_led(self, enabled: bool) -> None:
        """壁センサ LED の有効化。応答が無いコマンドなので送りっぱなし。"""
        self._send(f"WALL,{1 if enabled else 0}\n")
        time.sleep(0.05)

    # ---- センサ ----

    def read_sensors(self, timeout_s: float = 2.0) -> Optional[SensorFrame]:
        """SEN を要求して 1 フレーム読む。取得できなければ None。"""
        self._send("SEN\n")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._check_abort()
            line = self._read_line()
            if line is None:
                continue
            if line.startswith("SEN,"):
                frame = parse_sen_line(line)
                if frame is not None:
                    self.last_frame = frame
                    return frame
        return None
