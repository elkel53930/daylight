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

    def jog_forward(self, distance_mm: float) -> None:
        """低速固定速度(params.jog_mps)で指定距離前進。加減速なし。"""
        self._command_and_wait_done(f"JOGFWD,{distance_mm:.1f}\n")

    def jog_backward(self, distance_mm: float) -> None:
        """低速固定速度(params.jog_mps)で指定距離後退。加減速なし。"""
        self._command_and_wait_done(f"JOGBACK,{distance_mm:.1f}\n")

    def jog_turn(self, angle_rad: float) -> None:
        """低速固定速度(params.jog_turn_mps)でその場旋回。正: 左(CCW)。"""
        self._command_and_wait_done(f"JOGTURN,{angle_rad:.6f}\n")

    # ---- ラッチ動作(手動操作向け。LSTOP まで継続、DONE 無し) ----
    #
    # JOG* は「指定距離/角度だけ動いて自動停止・DONE」なのに対し、LATCH は
    # 「ボタンが押されている間だけ動かす」手動遠隔操作向けの動作(params.
    # latch_mps / latch_turn_mps で低速固定)。DONE を返さない送りっぱなし
    # コマンドなので、呼び出し側でボタン押下/解放に応じて開始・停止を
    # 呼び分ける(software/manual_controller/remote_controller.py 参照)。

    def latch_forward(self) -> None:
        """低速前進を開始する(latch_stop() まで継続)。"""
        self._send("LFWD\n")

    def latch_backward(self) -> None:
        """低速後退を開始する(latch_stop() まで継続)。"""
        self._send("LBACK\n")

    def latch_turn_left(self) -> None:
        """低速左旋回(CCW)を開始する(latch_stop() まで継続)。"""
        self._send("LTURNL\n")

    def latch_turn_right(self) -> None:
        """低速右旋回(CW)を開始する(latch_stop() まで継続)。"""
        self._send("LTURNR\n")

    def latch_stop(self) -> None:
        """latch_* 動作を停止する。"""
        self._send("LSTOP\n")

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

    def correct_angle(self, angle_rad: float) -> None:
        """外部の絶対基準(カメラ補正等)で角度を上書きする。

        RANG/RDST同様、セグメント間の停止中にのみ呼ぶこと(走行中の
        制御ループが参照する目標角には触れないため安全だが、動作中に
        呼ぶと基準角が汚染される)。
        """
        self._command_and_wait_done(f"SANG,{angle_rad:.6f}\n", timeout_s=3.0)

    def wall_led(self, enabled: bool) -> None:
        """壁センサ LED の有効化。応答が無いコマンドなので送りっぱなし。"""
        self._send(f"WALL,{1 if enabled else 0}\n")
        time.sleep(0.05)

    # ---- パラメータ(PGET/PSET、software/util/param_tui.py と同じ形式) ----

    def get_param(self, name: str, timeout_s: float = 2.0) -> Optional[float]:
        """単一パラメータを取得する(PGET,<name> → PVAL,<name>,<value>)。

        取得できなければ None(未知のパラメータ名・タイムアウト)。
        """
        self._send(f"PGET,{name}\n")
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
                continue
            if line.startswith("PVAL,"):
                parts = line.split(",")
                if len(parts) == 3 and parts[1] == name:
                    try:
                        return float(parts[2])
                    except ValueError:
                        return None
                continue
        return None

    def set_param(self, name: str, value: float) -> None:
        """パラメータを RAM 上で即時変更する(NVSには保存しない、送りっぱなし)。"""
        self._send(f"PSET,{name},{value}\n")
        time.sleep(0.05)

    # ---- リロードサーボ・ファン(ボール回収機構、DONE応答なし) ----

    def set_reload_servo(self, angle_deg: float) -> None:
        """リロードサーボ(mob 経由の RC サーボ)角度設定。"""
        self._send(f"SRV,{int(round(angle_deg))}\n")

    def set_fan_percent(self, percent: float) -> None:
        """吸引ファン Duty 設定(0-100%)。"""
        duty = max(0, min(255, round(255 * percent / 100.0)))
        self._send(f"FAN,{duty}\n")

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
