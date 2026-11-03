"""mob (ESP32-S3) シリアルドライバ。

=== 概要 ===
このモジュールはラズパイと ESP32-S3 (mob) の間のシリアル通信を担当します。
3Mbps の UART で通信し、テキストベースのコマンドを送受信します。

=== プロトコル ===
PC → mob : "JOGFWD,<mm>\\n" / "JOGTURN,<rad>\\n" / "MOT,0,0\\n" など
mob → PC : "DONE\\n" / "SEN,..." / "#<デバッグ>"

=== コマンド体系 ===
移動コマンド (DONE応答あり):
  - JOGFWD,<mm>: 指定距離を前進 (place_controller が位置を保持)
  - JOGBACK,<mm>: 指定距離を後退
  - JOGTURN,<rad>: 指定角度を旋回 (正=左/CCW)

設定コマンド (DONE応答あり):
  - GCAL: ジャイロキャリブレーション (静止状態で実行)
  - RDST: 距離オドメトリをリセット
  - RANG: 角度オドメトリをリセット
  - SANG,<rad>: 角度を絶対値で設定

即時コマンド (DONE応答なし):
  - MOT,<r>,<l>: モータ速度を直接設定 (-1023〜+1023)
  - WALL,<0|1>: 壁センサLEDのON/OFF

=== 重要な制約 ===
JOG コマンドは到達後も位置・角度を保持し続けます。
そのため SANG/RANG/RDST/GCAL の前に MOT,0,0 で保持を解除する必要があります。
保持中にこれらのコマンドを送ると、place_controller が暴走して機体が動きます。

=== API の対応表 ===
  forward(speed, accel, dist)  → JOGFWD (speed/accel は無視される)
  stop_at(speed, accel, dist)  → JOGFWD + MOT,0,0
  turn(angle)                  → JOGTURN
  emergency_stop()             → MOT,0,0 (2回送信)

=== 設計 ===
Twilight の mobile_base_threaded.py を同期版として再実装したもの。
スレッド化はせず、DONE 待ちループに abort_check コールバックを挿して
「待機中のボタン中断 → MOT,0,0」を実現する(判断点間の処理は全て同期で
足りるため。理由は README「Twilight からの移植」参照)。
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
    MOT,0,0 でモータを止めて AbortRequested を送出する。
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
        """シリアルポートに1行送信する。
        
        事前条件:
        - line はASCII文字列 (改行コード含む)
        - self.ser がオープン済み
        
        事後条件:
        - シリアルバッファがクリアされている
        - line が送信されている
        """
        try:
            self.ser.reset_input_buffer()
        except serial.SerialException:
            pass
        self.ser.write(line.encode("ascii", errors="ignore"))

    def _read_line(self) -> Optional[str]:
        """1行読む。空・制御文字は除去し、無ければ None。
        
        事前条件:
        - self.ser がオープン済み
        
        事後条件:
        - 戻り値が None の場合: タイムアウト (データなし)
        - 戻り値が文字列の場合: 制御文字を除去した1行
        
        不変条件:
        - raw_log_fn が設定されていれば、受信した行が記録される
        """
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
        """中断要求をチェックする。
        
        事前条件:
        - abort_check が設定されている場合、呼び出し可能
        
        事後条件:
        - abort_check() が True を返した場合:
          - emergency_stop が呼ばれている
          - AbortRequested が送出される
        - False の場合: 何もしない
        """
        if self.abort_check is not None and self.abort_check():
            self.emergency_stop()
            raise AbortRequested("aborted by user")

    def _wait_for(self, terminators: tuple, timeout_s: Optional[float]) -> str:
        """terminators のいずれかで始まる行を待つ。
        
        同期的なプロトコルの核心部:
        - コマンドを送信後、mob から "DONE" などの応答を待つ
        - タイムアウトまでに該当行が来なければ MobileBaseError を送出
        - SEN 行は待機中でも随時取り込んで last_frame に保持
        
        このメソッドは「コマンドを送って応答を待つ」という同期的な通信を実現します。
        非同期ではなく、各コマンドの完了を待ってから次のコマンドを送信します。
        """
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

    def _jog_timeout(self, distance_mm: float) -> float:
        """JOG 距離走行の DONE 待ちタイムアウト。

        JOG は平均 120mm/s 程度を下限目安に余裕をみる(実速度は
        place_pos_max 依存)。固定の command_timeout_s では長い直進
        (最短走行)で足りなくなるため距離比例で延ばす。
        """
        return max(self.timeout_s, 5.0 + abs(distance_mm) / 120.0)

    # ---- 走行コマンド ----
    #
    # 旧 FWD/STOP/TURN は mob から削除されたため、距離移動は JOGFWD、
    # 旋回は JOGTURN に置き換えている。speed/accel 引数は呼び出し側
    # (cell_runner / simulator)との互換のために残しているが、実機では
    # mob 側の JOG プロファイルが使われるため無視される。

    def forward(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """指定距離だけ前進する(JOGFWD、到達・整定で DONE、その場を保持)。
        
        事前条件:
        - distance_mm >= 0 (負の値は0に丸められる)
        - self.ser がオープン済み
        
        事後条件:
        - JOGFWD コマンドが送信され、DONE を受信済み
        - モバイルベースが指定距離前進している
        - place_controller が位置を保持中
        
        不変条件:
        - speed_mmps, accel_mmps2 は無視される (mob側の固定プロファイル使用)
        """
        distance_mm = max(0.0, float(distance_mm))
        if distance_mm <= 0.0:
            return
        self._command_and_wait_done(
            f"JOGFWD,{distance_mm:.1f}\n", self._jog_timeout(distance_mm)
        )

    def stop_at(self, speed_mmps: float, accel_mmps2: float, distance_mm: float) -> None:
        """指定距離だけ前進して停止する(JOGFWD 到達後 MOT,0,0 で保持解除)。
        
        事前条件:
        - distance_mm >= 0
        
        事後条件:
        - forward() が完了している (JOGFWD + DONE 受信済み)
        - MOT,0,0 が送信され、place_controller が解除されている
        - モバイルベースが停止状態 (IDLE)
        """
        self.forward(speed_mmps, accel_mmps2, distance_mm)
        self.motors_off()

    def turn(self, angle_rad: float) -> None:
        """その場旋回(JOGTURN、到達・整定で DONE)。正: 左(CCW)。
        
        事前条件:
        - abs(angle_rad) >= 1e-9 (微小角度は無視)
        - self.ser がオープン済み
        
        事後条件:
        - JOGTURN コマンドが送信され、DONE を受信済み
        - モバイルベースが指定角度旋回している
        - place_controller が角度を保持中
        """
        if abs(angle_rad) < 1e-9:
            return
        self._command_and_wait_done(f"JOGTURN,{angle_rad:.6f}\n")

    def jog_forward(self, distance_mm: float) -> None:
        """指定距離前進(JOGFWD、到達・整定で DONE、その場を保持)。"""
        distance_mm = abs(float(distance_mm))
        if distance_mm <= 0.0:
            return
        self._command_and_wait_done(
            f"JOGFWD,{distance_mm:.1f}\n", self._jog_timeout(distance_mm)
        )

    def jog_backward(self, distance_mm: float) -> None:
        """指定距離後退(JOGBACK、到達・整定で DONE、その場を保持)。"""
        distance_mm = abs(float(distance_mm))
        if distance_mm <= 0.0:
            return
        self._command_and_wait_done(
            f"JOGBACK,{distance_mm:.1f}\n", self._jog_timeout(distance_mm)
        )

    def jog_turn(self, angle_rad: float) -> None:
        """その場旋回(JOGTURN、到達・整定で DONE)。正: 左(CCW)。"""
        if abs(angle_rad) < 1e-9:
            return
        self._command_and_wait_done(f"JOGTURN,{angle_rad:.6f}\n")

    def quick_stop(self) -> float:
        """即時停止する(旧 QSTP 互換)。

        新プロトコルには QSTP が無いため MOT,0,0 を送る。残距離は
        mob から返らないので常に 0.0。
        """
        self.motors_off()
        return 0.0

    def motors_off(self) -> None:
        """即時にモータ指令をゼロにする(応答は待たない)。"""
        try:
            self.ser.write(b"MOT,0,0\n")
        except serial.SerialException:
            pass

    def emergency_stop(self) -> None:
        """安全停止: MOT,0,0 を送って即時停止する(応答は待たない)。
        
        事前条件:
        - self.ser がオープン済み (または例外が発生しても問題ない)
        
        事後条件:
        - MOT,0,0 が2回送信されている (冗長性確保)
        - モバイルベースが IDLE 状態 (モータ指令ゼロ)
        
        不変条件:
        - このメソッドは例外を送出しない (安全停止のため)
        - 応答を待たない (即時リターン)
        """
        self.motors_off()
        # 直前に走行コマンドがキューに残っていても最後に0が効くよう二重送出。
        self.motors_off()

    # ---- キャリブレーション・リセット ----

    def gyro_calibrate(self, timeout_s: float = 10.0) -> None:
        """ジャイロオフセット校正。静止状態で呼ぶこと。
        
        事前条件:
        - モバイルベースが完全に静止している (振動・移動なし)
        - place_controller が解除済み (MOT,0,0 送信済み)
        
        事後条件:
        - MOT,0,0 が送信され、place_controller が解除されている
        - GCAL コマンドが送信され、DONE を受信済み
        - ジャイロのオフセットがキャリブレーション済み
        
        不変条件:
        - 0.2秒の整定待ち時間が插入される
        """
        # JOG/TURN の保持中だと停止しきる前に校正してしまうため、
        # 先に保持を解除して整定を待つ。
        self.motors_off()
        time.sleep(0.2)
        self._command_and_wait_done("GCAL\n", timeout_s)

    def reset_distance(self) -> None:
        """距離オドメトリをリセットする。

        事前条件:
        - place_controller が解除済み (JOG/TURN 保持中でない)
        
        事後条件:
        - MOT,0,0 が送信され、place_controller が解除されている
        - RDST コマンドが送信され、DONE を受信済み
        - mob 側の odo_dist == 0
        
        不変条件:
        - 保持中に RDST を送ると、place_controller の位置基準と実オドメトリが
          ずれて機体が動き出すため、先に MOT,0,0 で保持を解除する
        """
        self.motors_off()
        self._command_and_wait_done("RDST\n", timeout_s=3.0)

    def reset_angle(self) -> None:
        """角度オドメトリをリセットする。
        
        事前条件:
        - place_controller が解除済み (JOG/TURN 保持中でない)
        
        事後条件:
        - MOT,0,0 が送信され、place_controller が解除されている
        - RANG コマンドが送信され、DONE を受信済み
        - mob 側の odo_ang == 0
        """
        self.motors_off()
        self._command_and_wait_done("RANG\n", timeout_s=3.0)

    def correct_angle(self, angle_rad: float) -> None:
        """外部の絶対基準(カメラ補正等)で角度を上書きする。

        事前条件:
        - place_controller が解除済み (JOG/TURN 保持中でない)
        - モバイルベースが静止している
        
        事後条件:
        - MOT,0,0 が送信され、place_controller が解除されている
        - SANG コマンドが送信され、DONE を受信済み
        - mob 側の odo_ang == angle_rad
        
        不変条件:
        - 保持中に SANG を送ると、place_controller が旧目標角へ戻そうと
          して機体が旋回してしまうため、先に MOT,0,0 で保持を解除する
        """
        self.motors_off()
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

    # ---- パスコントロール(壁センサフィードバック付き) ----

    def forward_with_wall_feedback(
        self,
        distance_mm: float,
        speed_mmps: float = 300.0,
        accel_mmps2: float = 1000.0,
        timeout_s: Optional[float] = None,
    ) -> None:
        """壁センサフィードバック付きで指定距離前進する。
        
        PCLEAR → PADD,STRAIGHT → PRUN を使用して、path_controller の
        壁センサフィードバック(path_wall_kp)を有効にして直進する。
        壁センサが左右の壁を検出している場合、機体を中央に寄せる
        補正が働く。
        
        事前条件:
        - distance_mm > 0
        - WALL,1 で壁センサLEDが有効になっている
        - self.ser がオープン済み
        
        事後条件:
        - PRUN が完了し、モータが停止している
        - モバイルベースが指定距離前進している
        
        不変条件:
        - #T テレメトリでセグメント完了を検出する
        - 完了後に MOT,0,0 で PATH_FOLLOW 状態を抜ける
        """
        if distance_mm <= 0.0:
            return
        
        timeout_s = timeout_s or self._jog_timeout(distance_mm)
        
        # パターンバッファをクリア
        self._send("PCLEAR\n")
        self._wait_for(("DONE",), timeout_s=2.0)
        
        # 直進セグメントを追加
        # PADD,STRAIGHT,<dist>,<v_start>,<v_cruise>,<v_end>
        v_start = 0.0
        v_cruise = speed_mmps
        v_end = 0.0
        self._send(f"PADD,STRAIGHT,{distance_mm:.1f},{v_start:.1f},{v_cruise:.1f},{v_end:.1f}\n")
        self._wait_for(("DONE",), timeout_s=2.0)
        
        # 走行開始
        self._send("PRUN\n")
        
        # #T テレメトリでセグメント完了を待つ
        # #T,<seg_index>,<tx>,<ty>,<rx>,<ry>,<rtheta>,<dist>,<hdg_err>
        deadline = time.monotonic() + timeout_s
        pattern_done = False
        
        while time.monotonic() < deadline and not pattern_done:
            self._check_abort()
            line = self._read_line()
            if line is None:
                continue
            if line.startswith("SEN,"):
                frame = parse_sen_line(line)
                if frame is not None:
                    self.last_frame = frame
                continue
            if line.startswith("#T,"):
                parts = line.split(",")
                if len(parts) >= 2:
                    try:
                        seg_index = int(parts[1])
                        # セグメントインデックスが1以上になったら完了
                        # (0 = 最初のセグメント実行中, 1 = 全セグメント完了)
                        if seg_index >= 1:
                            pattern_done = True
                    except ValueError:
                        pass
        
        # モータ停止して PATH_FOLLOW 状態を抜ける
        self.motors_off()

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
