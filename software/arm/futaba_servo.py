#!/usr/bin/env python3
"""
futaba_servo.py - Futabaコマンドサーボ(RS204MD/RS304MD)制御

ラズパイのUARTから直接コマンドパケットを送信してサーボを制御する。
robosweep_twilight(前身機)の software/arm/arm.py に含まれていた
Futaba制御部分を移植したもの(Arduino経由のランチャー/モータ制御は
Daylightには無いため除外している)。
"""

import argparse
import time
from typing import Optional, Union

import serial


class FutabaServo:
    """Futabaコマンドサーボ(RS204MD/RS304MD)コントローラー"""

    # メモリマップアドレス
    ADDR_TORQUE_ENABLE = 0x24
    ADDR_GOAL_POSITION = 0x1E

    def __init__(
        self,
        servo_id: int = 1,
        port: str = "/dev/ttyAMA0",
        baudrate: int = 115200,
        min_angle: float = -150.0,
        max_angle: float = 150.0,
        zero_offset_deg: float = -130.0,
    ) -> None:
        """
        Args:
            servo_id: サーボID (1-127)
            port: シリアルポート
            baudrate: 通信速度 [bps]
            min_angle, max_angle: 角度制限 [度](サーボの物理限界は±150度)
            zero_offset_deg: set_angle()の論理角度0に対応する物理角度 [度]。
                実機校正で確認した機構上の基準位置(-130度)がデフォルト
                (2026-07-23実測)。set_angle(angle)は内部で
                angle + zero_offset_deg を実際にサーボへ送信する。
        """
        self.servo_id = servo_id
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.zero_offset_deg = zero_offset_deg
        self._serial: Optional[serial.Serial] = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        time.sleep(0.1)  # 安定化待ち

    def close(self) -> None:
        """シリアルポートを閉じる"""
        if self._serial is not None and self._serial.is_open:
            self._serial.close()

    def __enter__(self) -> "FutabaServo":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()

    # ------------------------------------------------------------------
    # パケット送信
    # ------------------------------------------------------------------

    @staticmethod
    def _checksum(data: bytes) -> int:
        """チェックサム計算(XOR)"""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def _send_short_packet(self, flag: int, address: int, data: Union[int, bytes, bytearray]) -> None:
        """
        ショートパケットを送信する。

        Args:
            flag: フラグ
            address: メモリマップアドレス
            data: 送信データ(int または bytes/bytearray)
        """
        packet = bytearray()
        packet.append(0xFA)  # Header L
        packet.append(0xAF)  # Header H
        packet.append(self.servo_id)
        packet.append(flag)
        packet.append(address)
        packet.append(1 if isinstance(data, int) else len(data))  # Length
        packet.append(0x01)  # Count(ショートパケットは常に1)

        if isinstance(data, int):
            packet.append(data)
        else:
            packet.extend(data)

        packet.append(self._checksum(packet[2:]))  # チェックサムはID以降

        self._serial.write(packet)
        self._serial.flush()

    # ------------------------------------------------------------------
    # サーボ制御
    # ------------------------------------------------------------------

    def set_torque(self, enable: bool) -> None:
        """トルクON/OFF"""
        self._send_short_packet(0x00, self.ADDR_TORQUE_ENABLE, [0x01 if enable else 0x00])
        time.sleep(0.01)

    def set_angle(self, angle: float, move_time_ms: int = 0) -> None:
        """
        目標角度を設定する。

        Args:
            angle: 目標角度 [度](zero_offset_degを基準とした論理角度)
            move_time_ms: 移動時間 [ms](0の場合は最高速度)
        """
        if angle < self.min_angle or angle > self.max_angle:
            raise ValueError(f"角度が範囲外です: {angle}度 ({self.min_angle}度〜{self.max_angle}度)")

        physical_angle = angle + self.zero_offset_deg
        angle_value = int(physical_angle * 10)  # 0.1度単位
        if angle_value < -1500 or angle_value > 1500:
            raise ValueError(
                f"物理角度がサーボの物理的制限を超えています: "
                f"論理角度{angle}度(オフセット{self.zero_offset_deg}度適用後{physical_angle}度、±150.0度まで)"
            )

        if angle_value < 0:
            angle_value += 0x10000  # 2の補数表現

        data = bytearray()
        data.append(angle_value & 0xFF)         # Low byte
        data.append((angle_value >> 8) & 0xFF)  # High byte

        if move_time_ms > 0:
            time_value = int(move_time_ms / 10)  # 10ms単位
            data.append(time_value & 0xFF)
            data.append((time_value >> 8) & 0xFF)

        self._send_short_packet(0x00, self.ADDR_GOAL_POSITION, data)
        time.sleep(0.01)

    def set_brake(self) -> None:
        """ブレーキモードに設定する(トルクON状態のまま外力による回転に抵抗する)"""
        self._send_short_packet(0x00, self.ADDR_TORQUE_ENABLE, [0x02])
        time.sleep(0.01)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Futabaサーボを指定角度へ2秒かけて移動し、トルクを抜く")
    parser.add_argument("angle", type=float, help="目標角度 [度](zero_offset_degを基準とした論理角度)")
    args = parser.parse_args()

    with FutabaServo() as servo:
        servo.set_torque(True)
        servo.set_angle(args.angle, move_time_ms=2000)
        time.sleep(2.0)
        servo.set_torque(False)
