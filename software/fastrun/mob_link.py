"""mob_link.py — mob(ESP32)との簡易シリアル通信(2026-08-03〜)。

pattern.send_pattern が期待する .send(str) / .wait_for(prefix, timeout_s) を
実装し、加えて SEN 一括読み取りを備える。motion_test.py の MobLink と
同方針だが、fastrun パッケージ内で完結させて依存を減らす。
"""

from __future__ import annotations

import time
from typing import Optional

import serial

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 3_000_000


class MobLink:
    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def __enter__(self) -> "MobLink":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def send(self, cmd: str) -> None:
        self.ser.write((cmd + "\n").encode("ascii"))
        self.ser.flush()

    def _readline(self, timeout_s: float) -> Optional[str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if raw:
                return raw.decode("ascii", errors="replace").strip()
        return None

    def wait_for(self, prefix: str, timeout_s: float) -> Optional[str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = self._readline(timeout_s=max(0.0, deadline - time.monotonic()))
            if line is None:
                break
            if line.startswith(prefix):
                return line
        return None

    def read_sen(self, timeout_s: float = 0.5) -> Optional[dict]:
        """SEN を1回問い合わせてパースした dict を返す。取りこぼしたら None。

        SEN,<gyro_z>,<vbatt>,<lf>,<ls>,<rs>,<rf>,<enc_r>,<enc_l>,
            <odo_dist>,<odo_ang>,<ball_raw>,<ball_det>  (13フィールド)
        """
        self.ser.reset_input_buffer()
        self.send("SEN")
        line = self.wait_for("SEN,", timeout_s=timeout_s)
        if line is None:
            return None
        p = line.split(",")
        if len(p) != 13:
            return None
        try:
            return {
                "gyro_z": float(p[1]),
                "vbatt": float(p[2]),
                "lf": int(p[3]),
                "ls": int(p[4]),
                "rs": int(p[5]),
                "rf": int(p[6]),
                "enc_r": int(p[7]),
                "enc_l": int(p[8]),
                "odo_dist": float(p[9]),
                "odo_ang": float(p[10]),
                "ball_raw": int(p[11]),
                "ball_det": int(p[12]),
            }
        except ValueError:
            return None

    def gyro_calibrate(self, timeout_s: float = 5.0) -> bool:
        """GCAL(静止前提)。DONE を待つ。"""
        self.send("GCAL")
        return self.wait_for("DONE", timeout_s=timeout_s) is not None

    def reset_odometry(self, timeout_s: float = 2.0) -> None:
        """走行距離・積分角度をリセット(RDST/RANG、いずれも DONE 応答)。"""
        self.send("RDST")
        self.wait_for("DONE", timeout_s=timeout_s)
        self.send("RANG")
        self.wait_for("DONE", timeout_s=timeout_s)

    def stop(self) -> None:
        """走行停止(PATTERN/MOT いずれも MOT,0,0 で止まる)。"""
        self.send("MOT,0,0")
