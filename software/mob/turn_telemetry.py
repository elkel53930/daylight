#!/usr/bin/env python3
"""旋回振動のテレメトリログ取得スクリプト.

TURNコマンドを送信し、#Vテレメトリをキャプチャしてログに保存する。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "micromouse"))

from config import MicromouseConfig
from mobile_base import MobileBase

# #Vテレメトリの正規表現: #V,cmd,vr,vl,ur,ul,gz,ang,rem
# cmd=目標角速度, vr/vl=車輪速度, ur/ul=duty, gz=ジャイロ, ang=角度, rem=残り角
V_TELEMETRY_RE = re.compile(
    r"^#V,([-\d.]+),([-\d.]+),([-\d.]+),([-\d]+),([-\d]+),([-\d.]+),([-\d.]+),([-\d.]+)"
)


def capture_turn_telemetry(
    base: MobileBase,
    angle_rad: float,
    duration_s: float = 5.0,
    *,
    log_file: Path | None = None,
) -> list[dict]:
    """TURNコマンドを送信してテレメトリをキャプチャする.

    Args:
        base: MobileBase
        angle_rad: 旋回角度[rad]
        duration_s: キャプチャ時間[s]
        log_file: ログファイルパス(Noneの場合は出力しない)

    Returns:
        テレメトリデータのリスト
    """
    # ログファイルを開く
    if log_file is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file = Path(__file__).parent / f"turn_telemetry_{stamp}.csv"

    print(f"旋回テスト: {math.degrees(angle_rad):+.1f}度")
    print(f"ログファイル: {log_file}")

    # ログファイルのヘッダ
    with open(log_file, "w") as f:
        f.write("time_ms,cmd_omega,vr,vl,duty_r,duty_l,gyro_radps,angle_rad,remaining_rad\n")

    # キャプチャ開始
    base.reset_angle()
    time.sleep(0.2)

    # シリアルバッファをクリア
    base.ser.reset_input_buffer()

    # TURNコマンドを送信（DONEを待たずに即座にキャプチャ開始）
    print(f"TURN {math.degrees(angle_rad):+.1f}度 送信")
    start_time = time.monotonic()
    base.ser.write(f"TURN,{angle_rad:.6f}\n".encode("ascii"))

    # テレメトリをキャプチャ
    data_points = []
    capture_end = start_time + duration_s

    # シリアルポートから直接読み取る
    base.ser.timeout = 0.01
    while time.monotonic() < capture_end:
        try:
            line = base.ser.readline().decode("ascii", errors="replace").strip()
            if not line:
                continue

            t_ms = (time.monotonic() - start_time) * 1000

            # #Vテレメトリ
            m = V_TELEMETRY_RE.match(line)
            if m:
                cmd_omega = float(m.group(1))
                vr = float(m.group(2))
                vl = float(m.group(3))
                duty_r = int(m.group(4))
                duty_l = int(m.group(5))
                gyro = float(m.group(6))
                angle = float(m.group(7))
                remaining = float(m.group(8))

                data_points.append({
                    "time_ms": t_ms,
                    "cmd_omega": cmd_omega,
                    "vr": vr,
                    "vl": vl,
                    "duty_r": duty_r,
                    "duty_l": duty_l,
                    "gyro_radps": gyro,
                    "angle_rad": angle,
                    "remaining_rad": remaining,
                })

                # CSVに追記
                with open(log_file, "a") as f:
                    f.write(f"{t_ms:.1f},{cmd_omega:.3f},{vr:.3f},{vl:.3f},"
                            f"{duty_r},{duty_l},{gyro:.4f},{angle:.4f},{remaining:.4f}\n")
        except Exception:
            pass

    print(f"キャプチャ完了: {len(data_points)} サンプル")
    return data_points


def analyze_telemetry(data: list[dict], target_angle_rad: float) -> dict:
    """テレメトリデータを分析する.

    Args:
        data: テレメトリデータ
        target_angle_rad: 目標角度[rad]

    Returns:
        分析結果
    """
    if not data:
        return {"error": "no data"}

    # 最終角度
    final_angle = data[-1]["angle_rad"]
    angle_error = final_angle - target_angle_rad

    # 目標到達後のデータを抽出（目標角速度が0付近で、残り角が小さい）
    settled_data = [d for d in data if abs(d["remaining_rad"]) < 0.1]
    if settled_data:
        # 定常状態での角度誤差の統計
        errors = [d["angle_rad"] - target_angle_rad for d in settled_data]
        mean_error = sum(errors) / len(errors)
        max_error = max(abs(e) for e in errors)

        # 振動の検出（角度誤差が正負を行き来する回数）
        crossings = 0
        for i in range(1, len(settled_data)):
            prev_err = settled_data[i-1]["angle_rad"] - target_angle_rad
            curr_err = settled_data[i]["angle_rad"] - target_angle_rad
            if prev_err * curr_err < 0:
                crossings += 1

        return {
            "final_angle_rad": final_angle,
            "final_error_rad": angle_error,
            "final_error_deg": math.degrees(angle_error),
            "settled_samples": len(settled_data),
            "mean_error_rad": mean_error,
            "mean_error_deg": math.degrees(mean_error),
            "max_error_rad": max_error,
            "max_error_deg": math.degrees(max_error),
            "target_crossings": crossings,
        }
    else:
        return {
            "final_angle_rad": final_angle,
            "final_error_rad": angle_error,
            "final_error_deg": math.degrees(angle_error),
            "settled_samples": 0,
        }


def main() -> int:
    ap = argparse.ArgumentParser(description="Turn oscillation telemetry capture")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--angle", type=float, default=90.0, help="旋回角度[度]")
    ap.add_argument("--duration", type=float, default=5.0, help="キャプチャ時間[秒]")
    args = ap.parse_args()

    config = MicromouseConfig.load()
    base = MobileBase(
        args.port or config.port,
        args.baud or config.baud,
        timeout_s=config.command_timeout_s,
    )

    try:
        angle_rad = math.radians(args.angle)
        data = capture_turn_telemetry(base, angle_rad, args.duration)

        if data:
            analysis = analyze_telemetry(data, angle_rad)
            print("\n=== 分析結果 ===")
            print(f"最終角度: {analysis['final_angle_rad']:.4f} rad ({math.degrees(analysis['final_angle_rad']):+.2f} deg)")
            print(f"最終誤差: {analysis['final_error_deg']:+.2f} deg")
            print(f"定常サンプル数: {analysis['settled_samples']}")
            if analysis['settled_samples'] > 0:
                print(f"平均誤差: {analysis['mean_error_deg']:+.3f} deg")
                print(f"最大誤差: {analysis['max_error_deg']:.3f} deg")
                print(f"目標値交叉回数: {analysis['target_crossings']}")

        return 0
    except KeyboardInterrupt:
        print("\n中断")
        return 0
    finally:
        base.motors_off()
        base.close()


if __name__ == "__main__":
    raise SystemExit(main())
