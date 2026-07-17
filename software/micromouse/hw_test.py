#!/usr/bin/env python3
"""実機の段階的検証 CLI(18.3 Hardware Test)。

モータを回す前に sen / walls でセンサ系を確認すること。
実行順の推奨は README の「実機の段階的検証」を参照。

    python3 hw_test.py sen                # センサ値の連続表示
    python3 hw_test.py walls              # 壁判定の連続表示(しきい値校正)
    python3 hw_test.py gcal               # ジャイロキャリブレーション
    python3 hw_test.py fwd [--cells N]    # Nセル直進(既定1)
    python3 hw_test.py turn left|right|back
    python3 hw_test.py cycle              # 探索1サイクル(半セル→半セル)
    python3 hw_test.py stop               # モータ停止(MOT,0,0)
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from config import MicromouseConfig
from mobile_base import MobileBase
from wall_detector import WallDetector


def check_battery(base: MobileBase, config: MicromouseConfig) -> None:
    frame = base.read_sensors()
    if frame is None:
        raise SystemExit("SEN 応答なし: mob の接続・電源を確認してください")
    print(f"battery: {frame.vbatt:.2f} V")
    if frame.vbatt < config.battery_min_v:
        raise SystemExit(f"バッテリー電圧が低すぎます (< {config.battery_min_v} V)")


def cmd_sen(base: MobileBase, config: MicromouseConfig, args) -> None:
    print("Ctrl+C で終了")
    base.wall_led(True)
    try:
        while True:
            frame = base.read_sensors()
            if frame is None:
                print("SEN timeout")
            else:
                print(
                    f"gyro={frame.gyro_radps:+.3f}rad/s batt={frame.vbatt:.2f}V "
                    f"wall lf/ls/rs/rf={frame.lf}/{frame.ls}/{frame.rs}/{frame.rf} "
                    f"enc={frame.enc_r}/{frame.enc_l} "
                    f"odo={frame.odo_dist_mm:.1f}mm {frame.odo_ang_rad:+.3f}rad"
                )
            time.sleep(0.2)
    finally:
        base.wall_led(False)


def cmd_walls(base: MobileBase, config: MicromouseConfig, args) -> None:
    detector = WallDetector(
        left_threshold=config.wall_left_threshold,
        right_threshold=config.wall_right_threshold,
        front_threshold=config.wall_front_threshold,
    )
    print(
        f"しきい値 L/R/F = {config.wall_left_threshold}/"
        f"{config.wall_right_threshold}/{config.wall_front_threshold}"
        " (config/micromouse.yaml で調整)"
    )
    print("Ctrl+C で終了")
    base.wall_led(True)
    try:
        while True:
            frame = base.read_sensors()
            if frame is None:
                print("SEN timeout")
            else:
                obs = detector.detect(frame)

                def mark(flag: bool) -> str:
                    return "#" if flag else "."

                print(
                    f"L{mark(obs.left)} F{mark(obs.front)} R{mark(obs.right)}  "
                    f"(ls={frame.ls} front={min(frame.lf, frame.rf)} rs={frame.rs})"
                )
            time.sleep(0.2)
    finally:
        base.wall_led(False)


def cmd_gcal(base: MobileBase, config: MicromouseConfig, args) -> None:
    print("機体を静止させてください...")
    time.sleep(1.0)
    base.gyro_calibrate()
    print("完了")


def cmd_fwd(base: MobileBase, config: MicromouseConfig, args) -> None:
    check_battery(base, config)
    distance = args.cells * config.cell_size_mm
    print(f"{distance:.0f} mm 直進します")
    base.reset_distance()
    base.reset_angle()
    base.stop_at(config.explore_speed_mmps, config.explore_accel_mmps2, distance)
    frame = base.read_sensors()
    if frame is not None:
        print(f"odo: {frame.odo_dist_mm:.1f} mm, {frame.odo_ang_rad:+.4f} rad")


def cmd_turn(base: MobileBase, config: MicromouseConfig, args) -> None:
    import math

    check_battery(base, config)
    angle = {"left": math.pi / 2, "right": -math.pi / 2, "back": math.pi}[args.dir]
    print(f"旋回: {args.dir} ({math.degrees(angle):.0f} deg)")
    base.reset_angle()
    base.turn(angle)
    frame = base.read_sensors()
    if frame is not None:
        print(f"odo angle: {math.degrees(frame.odo_ang_rad):+.2f} deg")


def cmd_cycle(base: MobileBase, config: MicromouseConfig, args) -> None:
    """探索1サイクル: 半セル前進 → センサ読み → 半セル前進して停止。"""
    check_battery(base, config)
    detector = WallDetector(
        left_threshold=config.wall_left_threshold,
        right_threshold=config.wall_right_threshold,
        front_threshold=config.wall_front_threshold,
    )
    base.wall_led(True)
    try:
        base.reset_distance()
        base.reset_angle()
        half = config.half_cell_mm
        base.forward(config.explore_speed_mmps, config.explore_accel_mmps2, half)
        frame = base.read_sensors()
        if frame is not None:
            obs = detector.detect(frame)
            print(f"判断点: left={obs.left} front={obs.front} right={obs.right}")
        base.stop_at(config.explore_speed_mmps, config.explore_accel_mmps2, half)
        print("セル中心で停止")
    finally:
        base.wall_led(False)


def cmd_stop(base: MobileBase, config: MicromouseConfig, args) -> None:
    base.emergency_stop()
    print("モータ停止")


def main() -> int:
    ap = argparse.ArgumentParser(description="micromouse hardware test")
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--config", type=Path, default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("sen")
    sub.add_parser("walls")
    sub.add_parser("gcal")
    p_fwd = sub.add_parser("fwd")
    p_fwd.add_argument("--cells", type=int, default=1)
    p_turn = sub.add_parser("turn")
    p_turn.add_argument("dir", choices=["left", "right", "back"])
    sub.add_parser("cycle")
    sub.add_parser("stop")

    args = ap.parse_args()
    config = MicromouseConfig.load(args.config)

    base = MobileBase(
        args.port or config.port,
        args.baud or config.baud,
        timeout_s=config.command_timeout_s,
    )
    handlers = {
        "sen": cmd_sen,
        "walls": cmd_walls,
        "gcal": cmd_gcal,
        "fwd": cmd_fwd,
        "turn": cmd_turn,
        "cycle": cmd_cycle,
        "stop": cmd_stop,
    }
    try:
        handlers[args.cmd](base, config, args)
        return 0
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        base.motors_off()
        base.close()


if __name__ == "__main__":
    raise SystemExit(main())
