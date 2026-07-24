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
    python3 hw_test.py camera-capture     # カメラで1枚撮影しlogs/camera/へ保存
    python3 hw_test.py camera-sweep       # 既知の角度・横位置ズレで自動走行しカメラ較正用画像セットを撮影
    python3 hw_test.py camera-sweep-distance  # 既知の前進距離で自動走行し壁との距離較正用画像セットを撮影
    python3 hw_test.py camera-correct     # カメラで壁上面を撮影し、推定ヨー角でmobの角度を補正(暫定較正式、TODO.md参照)
    python3 hw_test.py camera-straighten  # 推定ヨー角・距離オフセットぶん機体を物理的に動かしまっすぐ・基準距離に戻す(デモ用)
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


def cmd_camera_capture(base: MobileBase, config: MicromouseConfig, args) -> None:
    """カメラで1枚撮影して logs/camera/latest.jpg に上書き保存する
    (カメラ角度補正の実機データ収集用。蓄積はしない。校正用に残したい
    カットは呼び出し側で別名に手動コピーすること)。

    カメラをパンさせる Futaba コマンドサーボ(software/arm/futaba_servo.py)は
    ラズパイの UART (/dev/ttyAMA0) 直結で mob(ESP32)とは独立した経路なので、
    MobileBase(mob用シリアル)は使わない。論理角度0度=機体前方固定
    (2026-07-24 ユーザー確認済み)。
    """
    sys.path.insert(0, str(Path(__file__).parent.parent / "arm"))
    from futaba_servo import FutabaServo
    from picamera2 import Picamera2
    from PIL import Image

    with FutabaServo() as servo:
        servo.set_torque(True)
        servo.set_angle(0.0, move_time_ms=500)
        time.sleep(0.8)  # サーボ静定待ち

        cam = Picamera2()
        still_config = cam.create_still_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"}
        )
        cam.configure(still_config)
        cam.start()
        time.sleep(1.0)  # 露出安定待ち
        try:
            array = cam.capture_array("main")
        finally:
            cam.stop()

    img = Image.fromarray(array[:, :, ::-1]).convert("RGB")

    log_dir = Path(config.log_dir) / "camera"
    log_dir.mkdir(parents=True, exist_ok=True)
    path = log_dir / "latest.jpg"
    img.save(path, format="JPEG", quality=95)
    print(f"saved: {path} ({img.width}x{img.height})")


# camera-sweep: 較正用に撮影する (角度[deg], 横位置ズレ[mm]) の組み合わせ。
# 角度・横位置ズレは基準姿勢(現在向いている方向・セル中心)からの相対値。
CAMERA_SWEEP_ANGLES_DEG = [-10, 0, 10]
CAMERA_SWEEP_LATERALS_MM = [-25, 0, 25]


def cmd_camera_sweep(base: MobileBase, config: MicromouseConfig, args) -> None:
    """既知の角度・横位置ズレのパターンで機体を動かし、カメラ較正用の画像セットを撮影する。

    実行前に機体をセル中心・直進方向(基準姿勢)に置いておくこと。
    横位置ズレは「90度旋回→直進→-90度旋回」で作る(2輪差動駆動には横移動
    (ストレイフ)機構が無いため)。各サンプル撮影後は同じ手順を逆順で辿り、
    毎回この基準姿勢に戻ってから次のサンプルへ進む(誤差の累積を防ぐため、
    サンプル間で移動を継ぎ足さない)。

    TURN/STOP は指令値通りに正確には止まらない(実測: 指令-10度に対し
    実測-11〜-14度程度、2026-07-24確認)。そのため較正の正解値には指令値
    ではなく、旋回・移動直後に機体自身のジャイロ(角度)・エンコーダ(距離)
    から読んだ実測オドメトリを使う: 角度は sweep 開始時に1回だけ reset_angle()
    した後の累積 odo_ang_rad(=基準姿勢からの絶対角度)、横ズレは各シフト
    直前に reset_distance() し直してから読む odo_dist_mm(=そのシフトで
    実際に移動した距離)。manifest には指令値(*_nominal)と実測値の両方を
    残すが、較正にはactualの方を使うこと。

    カメラ・Futabaサーボの扱いは cmd_camera_capture と同じ
    (mob/ESP32とは独立した経路)。撮影した画像とメタデータは
    logs/camera/calib/ 以下に残す(較正用データセットのため、
    camera-capture と異なり蓄積する方針)。
    """
    import json
    import math

    check_battery(base, config)
    sys.path.insert(0, str(Path(__file__).parent.parent / "arm"))
    from futaba_servo import FutabaServo
    from picamera2 import Picamera2
    from PIL import Image

    speed = config.explore_speed_mmps
    accel = config.explore_accel_mmps2

    log_dir = Path(config.log_dir) / "camera" / "calib"
    log_dir.mkdir(parents=True, exist_ok=True)
    manifest = []

    base.reset_angle()
    base.reset_distance()

    with FutabaServo() as servo:
        servo.set_torque(True)
        servo.set_angle(0.0, move_time_ms=500)
        time.sleep(0.8)  # サーボ静定待ち

        cam = Picamera2()
        still_config = cam.create_still_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"}
        )
        cam.configure(still_config)
        cam.start()
        time.sleep(1.0)  # 露出安定待ち

        try:
            for lateral_mm in CAMERA_SWEEP_LATERALS_MM:
                shift_sign = 1.0 if lateral_mm >= 0 else -1.0
                lateral_mm_actual = 0.0
                if lateral_mm != 0:
                    base.turn(shift_sign * math.pi / 2)
                    base.reset_distance()
                    base.stop_at(speed, accel, abs(lateral_mm))
                    shift_frame = base.read_sensors()
                    measured = shift_frame.odo_dist_mm if shift_frame is not None else abs(lateral_mm)
                    lateral_mm_actual = shift_sign * measured
                    base.turn(-shift_sign * math.pi / 2)  # 基準姿勢の向きに戻す(横にはズレたまま)

                for angle_deg in CAMERA_SWEEP_ANGLES_DEG:
                    if angle_deg != 0:
                        base.turn(math.radians(angle_deg))

                    time.sleep(0.3)  # 撮影姿勢での露出再安定待ち
                    array = cam.capture_array("main")
                    img = Image.fromarray(array[:, :, ::-1]).convert("RGB")
                    name = f"calib_a{angle_deg:+03d}_l{lateral_mm:+03d}.jpg"
                    img.save(log_dir / name, format="JPEG", quality=95)

                    frame = base.read_sensors()
                    entry = {
                        "file": name,
                        "angle_deg_nominal": angle_deg,
                        "lateral_mm_nominal": lateral_mm,
                        "angle_deg_actual": math.degrees(frame.odo_ang_rad) if frame is not None else None,
                        "lateral_mm_actual": lateral_mm_actual,
                    }
                    manifest.append(entry)
                    print(f"saved {name}: {entry}")

                    if angle_deg != 0:
                        base.turn(-math.radians(angle_deg))

                if lateral_mm != 0:
                    base.turn(-shift_sign * math.pi / 2)
                    base.reset_distance()
                    base.stop_at(speed, accel, abs(lateral_mm))
                    base.turn(shift_sign * math.pi / 2)
        finally:
            cam.stop()

    manifest_path = log_dir / "manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"done: {len(manifest)} images, manifest: {manifest_path}")


# camera-sweep-distance: 較正用に撮影する、基準姿勢からの累積前進距離[mm]。
# 角度は変えず前進方向にのみ動かす(camera-sweepの横位置ズレとは直交する軸)。
# 壁までの実際の余裕距離が不明なため保守的に小さめの値にしてある。
CAMERA_DIST_SWEEP_STEPS_MM = [0, 10, 20, 30]


def cmd_camera_sweep_distance(base: MobileBase, config: MicromouseConfig, args) -> None:
    """既知の前進距離のパターンで機体を動かし、壁との距離較正用の画像セットを撮影する。

    実行前に機体をセル中心・直進方向(基準姿勢)に置いておくこと。角度・
    横位置ズレは変えず、前進方向にのみ既知の距離だけ進めながら撮影する。
    最後に180度旋回して合計移動距離ぶん戻り、元の位置・向きへ復帰する
    (camera-sweepの横ズレ用90度旋回トリックと同じ考え方)。

    カメラ・Futabaサーボの扱いは camera-capture と同じ(mob/ESP32とは
    独立した経路)。撮影した画像とメタデータ(指令の累積距離、実測
    オドメトリ)は logs/camera/calib_dist/ 以下に残す(較正用データセット
    のため蓄積する方針)。
    """
    import json
    import math

    check_battery(base, config)
    sys.path.insert(0, str(Path(__file__).parent.parent / "arm"))
    from futaba_servo import FutabaServo
    from picamera2 import Picamera2
    from PIL import Image

    speed = config.explore_speed_mmps
    accel = config.explore_accel_mmps2

    log_dir = Path(config.log_dir) / "camera" / "calib_dist"
    log_dir.mkdir(parents=True, exist_ok=True)
    manifest = []

    base.reset_distance()

    with FutabaServo() as servo:
        servo.set_torque(True)
        servo.set_angle(0.0, move_time_ms=500)
        time.sleep(0.8)  # サーボ静定待ち

        cam = Picamera2()
        still_config = cam.create_still_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"}
        )
        cam.configure(still_config)
        cam.start()
        time.sleep(1.0)  # 露出安定待ち

        try:
            prev_target_mm = 0
            for target_mm in CAMERA_DIST_SWEEP_STEPS_MM:
                increment_mm = target_mm - prev_target_mm
                if increment_mm > 0:
                    base.stop_at(speed, accel, increment_mm)
                prev_target_mm = target_mm

                time.sleep(0.3)  # 撮影姿勢での露出再安定待ち
                array = cam.capture_array("main")
                img = Image.fromarray(array[:, :, ::-1]).convert("RGB")
                name = f"calib_dist_d{target_mm:+04d}.jpg"
                img.save(log_dir / name, format="JPEG", quality=95)

                frame = base.read_sensors()
                entry = {
                    "file": name,
                    "target_mm": target_mm,
                    "dist_mm_actual": frame.odo_dist_mm if frame is not None else None,
                }
                manifest.append(entry)
                print(f"saved {name}: {entry}")

            total_mm = prev_target_mm
            if total_mm > 0:
                base.turn(math.pi)
                base.stop_at(speed, accel, total_mm)
                base.turn(math.pi)
        finally:
            cam.stop()

    manifest_path = log_dir / "manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, ensure_ascii=False, indent=2)
    print(f"done: {len(manifest)} images, manifest: {manifest_path}")


def cmd_camera_correct(base: MobileBase, config: MicromouseConfig, args) -> None:
    """カメラで壁上面(赤帯)を撮影し、推定したヨー角でmobの角度(SANG)を補正する。

    現状の較正式は camera-sweep で撮った9点のみの経験的フィットで精度は
    未検証(TODO.md参照)。探索走行本体(state_machine.py)での自動組み込みは
    camera_correction.CameraCorrector が行う。こちらは手動で1回試すための
    実装で、角度のみを補正し機体は動かさない。
    """
    import math

    from camera_correction import CAMERA_CORRECTION_MAX_YAW_DEG, CameraCorrector, estimate_pose, is_confident

    check_battery(base, config)

    frame_before = base.read_sensors()
    print(f"補正前 odo_ang_rad = {frame_before.odo_ang_rad if frame_before is not None else None}")

    corrector = CameraCorrector(args.width, args.height)
    try:
        estimate = estimate_pose(corrector.capture())
    finally:
        corrector.close()

    if estimate is None:
        print("赤帯を検出できませんでした。補正は行いません。")
        return

    print(
        f"検出: 推定ヨー角={estimate.yaw_deg:.2f}deg "
        f"n={estimate.inlier_count} resid={estimate.residual_px:.2f}px"
    )
    if not is_confident(estimate):
        print(f"信頼度ゲートで棄却(|yaw|>{CAMERA_CORRECTION_MAX_YAW_DEG}度 等)。補正は行いません。")
        return

    yaw_rad = math.radians(estimate.yaw_deg)
    print(f"推定ヨー角: {estimate.yaw_deg:.2f}deg ({yaw_rad:.4f}rad) -> correct_angle()")
    base.correct_angle(yaw_rad)

    frame_after = base.read_sensors()
    print(f"補正後 odo_ang_rad = {frame_after.odo_ang_rad if frame_after is not None else None}")


CAMERA_STRAIGHTEN_DEFAULT_ITERATIONS = 3


def cmd_camera_straighten(base: MobileBase, config: MicromouseConfig, args) -> None:
    """カメラで推定したヨー角・壁との距離ぶん機体を物理的に動かし、
    まっすぐ・基準距離に戻す。これを複数回(既定3回)繰り返す。

    camera-correct は mob 内部の角度(SANG)を書き換えるだけで機体は
    動かさない(本来の使い方: 判断点でのオドメトリ補正)。こちらは
    実際に姿勢が戻ることをその場で確認するためのデモ用に、まず推定
    ヨー角の分だけ JOGTURN で低速旋回・角度を補正してから、その後の
    推定距離オフセットの分だけ JOGFWD/JOGBACK で低速前後移動・距離を
    補正する(角度がズレたままだと距離推定も乱れるため、角度→距離の
    順で行う)。通常のTURN/STOPは速度が高くオーバーシュート・慣性が
    残差の一因になっていたため、低速固定速度のJOG系コマンドに変更した
    (2026-07-24)。

    較正済み範囲(±10度程度)を大きく超えるズレでは1回の推定・補正が
    不正確なことがある(2026-07-24 実機で確認: 残差145px相当の壊れた
    推定から旋回した結果、間違った角度(+17度付近)に収束してしまった
    例あり)。camera_correction.is_confident() による信頼度ゲートで、
    較正範囲から大きく外れた推定はスキップするようにして再発を防ぐ。
    ゲートを通った推定については、複数回繰り返すことで徐々に真値へ
    収束することを期待する(過去の手動再実行での収束を踏まえた設計)。
    較正式はどちらも暫定(TODO.md参照)。
    """
    import math

    from camera_correction import CameraCorrector, estimate_pose, is_confident

    check_battery(base, config)
    n = args.iterations

    corrector = CameraCorrector(args.width, args.height)
    last_estimate = None
    try:
        for i in range(n):
            estimate = estimate_pose(corrector.capture())
            if estimate is None:
                print(f"[{i + 1}/{n}] 赤帯を検出できませんでした。ここで打ち切ります。")
                break
            print(
                f"[{i + 1}/{n}] 補正前: 推定ヨー角={estimate.yaw_deg:.2f}deg "
                f"推定距離オフセット={estimate.dist_offset_mm:.1f}mm (resid={estimate.residual_px:.2f}px)"
            )
            if not is_confident(estimate):
                print(f"[{i + 1}/{n}] 信頼度ゲートで棄却。この回は補正しません。")
                last_estimate = estimate
                continue

            base.jog_turn(math.radians(-estimate.yaw_deg))
            base.correct_angle(0.0)  # ここを新しい基準(まっすぐ)とする

            time.sleep(0.3)  # 露出再安定待ち
            dist_estimate = estimate_pose(corrector.capture())
            if dist_estimate is None:
                print(f"[{i + 1}/{n}] 角度補正後、赤帯を検出できませんでした。距離補正は行いません。")
                last_estimate = dist_estimate
                continue
            if not is_confident(dist_estimate):
                print(f"[{i + 1}/{n}] 角度補正後の距離推定が信頼度ゲートで棄却。距離補正は行いません。")
                last_estimate = dist_estimate
                continue

            if dist_estimate.dist_offset_mm > 0:
                base.jog_forward(dist_estimate.dist_offset_mm)
            elif dist_estimate.dist_offset_mm < 0:
                base.jog_backward(abs(dist_estimate.dist_offset_mm))
            base.reset_distance()  # ここを新しい基準距離とする

            time.sleep(0.3)  # 露出再安定待ち
            last_estimate = estimate_pose(corrector.capture())
            if last_estimate is not None:
                print(
                    f"[{i + 1}/{n}] 補正後: 推定ヨー角={last_estimate.yaw_deg:.2f}deg "
                    f"推定距離オフセット={last_estimate.dist_offset_mm:.1f}mm "
                    f"(resid={last_estimate.residual_px:.2f}px)"
                )
    finally:
        corrector.close()

    if last_estimate is None:
        print("最終的に赤帯を検出できませんでした。")
        return
    print(
        f"最終結果: 推定ヨー角={last_estimate.yaw_deg:.2f}deg "
        f"推定距離オフセット={last_estimate.dist_offset_mm:.1f}mm "
        f"(resid={last_estimate.residual_px:.2f}px)"
    )


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
    p_cam = sub.add_parser("camera-capture")
    p_cam.add_argument("--width", type=int, default=2304)
    p_cam.add_argument("--height", type=int, default=1296)
    p_sweep = sub.add_parser("camera-sweep")
    p_sweep.add_argument("--width", type=int, default=2304)
    p_sweep.add_argument("--height", type=int, default=1296)
    p_sweep_dist = sub.add_parser("camera-sweep-distance")
    p_sweep_dist.add_argument("--width", type=int, default=2304)
    p_sweep_dist.add_argument("--height", type=int, default=1296)
    p_correct = sub.add_parser("camera-correct")
    p_correct.add_argument("--width", type=int, default=2304)
    p_correct.add_argument("--height", type=int, default=1296)
    p_straighten = sub.add_parser("camera-straighten")
    p_straighten.add_argument("--width", type=int, default=2304)
    p_straighten.add_argument("--height", type=int, default=1296)
    p_straighten.add_argument(
        "--iterations", type=int, default=CAMERA_STRAIGHTEN_DEFAULT_ITERATIONS
    )

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
        "camera-capture": cmd_camera_capture,
        "camera-sweep": cmd_camera_sweep,
        "camera-sweep-distance": cmd_camera_sweep_distance,
        "camera-correct": cmd_camera_correct,
        "camera-straighten": cmd_camera_straighten,
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
