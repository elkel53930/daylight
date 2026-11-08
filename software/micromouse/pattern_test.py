#!/usr/bin/env python3
"""固定走行パターンテスト。

迷路探索・センサ判定を一切使わず、決め打ちの動作を実行する実機テスト。
hw_test.py の個別コマンド(fwd/turn)を単発で確認するのとは違い、
複数の旋回を連続実行したときの誤差累積(オーバーシュート・振動など)を
見るためのもの。

走行パターン: 1マス直進 → 右旋回 → 1マス直進 → 左旋回 → 2マス直進
→ 左旋回 → 1マス直進 → 左旋回 → 3マス直進 → 180度旋回。
開始位置・開始向きに戻る閉路(誤差累積がそのまま最終位置ずれに出る)。
(旋回のみの3ステップ版は git 履歴を参照。旋回チューニング完了
(2026-07-24、左右速度同期の再有効化)を受けて直進を組み込んだ。)

各区間は cell_runner.CellRunner.run_motion() を使う(TURN系はその場
旋回のみで、STRAIGHTのような速度プロファイルは使わない)。

実行:
    python3 pattern_test.py                    # 実機、既定は explore 速度
    python3 pattern_test.py --speed 400 --accel 1200
    python3 pattern_test.py --no-ui            # ui_server 無し(コンソール)
    python3 pattern_test.py --no-ui --autostart  # 確認なしで即走行

default_app の Applications メニューからも起動できる
(config/applications.yaml.example と /etc/robot-ui/applications.yaml
参照)。
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

sys.path.insert(0, str(Path(__file__).parent))

from cell_runner import CellRunner
from config import MicromouseConfig
from errors import AbortRequested, MobileBaseError
from path_planner import Motion, MotionType

# ui_client は software/ui にある(リポジトリレイアウトで自動解決)
sys.path.insert(0, str(Path(__file__).parent.parent / "ui"))

MELODY_START = "ceg"
MELODY_FINISH = "CCGG"
MELODY_ERROR = "CcCc"

# 固定テストパターン(壁センサFB確認用: 壁付きコリドーで3マス直進のみ。
# 閉路パターンは git 履歴を参照)
PATTERN: List[Motion] = [
    Motion(MotionType.STRAIGHT, 3),
]


class PatternLogger:
    """mob からの生シリアル行(#V テレメトリ含む)を JSONL に記録する。

    mobile_base.MobileBase は SEN/DONE 以外の行(#V,... など)を
    _wait_for() 内の一時バッファ(直近5行、タイムアウト時のみ表示)
    以外には残さないため、raw_log_fn 経由でここに逐次書き出す。
    """

    def __init__(self, log_dir: Path):
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = log_dir / f"pattern_{stamp}.jsonl"
        self._f = open(self.path, "a", encoding="utf-8")
        self.step: Optional[int] = None
        self.motion: Optional[str] = None

    def set_context(self, step: Optional[int], motion: Optional[str]) -> None:
        self.step = step
        self.motion = motion

    def log_line(self, line: str) -> None:
        self._write({"event": "serial", "line": line})

    def log_event(self, **record) -> None:
        self._write({"event": "note", **record})

    def _write(self, record: dict) -> None:
        record.setdefault("t", time.time())
        record.setdefault("step", self.step)
        record.setdefault("motion", self.motion)
        self._f.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._f.flush()

    def close(self) -> None:
        self._f.close()


class ConsoleUI:
    """ui_server 無し環境用のフォールバック。

    表示はコンソールのみだが、走行開始メロディだけは(鳴らせる環境なら)
    ui_server 経由でブザーを鳴らす(いつ走り出すか分かりにくいため)。
    """

    def __init__(self, autostart: bool):
        self.autostart = autostart
        self._buzzer = None

    def notify_start(self) -> None:
        print("走行開始...")
        try:
            from ui_client import UIClient

            if self._buzzer is None:
                self._buzzer = UIClient()
                self._buzzer.connect(priority=90)
            self._buzzer.play(MELODY_START)
        except Exception:
            self._buzzer = None

    def wait_start(self) -> bool:
        if self.autostart:
            return True
        try:
            input("Enter で走行開始 (Ctrl+C で終了): ")
            return True
        except (KeyboardInterrupt, EOFError):
            return False

    def abort_requested(self) -> bool:
        return False  # Ctrl+C (KeyboardInterrupt) で中断する

    def show_progress(self, i: int, total: int, motion: Motion) -> None:
        print(f"[{i + 1}/{total}] {motion!r}")

    def show_result(self, ok: bool, message: str) -> None:
        print(("完了: " if ok else "エラー: ") + message)

    def retry(self) -> bool:
        if self.autostart:
            return False
        try:
            ans = input("もう一度実行しますか? [y/N]: ").strip().lower()
            return ans in ("y", "yes")
        except (KeyboardInterrupt, EOFError):
            return False

    def close(self) -> None:
        if self._buzzer is not None:
            try:
                self._buzzer.disconnect()
            except Exception:
                pass


class OledUI:
    """ui_server 経由の OLED / ボタン / ブザー(micromouse_app.py と同形式)。"""

    POLL_INTERVAL_S = 0.15

    def __init__(self, priority: int = 20):
        from PIL import Image, ImageDraw  # noqa: F401 (依存確認)
        from ui_client import UIClient

        self._Image = Image
        self._ImageDraw = ImageDraw
        self.client = UIClient()
        self.client.connect(priority=priority)
        self._last_poll = 0.0
        self._abort_latched = False

    def _draw(self, lines: List[str]) -> None:
        try:
            img = self._Image.new("RGB", (96, 64), "black")
            d = self._ImageDraw.Draw(img)
            for i, line in enumerate(lines[:5]):
                d.text((2, 2 + i * 12), line, fill="white")
            self.client.display(img)
        except Exception:
            pass

    def _play(self, melody: str) -> None:
        try:
            self.client.play(melody)
        except Exception:
            pass

    def _buttons(self) -> dict:
        try:
            return self.client.get_buttons()
        except Exception:
            return {}

    def _wait_button(self) -> Optional[str]:
        while True:
            b = self._buttons()
            if b.get("left") == "released" and b.get("right") == "released":
                break
            time.sleep(0.05)
        while True:
            b = self._buttons()
            if b.get("right") in ("pressed", "long_pressed"):
                return "right"
            if b.get("left") in ("pressed", "long_pressed"):
                return "left"
            time.sleep(0.05)

    def wait_start(self) -> bool:
        self._abort_latched = False
        self._draw(["PATTERN TEST", "", "R: Start", "L: Quit"])
        return self._wait_button() == "right"

    def abort_requested(self) -> bool:
        if self._abort_latched:
            return True
        now = time.monotonic()
        if now - self._last_poll < self.POLL_INTERVAL_S:
            return False
        self._last_poll = now
        if self._buttons().get("left") in ("pressed", "long_pressed"):
            self._abort_latched = True
            return True
        return False

    def notify_start(self) -> None:
        self._draw(["PATTERN TEST", "", "starting..."])
        self._play(MELODY_START)

    def show_progress(self, i: int, total: int, motion: Motion) -> None:
        self._draw(["PATTERN TEST", f"step {i + 1}/{total}", repr(motion), ""])

    def show_result(self, ok: bool, message: str) -> None:
        self._play(MELODY_FINISH if ok else MELODY_ERROR)
        head = "DONE" if ok else "ERROR"
        lines = [message[i:i + 16] for i in range(0, len(message), 16)]
        self._draw([head] + lines[:2] + ["R: Retry  L: Quit"])

    def retry(self) -> bool:
        return self._wait_button() == "right"

    def close(self) -> None:
        try:
            self.client.disconnect()
        except Exception:
            pass


def run_pattern(
    base: MobileBase,
    config: MicromouseConfig,
    ui,
    logger: Optional[PatternLogger] = None,
) -> Tuple[bool, str]:
    """固定パターンを1回走行する。戻り値は (成功したか, メッセージ)。"""
    frame = base.read_sensors()
    if frame is None:
        return False, "sensor read failed"
    if frame.vbatt < config.battery_min_v:
        return False, f"battery low: {frame.vbatt:.2f}V"

    base.gyro_calibrate()
    base.reset_distance()
    base.reset_angle()
    # 壁センサLEDを点灯(壁FBの実機確認用。壁が無い環境ではセンサ値が
    # 閾値未満になり補正0なので挙動は変わらない)
    base.wall_led(True)

    # 走行開始をメロディで予告し、1秒置いてから動き出す
    ui.notify_start()
    time.sleep(1.0)

    runner = CellRunner(base, config)
    total = len(PATTERN)
    for i, motion in enumerate(PATTERN):
        if logger is not None:
            logger.set_context(i, repr(motion))
            logger.log_event(event="motion_start", index=i, total=total)
        ui.show_progress(i, total, motion)
        runner.run_motion(motion)

    if logger is not None:
        logger.set_context(None, None)

    # mob側はSTOP/TURN完了後0.5秒の角度維持ホールドを終えてからDONEを
    # 返すため、この時点で機体は整定済み
    frame = base.read_sensors()
    if frame is None:
        return True, "done (odo read failed)"
    return True, (
        f"odo {frame.odo_dist_mm:.0f}mm {math.degrees(frame.odo_ang_rad):+.1f}deg"
    )


def run_once(
    args, config: MicromouseConfig, ui, logger: Optional[PatternLogger] = None
) -> Tuple[bool, str]:
    from mobile_base import MobileBase

    base = MobileBase(
        args.port or config.port,
        args.baud or config.baud,
        timeout_s=config.command_timeout_s,
        abort_check=ui.abort_requested,
        raw_log_fn=(logger.log_line if logger is not None else None),
    )
    try:
        return run_pattern(base, config, ui, logger)
    except AbortRequested:
        return False, "aborted by user"
    except MobileBaseError as e:
        base.emergency_stop()
        return False, str(e)
    except Exception as e:
        base.emergency_stop()
        return False, f"unexpected: {e}"
    finally:
        try:
            base.wall_led(False)
        except Exception:
            pass
        base.motors_off()
        base.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="Daylight micromouse fixed pattern test")
    ap.add_argument("--port", default=None, help="mob シリアルポート(既定は設定値)")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--config", type=Path, default=None, help="設定 YAML のパス")
    ap.add_argument("--speed", type=float, default=None,
                    help="直進速度 mm/s(既定は config の explore_speed_mmps)")
    ap.add_argument("--accel", type=float, default=None,
                    help="加減速度 mm/s^2(既定は config の explore_accel_mmps2)")
    ap.add_argument("--no-ui", action="store_true", help="ui_server を使わない")
    ap.add_argument("--autostart", action="store_true",
                    help="ボタン/入力を待たず即開始・即終了")
    args = ap.parse_args()

    config = MicromouseConfig.load(args.config)
    config = dataclasses.replace(
        config,
        speed_run_speed_mmps=args.speed or config.explore_speed_mmps,
        speed_run_accel_mmps2=args.accel or config.explore_accel_mmps2,
    )

    ui = None
    if not args.no_ui:
        try:
            ui = OledUI()
        except Exception as e:
            print(f"UI サーバーに接続できないためコンソールモードで動作: {e}")
    if ui is None:
        ui = ConsoleUI(autostart=args.autostart)

    try:
        while True:
            if not ui.wait_start():
                return 0
            logger = PatternLogger(Path(config.log_dir))
            try:
                ok, message = run_once(args, config, ui, logger)
            finally:
                logger.close()
            ui.show_result(ok, message)
            if isinstance(ui, ConsoleUI):
                if not ui.retry():
                    return 0 if ok else 1
                continue
            if not ui.retry():
                return 0
    finally:
        ui.close()


if __name__ == "__main__":
    raise SystemExit(main())
