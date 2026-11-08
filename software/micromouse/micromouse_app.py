#!/usr/bin/env python3
"""マイクロマウスアプリケーション(エントリポイント)。

実機:
    python3 micromouse_app.py --port /dev/ttyUSB0

シミュレーション(ハードウェア不要):
    python3 micromouse_app.py --sim maze_files/AllJapan_002_1981_classic___16x16.txt \
        --no-ui --autostart

UI(ui_server)がある場合は OLED に状態を表示し、
    R ボタン: 開始 / 最短走行の承認
    L ボタン: 中断(EMERGENCY_STOP) / 終了
UI が無い場合(--no-ui)はコンソールで Enter 開始 / Ctrl+C 中断。

既存の手動操作機能(default_app)とはモードとして分離されており、
本アプリは default_app のメニューから起動されると ui_server の所有権を
引き継ぎ、終了すると default_app に返す。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).parent))

from config import MicromouseConfig
from maze import load_maze_text
from state_machine import MicromouseMission, MissionState

# ui_client は software/ui にある(リポジトリレイアウトで自動解決)
sys.path.insert(0, str(Path(__file__).parent.parent / "ui"))

MELODY_START = "ceg"
MELODY_GOAL = "gCEG"
MELODY_FINISH = "CCGG"
MELODY_ERROR = "CcCc"


class RunLogger:
    """JSONL 走行ログ + 迷路スナップショット保存。"""

    def __init__(self, log_dir: Path):
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.jsonl_path = log_dir / f"run_{stamp}.jsonl"
        self.maze_path = log_dir / f"maze_{stamp}.json"
        self._f = open(self.jsonl_path, "a", encoding="utf-8")

    def log(self, record: dict) -> None:
        self._f.write(json.dumps(record, ensure_ascii=False, default=str) + "\n")
        self._f.flush()

    def save_maze(self, maze) -> None:
        with open(self.maze_path, "w", encoding="utf-8") as f:
            json.dump(maze.to_dict(), f)

    def close(self) -> None:
        self._f.close()


class ConsoleUI:
    """UI サーバー無し環境用のフォールバック。"""

    def __init__(self, autostart: bool):
        self.autostart = autostart

    def wait_start(self) -> bool:
        if self.autostart:
            return True
        try:
            input("Enter で走行開始 (Ctrl+C で終了): ")
            return True
        except (KeyboardInterrupt, EOFError):
            return False

    def confirm_speed_run(self) -> bool:
        if self.autostart:
            return True
        try:
            ans = input("最短走行を開始しますか? [Y/n]: ").strip().lower()
            return ans in ("", "y", "yes")
        except (KeyboardInterrupt, EOFError):
            return False

    def abort_requested(self) -> bool:
        return False  # コンソールでは Ctrl+C (KeyboardInterrupt) で中断

    def show_state(self, state: MissionState, info: dict, mission) -> None:
        print(f"[STATE] {state.value} {info if info else ''}")

    def show_message(self, *lines: str) -> None:
        for line in lines:
            print(line)

    def close(self) -> None:
        pass


class OledUI:
    """ui_server 経由の OLED / ボタン / ブザー。

    UI 通信の失敗で走行を止めないよう、表示系のエラーは全て握りつぶす
    (中断判定 abort_requested だけは失敗時 False を返し、最終的な
    安全は Ctrl+C とミッション側の異常検出に任せる)。
    """

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

    def _draw(self, lines: list[str]) -> None:
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
        """L/R どちらかが押されるまで待つ。押された方を返す。"""
        # ボタンが離されるのを待ってから新しい押下を検出する
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
        self._abort_latched = False  # 前回走行の中断フラグを持ち越さない
        self._draw(["MICROMOUSE", "", "R: Start", "L: Quit"])
        return self._wait_button() == "right"

    def confirm_speed_run(self) -> bool:
        self._play(MELODY_GOAL)
        self._draw(["PATH READY", "", "R: Speed run", "L: Skip"])
        ok = self._wait_button() == "right"
        self._abort_latched = False  # 承認待ちの L は中断ではない
        return ok

    def abort_requested(self) -> bool:
        """走行中の中断判定(base の abort_check から呼ばれる)。"""
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

    def show_state(self, state: MissionState, info: dict, mission) -> None:
        pose = mission.pose
        batt = ""
        frame = getattr(mission.base, "last_frame", None)
        if frame is not None:
            batt = f"{frame.vbatt:.2f}V"
        self._draw(
            [
                state.value[:16],
                f"cell ({pose.x},{pose.y}) {pose.heading.name[0]}",
                f"step {mission.step_count}",
                batt,
            ]
        )
        if state == MissionState.EXPLORATION and mission.step_count == 0:
            self._play(MELODY_START)
        elif state == MissionState.FINISHED:
            self._play(MELODY_FINISH)
        elif state in (MissionState.ERROR, MissionState.EMERGENCY_STOP):
            self._play(MELODY_ERROR)

    def show_message(self, *lines: str) -> None:
        self._draw(list(lines))

    def close(self) -> None:
        try:
            self.client.disconnect()
        except Exception:
            pass


def make_base(args, config: MicromouseConfig, abort_check):
    """実機 or シミュレータの走行ベースを生成する。"""
    if args.sim:
        from simulator import SimMobileBase

        text = Path(args.sim).read_text(encoding="utf-8")
        true_maze, _ = load_maze_text(text, size=config.maze_size)
        return SimMobileBase(
            true_maze, cell_size_mm=config.cell_size_mm, abort_check=abort_check
        )

    from mobile_base import MobileBase

    return MobileBase(
        args.port or config.port,
        args.baud or config.baud,
        timeout_s=config.command_timeout_s,
        abort_check=abort_check,
    )


def run_mission(args, config: MicromouseConfig, ui) -> tuple[MissionState, str]:
    logger = RunLogger(Path(config.log_dir))
    base = make_base(args, config, ui.abort_requested)
    mission = MicromouseMission(
        base,
        config,
        observer=lambda state, info: ui.show_state(state, info, mission),
        log_fn=logger.log,
        confirm_speed_run=ui.confirm_speed_run,
    )
    try:
        final = mission.run()
    except KeyboardInterrupt:
        base.emergency_stop()
        final = MissionState.EMERGENCY_STOP
        print("\n中断しました(モータ停止済み)")
    finally:
        logger.save_maze(mission.maze)
        logger.close()
        base.close()

    print(f"\n=== 結果: {final.value} ===")
    if mission.error_message:
        print(f"エラー: {mission.error_message}")
    print(mission.maze.render_text(pose=mission.pose, goal=mission.goal))
    print(f"走行ログ: {logger.jsonl_path}")
    return final, mission.error_message


def main() -> int:
    ap = argparse.ArgumentParser(description="Daylight micromouse")
    ap.add_argument("--port", default=None, help="mob シリアルポート(既定は設定値)")
    ap.add_argument("--baud", type=int, default=None)
    ap.add_argument("--config", type=Path, default=None, help="設定 YAML のパス")
    ap.add_argument("--sim", metavar="MAZE_FILE", default=None,
                    help="シミュレーションモード(迷路テキストファイル)")
    ap.add_argument("--no-ui", action="store_true", help="ui_server を使わない")
    ap.add_argument("--autostart", action="store_true",
                    help="ボタン/入力を待たず即開始(最短走行も自動承認)")
    args = ap.parse_args()

    config = MicromouseConfig.load(args.config)

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
            final, error_message = run_mission(args, config, ui)
            if error_message:
                # OLED は1行約16文字。エラー理由を2行まで表示する
                lines = [f"DONE: {final.value}"[:16]]
                lines += [error_message[i:i + 16] for i in (0, 16)]
                lines += ["R: Retry", "L: Quit"]
                ui.show_message(*[l for l in lines if l][:5])
            else:
                ui.show_message("DONE:", final.value, "", "R: Retry", "L: Quit")
            if isinstance(ui, ConsoleUI):
                return 0 if final == MissionState.FINISHED else 1
            # OLED: ボタンで再走行か終了かを選ぶ
            if ui._wait_button() != "right":
                return 0
    finally:
        ui.close()


if __name__ == "__main__":
    raise SystemExit(main())
