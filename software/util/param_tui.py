#!/usr/bin/env python3
"""mob (ESP32-S3) のチューニングパラメータをその場で調整するTUIツール。

PGET/PSET/PSAVE/PLOAD/PRESET(software/mob/params.h)を操作する。
パラメータ一覧はハードコードせず起動時に PGET で mob から取得するため、
ファームウェア側で params.h にパラメータを追加/削除しても本ツールの
修正は不要。SEN の値も上部に常時表示する。

使い方:
    software/venv/bin/python3 software/util/param_tui.py

キー操作:
    ↑/↓        パラメータ選択
    ←/→        選択中の値を ±ステップ で変更(即PSET、RAMのみ反映)
    Enter       選択中の値を直接入力して変更(即PSET)
    [ / ]       ステップを ÷10 / ×10
    s           PSAVE(現在値を機体のNVSへ恒久保存)
    l           PLOAD(NVSから再読込)
    r           PRESET(RAM値をビルド時デフォルトへ、NVSは無変更)
    g           PGET で一覧を再取得(表示をmobの実際値に同期)
    /           名前でフィルタ(Enterで確定、Escで解除)
    q           終了
"""

from __future__ import annotations

import argparse
import curses
import time
from dataclasses import dataclass
from typing import Optional

import serial


@dataclass
class SenFrame:
    gyro: float
    vbatt: float
    lf: int
    ls: int
    rs: int
    rf: int
    enc_r: int
    enc_l: int
    odo_dist: float
    odo_ang: float
    ball_raw: int
    ball_det: int


def parse_sen_line(line: str) -> Optional[SenFrame]:
    parts = line.split(",")
    if len(parts) != 13 or parts[0] != "SEN":
        return None
    try:
        return SenFrame(
            gyro=float(parts[1]),
            vbatt=float(parts[2]),
            lf=int(parts[3]),
            ls=int(parts[4]),
            rs=int(parts[5]),
            rf=int(parts[6]),
            enc_r=int(parts[7]),
            enc_l=int(parts[8]),
            odo_dist=float(parts[9]),
            odo_ang=float(parts[10]),
            ball_raw=int(parts[11]),
            ball_det=int(parts[12]),
        )
    except ValueError:
        return None


class MobLink:
    """mob との同期シリアル通信(このツール専用の薄いラッパー)。"""

    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        self.ser.reset_input_buffer()

    def close(self) -> None:
        self.ser.close()

    def _readline(self, timeout_s: float) -> Optional[str]:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self.ser.readline()
            if raw:
                return raw.decode("ascii", errors="replace").strip()
        return None

    def fetch_params(self) -> list[tuple[str, float]]:
        """PGET を発行し (name, value) のリストを順序を保って返す。"""
        self.ser.reset_input_buffer()
        self.ser.write(b"PGET\n")
        result: list[tuple[str, float]] = []
        while True:
            line = self._readline(timeout_s=2.0)
            if line is None or line == "PLISTEND":
                break
            if line.startswith("PVAL,"):
                _, name, value = line.split(",", 2)
                try:
                    result.append((name, float(value)))
                except ValueError:
                    continue
        return result

    def set_param(self, name: str, value: float) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(f"PSET,{name},{value}\n".encode("ascii"))
        line = self._readline(timeout_s=1.0)
        return line or "(no response)"

    def save(self) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(b"PSAVE\n")
        line = self._readline(timeout_s=2.0)
        return line or "(no response)"

    def load(self) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(b"PLOAD\n")
        lines = []
        while True:
            line = self._readline(timeout_s=2.0)
            if line is None:
                break
            lines.append(line)
            if line == "DONE":
                break
        return " / ".join(lines) if lines else "(no response)"

    def reset_defaults(self) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(b"PRESET\n")
        line = self._readline(timeout_s=2.0)
        return line or "(no response)"

    def read_sen(self) -> Optional[SenFrame]:
        self.ser.reset_input_buffer()
        self.ser.write(b"SEN\n")
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            line = self._readline(timeout_s=0.5)
            if line is None:
                break
            if line.startswith("SEN,"):
                return parse_sen_line(line)
        return None


def fmt_value(v: float) -> str:
    return f"{v:.6g}"


def draw(
    stdscr,
    port: str,
    step: float,
    status: str,
    sen: Optional[SenFrame],
    names: list[str],
    values: dict[str, float],
    selected: int,
    scroll: int,
    filter_text: str,
    filtering: bool,
    editing: bool,
    edit_buf: str,
) -> None:
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    stdscr.addnstr(0, 0, f" mob param tui  port={port}  step={fmt_value(step)}", w - 1, curses.A_REVERSE)

    row = 1
    if sen is not None:
        stdscr.addnstr(
            row, 0,
            f" SEN vbatt={sen.vbatt:.2f}V gyro={sen.gyro:+.3f} "
            f"lf={sen.lf} ls={sen.ls} rs={sen.rs} rf={sen.rf}",
            w - 1,
        )
        row += 1
        stdscr.addnstr(
            row, 0,
            f"     enc_r={sen.enc_r} enc_l={sen.enc_l} "
            f"dist={sen.odo_dist:.1f}mm ang={sen.odo_ang:+.3f}rad "
            f"ball_raw={sen.ball_raw} ball_det={sen.ball_det}",
            w - 1,
        )
    else:
        stdscr.addnstr(row, 0, " SEN: (no response)", w - 1)
        row += 1
    row += 1

    list_top = row
    list_bottom = h - 3
    list_height = max(1, list_bottom - list_top)

    if selected < scroll:
        scroll = selected
    if selected >= scroll + list_height:
        scroll = selected - list_height + 1

    for i in range(list_height):
        idx = scroll + i
        if idx >= len(names):
            break
        name = names[idx]
        value_str = fmt_value(values[name])
        line = f" {name:<20s} {value_str:>14s}"
        attr = curses.A_REVERSE if idx == selected else curses.A_NORMAL
        stdscr.addnstr(list_top + i, 0, line, w - 1, attr)

    footer1 = " ↑↓:select  ←→:±step  Enter:type value  [ ]:step÷10/×10"
    footer2 = " s:PSAVE  l:PLOAD  r:PRESET  g:refresh  /:filter  q:quit"
    stdscr.addnstr(h - 2, 0, footer1, w - 1)

    if editing:
        current = fmt_value(values[names[selected]]) if names else "?"
        stdscr.addnstr(h - 1, 0, f" new value (current={current}): {edit_buf}", w - 1, curses.A_REVERSE)
    elif filtering:
        stdscr.addnstr(h - 1, 0, f" filter: {filter_text}", w - 1, curses.A_REVERSE)
    else:
        stdscr.addnstr(h - 1, 0, f" {footer2}  | {status}", w - 1)

    stdscr.refresh()


def run(stdscr, link: MobLink, port: str) -> None:
    curses.curs_set(0)
    stdscr.timeout(150)

    order = link.fetch_params()
    values: dict[str, float] = dict(order)
    all_names = [name for name, _ in order]
    names = list(all_names)

    status = f"Loaded {len(all_names)} params"
    selected = 0
    scroll = 0
    step = 1.0
    filter_text = ""
    filtering = False
    editing = False
    edit_buf = ""

    sen: Optional[SenFrame] = None
    last_sen_t = 0.0
    sen_interval_s = 0.2

    while True:
        now = time.monotonic()
        if now - last_sen_t >= sen_interval_s:
            sen = link.read_sen()
            last_sen_t = now

        draw(
            stdscr, port, step, status, sen,
            names, values, selected, scroll,
            filter_text, filtering, editing, edit_buf,
        )

        ch = stdscr.getch()
        if ch == -1:
            continue

        if editing:
            if ch in (10, 13, curses.KEY_ENTER):
                if names and edit_buf:
                    name = names[selected]
                    try:
                        new_value = float(edit_buf)
                    except ValueError:
                        status = f"invalid number: {edit_buf}"
                    else:
                        resp = link.set_param(name, new_value)
                        values[name] = new_value
                        status = resp
                editing = False
                edit_buf = ""
            elif ch == 27:  # Esc
                editing = False
                edit_buf = ""
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                edit_buf = edit_buf[:-1]
            elif 32 <= ch < 127:
                edit_buf += chr(ch)
            continue

        if filtering:
            if ch in (10, 13, curses.KEY_ENTER):
                filtering = False
            elif ch == 27:  # Esc
                filtering = False
                filter_text = ""
                names = list(all_names)
                selected = 0
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                filter_text = filter_text[:-1]
                names = [n for n in all_names if filter_text in n] or list(all_names)
                selected = 0
            elif 32 <= ch < 127:
                filter_text += chr(ch)
                names = [n for n in all_names if filter_text in n] or list(all_names)
                selected = 0
            continue

        if ch in (curses.KEY_UP, ord("k")):
            selected = max(0, selected - 1)
        elif ch in (curses.KEY_DOWN, ord("j")):
            selected = min(len(names) - 1, selected + 1) if names else 0
        elif ch == curses.KEY_LEFT:
            if names:
                name = names[selected]
                new_value = values[name] - step
                resp = link.set_param(name, new_value)
                values[name] = new_value
                status = resp
        elif ch == curses.KEY_RIGHT:
            if names:
                name = names[selected]
                new_value = values[name] + step
                resp = link.set_param(name, new_value)
                values[name] = new_value
                status = resp
        elif ch in (10, 13, curses.KEY_ENTER):
            if names:
                editing = True
                edit_buf = ""
        elif ch == ord("["):
            step /= 10.0
        elif ch == ord("]"):
            step *= 10.0
        elif ch == ord("/"):
            filtering = True
            filter_text = ""
        elif ch == ord("s"):
            status = link.save()
        elif ch == ord("l"):
            status = link.load()
            order = link.fetch_params()
            values = dict(order)
            all_names = [n for n, _ in order]
            names = [n for n in all_names if filter_text in n] or list(all_names)
        elif ch == ord("r"):
            status = link.reset_defaults()
            order = link.fetch_params()
            values = dict(order)
            all_names = [n for n, _ in order]
            names = [n for n in all_names if filter_text in n] or list(all_names)
        elif ch == ord("g"):
            order = link.fetch_params()
            values = dict(order)
            all_names = [n for n, _ in order]
            names = [n for n in all_names if filter_text in n] or list(all_names)
            status = f"Reloaded {len(all_names)} params"
        elif ch == ord("q"):
            return


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0", help="mob のシリアルポート")
    parser.add_argument("--baud", type=int, default=3000000)
    args = parser.parse_args()

    link = MobLink(args.port, args.baud)
    try:
        curses.wrapper(run, link, args.port)
    finally:
        link.close()


if __name__ == "__main__":
    main()
