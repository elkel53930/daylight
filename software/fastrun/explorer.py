"""explorer.py — 壁センサを真値にしたセル単位の迷路探索/マッピング(2026-08-03〜)。

各セルで stop-and-read: 停止→前/左/右の壁センサ読み→地図更新→フラッドフィルで
ゴールへ近づく向きを決定→その場旋回(TURN)+1セル前進(180mm)。前進の直前に
前壁センサで安全確認し、想定外の壁なら地図に足して再決定する(衝突回避)。

局所フレーム(開始セル=(START,START)、開始向き=Direction.N=ロボットの現在の
前方向)で動く。俯瞰カメラは安全確認・記録用にDiscordへ移動後の画像を送る。

⚠️ 現状はgyroベースの直進/その場旋回のみで、側壁による横位置補正(壁制御)は
未実装。数セル程度なら問題ないが、多数セルでは累積ドリフトしうる(今後の課題)。
"""

from __future__ import annotations

import math
import time
from typing import List, Optional, Sequence, Tuple

from floodfill import flood_fill, next_direction
from geometry import CELL_MM, Direction, turn_between
from mapping import WALL_THRESHOLD, update_walls
from maze import WallMap

Cell = Tuple[int, int]

GRID = 9          # 局所グリッド(十分広く取る)
START = 4         # 開始セル座標(中央)


class Explorer:
    # 前壁センサ(lf)は壁がセル境界にしか無いため実質3値になる(2026-08-03実測):
    #   壁@90mm(現セル前縁=前進不可) → lf≈800
    #   壁@270mm(1.5セル先=1セル進めば90mmで到達可) → lf≈188
    #   近くに壁なし → lf≈2
    # よって「今このセルの前が塞がっているか(=90mmに壁)」は lf>FRONT_GATE で判定
    # できる。中間値が無いので 400 で 90mm壁(~800)だけを拾い、1.5セル先(~188)は
    # 拾わない(=1セル進めるので前進を止めない)。旧値130は188も拾い、遠い壁で
    # 前進を止める誤判定をしていた。
    # ⚠️ ただし前壁センサは**近すぎると値が下がる非単調特性**がある(ユーザー
    # 指摘2026-08-03、CLAUDE.md参照)。この lf 判定はマス中心(前壁90mm)に正しく
    # 居るときだけ信頼でき、前へ変位して90mmより近づくと lf が下がり「開放」と
    # 誤認する。近距離前壁の確実な検出は**カメラ**へ移すのが本筋(次のカメラ統合)。
    FRONT_GATE = 400

    def __init__(self, link, *, cruise_mmps: float = 200.0, cell_mm: float = CELL_MM,
                 on_moved=None):
        self.link = link
        self.cruise = cruise_mmps
        self.cell_mm = cell_mm
        self.on_moved = on_moved  # 移動後コールバック(俯瞰→Discord等)
        self.wm = WallMap(GRID, GRID)
        self.cell: Cell = (START, START)
        self.heading = Direction.N

    # ---- 低レベル動作 ----
    def _read_walls(self, n: int = 5) -> Tuple[bool, bool, bool, Tuple[int, int, int]]:
        """前/左/右の壁有無(bool)と生値(median)を返す。WALLは有効前提。"""
        fs, ls_, rs_ = [], [], []
        for _ in range(n):
            s = self.link.read_sen()
            if s:
                fs.append(s["lf"]); ls_.append(s["ls"]); rs_.append(s["rs"])
            time.sleep(0.03)
        import statistics
        f = int(statistics.median(fs)) if fs else 0
        l = int(statistics.median(ls_)) if ls_ else 0
        r = int(statistics.median(rs_)) if rs_ else 0
        # 前壁だけ専用しきい値(遠いぶん低く出る)。側壁は WALL_THRESHOLD。
        return (f > self.FRONT_GATE, l > WALL_THRESHOLD, r > WALL_THRESHOLD, (f, l, r))

    def _front_raw(self) -> int:
        s = self.link.read_sen()
        return s["lf"] if s else 0

    def _turn(self, steps: int) -> None:
        """turn_between の回転ステップ数(45°単位、+1=右/CW, -1=左/CCW,
        ±2=90°, ±4=180°)だけその場旋回。"""
        if steps == 0:
            return
        # TURNは正=左/CCW。右(steps>0)は負のrad。
        turn_rad = -steps * (math.pi / 4.0)
        self.link.send(f"TURN,{turn_rad:.5f}")
        time.sleep(0.8 + abs(turn_rad) * 0.45)
        self.link.stop()
        time.sleep(0.2)

    # 走行中の前壁・緊急停止しきい値。**距離ゲート必須**:
    # 前壁のあるセルへ正常移動すると、その壁は到達時に90mm(lf≈800)まで近づく
    # ので、単純な lf 閾値では正常到着の途中で誤abortする(旧FRONT_ABORT=450の
    # バグ=探索破綻の主因)。壁はセル境界にしか無く、前進前ゲート(FRONT_GATE)で
    # 「今90mmに壁が無い」ことを確認済みなので、1セル(cell_mm)移動は本来常に安全
    # (最悪でも到達時に次セル前縁の壁へ90mm)。よって緊急停止は「まだ大して
    # 進んでいないのに壁が至近」= 想定外に近い場合だけに限定する。
    FRONT_ABORT = 800          # これ以上=90mmより近い(ほぼ接触寸前)
    FRONT_ABORT_DIST_FRAC = 0.6  # 移動距離がこの割合未満のときだけ緊急停止を有効化

    def _forward_one_cell(self) -> Tuple[List[str], bool]:
        """1セル前進(台形、停止)。走行中の#WEDGEを集め、前壁接近で中断。

        戻り値 (wedges, aborted)。aborted=True なら前壁接近で途中停止した。
        """
        self.link.send("PCLEAR"); self.link.wait_for("DONE", 1.0)
        self.link.send(f"PADD,STRAIGHT,{self.cell_mm:.0f},0,{self.cruise:.0f},0")
        self.link.wait_for("DONE", 1.0)
        self.link.send("RDST"); self.link.wait_for("DONE", 1.0)
        self.link.send("PRUN")
        wedges: List[str] = []
        aborted = False
        t0 = time.monotonic()
        last_sen = 0.0
        while time.monotonic() - t0 < self.cell_mm / self.cruise + 2.0:
            if time.monotonic() - last_sen > 0.12:
                self.link.send("SEN"); last_sen = time.monotonic()
            raw = self.link.ser.readline()
            if not raw:
                continue
            ln = raw.decode("ascii", "replace").strip()
            if ln.startswith("#WEDGE"):
                wedges.append(ln)
            elif ln.startswith("SEN,"):
                p = ln.split(",")
                if len(p) == 13:
                    try:
                        lf = int(p[3])
                        traveled = float(p[9])  # odo_dist(RDST済みなので移動量)
                    except ValueError:
                        continue
                    # 「まだ大して進んでいないのに壁が至近」= 想定外に近い場合のみ緊急停止。
                    # 正常到着(終盤にlf~800)では距離ゲートで無効化され誤abortしない。
                    if lf > self.FRONT_ABORT and traveled < self.FRONT_ABORT_DIST_FRAC * self.cell_mm:
                        aborted = True
                        print(f"  !! 前壁が想定外に至近(lf={lf}, 進行{traveled:.0f}/{self.cell_mm:.0f}mm)で緊急停止")
                        break
        self.link.stop()
        return wedges, aborted

    def face(self, target: Direction) -> None:
        self._turn(turn_between(self.heading, target))
        self.heading = target

    # ---- 探索ループ ----
    def explore(self, goals: Sequence[Cell], max_steps: int = 12) -> bool:
        """goals に到達するまで探索。到達で True、行き詰まり/step超過で False。"""
        self.link.gyro_calibrate()
        self.link.send("RANG"); self.link.wait_for("DONE", 2.0)
        self.link.send("WALL,1"); time.sleep(0.2)

        for step in range(max_steps):
            f, l, r, raw = self._read_walls()
            update_walls(self.wm, self.cell, self.heading, f, l, r)
            print(f"[step {step}] cell={self.cell} head={self.heading.name} "
                  f"F={f} L={l} R={r} raw={raw}")

            if tuple(self.cell) in set(goals):
                print("*** ゴール到達 ***")
                return True

            dist = flood_fill(self.wm, goals)
            nd = next_direction(self.wm, self.cell, dist)
            if nd is None:
                print("!! 進める方向が無い(袋小路/到達不能)")
                return False

            self.face(nd)
            # 前進直前の安全確認: 前壁があれば地図に足して再決定
            if self._front_raw() > self.FRONT_GATE:
                print(f"  安全確認: 前方{nd.name}に想定外の壁 → 地図更新して再決定")
                update_walls(self.wm, self.cell, self.heading, True, False, False)
                continue

            wedges, aborted = self._forward_one_cell()
            if aborted:
                # 前壁接近で中断: その向きに壁を記録して姿勢は進めない(再決定)
                update_walls(self.wm, self.cell, self.heading, True, False, False)
                if self.on_moved:
                    self.on_moved(self, step, wedges)
                continue
            dx, dy = self.heading.delta
            self.cell = (self.cell[0] + dx, self.cell[1] + dy)
            print(f"  前進 -> cell={self.cell} #WEDGE={len(wedges)} {wedges}")
            if self.on_moved:
                self.on_moved(self, step, wedges)

        print("!! max_steps 到達")
        return False
