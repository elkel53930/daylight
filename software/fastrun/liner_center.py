"""liner_center.py — Liner をマス中心へ戻す(壁上面補正、L1、2026-08-04)。

RoboSweep の Liner は高速移動でマス中心からズレる。到達後にカメラで
ヨー+X+Y をマス中心へ戻す。使う要素は全て検証済み:
  - 位置成分: recenter.center_on_front_wall(前方の壁へ90mmまで寄せる)
  - 角度成分: recenter.reanchor_heading(壁に正対しヨーを絶対方位へ貼り直す)
  - 面を向く: recenter.turn_to(ジャイロ方位へ超信地旋回)

1軸(1つの面)の補正 = その面を向く→位置を90mmへ→ヨーを絶対化。前壁センサでなく
カメラ(唯一の絶対基準)を使う。X軸はE/W壁、Y軸はN/S壁に対して行う。ゴールマスに
その軸の壁が無ければ、近傍マス(壁を共有し進入可能な隣)へ移って補正してよい
(XとYを別マスで補正してよい、というユーザー方針)。

⚠️ 使う壁は maze(既知)から選ぶ。壁に近づきすぎる横壁センサは使わない(カメラのみ)。
旋回は左右バランス(ケーブルよじれ対策、recenter.turn_to が考慮)。
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

from geometry import Direction
from liner_pose import LinerPose, direction_to_gyro_deg
from maze import WallMap

import recenter


# X軸を決める面(東西)と Y軸を決める面(南北)
_X_FACES = (Direction.E, Direction.W)
_Y_FACES = (Direction.N, Direction.S)


def available_faces(maze: WallMap, cx: int, cy: int) -> tuple[list, list]:
    """セル(cx,cy)で壁上面補正に使える面を (x_faces, y_faces) で返す(純関数)。

    x_faces は東西の壁がある向き、y_faces は南北の壁がある向き。外周壁も使える
    (has_wall は外周を常に True とする)。近い方/汚染しにくい方の選択は呼び出し側。
    """
    x_faces = [d for d in _X_FACES if maze.has_wall(cx, cy, d)]
    y_faces = [d for d in _Y_FACES if maze.has_wall(cx, cy, d)]
    return x_faces, y_faces


def neighbor_for_axis(maze: WallMap, cx: int, cy: int, axis: str) -> Optional[tuple]:
    """指定軸('x' or 'y')の壁が現在セルに無いとき、その軸の壁を持ち、かつ現在
    セルから壁なしで進入できる隣接セルを探して (ncx, ncy, move_dir) を返す(純関数)。

    move_dir は現在セルから隣へ進む向き。見つからなければ None。XとYを別マスで
    補正してよいので、隣で補正→その隣を新しい既知セルとして扱う。
    """
    faces = _X_FACES if axis == "x" else _Y_FACES
    # 4近傍のうち、壁なしで進入でき、その隣セルが目的軸の壁を持つものを探す。
    for move in (Direction.N, Direction.E, Direction.S, Direction.W):
        if maze.has_wall(cx, cy, move):
            continue  # そちらへは壁で進めない
        dx, dy = move.delta
        ncx, ncy = cx + dx, cy + dy
        if not maze.in_bounds(ncx, ncy):
            continue
        if any(maze.has_wall(ncx, ncy, d) for d in faces):
            return (ncx, ncy, move)
    return None


@dataclass(frozen=True)
class AxisResult:
    face: Direction
    offset_mm: float
    ok: bool


def center_axis(link, cam, face: Direction, *,
                center_tol_mm: float = 4.0) -> Optional[AxisResult]:
    """face 方向の壁に対して1軸を補正する(位置90mm + ヨー絶対化)。

    手順: (1)turn_to で face をおおまかに向く、(2)center_on_front_wall で前壁90mmへ
    (row駆動で遠くても安全に寄る)、(3)reanchor_heading で90mmの壁にヨー正対させ
    odo_ang を face の絶対方位へSANG。撮影は静止時、旋回はhold/左右バランス。
    戻り値: AxisResult(face, 到達offset, ok)。壁を掴めない等で失敗なら ok=False。
    """
    target_deg = direction_to_gyro_deg(face)
    recenter.turn_to(link, target_deg)
    time.sleep(0.2)
    off = recenter.center_on_front_wall(link, cam)
    if off is None or abs(off.offset_mm) > center_tol_mm or off.residual_px > recenter.FORWARD_OFFSET_MAX_RES:
        return AxisResult(face=face, offset_mm=(off.offset_mm if off else float("nan")), ok=False)
    # 90mm に居るのでHALFCELL較正が有効。ヨーを絶対方位へ貼り直す。
    yaw_ok = recenter.reanchor_heading(link, cam, target_deg)
    return AxisResult(face=face, offset_mm=off.offset_mm, ok=yaw_ok)


def recenter_cell(link, cam, maze: WallMap, pose: LinerPose) -> dict:
    """現在セルでヨー+X+Yをマス中心へ戻す(壁上面補正、両軸)。

    現在セルに壁がある軸はその面で補正。壁が無い軸は近傍マスへ移って補正する
    (XとYを別マスで行ってよい)。戻り値は {'x': AxisResult|None, 'y': AxisResult|None,
    'pose': 補正後の推定LinerPose}。近傍へ移った場合は pose のセルが変わる。

    注意(現状の実装範囲): 近傍フォールバックの「移動」は L2 の高速移動が入るまで、
    JOGで1セル進む簡易版を使う(呼び出し側でmove_one_cellを与える)。まずは現在セルに
    両軸の壁がある場合(角のセル等)で完結する経路を実装・検証し、フォールバックは
    段階的に足す。
    """
    cx, cy = pose.cell
    x_faces, y_faces = available_faces(maze, cx, cy)
    results: dict = {"x": None, "y": None, "pose": pose}

    # Y軸(南北)を先に(ケーブルよじれはturn_toが正味回転で吸収)
    if y_faces:
        results["y"] = center_axis(link, cam, y_faces[0])
    if x_faces:
        results["x"] = center_axis(link, cam, x_faces[0])
    return results
