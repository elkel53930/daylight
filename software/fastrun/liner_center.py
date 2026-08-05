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

import camera_model
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
    camera_yaw_deg: Optional[float] = None
    camera_dist_mm: Optional[float] = None
    camera_row_calib: Optional[float] = None


YAW_GATE_DEG = 15.0  # これ超のヨー推定は較正外/汚染として動かさない
RANGE_ADJUST_MM = 15.0
MAX_RANGE_ADJUST_STEPS = 4


def center_axis(link, cam, face: Direction, *,
                center_tol_mm: float = 4.0,
                yaw_deadband_deg: float = 0.8) -> Optional[AxisResult]:
    """face 方向の壁に対して1軸を補正する(オープンループ、2026-08-04ユーザー指示)。

    フィードバック反復をやめ、2測定・2一発補正にする(高速化):
      (1) turn_to で face をおおまかに向く(ジャイロ)
      (2) 【測定1】下端エッジで角度(ヨー)を測り、-yaw だけ JOGTURN で一発旋回。
          停止して SANG で face の絶対方位を確定(絶対基準の貼り直し)。
      (3) 【測定2】下端エッジで前後距離オフセットを測り、その分を JOGFWD/JOGBACK で
          一発移動して 90mm(マス中心)へ。
    各測定はフレーム数を減らさず(信頼性維持)中央値で頑健化する。撮影は静止時。
    戻り値: AxisResult(face, 測定した距離offset[mm], ok)。offset は補正した検出値
    (オープンループなので補正後の残差は測らない)。汚染/範囲外で ok=False。
    """
    import math
    t_axis_start = time.perf_counter()

    def measure_in_range(stage: str) -> Optional[recenter.WallMeasure]:
        for attempt in range(MAX_RANGE_ADJUST_STEPS + 1):
            t_attempt = time.perf_counter()
            measure = recenter.measure_wall(cam)
            dt_measure = time.perf_counter() - t_attempt
            if measure is None:
                print(
                    f"center_axis perf: face={face.name} {stage} attempt={attempt} "
                    f"measure_wall={dt_measure:.3f}s -> None"
                )
                return None
            range_state = camera_model.row_range_state(measure.row_calib)
            print(
                f"center_axis perf: face={face.name} {stage} attempt={attempt} "
                f"measure_wall={dt_measure:.3f}s range={range_state} "
                f"ok={measure.ok} dist={measure.dist_mm:.1f} yaw={measure.yaw_deg:+.2f} "
                f"offset={measure.offset_mm:+.1f} row={measure.row_calib:.1f}"
            )
            if measure.ok or range_state == "in":
                return measure
            if attempt >= MAX_RANGE_ADJUST_STEPS:
                return measure

            t_adjust = time.perf_counter()
            if range_state == "far":
                print(
                    f"center_axis {stage}: face={face.name} row_calib={measure.row_calib:.1f} too far -> "
                    f"JOGFWD {RANGE_ADJUST_MM:.1f} mm"
                )
                recenter._jog(link, f"JOGFWD,{RANGE_ADJUST_MM:.1f}")
            elif range_state == "near":
                print(
                    f"center_axis {stage}: face={face.name} row_calib={measure.row_calib:.1f} too near -> "
                    f"JOGBACK {RANGE_ADJUST_MM:.1f} mm"
                )
                recenter._jog(link, f"JOGBACK,{RANGE_ADJUST_MM:.1f}")
            print(
                f"center_axis perf: face={face.name} {stage} attempt={attempt} "
                f"range_adjust={time.perf_counter()-t_adjust:.3f}s"
            )
            t_sleep = time.perf_counter()
            time.sleep(0.2)
            print(
                f"center_axis perf: face={face.name} {stage} attempt={attempt} "
                f"post_adjust_sleep={time.perf_counter()-t_sleep:.3f}s"
            )
        return None

    target_deg = direction_to_gyro_deg(face)
    t_start = time.perf_counter()
    recenter.turn_to(link, target_deg)          # ジャイロで概ね壁へ向く
    t_sleep = time.perf_counter()
    time.sleep(0.2)
    print(
        f"center_axis perf: face={face.name} turn_to_total={time.perf_counter()-t_start:.3f}s "
        f"post_turn_sleep={time.perf_counter()-t_sleep:.3f}s"
    )

    # --- 測定1: 距離認識モデルで (距離,ヨー) → ヨーを一発旋回 → SANG で絶対方位確定 ---
    t_m1 = time.perf_counter()
    m1 = measure_in_range("measure1")
    if m1 is None or not m1.ok or abs(m1.yaw_deg) > YAW_GATE_DEG:
        print(f"center_axis perf: measure1={time.perf_counter()-t_m1:.3f}s", end="")
        if m1 is None:
            print(f"center_axis skip: face={face.name} measure1 unavailable")
        else:
            print(
                f"center_axis skip: face={face.name} measure1 dist_mm={m1.dist_mm:.1f} "
                f"yaw_deg={m1.yaw_deg:+.2f} offset_mm={m1.offset_mm:+.1f} row_calib={m1.row_calib:.1f} "
                f"res_px={m1.res_px:.2f} "
                f"n_clean={m1.n_clean} ok={m1.ok}"
            )
        return AxisResult(
            face=face,
            offset_mm=float("nan"),
            ok=False,
            camera_yaw_deg=(m1.yaw_deg if m1 is not None else None),
            camera_dist_mm=(m1.dist_mm if m1 is not None else None),
            camera_row_calib=(m1.row_calib if m1 is not None else None),
        )
    print(
        f"center_axis perf: face={face.name} measure1_total={time.perf_counter()-t_m1:.3f}s "
        f"yaw_gate={YAW_GATE_DEG:.1f}deg yaw={m1.yaw_deg:+.2f}deg"
    )
    if abs(m1.yaw_deg) > yaw_deadband_deg:
        turn_rad = math.radians(-m1.yaw_deg)
        planned_wait = 1.0
        t_jog = time.perf_counter()
        link.send(f"TURN,{turn_rad:.5f}")
        t_turn_sleep = time.perf_counter()
        time.sleep(planned_wait)
        print(
            f"center_axis perf: TURN yaw={m1.yaw_deg:+.2f}deg rad={turn_rad:+.5f} "
            f"planned_wait={planned_wait:.3f}s actual_wait={time.perf_counter()-t_turn_sleep:.3f}s "
            f"total={time.perf_counter()-t_jog:.3f}s"
        )
    link.stop()
    t_stop_sleep = time.perf_counter()
    time.sleep(0.15)
    t_sang = time.perf_counter()
    link.send(f"SANG,{math.radians(target_deg):.5f}")
    sang_done = link.wait_for("DONE", timeout_s=1.0) is not None
    print(
        f"center_axis step1: face={face.name} SANG heading={target_deg:+.1f} deg "
        f"wait={time.perf_counter()-t_sang:.3f}s done={sang_done}"
    )
    t_post_sang_sleep = time.perf_counter()
    time.sleep(0.1)
    print(
        f"center_axis perf: face={face.name} post_stop_sleep={time.perf_counter()-t_stop_sleep:.3f}s "
        f"post_sang_sleep={time.perf_counter()-t_post_sang_sleep:.3f}s"
    )

    # --- 測定2: 正対後に距離を測り一発移動(正対後なので row-yaw結合がなく距離が正確) ---
    t_m2 = time.perf_counter()
    m2 = measure_in_range("measure2")
    if m2 is None or not m2.ok:
        t_elapsed = time.perf_counter() - t_m2
        print(f"center_axis perf: measure2={t_elapsed:.3f}s")
        if m2 is not None:
            print(
                f"center_axis skip: face={face.name} measure2 dist_mm={m2.dist_mm:.1f} "
                f"yaw_deg={m2.yaw_deg:+.2f} offset_mm={m2.offset_mm:+.1f} row_calib={m2.row_calib:.1f} "
                f"res_px={m2.res_px:.2f} n_clean={m2.n_clean} ok={m2.ok}"
            )
        return AxisResult(
            face=face,
            offset_mm=(m2.offset_mm if m2 else float("nan")),
            ok=False,
            camera_yaw_deg=(m2.yaw_deg if m2 is not None else m1.yaw_deg),
            camera_dist_mm=(m2.dist_mm if m2 is not None else m1.dist_mm),
            camera_row_calib=(m2.row_calib if m2 is not None else m1.row_calib),
        )
    print(
        f"center_axis perf: face={face.name} measure2_total={time.perf_counter()-t_m2:.3f}s "
        f"offset={m2.offset_mm:+.1f}mm tol={center_tol_mm:.1f}mm"
    )
    if abs(m2.offset_mm) > center_tol_mm:
        t_jog = time.perf_counter()
        # offset>0 = 中心より前(壁に近い)→ JOGBACK で後退。offset<0 → JOGFWD。
        if m2.offset_mm > 0:
            print(
                f"center_axis step2: face={face.name} offset_mm={m2.offset_mm:+.1f} -> "
                f"JOGBACK {m2.offset_mm:.1f} mm"
            )
            recenter._jog(link, f"JOGBACK,{m2.offset_mm:.1f}")
        else:
            print(
                f"center_axis step2: face={face.name} offset_mm={m2.offset_mm:+.1f} -> "
                f"JOGFWD {-m2.offset_mm:.1f} mm"
            )
            recenter._jog(link, f"JOGFWD,{-m2.offset_mm:.1f}")
        print(f"center_axis perf: final_jog time={time.perf_counter()-t_jog:.3f}s")
    else:
        print(f"center_axis step2: face={face.name} offset_mm={m2.offset_mm:+.1f} within tolerance")
    print(f"center_axis perf: total_measure2={time.perf_counter()-t_m2:.3f}s")
    print(f"center_axis perf: face={face.name} axis_total={time.perf_counter()-t_axis_start:.3f}s")
    return AxisResult(
        face=face,
        offset_mm=m2.offset_mm,
        ok=True,
        camera_yaw_deg=m2.yaw_deg,
        camera_dist_mm=m2.dist_mm,
        camera_row_calib=m2.row_calib,
    )


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
