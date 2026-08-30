"""mission.py — Eiffel の迷路情報で Liner を指定座標まで走らせる(2026-08-30)。

Liner のミッション(memory liner-dev-plan の L3 統合、設計 DESIGN_eiffel_liner.md):
  ゴール座標受領 → Eiffel(視覚エージェント)から迷路情報を取得 → 経路計画 →
  **走り出す前に壁上面補正(recenter)でスタットをマス中心・正対に確定** → 高速移動 →
  到達 → (任意)到達セルで壁上面補正。

迷路情報は Eiffel 1号機(role=primary)の /status.json の combined(2台マージ済み・
グローバル座標)を eiffel_client 経由で取得する。座標系は Liner と一致(原点=南西、
col=cx=東、row=cy=北)。スタート姿勢は迷路ごとに変わるため YAML/CLI で与える。

使い方:
    # 計画だけ表示(ハード不要。Eiffel からマップ取得 → 経路 → 区間列)
    mission.py --goal 3,7 --dry-run

    # 実機走行(スタートで recenter → 走行 → 到達で recenter)
    mission.py --goal 3,7

    # 設定ファイルで既定を上書き(CLI が最優先)
    mission.py --config mission.yaml --goal 3,7
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
sys.path.insert(0, str(_HERE.parent / "mob"))

import eiffel_client  # noqa: E402
from drive_runner import (COLLIDE_ANG_RAD, COLLIDE_CONSECUTIVE, COLLIDE_DIST_MM,  # noqa: E402
                          drive_segments)
from geometry import Direction  # noqa: E402
from liner_center import recenter_cell  # noqa: E402
from liner_pose import LinerPose, direction_to_gyro_deg  # noqa: E402
from planner import PlannerConfig, find_path, plan  # noqa: E402

DEFAULTS = {
    "eiffel": {"host": eiffel_client.DEFAULT_HOST, "port": eiffel_client.DEFAULT_PORT,
               "timeout_s": eiffel_client.DEFAULT_TIMEOUT_S},
    "maze": {"cols": 16, "rows": 8},
    "liner": {"start_cell": (0, 7), "start_dir": "S"},
    "map_sync": {"valid_policy": "conservative", "retries": 5, "backoff_s": 0.5,
                 "require_region_valid": True},
    "drive": {"straight_mmps": 700.0, "slalom_mmps": 550.0,
              "collide_dist_mm": COLLIDE_DIST_MM, "collide_ang_rad": COLLIDE_ANG_RAD,
              "port": "/dev/ttyUSB0"},
}


class MissionError(Exception):
    """ミッション遂行を中止すべき致命的状態(到達不能・マップ未確定など)。"""


# ----- 設定 -----

def load_config(path: Optional[str]) -> dict:
    """YAML 設定を DEFAULTS に重ねて返す(セクション単位でキーをマージ)。"""
    import copy
    cfg = copy.deepcopy(DEFAULTS)
    if not path:
        return cfg
    import yaml
    loaded = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    for section, vals in loaded.items():
        if section in cfg and isinstance(vals, dict):
            cfg[section].update(vals)
        else:
            cfg[section] = vals
    # start が {cell:[..], heading:..} 形式でも受ける
    liner = cfg.get("liner", {})
    if "start" in liner and isinstance(liner["start"], dict):
        st = liner["start"]
        if "cell" in st:
            liner["start_cell"] = tuple(st["cell"])
        if "heading" in st:
            liner["start_dir"] = st["heading"]
    return cfg


def _parse_cell(s: str) -> Tuple[int, int]:
    x, y = s.split(",")
    return int(x), int(y)


# ----- Eiffel マップ取得 -----

def fetch_map(cfg: dict) -> eiffel_client.EiffelSnapshot:
    """Eiffel から迷路スナップショットを取得(バックオフ再試行つき)。"""
    e = cfg["eiffel"]
    ms = cfg["map_sync"]
    last: Optional[Exception] = None
    for attempt in range(1, int(ms["retries"]) + 1):
        try:
            snap = eiffel_client.fetch_snapshot(
                host=e["host"], port=e["port"], timeout_s=e["timeout_s"],
                valid_policy=ms["valid_policy"])
            print(f"Eiffel 取得OK: {snap.source} {snap.cols}x{snap.rows} "
                  f"role={snap.role} peer={'接続' if snap.peer_connected else '未接続'} "
                  f"invalid_inner={snap.invalid_inner_edges} undetected={snap.undetected_cells}")
            return snap
        except eiffel_client.EiffelUnavailable as ex:
            last = ex
            print(f"[{attempt}/{ms['retries']}] Eiffel 未取得: {ex}")
            time.sleep(float(ms["backoff_s"]) * attempt)
    raise MissionError(f"Eiffel からマップを取得できません: {last}")


# ----- 計画 -----

def plan_route(snap: eiffel_client.EiffelSnapshot, start: Tuple[int, int],
               start_dir: Direction, goal: Tuple[int, int], cfg: dict):
    """経路探索 → 区間列。到達不能/範囲外/未検出経路は MissionError。

    戻り値: (segs, cells, final_dir)。cells は通過セル列、final_dir は到達時の向き。
    """
    wm = snap.wm
    if not wm.in_bounds(*goal):
        raise MissionError(f"ゴール {goal} が迷路範囲外(0..{wm.width-1},0..{wm.height-1})")
    if not wm.in_bounds(*start):
        raise MissionError(f"スタート {start} が迷路範囲外")

    pcfg = PlannerConfig(straight_cruise_mmps=cfg["drive"]["straight_mmps"],
                         slalom_mmps=cfg["drive"]["slalom_mmps"])
    try:
        path = find_path(wm, start, start_dir, goal, pcfg)
    except ValueError as e:
        raise MissionError(f"到達不能: {start}{start_dir.name} → {goal}({e})") from e
    cells = [(x, y) for (x, y, _) in path]
    final_dir = path[-1][2]

    if cfg["map_sync"].get("require_region_valid", True):
        if not snap.region_valid(cells):
            bad = [c for c in cells if not snap.cell_all_valid(*c)]
            raise MissionError(
                f"経路上に未検出(walls_valid=False)のマスがあります: {bad[:8]}"
                f"{' …' if len(bad) > 8 else ''}"
                + ("(2号機未接続)" if not snap.peer_connected else ""))

    segs = plan(wm, start, start_dir, goal, pcfg)
    return segs, cells, final_dir


def describe(segs, cells) -> None:
    print(f"経路 {len(cells)} セル: {cells}")
    print(f"区間 {len(segs)}:")
    from pattern import Slalom, Straight
    for i, s in enumerate(segs):
        if isinstance(s, Straight):
            print(f"  [{i}] STRAIGHT {s.distance_mm:6.1f}mm "
                  f"v:{s.v_start_mmps:.0f}->{s.v_cruise_mmps:.0f}->{s.v_end_mmps:.0f}")
        elif isinstance(s, Slalom):
            print(f"  [{i}] SLALOM {s.dir} r{s.radius_mm:.0f} "
                  f"{s.angle_deg:.0f}deg @{s.v_mmps:.0f}mm/s")


# ----- 走り出す前の recenter(必須) -----

def recenter_before_drive(link, cam, wm, pose: LinerPose) -> dict:
    """走り出す前にスタートセルを壁上面補正でマス中心・正対に確定する。

    liner_center.recenter_cell でヨー+X+Y をマス中心へ戻し、最後に start_dir へ正対して
    SANG で絶対方位を貼り直す(以後の区間走行の絶対フレームを確定)。
    verify_loop.recenter_start と同方針だが、任意のセル・向き・WallMap に対応する。
    """
    deg = direction_to_gyro_deg(pose.heading)
    link.stop()
    time.sleep(0.15)
    link.send(f"SANG,{math.radians(deg):.5f}")
    if link.wait_for("DONE", timeout_s=1.0) is None:
        raise MissionError("recenter: 初回 SANG timeout")

    result = recenter_cell(link, cam, wm, pose)
    link.stop()
    time.sleep(0.15)
    ok_x = result.get("x") is not None and result["x"].ok
    ok_y = result.get("y") is not None and result["y"].ok
    if not (ok_x or ok_y):
        raise MissionError(f"recenter 失敗(両軸とも補正できず): {result}")
    if not (ok_x and ok_y):
        print(f"[WARN] recenter 片軸のみ成功: x_ok={ok_x} y_ok={ok_y}(片軸は壁が無い可能性)")

    # start_dir へ正対して絶対方位を確定(区間走行の基準フレーム)。
    import recenter as recenter_ops
    recenter_ops.turn_to(link, deg)
    link.stop()
    time.sleep(0.15)
    link.send(f"SANG,{math.radians(deg):.5f}")
    if link.wait_for("DONE", timeout_s=1.0) is None:
        raise MissionError("recenter: 正対 SANG timeout")
    print(f"recenter 完了: {pose.cell} マス中心・{pose.heading.name}向き確定")
    return result


# ----- ミッション本体 -----

def run_mission(cfg: dict, start: Tuple[int, int], start_dir: Direction,
                goal: Tuple[int, int], *, dry_run: bool, recenter_goal: bool,
                snap: Optional["eiffel_client.EiffelSnapshot"] = None) -> int:
    if snap is None:
        snap = fetch_map(cfg)
    if (snap.cols, snap.rows) != (cfg["maze"]["cols"], cfg["maze"]["rows"]):
        print(f"[WARN] マップサイズ {snap.cols}x{snap.rows} が設定 "
              f"{cfg['maze']['cols']}x{cfg['maze']['rows']} と不一致。Eiffel 側を採用します")

    segs, cells, final_dir = plan_route(snap, start, start_dir, goal, cfg)
    print(f"スタート {start} {start_dir.name} → ゴール {goal}(到達時 {final_dir.name}向き)")
    describe(segs, cells)

    if dry_run:
        print("(--dry-run: 走行しません。実走は機体をスタットへ手で置いてから)")
        return 0

    from camera_align import OnboardCamera
    from mob_link import MobLink

    with MobLink(cfg["drive"]["port"]) as link:
        sen = link.read_sen()
        if sen is None:
            raise MissionError("SEN 応答なし。ポート・電源を確認して再実行してください")
        print(f"接続OK vbatt={sen['vbatt']:.2f}V")
        print("GCAL(静止確認)...")
        if not link.gyro_calibrate():
            raise MissionError("GCAL timeout")
        link.send("WALL,0")  # 横壁センサは使わない(カメラが絶対基準)
        time.sleep(0.1)

        # --- 走り出す前の recenter(必須) ---
        print(f"[recenter] スタット {start} {start_dir.name} を壁上面補正で確定 ...")
        with OnboardCamera() as cam:
            recenter_before_drive(link, cam, snap.wm,
                                  LinerPose(start[0], start[1], start_dir))

        # --- 走行 ---
        print("[drive] 走行開始 ...")
        result = drive_segments(
            link, segs,
            tag=f"mission_{goal[0]}_{goal[1]}",
            collide_dist_mm=cfg["drive"]["collide_dist_mm"],
            collide_ang_rad=cfg["drive"]["collide_ang_rad"],
            collide_consecutive=COLLIDE_CONSECUTIVE,
        )
        if result.oscillating:
            print("[WARN] hdg_err の符号反転が多い(発振の疑い)。ログを確認すること")
        if result.collision is not None:
            print("[FAIL] 壁衝突で中止。到達できませんでした(要 RECOVER:カメラ再ローカライズ)")
            print(f"ログ: {result.log_path}")
            return 2
        if not result.reached:
            print("[FAIL] タイムアウト等で未到達。ログ確認")
            print(f"ログ: {result.log_path}")
            return 2

        print(f"[到達] ゴール {goal} 到達。ログ: {result.log_path}")

        # --- 到達セルで壁上面補正(任意、L1) ---
        if recenter_goal:
            print(f"[recenter] 到達セル {goal} {final_dir.name} をマス中心へ補正 ...")
            try:
                with OnboardCamera() as cam:
                    recenter_before_drive(link, cam, snap.wm,
                                          LinerPose(goal[0], goal[1], final_dir))
            except MissionError as e:
                print(f"[WARN] 到達セルの recenter 失敗(壁が無い軸かも): {e}")
        print("ミッション完了。")
    return 0


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--goal", required=True, help='ゴール "x,y"')
    ap.add_argument("--start", help='スタット "x,y"(既定: 設定/(0,7))')
    ap.add_argument("--dir", choices=[d.name for d in Direction],
                    help="スタート向き(既定: 設定/S)")
    ap.add_argument("--config", help="mission.yaml のパス(任意)")
    ap.add_argument("--host", help="Eiffel 1号機ホスト(既定 172.20.10.4)")
    ap.add_argument("--port", help="mob シリアルポート(既定 /dev/ttyUSB0)")
    ap.add_argument("--straight-mmps", type=float)
    ap.add_argument("--slalom-mmps", type=float)
    ap.add_argument("--dry-run", action="store_true", help="計画だけ表示(走らない)")
    ap.add_argument("--no-goal-recenter", action="store_true",
                    help="到達後の壁上面補正をしない")
    args = ap.parse_args()

    cfg = load_config(args.config)
    if args.host:
        cfg["eiffel"]["host"] = args.host
    if args.port:
        cfg["drive"]["port"] = args.port
    if args.straight_mmps:
        cfg["drive"]["straight_mmps"] = args.straight_mmps
    if args.slalom_mmps:
        cfg["drive"]["slalom_mmps"] = args.slalom_mmps

    start = _parse_cell(args.start) if args.start else tuple(cfg["liner"]["start_cell"])
    start_dir = Direction[args.dir] if args.dir else Direction[cfg["liner"]["start_dir"]]
    goal = _parse_cell(args.goal)

    try:
        rc = run_mission(cfg, start, start_dir, goal,
                         dry_run=args.dry_run,
                         recenter_goal=not args.no_goal_recenter)
    except MissionError as e:
        print(f"[MISSION FAIL] {e}")
        raise SystemExit(1)
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
