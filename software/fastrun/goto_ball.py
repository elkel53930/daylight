"""goto_ball.py — Eiffel のボール検出 → 経路決定 → そのボールへ走行(2026-08-30)。

Liner のミッションの一手(memory liner-dev-plan、設計 DESIGN_eiffel_liner.md):
  Eiffel(視覚エージェント)の combined.balls からボール位置を得て、自陣内で到達可能な
  ボールを1つ選び、走り出す前に壁上面補正(recenter)してから、そのマスへ高速移動する。

ボール検出はフレームごとに揺れる(黄色画素割合の閾値判定)。誤検出・取りこぼしに強くする
ため、複数フレームを取得して**過半数のフレームで見えたボールだけ**を採用する。走行・経路・
recenter の実体は mission.py を再利用する。

使い方:
    goto_ball.py --dry-run                 # 検出・選択・経路の表示だけ(走らない)
    goto_ball.py                           # 最寄りの到達可能ボールへ走行
    goto_ball.py --select most-yellow      # 最も黄色が濃いボールを選ぶ
    goto_ball.py --start 5,1 --dir N       # 現在姿勢を指定(既定は設定/(0,7)S)
"""
from __future__ import annotations

import argparse
import time
from collections import Counter
from typing import List, Optional, Tuple

import eiffel_client
from geometry import Direction
from mission import (MissionError, _parse_cell, fetch_map, load_config,
                     run_mission)
from planner import PlannerConfig, find_path

Ball = Tuple[int, int]


def collect_stable_balls(
    cfg: dict, *, frames: int, interval_s: float, min_seen: int
) -> Tuple[List[Ball], "eiffel_client.EiffelSnapshot", dict]:
    """複数フレームを取得し、過半数(min_seen 以上)で見えたボールだけ返す。

    戻り値: (安定ボール列, 最終スナップショット, ボール毎の yellow_frac 最大値dict)。
    yellow_frac は生 /status.json から拾う(EiffelSnapshot は座標のみ持つため)。
    """
    e = cfg["eiffel"]
    ms = cfg["map_sync"]
    seen: Counter = Counter()
    frac_max: dict = {}
    last_snap: Optional[eiffel_client.EiffelSnapshot] = None
    for i in range(frames):
        status = eiffel_client.fetch_status(e["host"], e["port"], e["timeout_s"])
        last_snap = eiffel_client.snapshot_from_status(
            status, valid_policy=ms["valid_policy"])
        # combined(or standalone) の balls から frac も拾う。
        map_dict, _ = eiffel_client._map_dict_from_status(status)
        frame_balls = set()
        for b in map_dict.get("balls", []):
            cell = (int(b["col"]), int(b["row"]))
            frame_balls.add(cell)
            frac_max[cell] = max(frac_max.get(cell, 0.0),
                                 float(b.get("yellow_frac", 0.0)))
        for cell in frame_balls:
            seen[cell] += 1
        if i < frames - 1:
            time.sleep(interval_s)
    if last_snap is None:
        raise MissionError("Eiffel からボール情報を取得できませんでした")
    stable = sorted(b for b, c in seen.items() if c >= min_seen)
    print(f"ボール検出: {frames}フレーム中の観測 {dict(seen)}")
    print(f"安定ボール(>= {min_seen}フレーム): {stable}")
    return stable, last_snap, frac_max


def select_ball(
    snap: "eiffel_client.EiffelSnapshot",
    balls: List[Ball],
    start: Ball,
    start_dir: Direction,
    cfg: dict,
    *,
    select: str = "nearest",
    frac_max: Optional[dict] = None,
) -> Tuple[Optional[Ball], List[Tuple[int, Ball]]]:
    """自陣内で到達可能なボールを絞り、選択方針に従って1つ選ぶ(純関数)。

    到達可能 = find_path 成功 かつ(設定で要求すれば)経路上が全 valid。
    select: "nearest"(経路が最短)/ "most-yellow"(yellow_frac 最大)。
    戻り値: (選択ボール or None, 到達可能候補 [(経路セル数, ボール), ...] 経路昇順)。
    """
    pcfg = PlannerConfig(straight_cruise_mmps=cfg["drive"]["straight_mmps"],
                         slalom_mmps=cfg["drive"]["slalom_mmps"])
    require_valid = cfg["map_sync"].get("require_region_valid", True)
    cands: List[Tuple[int, Ball]] = []
    for b in balls:
        if not snap.wm.in_bounds(*b):
            continue
        try:
            path = find_path(snap.wm, start, start_dir, b, pcfg)
        except ValueError:
            continue  # 到達不能(敵陣/壁分離)
        cells = [(x, y) for (x, y, _) in path]
        if require_valid and not snap.region_valid(cells):
            continue
        cands.append((len(path), b))
    cands.sort()
    if not cands:
        return None, cands
    if select == "most-yellow" and frac_max:
        target = max((b for _, b in cands), key=lambda b: frac_max.get(b, 0.0))
    else:  # nearest
        target = cands[0][1]
    return target, cands


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", help="mission.yaml のパス(任意)")
    ap.add_argument("--start", help='現在セル "x,y"(既定: 設定/(0,7))')
    ap.add_argument("--dir", choices=[d.name for d in Direction],
                    help="現在の向き(既定: 設定/S)")
    ap.add_argument("--host", help="Eiffel 1号機ホスト")
    ap.add_argument("--port", help="mob シリアルポート")
    ap.add_argument("--straight-mmps", type=float)
    ap.add_argument("--slalom-mmps", type=float)
    ap.add_argument("--frames", type=int, default=3, help="ボール検出フレーム数(既定3)")
    ap.add_argument("--interval-s", type=float, default=0.3,
                    help="フレーム間隔[s](既定0.3)")
    ap.add_argument("--min-seen", type=int, default=None,
                    help="採用に必要な観測フレーム数(既定: 過半数)")
    ap.add_argument("--select", choices=["nearest", "most-yellow"],
                    default="nearest", help="選択方針(既定: 最寄り)")
    ap.add_argument("--dry-run", action="store_true", help="検出・選択・経路のみ表示")
    ap.add_argument("--goal-recenter", action="store_true",
                    help="到達後もマス中心へ壁上面補正する(既定: しない=ボールを押さない)")
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
    min_seen = args.min_seen if args.min_seen is not None else (args.frames // 2 + 1)

    try:
        stable, snap, frac_max = collect_stable_balls(
            cfg, frames=args.frames, interval_s=args.interval_s, min_seen=min_seen)
        target, cands = select_ball(snap, stable, start, start_dir, cfg,
                                    select=args.select, frac_max=frac_max)
        print(f"到達可能候補(経路セル数, ボール): {cands}")
        if target is None:
            raise MissionError(
                "到達可能なボールがありません"
                + ("(検出0)" if not stable else "(全て敵陣/壁分離/未検出経路)"))
        print(f"選択ボール: {target}  (方針={args.select}, "
              f"yellow_frac={frac_max.get(target, 0.0):.3f})")

        rc = run_mission(cfg, start, start_dir, target,
                         dry_run=args.dry_run,
                         recenter_goal=args.goal_recenter,
                         snap=snap)
    except MissionError as e:
        print(f"[MISSION FAIL] {e}")
        raise SystemExit(1)
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
