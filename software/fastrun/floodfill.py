"""floodfill.py — 迷路探索のフラッドフィル距離計算(2026-08-03〜)。

既知の壁マップ(WallMap)上で、ゴール領域からの最短ステップ数(マンハッタン
的な距離、ただし壁を考慮)を各セルへ配る。未知の壁はWallMapに記録が無い=
can_move()がTrueを返す=開放とみなす(楽観的探索、古典的マイクロマウス)。

探索走行では、現在セルから「距離が1つ小さい隣接セル」へ進めばゴールに
近づく。next_direction() がその向きを返す。
"""

from __future__ import annotations

from collections import deque
from typing import Dict, Iterable, Optional, Tuple

from geometry import Direction
from maze import WallMap

Cell = Tuple[int, int]


def flood_fill(wm: WallMap, goals: Iterable[Cell]) -> Dict[Cell, int]:
    """ゴール領域から各セルへの最短ステップ数を返す(壁を考慮したBFS)。

    到達不能なセルは辞書に含めない。
    """
    dist: Dict[Cell, int] = {}
    q: deque[Cell] = deque()
    for g in goals:
        gx, gy = g
        if wm.in_bounds(gx, gy) and (gx, gy) not in dist:
            dist[(gx, gy)] = 0
            q.append((gx, gy))
    while q:
        x, y = q.popleft()
        d = dist[(x, y)]
        for nd in Direction:
            if not wm.can_move(x, y, nd):
                continue
            dx, dy = nd.delta
            nc = (x + dx, y + dy)
            if nc not in dist:
                dist[nc] = d + 1
                q.append(nc)
    return dist


def next_direction(
    wm: WallMap, cell: Cell, dist: Dict[Cell, int]
) -> Optional[Direction]:
    """cell から見て距離が最小の開放隣接セルへの向きを返す。

    現在セルより距離が小さい隣接セルの中で最小のものへ向かう。候補が無い
    (ゴール上、または袋小路で全隣接が現在以上)なら None。
    """
    cx, cy = cell
    here = dist.get(cell)
    if here is None or here == 0:
        return None
    best: Optional[Direction] = None
    best_d = here
    for nd in Direction:
        if not wm.can_move(cx, cy, nd):
            continue
        dx, dy = nd.delta
        nc = (cx + dx, cy + dy)
        nd_d = dist.get(nc)
        if nd_d is not None and nd_d < best_d:
            best_d = nd_d
            best = nd
    return best
