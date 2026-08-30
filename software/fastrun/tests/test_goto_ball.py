"""goto_ball のボール選択ロジックのユニットテスト(ハード・ネットワーク非依存)。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from eiffel_client import snapshot_from_status
from geometry import Direction
from goto_ball import select_ball
from mission import DEFAULTS


def _cell(col, row, walls=None):
    w = {"N": False, "E": False, "S": False, "W": False}
    if walls:
        w.update(walls)
    v = {"N": True, "E": True, "S": True, "W": True}
    return {"col": col, "row": row, "walls": w, "walls_valid": v}


def _open_status(cols, rows, balls):
    """壁なし(外周のみ)の迷路 status を作る。balls は (col,row) 列。"""
    cells = [_cell(c, r) for r in range(rows) for c in range(cols)]
    combined = {"maze_cols": cols, "maze_rows": rows, "cells": cells,
                "balls": [{"col": c, "row": r, "yellow_frac": 0.1} for c, r in balls],
                "peer_last_update": "t"}
    return {"ready": True, "role": "primary", "last_update": "t",
            "maze_cols": cols, "maze_rows": rows, "cells": [], "balls": [],
            "combined": combined}


class TestSelectBall(unittest.TestCase):
    def setUp(self):
        import copy
        self.cfg = copy.deepcopy(DEFAULTS)

    def test_nearest_selected(self):
        snap = snapshot_from_status(_open_status(8, 8, [(1, 0), (5, 5), (0, 3)]))
        target, cands = select_ball(snap, [(1, 0), (5, 5), (0, 3)],
                                    (0, 0), Direction.N, self.cfg, select="nearest")
        # (0,3) と (1,0) が近い。最短経路のものが選ばれる。
        self.assertIn(target, {(1, 0), (0, 3)})
        self.assertEqual(cands[0][1], target)
        # 候補は経路昇順。
        self.assertEqual([c[0] for c in cands], sorted(c[0] for c in cands))

    def test_most_yellow_selected(self):
        snap = snapshot_from_status(_open_status(8, 8, [(1, 0), (7, 7)]))
        frac = {(1, 0): 0.1, (7, 7): 0.9}
        target, _ = select_ball(snap, [(1, 0), (7, 7)], (0, 0), Direction.N,
                                self.cfg, select="most-yellow", frac_max=frac)
        self.assertEqual(target, (7, 7))

    def test_unreachable_filtered(self):
        # 壁で囲まれたボールは到達不能として候補から外れる。
        status = _open_status(4, 4, [(3, 3), (1, 0)])
        for c in status["combined"]["cells"]:
            if (c["col"], c["row"]) == (3, 3):
                c["walls"] = {"N": True, "E": True, "S": True, "W": True}
            if (c["col"], c["row"]) in {(2, 3), (3, 2)}:
                # (3,3) の隣も壁で塞ぐ(共有壁で to (3,3) を封鎖)
                pass
        snap = snapshot_from_status(status)
        target, cands = select_ball(snap, [(3, 3), (1, 0)], (0, 0), Direction.N,
                                    self.cfg, select="nearest")
        self.assertNotIn((3, 3), [b for _, b in cands])
        self.assertEqual(target, (1, 0))

    def test_no_balls_returns_none(self):
        snap = snapshot_from_status(_open_status(8, 8, []))
        target, cands = select_ball(snap, [], (0, 0), Direction.N, self.cfg)
        self.assertIsNone(target)
        self.assertEqual(cands, [])


if __name__ == "__main__":
    unittest.main()
