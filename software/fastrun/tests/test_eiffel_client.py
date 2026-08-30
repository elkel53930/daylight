"""eiffel_client のユニットテスト(ハード非依存)。

実機 two-unit-network の /status.json を tests/data/eiffel_status_primary.json に
保存済み。2号機オフライン(未検出=東半分)や not ready などの異常系は合成して検証する。
"""
import copy
import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from eiffel_client import (
    EiffelContractError,
    EiffelSnapshot,
    EiffelUnavailable,
    snapshot_from_status,
    wallmap_from_map,
)
from geometry import Direction

_DATA = Path(__file__).resolve().parent / "data" / "eiffel_status_primary.json"


def _load_status() -> dict:
    return json.loads(_DATA.read_text())


def _make_cell(col, row, walls=None, valid=True):
    w = {"N": False, "E": False, "S": False, "W": False}
    if walls:
        w.update(walls)
    if isinstance(valid, bool):
        v = {d: valid for d in ("N", "E", "S", "W")}
    else:
        v = {"N": True, "E": True, "S": True, "W": True}
        v.update(valid)
    return {"col": col, "row": row, "walls": w, "walls_valid": v}


def _synthetic_status(cells, cols, rows, *, role="primary",
                      peer_last_update="2026-08-30T15:00:00", balls=None):
    """combined を持つ最小の primary status を組み立てる。"""
    combined = {
        "maze_cols": cols, "maze_rows": rows, "cells": cells,
        "balls": balls or [], "peer_last_update": peer_last_update,
    }
    return {
        "ready": True, "role": role, "last_update": "2026-08-30T15:00:00",
        "last_error": None, "maze_cols": 8, "maze_rows": 8, "threshold": 37.0,
        "cells": [], "balls": [], "combined": combined,
    }


class TestRealFixture(unittest.TestCase):
    """保存した実 /status.json に対する検証。"""

    def setUp(self):
        self.status = _load_status()

    def test_snapshot_basic(self):
        snap = snapshot_from_status(self.status)
        self.assertIsInstance(snap, EiffelSnapshot)
        self.assertEqual(snap.source, "combined")
        self.assertEqual(snap.role, "primary")
        self.assertEqual((snap.cols, snap.rows), (16, 8))
        self.assertEqual(snap.wm.width, 16)
        self.assertEqual(snap.wm.height, 8)

    def test_real_map_is_clean(self):
        # 両号機稼働時: 内壁 invalid=0・未検出0(外周4隅の invalid は無視される)。
        snap = snapshot_from_status(self.status)
        self.assertEqual(snap.invalid_inner_edges, 0)
        self.assertEqual(snap.undetected_cells, 0)
        self.assertTrue(snap.all_valid())
        self.assertTrue(snap.peer_connected)

    def test_wallmap_matches_source(self):
        # combined.cells の walls=True 内辺が WallMap に立っていること。
        snap = snapshot_from_status(self.status)
        combined = self.status["combined"]
        checked = 0
        for c in combined["cells"]:
            col, row = c["col"], c["row"]
            for d in ("N", "E", "S", "W"):
                if c["walls"][d] and c["walls_valid"][d]:
                    self.assertTrue(
                        snap.wm.has_wall(col, row, Direction[d]),
                        f"({col},{row},{d}) が WallMap に無い",
                    )
                    checked += 1
        self.assertGreater(checked, 0)

    def test_balls_parsed(self):
        snap = snapshot_from_status(self.status)
        expected = {(b["col"], b["row"]) for b in self.status["combined"]["balls"]}
        self.assertEqual(set(snap.balls), expected)


class TestPeerOffline(unittest.TestCase):
    """2号機オフライン(東半分が未検出プレースホルダ)の合成ケース。"""

    def _cells_peer_offline(self, cols=16, rows=8):
        cells = []
        for r in range(rows):
            for c in range(cols):
                if c < cols // 2:  # 西半分=検出済み(壁なしの素通しマス)
                    cells.append(_make_cell(c, r, valid=True))
                else:  # 東半分=未検出(walls_valid 全 false)
                    cells.append(_make_cell(c, r, valid=False))
        return cells

    def test_undetected_counted(self):
        status = _synthetic_status(self._cells_peer_offline(), 16, 8,
                                   peer_last_update=None)
        snap = snapshot_from_status(status)
        self.assertFalse(snap.peer_connected)
        self.assertFalse(snap.all_valid())
        self.assertEqual(snap.undetected_cells, 8 * 8)  # 東半分 8×8

    def test_region_valid_gate(self):
        status = _synthetic_status(self._cells_peer_offline(), 16, 8,
                                   peer_last_update=None)
        snap = snapshot_from_status(status)
        # 西半分の経路は valid、東半分を含むと invalid。
        self.assertTrue(snap.region_valid([(0, 0), (1, 0), (7, 7)]))
        self.assertFalse(snap.region_valid([(7, 0), (8, 0)]))

    def test_conservative_blocks_unknown(self):
        # conservative では未検出マスの内辺が封鎖され、そこへ進入できない。
        status = _synthetic_status(self._cells_peer_offline(), 16, 8,
                                   peer_last_update=None)
        snap = snapshot_from_status(status, valid_policy="conservative")
        # (7,0)→(8,0)(東=未検出へ)は封鎖される。
        self.assertFalse(snap.wm.can_move(7, 0, Direction.E))

    def test_strict_leaves_unknown_open(self):
        status = _synthetic_status(self._cells_peer_offline(), 16, 8,
                                   peer_last_update=None)
        snap = snapshot_from_status(status, valid_policy="strict")
        # strict では未知辺は開放のまま(封鎖しない)。
        self.assertTrue(snap.wm.can_move(7, 0, Direction.E))


class TestPolicyAndEdges(unittest.TestCase):
    def test_valid_wall_is_set_both_policies(self):
        cells = [
            _make_cell(0, 0, walls={"E": True}, valid=True),
            _make_cell(1, 0, walls={"W": True}, valid=True),
        ]
        for policy in ("strict", "conservative"):
            wm, stats = wallmap_from_map(
                {"maze_cols": 2, "maze_rows": 1, "cells": cells}, valid_policy=policy)
            self.assertTrue(wm.has_wall(0, 0, Direction.E), policy)
            self.assertEqual(stats["invalid_inner_edges"], 0, policy)

    def test_outer_invalid_ignored(self):
        # (0,0) の S/W(外周)が invalid でも invalid_inner_edges に数えない。
        cells = [_make_cell(0, 0, valid={"S": False, "W": False})]
        wm, stats = wallmap_from_map({"maze_cols": 1, "maze_rows": 1, "cells": cells})
        self.assertEqual(stats["invalid_inner_edges"], 0)

    def test_corner_cell_outer_invalid_is_valid(self):
        # 外周辺のみ invalid の隅セルは region_valid で弾かれない(内辺は valid)。
        cells = [_make_cell(0, 0, valid={"S": False, "W": False}),
                 _make_cell(1, 0, valid=True)]
        status = _synthetic_status(cells, 2, 1)
        snap = snapshot_from_status(status)
        self.assertTrue(snap.cell_all_valid(0, 0))
        self.assertTrue(snap.region_valid([(0, 0), (1, 0)]))

    def test_unknown_policy_raises(self):
        with self.assertRaises(ValueError):
            wallmap_from_map({"maze_cols": 1, "maze_rows": 1, "cells": []},
                             valid_policy="bogus")

    def test_shared_wall_consistency(self):
        # 片側だけ walls=True でも WallMap が隣接セルの反対側にも立てる。
        cells = [
            _make_cell(0, 0, walls={"E": True}, valid=True),
            _make_cell(1, 0, valid=True),  # W は false だが共有で立つ
        ]
        wm, _ = wallmap_from_map({"maze_cols": 2, "maze_rows": 1, "cells": cells})
        self.assertTrue(wm.has_wall(1, 0, Direction.W))


class TestRoleAndReadiness(unittest.TestCase):
    def test_not_ready_raises_unavailable(self):
        with self.assertRaises(EiffelUnavailable):
            snapshot_from_status({"ready": False, "role": "primary",
                                  "last_error": None})

    def test_secondary_unavailable(self):
        status = {"ready": True, "role": "secondary", "maze_cols": 8,
                  "maze_rows": 8, "cells": [], "balls": []}
        with self.assertRaises(EiffelUnavailable):
            snapshot_from_status(status)

    def test_primary_without_combined_is_contract_error(self):
        with self.assertRaises(EiffelContractError):
            snapshot_from_status({"ready": True, "role": "primary"})

    def test_standalone_uses_toplevel(self):
        cells = [_make_cell(0, 0, walls={"E": True}, valid=True),
                 _make_cell(1, 0, valid=True)]
        status = {"ready": True, "role": "standalone", "maze_cols": 2,
                  "maze_rows": 1, "cells": cells, "balls": [],
                  "last_update": "t"}
        snap = snapshot_from_status(status)
        self.assertEqual(snap.source, "standalone")
        self.assertIsNone(snap.peer_last_update)
        self.assertTrue(snap.wm.has_wall(0, 0, Direction.E))


if __name__ == "__main__":
    unittest.main()
