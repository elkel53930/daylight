"""planner / geometry / maze のユニットテスト(ハード非依存)。"""

import sys
import unittest
from pathlib import Path

# software/fastrun をimport探索パスに入れる(flat import)。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from geometry import CELL_MM, Direction, slalom_dir_symbol, turn_between
from maze import WallMap
from planner import (
    PlannerConfig,
    _states_to_moves,
    find_path,
    moves_to_segments,
    plan,
)
from pattern import Slalom, Straight


class TestGeometry(unittest.TestCase):
    def test_delta_north_is_plus_y(self):
        self.assertEqual(Direction.N.delta, (0, 1))
        self.assertEqual(Direction.E.delta, (1, 0))
        self.assertEqual(Direction.S.delta, (0, -1))
        self.assertEqual(Direction.W.delta, (-1, 0))

    def test_turned(self):
        self.assertEqual(Direction.N.turned(1), Direction.E)   # 右
        self.assertEqual(Direction.N.turned(-1), Direction.W)  # 左
        self.assertEqual(Direction.N.turned(2), Direction.S)

    def test_turn_between(self):
        self.assertEqual(turn_between(Direction.N, Direction.N), 0)
        self.assertEqual(turn_between(Direction.N, Direction.E), 1)   # 右90
        self.assertEqual(turn_between(Direction.N, Direction.W), -1)  # 左90
        self.assertEqual(turn_between(Direction.N, Direction.S), 2)   # Uターン

    def test_slalom_symbol(self):
        self.assertEqual(slalom_dir_symbol(1), "R")
        self.assertEqual(slalom_dir_symbol(-1), "L")
        with self.assertRaises(ValueError):
            slalom_dir_symbol(2)


class TestWallMap(unittest.TestCase):
    def test_bounds_are_walls(self):
        wm = WallMap(4, 4)
        self.assertTrue(wm.has_wall(0, 0, Direction.S))
        self.assertTrue(wm.has_wall(0, 0, Direction.W))
        self.assertFalse(wm.has_wall(0, 0, Direction.N))
        self.assertFalse(wm.has_wall(0, 0, Direction.E))

    def test_wall_is_shared(self):
        wm = WallMap(4, 4)
        wm.add_wall(1, 1, Direction.N)
        self.assertTrue(wm.has_wall(1, 1, Direction.N))
        # 隣セル (1,2) の S 側にも立っているはず
        self.assertTrue(wm.has_wall(1, 2, Direction.S))
        self.assertFalse(wm.can_move(1, 1, Direction.N))


class TestFindPath(unittest.TestCase):
    def test_straight_line_no_walls(self):
        wm = WallMap(4, 4)
        # (0,0) 北向き -> (0,3): まっすぐ北へ3セル
        path = find_path(wm, (0, 0), Direction.N, (0, 3))
        cells = [(x, y) for (x, y, _) in path]
        self.assertEqual(cells, [(0, 0), (0, 1), (0, 2), (0, 3)])

    def test_prefers_fewer_turns(self):
        # 開けた盤面で (0,0)N -> (3,3)。ターンは1回であるべき
        # (北へ3 → 東へ3、または東へ3 → 北へ3)。
        wm = WallMap(4, 4)
        path = find_path(wm, (0, 0), Direction.N, (3, 3))
        dirs = [path[i + 1][2] for i in range(len(path) - 1)]
        turns = sum(
            1 for i in range(1, len(dirs)) if dirs[i] != dirs[i - 1]
        )
        # 先頭の向き変更も1回に含める
        first_turn = 1 if dirs and dirs[0] != Direction.N else 0
        self.assertEqual(turns + first_turn, 1)

    def test_no_path_raises(self):
        wm = WallMap(2, 2)
        # (0,0) を四方壁で閉じ込める
        wm.add_wall(0, 0, Direction.N)
        wm.add_wall(0, 0, Direction.E)
        with self.assertRaises(ValueError):
            find_path(wm, (0, 0), Direction.N, (1, 1))


class TestSegments(unittest.TestCase):
    def test_pure_straight(self):
        # 北へ3セル、ターン無し: 単一 Straight 540mm、発進停止
        wm = WallMap(4, 4)
        segs = plan(wm, (0, 0), Direction.N, (0, 3))
        self.assertEqual(len(segs), 1)
        s = segs[0]
        self.assertIsInstance(s, Straight)
        self.assertAlmostEqual(s.distance_mm, 3 * CELL_MM)
        self.assertEqual(s.v_start_mmps, 0.0)
        self.assertEqual(s.v_end_mmps, 0.0)
        self.assertEqual(s.v_cruise_mmps, 400.0)

    def test_l_shape_has_one_slalom(self):
        # (0,0)N -> (3,3): 直進 → 右スラローム → 直進。
        wm = WallMap(4, 4)
        segs = plan(wm, (0, 0), Direction.N, (3, 3))
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(len(straights), 2)
        # スラロームで前後の直進が半径分(90mm)短縮される
        R = 90.0
        # 3セル北→曲がる: 最初の直進 = 3*180 - 90
        self.assertAlmostEqual(straights[0].distance_mm, 3 * CELL_MM - R)
        # 曲がった後 3セル東: 3*180 - 90
        self.assertAlmostEqual(straights[1].distance_mm, 3 * CELL_MM - R)
        # 直進とスラロームの境界速度は slalom 速度
        self.assertEqual(straights[0].v_start_mmps, 0.0)
        self.assertEqual(straights[0].v_end_mmps, 360.0)
        self.assertEqual(straights[1].v_start_mmps, 360.0)
        self.assertEqual(straights[1].v_end_mmps, 0.0)

    def test_initial_turn_from_start(self):
        # スタートで既に東を向いていて (0,0)E -> (3,0): ターン無しの直進のみ
        wm = WallMap(4, 4)
        segs = plan(wm, (0, 0), Direction.E, (3, 0))
        self.assertEqual(len(segs), 1)
        self.assertIsInstance(segs[0], Straight)

    def test_zigzag_zero_length_straight_dropped(self):
        # 半径90=半セルなので、1セルおきのジグザグは直進0mmになり Slalom 連続
        moves = _states_to_moves(
            # 手動で状態列を作る: (0,0)N ->(0,1)N ->(1,1)E ->(1,2)N
            [
                (0, 0, Direction.N),
                (0, 1, Direction.N),
                (1, 1, Direction.E),
                (1, 2, Direction.N),
            ],
            Direction.N,
        )
        segs = moves_to_segments(moves, PlannerConfig())
        # 中間の 1セル直進(turnに挟まれる)は 180-90-90=0 で除去される
        # 少なくともスラロームが2つ含まれる
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        self.assertEqual(len(slaloms), 2)


if __name__ == "__main__":
    unittest.main()
