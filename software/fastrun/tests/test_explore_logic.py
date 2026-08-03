"""floodfill / mapping の純ロジックのユニットテスト(ハード非依存)。"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from floodfill import flood_fill, next_direction
from geometry import Direction
from mapping import update_walls, walls_from_raw
from maze import WallMap


class TestFloodFill(unittest.TestCase):
    def test_open_maze_manhattan(self):
        wm = WallMap(4, 4)
        dist = flood_fill(wm, [(3, 3)])
        self.assertEqual(dist[(3, 3)], 0)
        self.assertEqual(dist[(3, 2)], 1)
        self.assertEqual(dist[(0, 0)], 6)  # 3+3

    def test_wall_increases_distance(self):
        wm = WallMap(3, 1)  # 横1列 3セル
        # (0,0)-(1,0)間に壁を立てると(0,0)から(2,0)へ到達不能(1列なので)
        wm.add_wall(1, 0, Direction.W)
        dist = flood_fill(wm, [(2, 0)])
        self.assertIn((1, 0), dist)
        self.assertNotIn((0, 0), dist)  # 壁で隔離

    def test_next_direction_toward_goal(self):
        wm = WallMap(4, 4)
        dist = flood_fill(wm, [(3, 3)])
        # (0,0)から見て N か E(距離5)へ進むはず
        nd = next_direction(wm, (0, 0), dist)
        self.assertIn(nd, (Direction.N, Direction.E))

    def test_next_direction_none_at_goal(self):
        wm = WallMap(4, 4)
        dist = flood_fill(wm, [(3, 3)])
        self.assertIsNone(next_direction(wm, (3, 3), dist))


class TestMapping(unittest.TestCase):
    def test_walls_from_raw(self):
        self.assertEqual(walls_from_raw(300, 10, 250), (True, False, True))
        self.assertEqual(walls_from_raw(0, 0, 0), (False, False, False))

    def test_update_walls_facing_north(self):
        wm = WallMap(4, 4)
        # 北向きで 前=壁, 左=壁, 右=開放
        update_walls(wm, (1, 1), Direction.N, front=True, left=True, right=False)
        self.assertTrue(wm.has_wall(1, 1, Direction.N))   # 前=北
        self.assertTrue(wm.has_wall(1, 1, Direction.W))   # 左=西
        self.assertFalse(wm.has_wall(1, 1, Direction.E))  # 右=東は開放

    def test_update_walls_facing_east(self):
        wm = WallMap(4, 4)
        # 東向きで 前=壁, 左=開放, 右=壁
        update_walls(wm, (2, 2), Direction.E, front=True, left=False, right=True)
        self.assertTrue(wm.has_wall(2, 2, Direction.E))   # 前=東
        self.assertFalse(wm.has_wall(2, 2, Direction.N))  # 左=北は開放
        self.assertTrue(wm.has_wall(2, 2, Direction.S))   # 右=南

    def test_wall_shared_with_neighbor(self):
        wm = WallMap(4, 4)
        update_walls(wm, (1, 1), Direction.N, front=True, left=False, right=False)
        # 隣接セル(1,2)の南側にも壁
        self.assertTrue(wm.has_wall(1, 2, Direction.S))


if __name__ == "__main__":
    unittest.main()
