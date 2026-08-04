"""dev_maze(開発用4×4既知迷路)の構造テスト(ハード非依存)。

俯瞰(2026-08-04)から起こした壁が正しく載っているか、開放部が開いているかを確認。
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dev_maze import build_dev_maze
from geometry import Direction


class TestDevMaze(unittest.TestCase):
    def setUp(self):
        self.wm = build_dev_maze()

    def test_size(self):
        self.assertEqual((self.wm.width, self.wm.height), (4, 4))

    def test_detected_walls_present(self):
        N, E, S, W = Direction.N, Direction.E, Direction.S, Direction.W
        self.assertTrue(self.wm.has_wall(0, 3, S))
        self.assertTrue(self.wm.has_wall(2, 3, S))
        self.assertTrue(self.wm.has_wall(2, 2, S))
        self.assertTrue(self.wm.has_wall(1, 1, S))
        self.assertTrue(self.wm.has_wall(1, 1, W))
        self.assertTrue(self.wm.has_wall(1, 0, W))
        self.assertTrue(self.wm.has_wall(2, 1, E))

    def test_shared_walls_mirror(self):
        # 壁は共有: (2,1)E は (3,1)W と同一。
        self.assertTrue(self.wm.has_wall(3, 1, Direction.W))
        self.assertTrue(self.wm.has_wall(0, 2, Direction.N))  # (0,3)S の反対側

    def test_known_open_passages(self):
        # (1,3) は内壁なし(北の外周のみ)。ロボットが居たセル。
        self.assertFalse(self.wm.has_wall(1, 3, Direction.S))
        self.assertFalse(self.wm.has_wall(1, 3, Direction.E))
        self.assertFalse(self.wm.has_wall(1, 3, Direction.W))
        # (2,2) は東西開放(N/Sは壁)。
        self.assertFalse(self.wm.has_wall(2, 2, Direction.E))
        self.assertFalse(self.wm.has_wall(2, 2, Direction.W))

    def test_outer_boundary_is_wall(self):
        self.assertTrue(self.wm.has_wall(0, 0, Direction.S))
        self.assertTrue(self.wm.has_wall(0, 0, Direction.W))
        self.assertTrue(self.wm.has_wall(3, 3, Direction.N))
        self.assertTrue(self.wm.has_wall(3, 3, Direction.E))


if __name__ == "__main__":
    unittest.main()
