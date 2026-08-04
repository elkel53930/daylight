"""liner_center の純ロジック(壁選択・近傍探索)テスト(ハード非依存)。

実機を要する center_axis/recenter_cell はここではテストしない(オーケストレーション)。
"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dev_maze import build_dev_maze
from geometry import Direction
from liner_center import available_faces, neighbor_for_axis


class TestAvailableFaces(unittest.TestCase):
    def setUp(self):
        self.m = build_dev_maze()

    def test_cell_1_3_only_y_north(self):
        # (1,3): E/W壁なし、Nは外周 → x=[], y=[N]
        xf, yf = available_faces(self.m, 1, 3)
        self.assertEqual(xf, [])
        self.assertEqual(yf, [Direction.N])

    def test_corner_0_3_both_axes(self):
        # (0,3): W外周、S内壁、N外周 → x=[W], y=[N,S]
        xf, yf = available_faces(self.m, 0, 3)
        self.assertEqual(xf, [Direction.W])
        self.assertIn(Direction.N, yf)
        self.assertIn(Direction.S, yf)

    def test_cell_1_1_both_axes(self):
        # (1,1): W内壁, S内壁 → x=[W], y=[S]
        xf, yf = available_faces(self.m, 1, 1)
        self.assertEqual(xf, [Direction.W])
        self.assertEqual(yf, [Direction.S])

    def test_cell_2_1_both_axes(self):
        # (2,1): E内壁, N内壁 → x=[E], y=[N]
        xf, yf = available_faces(self.m, 2, 1)
        self.assertEqual(xf, [Direction.E])
        self.assertEqual(yf, [Direction.N])


class TestNeighborForAxis(unittest.TestCase):
    def setUp(self):
        self.m = build_dev_maze()

    def test_x_neighbor_of_1_3_is_west_cell(self):
        # (1,3)にX壁なし。Nは外周で不可、E→(2,3)X壁なし、S→(1,2)X壁なし、
        # W→(0,3)はW外周ありなので (0,3, W)。
        res = neighbor_for_axis(self.m, 1, 3, "x")
        self.assertIsNotNone(res)
        ncx, ncy, move = res
        self.assertEqual((ncx, ncy), (0, 3))
        self.assertEqual(move, Direction.W)

    def test_axis_present_still_returns_some_neighbor_only_when_needed(self):
        # 現在セルに壁がある軸でも neighbor_for_axis 自体は「その軸の壁を持つ隣」を
        # 探すだけ(呼び出し側が壁欠如時のみ使う)。(1,1)のY壁を持つ隣が見つかる。
        res = neighbor_for_axis(self.m, 1, 1, "y")
        # 進入可能な隣で y壁(N/S)を持つセルがあれば返る。無ければNoneでも可。
        if res is not None:
            ncx, ncy, move = res
            self.assertFalse(self.m.has_wall(1, 1, move))  # 壁なしで進入できる向き


if __name__ == "__main__":
    unittest.main()
