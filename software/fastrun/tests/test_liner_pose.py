"""liner_pose(Liner姿勢+座標変換)のユニットテスト(ハード非依存)。"""
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from geometry import Direction
from liner_pose import (
    LinerPose,
    cell_center_mm,
    direction_to_gyro_deg,
)


class TestCellCenter(unittest.TestCase):
    def test_origin_cell_center(self):
        self.assertEqual(cell_center_mm(0, 0), (90.0, 90.0))

    def test_ne_cell_center(self):
        # (3,3) 中心 = (90+540, 90+540) = (630,630)
        self.assertEqual(cell_center_mm(3, 3), (630.0, 630.0))

    def test_generic_cell(self):
        self.assertEqual(cell_center_mm(1, 2), (270.0, 450.0))


class TestGyroHeading(unittest.TestCase):
    def test_mapping(self):
        # recenter規約: 北=0, 西=+90, 東=-90, 南=180
        self.assertEqual(direction_to_gyro_deg(Direction.N), 0.0)
        self.assertEqual(direction_to_gyro_deg(Direction.W), 90.0)
        self.assertEqual(direction_to_gyro_deg(Direction.E), -90.0)
        self.assertEqual(direction_to_gyro_deg(Direction.S), 180.0)


class TestLinerPose(unittest.TestCase):
    def test_center_and_heading(self):
        p = LinerPose(1, 3, Direction.N)
        self.assertEqual(p.cell, (1, 3))
        self.assertEqual(p.center_mm(), (270.0, 630.0))
        self.assertEqual(p.heading_deg(), 0.0)

    def test_moved_keeps_heading(self):
        p = LinerPose(1, 1, Direction.N)
        q = p.moved(Direction.E, 2)
        self.assertEqual((q.cx, q.cy), (3, 1))
        self.assertEqual(q.heading, Direction.N)

    def test_advanced_uses_heading(self):
        p = LinerPose(0, 0, Direction.N)
        self.assertEqual(p.advanced().cell, (0, 1))
        p2 = LinerPose(0, 0, Direction.E)
        self.assertEqual(p2.advanced(3).cell, (3, 0))

    def test_facing_changes_only_heading(self):
        p = LinerPose(2, 2, Direction.N).facing(Direction.W)
        self.assertEqual(p.cell, (2, 2))
        self.assertEqual(p.heading, Direction.W)


if __name__ == "__main__":
    unittest.main()
