import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from maze import Direction, Maze, WallState
from path_planner import Motion, MotionType, path_to_motions, plan_cell_path


def open_all(maze: Maze) -> None:
    """内部の壁を全て OPEN にする(テスト用)。"""
    for y in range(maze.size):
        for x in range(maze.size):
            for d in Direction:
                dx, dy = d.vector
                if maze.in_bounds(x + dx, y + dy):
                    maze.set_wall(x, y, d, WallState.OPEN)


class TestPlanCellPath(unittest.TestCase):
    def test_straight_line(self):
        maze = Maze(4)
        open_all(maze)
        path = plan_cell_path(maze, (0, 0), [(0, 3)])
        self.assertEqual(path, [(0, 0), (0, 1), (0, 2), (0, 3)])

    def test_no_path_when_unknown(self):
        maze = Maze(4)  # 内部が全て UNKNOWN、既定は unknown_as_open=False
        path = plan_cell_path(maze, (0, 0), [(3, 3)])
        self.assertIsNone(path)

    def test_unknown_as_open_allows_path(self):
        maze = Maze(4)
        path = plan_cell_path(maze, (0, 0), [(3, 3)], unknown_as_open=True)
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (3, 3))
        self.assertEqual(len(path), 7)  # 最短 6 手 + 始点

    def test_detour_around_wall(self):
        maze = Maze(4)
        open_all(maze)
        # (0,0)→(0,1) をふさぐ
        maze.set_wall(0, 0, Direction.NORTH, WallState.WALL)
        path = plan_cell_path(maze, (0, 0), [(0, 1)])
        self.assertEqual(path, [(0, 0), (1, 0), (1, 1), (0, 1)])

    def test_start_in_goal(self):
        maze = Maze(4)
        path = plan_cell_path(maze, (2, 2), [(2, 2)])
        self.assertEqual(path, [(2, 2)])

    def test_multiple_goals_pick_nearest(self):
        maze = Maze(4)
        open_all(maze)
        path = plan_cell_path(maze, (0, 0), [(3, 3), (1, 0)])
        self.assertEqual(path[-1], (1, 0))


class TestPathToMotions(unittest.TestCase):
    def test_straight_only(self):
        path = [(0, 0), (0, 1), (0, 2), (0, 3)]
        motions, heading = path_to_motions(path, Direction.NORTH)
        self.assertEqual(motions, [Motion(MotionType.STRAIGHT, 3)])
        self.assertEqual(heading, Direction.NORTH)

    def test_turn_right(self):
        # 北へ2、東へ1
        path = [(0, 0), (0, 1), (0, 2), (1, 2)]
        motions, heading = path_to_motions(path, Direction.NORTH)
        self.assertEqual(
            motions,
            [
                Motion(MotionType.STRAIGHT, 2),
                Motion(MotionType.TURN_RIGHT),
                Motion(MotionType.STRAIGHT, 1),
            ],
        )
        self.assertEqual(heading, Direction.EAST)

    def test_turn_left(self):
        path = [(2, 0), (2, 1), (1, 1)]
        motions, heading = path_to_motions(path, Direction.NORTH)
        self.assertEqual(
            motions,
            [
                Motion(MotionType.STRAIGHT, 1),
                Motion(MotionType.TURN_LEFT),
                Motion(MotionType.STRAIGHT, 1),
            ],
        )
        self.assertEqual(heading, Direction.WEST)

    def test_initial_turn_back(self):
        # 南向きで北へ行く経路 → まず 180 度旋回
        path = [(0, 0), (0, 1)]
        motions, heading = path_to_motions(path, Direction.SOUTH)
        self.assertEqual(
            motions,
            [Motion(MotionType.TURN_BACK), Motion(MotionType.STRAIGHT, 1)],
        )
        self.assertEqual(heading, Direction.NORTH)

    def test_empty_path(self):
        motions, heading = path_to_motions([(0, 0)], Direction.EAST)
        self.assertEqual(motions, [])
        self.assertEqual(heading, Direction.EAST)

    def test_zigzag(self):
        path = [(0, 0), (1, 0), (1, 1), (2, 1), (2, 2)]
        motions, heading = path_to_motions(path, Direction.EAST)
        self.assertEqual(
            motions,
            [
                Motion(MotionType.STRAIGHT, 1),
                Motion(MotionType.TURN_LEFT),
                Motion(MotionType.STRAIGHT, 1),
                Motion(MotionType.TURN_RIGHT),
                Motion(MotionType.STRAIGHT, 1),
                Motion(MotionType.TURN_LEFT),
                Motion(MotionType.STRAIGHT, 1),
            ],
        )
        self.assertEqual(heading, Direction.NORTH)


class TestPlanAndConvertIntegration(unittest.TestCase):
    def test_total_cells_matches_path_length(self):
        maze = Maze(8)
        open_all(maze)
        maze.set_wall(3, 3, Direction.NORTH, WallState.WALL)
        maze.set_wall(3, 3, Direction.EAST, WallState.WALL)
        path = plan_cell_path(maze, (0, 0), [(7, 7)])
        self.assertIsNotNone(path)
        motions, _ = path_to_motions(path, Direction.NORTH)
        total = sum(m.cells for m in motions if m.type == MotionType.STRAIGHT)
        self.assertEqual(total, len(path) - 1)


if __name__ == "__main__":
    unittest.main()
