import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from maze import Direction, GoalRegion, INF, Maze, Pose, WallState, load_maze_text


class TestDirection(unittest.TestCase):
    def test_rotation(self):
        self.assertEqual(Direction.NORTH.left(), Direction.WEST)
        self.assertEqual(Direction.NORTH.right(), Direction.EAST)
        self.assertEqual(Direction.NORTH.back(), Direction.SOUTH)
        self.assertEqual(Direction.WEST.right(), Direction.NORTH)
        self.assertEqual(Direction.SOUTH.back(), Direction.NORTH)

    def test_vector(self):
        self.assertEqual(Direction.NORTH.vector, (0, 1))
        self.assertEqual(Direction.EAST.vector, (1, 0))
        self.assertEqual(Direction.SOUTH.vector, (0, -1))
        self.assertEqual(Direction.WEST.vector, (-1, 0))


class TestMazeWalls(unittest.TestCase):
    def test_initial_state(self):
        maze = Maze(4)
        # 外周は WALL
        self.assertEqual(maze.wall(0, 0, Direction.SOUTH), WallState.WALL)
        self.assertEqual(maze.wall(0, 0, Direction.WEST), WallState.WALL)
        self.assertEqual(maze.wall(3, 3, Direction.NORTH), WallState.WALL)
        self.assertEqual(maze.wall(3, 3, Direction.EAST), WallState.WALL)
        # 内部は UNKNOWN
        self.assertEqual(maze.wall(0, 0, Direction.NORTH), WallState.UNKNOWN)
        self.assertEqual(maze.wall(1, 1, Direction.EAST), WallState.UNKNOWN)

    def test_wall_consistency_between_neighbors(self):
        maze = Maze(4)
        maze.set_wall(1, 1, Direction.EAST, WallState.WALL)
        self.assertEqual(maze.wall(2, 1, Direction.WEST), WallState.WALL)

        maze.set_wall(2, 2, Direction.SOUTH, WallState.OPEN)
        self.assertEqual(maze.wall(2, 1, Direction.NORTH), WallState.OPEN)

    def test_observe_wall(self):
        maze = Maze(4)
        maze.observe_wall(0, 0, Direction.NORTH, True)
        self.assertEqual(maze.wall(0, 1, Direction.SOUTH), WallState.WALL)
        maze.observe_wall(0, 0, Direction.EAST, False)
        self.assertEqual(maze.wall(1, 0, Direction.WEST), WallState.OPEN)

    def test_boundary_cannot_be_opened(self):
        maze = Maze(4)
        with self.assertRaises(ValueError):
            maze.set_wall(0, 0, Direction.SOUTH, WallState.OPEN)

    def test_out_of_bounds_raises(self):
        maze = Maze(4)
        with self.assertRaises(ValueError):
            maze.wall(4, 0, Direction.NORTH)
        with self.assertRaises(ValueError):
            maze.set_wall(-1, 0, Direction.NORTH, WallState.WALL)

    def test_visited(self):
        maze = Maze(4)
        self.assertFalse(maze.visited(1, 2))
        maze.mark_visited(1, 2)
        self.assertTrue(maze.visited(1, 2))
        self.assertEqual(maze.visited_count(), 1)


class TestCanPass(unittest.TestCase):
    def test_boundary_blocks(self):
        maze = Maze(4)
        self.assertFalse(maze.can_pass(0, 0, Direction.SOUTH, unknown_as_open=True))
        self.assertFalse(maze.can_pass(3, 0, Direction.EAST, unknown_as_open=True))

    def test_unknown_policy(self):
        maze = Maze(4)
        self.assertTrue(maze.can_pass(1, 1, Direction.NORTH, unknown_as_open=True))
        self.assertFalse(maze.can_pass(1, 1, Direction.NORTH, unknown_as_open=False))

    def test_known_walls(self):
        maze = Maze(4)
        maze.set_wall(1, 1, Direction.NORTH, WallState.WALL)
        maze.set_wall(1, 1, Direction.EAST, WallState.OPEN)
        self.assertFalse(maze.can_pass(1, 1, Direction.NORTH, unknown_as_open=True))
        self.assertTrue(maze.can_pass(1, 1, Direction.EAST, unknown_as_open=False))


class TestDistanceMap(unittest.TestCase):
    def test_open_maze_manhattan(self):
        maze = Maze(4)
        dist = maze.distance_map([(3, 3)], unknown_as_open=True)
        self.assertEqual(dist[3][3], 0)
        self.assertEqual(dist[0][0], 6)  # マンハッタン距離
        self.assertEqual(dist[3][0], 3)

    def test_wall_detour(self):
        maze = Maze(4)
        # (0,0) の北をふさぐ → (0,1) へは東回り
        maze.set_wall(0, 0, Direction.NORTH, WallState.WALL)
        dist = maze.distance_map([(0, 1)], unknown_as_open=True)
        self.assertEqual(dist[0][0], 3)  # (0,0)→(1,0)→(1,1)→(0,1)

    def test_unreachable_with_unknown_as_wall(self):
        maze = Maze(4)  # 内部が全て UNKNOWN
        dist = maze.distance_map([(3, 3)], unknown_as_open=False)
        self.assertEqual(dist[3][3], 0)
        self.assertEqual(dist[0][0], INF)

    def test_multi_source(self):
        maze = Maze(4)
        dist = maze.distance_map([(0, 3), (3, 0)], unknown_as_open=True)
        self.assertEqual(dist[3][0], 0)
        self.assertEqual(dist[0][3], 0)
        self.assertEqual(dist[0][0], 3)


class TestGoalRegion(unittest.TestCase):
    def test_center_2x2(self):
        goal = GoalRegion.center_2x2(16)
        self.assertEqual(goal, GoalRegion(7, 8, 7, 8))
        self.assertEqual(len(goal.cells()), 4)
        self.assertTrue(goal.contains(7, 7))
        self.assertTrue(goal.contains(8, 8))
        self.assertFalse(goal.contains(6, 7))
        self.assertFalse(goal.contains(7, 9))

    def test_single_cell(self):
        goal = GoalRegion(2, 2, 3, 3)
        self.assertEqual(goal.cells(), [(2, 3)])

    def test_invalid(self):
        with self.assertRaises(ValueError):
            GoalRegion(3, 2, 0, 0)


class TestPersistence(unittest.TestCase):
    def test_roundtrip(self):
        maze = Maze(4)
        maze.set_wall(1, 1, Direction.NORTH, WallState.WALL)
        maze.set_wall(2, 0, Direction.EAST, WallState.OPEN)
        maze.mark_visited(0, 0)
        maze.mark_visited(2, 3)

        restored = Maze.from_dict(maze.to_dict())
        self.assertEqual(restored.size, 4)
        self.assertEqual(restored.wall(1, 1, Direction.NORTH), WallState.WALL)
        self.assertEqual(restored.wall(1, 2, Direction.SOUTH), WallState.WALL)
        self.assertEqual(restored.wall(2, 0, Direction.EAST), WallState.OPEN)
        self.assertEqual(restored.wall(0, 3, Direction.EAST), WallState.UNKNOWN)
        self.assertTrue(restored.visited(2, 3))
        self.assertFalse(restored.visited(1, 1))


SIMPLE_MAZE_TEXT = """\
+-+-+-+-+
|     | |
+ +-+ + +
| | |   |
+ + +-+ +
| |   | |
+ +-+ + +
|   |  G|
+-+-+-+-+
"""


class TestLoadMazeText(unittest.TestCase):
    def test_load(self):
        maze, goal = load_maze_text(SIMPLE_MAZE_TEXT, size=4)
        self.assertEqual(goal, (3, 0))
        # 外周
        self.assertEqual(maze.wall(0, 0, Direction.WEST), WallState.WALL)
        self.assertEqual(maze.wall(3, 3, Direction.NORTH), WallState.WALL)
        # 内部の壁(上の図から): (0,0)の東は開き、(1,0)の北は壁
        self.assertEqual(maze.wall(0, 0, Direction.EAST), WallState.OPEN)
        self.assertEqual(maze.wall(1, 0, Direction.NORTH), WallState.WALL)
        # (2,3)の東は壁 (最上段の "| |")
        self.assertEqual(maze.wall(2, 3, Direction.EAST), WallState.WALL)
        # UNKNOWN が残っていないこと
        for y in range(4):
            for x in range(4):
                for d in Direction:
                    self.assertNotEqual(maze.wall(x, y, d), WallState.UNKNOWN)

    def test_render_roundtrip_smoke(self):
        maze, _ = load_maze_text(SIMPLE_MAZE_TEXT, size=4)
        text = maze.render_text(pose=Pose(0, 0, Direction.NORTH))
        self.assertIn("^", text)


if __name__ == "__main__":
    unittest.main()
