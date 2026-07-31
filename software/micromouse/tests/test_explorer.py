import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from explorer import Explorer, WallObservation, observation_to_absolute
from maze import Direction, Maze, Pose, WallState


class TestObservationToAbsolute(unittest.TestCase):
    def test_facing_north(self):
        obs = WallObservation(left=True, front=False, right=True)
        result = observation_to_absolute(Direction.NORTH, obs)
        self.assertEqual(
            result,
            {Direction.WEST: True, Direction.NORTH: False, Direction.EAST: True},
        )

    def test_facing_east(self):
        obs = WallObservation(left=True, front=True, right=False)
        result = observation_to_absolute(Direction.EAST, obs)
        self.assertEqual(
            result,
            {Direction.NORTH: True, Direction.EAST: True, Direction.SOUTH: False},
        )

    def test_facing_south_and_west(self):
        obs = WallObservation(left=False, front=True, right=False)
        self.assertEqual(
            observation_to_absolute(Direction.SOUTH, obs),
            {Direction.EAST: False, Direction.SOUTH: True, Direction.WEST: False},
        )
        self.assertEqual(
            observation_to_absolute(Direction.WEST, obs),
            {Direction.SOUTH: False, Direction.WEST: True, Direction.NORTH: False},
        )

    def test_none_is_not_observed(self):
        obs = WallObservation(left=None, front=True, right=None)
        result = observation_to_absolute(Direction.NORTH, obs)
        self.assertEqual(result, {Direction.NORTH: True})


class TestExplorer(unittest.TestCase):
    def test_update_walls_keeps_consistency(self):
        maze = Maze(4)
        explorer = Explorer(maze, [(3, 3)])
        pose = Pose(1, 1, Direction.NORTH)
        explorer.update_walls(
            pose, WallObservation(left=True, front=False, right=True)
        )
        self.assertEqual(maze.wall(1, 1, Direction.WEST), WallState.WALL)
        self.assertEqual(maze.wall(0, 1, Direction.EAST), WallState.WALL)
        self.assertEqual(maze.wall(1, 1, Direction.NORTH), WallState.OPEN)
        self.assertEqual(maze.wall(1, 2, Direction.SOUTH), WallState.OPEN)

    def test_next_heading_moves_toward_goal(self):
        maze = Maze(4)
        explorer = Explorer(maze, [(3, 3)])
        # 開けた迷路では前(北)も右(東)も距離は同じ → 直進優先
        heading = explorer.next_heading(Pose(0, 0, Direction.NORTH))
        self.assertEqual(heading, Direction.NORTH)
        # 東向きなら東が「前」
        heading = explorer.next_heading(Pose(0, 0, Direction.EAST))
        self.assertEqual(heading, Direction.EAST)

    def test_next_heading_avoids_walls(self):
        maze = Maze(4)
        explorer = Explorer(maze, [(3, 0)])
        # 東をふさぐと北か南へ迂回するしかない
        maze.set_wall(0, 0, Direction.EAST, WallState.WALL)
        heading = explorer.next_heading(Pose(0, 0, Direction.EAST))
        self.assertEqual(heading, Direction.NORTH)

    def test_dead_end_turns_back(self):
        maze = Maze(4)
        explorer = Explorer(maze, [(3, 3)])
        # (1,1) を北向きで行き止まりに: 北・東・西が壁
        maze.set_wall(1, 1, Direction.NORTH, WallState.WALL)
        maze.set_wall(1, 1, Direction.EAST, WallState.WALL)
        maze.set_wall(1, 1, Direction.WEST, WallState.WALL)
        heading = explorer.next_heading(Pose(1, 1, Direction.NORTH))
        self.assertEqual(heading, Direction.SOUTH)

    def test_unreachable_goal_returns_none(self):
        maze = Maze(4)
        explorer = Explorer(maze, [(3, 3)])
        # ゴールを完全に囲う
        maze.set_wall(3, 3, Direction.SOUTH, WallState.WALL)
        maze.set_wall(3, 3, Direction.WEST, WallState.WALL)
        heading = explorer.next_heading(Pose(0, 0, Direction.NORTH))
        self.assertIsNone(heading)

    def test_set_goals_recomputes(self):
        maze = Maze(4)
        explorer = Explorer(maze, [(3, 3)])
        explorer.set_goals([(0, 3)])
        self.assertTrue(explorer.is_goal(0, 3))
        self.assertFalse(explorer.is_goal(3, 3))
        heading = explorer.next_heading(Pose(0, 0, Direction.NORTH))
        self.assertEqual(heading, Direction.NORTH)

    def test_goal_region_cells(self):
        maze = Maze(16)
        from maze import GoalRegion

        goal = GoalRegion.center_2x2(16)
        explorer = Explorer(maze, goal.cells())
        self.assertTrue(explorer.is_goal(7, 8))
        self.assertEqual(explorer.dist[7][7], 0)


if __name__ == "__main__":
    unittest.main()
