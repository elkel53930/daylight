"""シミュレーションテスト(18.2 Simulation Test)。

仮想迷路 + 仮想センサ + 仮想走行で、状態機械の全シーケンス
(探索 → ゴール → 帰還 → 経路計画 → 最短走行)を検証する。
SimMobileBase は壁を突き抜ける走行を SimulationCrash にするため、
探索・走行アルゴリズムの整合性がここで保証される。
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from config import MicromouseConfig
from maze import Direction, Maze, WallState, load_maze_text
from simulator import SimMobileBase, SimulationCrash
from state_machine import MicromouseMission, MissionState

MAZE_DIR = Path(__file__).parent.parent / "maze_files"

# 4x4 の全探索可能な迷路(ゴールは中央 2x2 の (1,1)-(2,2))
SMALL_MAZE = """\
+-+-+-+-+
|   |   |
+ + + + +
| |   | |
+ + + + +
| | | | |
+ + + + +
|S|     |
+-+-+-+-+
"""


def small_config(**overrides) -> MicromouseConfig:
    config = MicromouseConfig(
        maze_size=4,
        goal_x_min=1,
        goal_x_max=2,
        goal_y_min=1,
        goal_y_max=2,
        max_exploration_steps=200,
    )
    for k, v in overrides.items():
        setattr(config, k, v)
    return config


class TestSmallMazeMission(unittest.TestCase):
    def setUp(self):
        true_maze, _ = load_maze_text(SMALL_MAZE, size=4)
        self.true_maze = true_maze
        self.trace = []
        self.base = SimMobileBase(true_maze, trace=self.trace)
        self.states = []
        self.log = []
        self.mission = MicromouseMission(
            self.base,
            small_config(),
            observer=lambda s, info: self.states.append(s),
            log_fn=self.log.append,
        )

    def test_full_mission(self):
        final = self.mission.run()
        self.assertEqual(final, MissionState.FINISHED)

        # 状態遷移の順序
        expected = [
            MissionState.CALIBRATION,
            MissionState.MICROMOUSE_START,
            MissionState.EXPLORATION,
            MissionState.GOAL_REACHED,
            MissionState.RETURN_TO_START,
            MissionState.PATH_PLANNING,
            MissionState.SPEED_RUN,
            MissionState.FINISHED,
        ]
        self.assertEqual(self.states, expected)

        # 最短走行後、ロボットは物理的にゴールセルの中心にいる
        gx, gy = self.base.current_cell
        self.assertTrue(self.mission.goal.contains(gx, gy))
        self.assertTrue(self.base.is_at_center_of((gx, gy)))

        # 探索済み(訪問した)セルの壁情報は真の迷路と一致する
        maze = self.mission.maze
        for y in range(4):
            for x in range(4):
                if not maze.visited(x, y):
                    continue
                for d in Direction:
                    got = maze.wall(x, y, d)
                    if got == WallState.UNKNOWN:
                        continue
                    self.assertEqual(
                        got,
                        self.true_maze.wall(x, y, d),
                        f"wall mismatch at ({x},{y}) {d.name}",
                    )

        # ゴールとスタートは訪問済み
        self.assertTrue(maze.visited(0, 0))
        self.assertTrue(any(maze.visited(x, y) for (x, y) in self.mission.goal.cells()))

    def test_speed_run_skipped_when_not_confirmed(self):
        self.mission.confirm_speed_run = lambda: False
        final = self.mission.run()
        self.assertEqual(final, MissionState.FINISHED)
        self.assertNotIn(MissionState.SPEED_RUN, self.states)
        # 帰還後スタートセル中心で終わる
        self.assertTrue(self.base.is_at_center_of((0, 0)))

    def test_run_log_recorded(self):
        self.mission.run()
        events = {r.get("event") for r in self.log}
        self.assertIn("observe", events)
        self.assertIn("action", events)
        self.assertIn("plan", events)
        self.assertIn("motion", events)
        # observe レコードにセンサ生値と壁判定が入っている
        obs = next(r for r in self.log if r.get("event") == "observe")
        self.assertIn("sensors", obs)
        self.assertIn("walls", obs)
        self.assertIn("pose", obs)


class TestAbortAndErrors(unittest.TestCase):
    def _make_mission(self, base, **config_overrides):
        states = []
        mission = MicromouseMission(
            base,
            small_config(**config_overrides),
            observer=lambda s, info: states.append(s),
        )
        return mission, states

    def test_abort_leads_to_emergency_stop(self):
        true_maze, _ = load_maze_text(SMALL_MAZE, size=4)
        calls = {"n": 0}

        def abort_after_a_while() -> bool:
            calls["n"] += 1
            return calls["n"] > 10

        base = SimMobileBase(true_maze, abort_check=abort_after_a_while)
        mission, states = self._make_mission(base)
        final = mission.run()
        self.assertEqual(final, MissionState.EMERGENCY_STOP)

    def test_low_battery_leads_to_error(self):
        true_maze, _ = load_maze_text(SMALL_MAZE, size=4)
        base = SimMobileBase(true_maze, battery_v=6.0)
        mission, states = self._make_mission(base)
        final = mission.run()
        self.assertEqual(final, MissionState.ERROR)
        self.assertIn("battery", mission.error_message)

    def test_unreachable_goal_leads_to_error(self):
        # ゴール(中央2x2)を完全に壁で囲った迷路
        blocked = """\
+-+-+-+-+
|   |   |
+ +-+-+ +
| |   | |
+ + + + +
| |   | |
+ +-+-+ +
|S      |
+-+-+-+-+
"""
        true_maze, _ = load_maze_text(blocked, size=4)
        base = SimMobileBase(true_maze)
        mission, states = self._make_mission(base)
        final = mission.run()
        self.assertEqual(final, MissionState.ERROR)
        self.assertIn("no path", mission.error_message)


class TestFullSizeMazes(unittest.TestCase):
    """実際の全日本大会迷路(16x16)での完走テスト。"""

    def run_maze_file(self, name: str) -> None:
        text = (MAZE_DIR / name).read_text(encoding="utf-8")
        true_maze, goal = load_maze_text(text, size=16)
        base = SimMobileBase(true_maze)
        mission = MicromouseMission(base, MicromouseConfig(max_exploration_steps=2000))
        final = mission.run()
        self.assertEqual(final, MissionState.FINISHED, mission.error_message)
        gx, gy = base.current_cell
        self.assertTrue(mission.goal.contains(gx, gy))
        self.assertTrue(base.is_at_center_of((gx, gy)))

    def test_alljapan_1981(self):
        self.run_maze_file("AllJapan_002_1981_classic___16x16.txt")

    def test_alljapan_1993(self):
        self.run_maze_file("AllJapan_014_1993_classic_exp_fin_16x16.txt")


class TestSimulatorItself(unittest.TestCase):
    def test_crash_detection(self):
        true_maze, _ = load_maze_text(SMALL_MAZE, size=4)
        base = SimMobileBase(true_maze)
        base.heading = Direction.EAST  # (0,0) の東は壁
        with self.assertRaises(SimulationCrash):
            base.forward(300, 1000, 180)

    def test_turn_directions(self):
        true_maze, _ = load_maze_text(SMALL_MAZE, size=4)
        base = SimMobileBase(true_maze)
        import math

        base.turn(math.pi / 2)  # 左
        self.assertEqual(base.heading, Direction.WEST)
        base.turn(-math.pi / 2)  # 右
        self.assertEqual(base.heading, Direction.NORTH)
        base.turn(math.pi)  # 180度
        self.assertEqual(base.heading, Direction.SOUTH)

    def test_sensor_synthesis_at_boundary(self):
        true_maze, _ = load_maze_text(SMALL_MAZE, size=4)
        base = SimMobileBase(true_maze)
        base.wall_led(True)
        # スタートセル中心から 90mm 前進 → (0,0)/(0,1) 境界
        base.forward(300, 1000, 90)
        frame = base.read_sensors()
        # (0,1): 西=外周壁, 東=壁('| |' の列), 北=開き
        self.assertGreaterEqual(frame.ls, 100)   # 左(西)壁あり
        self.assertGreaterEqual(frame.rs, 100)   # 右(東)壁あり
        self.assertLess(min(frame.lf, frame.rf), 50)  # 前(北)壁なし

    def test_sensor_zero_when_led_off(self):
        true_maze, _ = load_maze_text(SMALL_MAZE, size=4)
        base = SimMobileBase(true_maze)
        frame = base.read_sensors()
        self.assertEqual((frame.lf, frame.ls, frame.rs, frame.rf), (0, 0, 0, 0))


if __name__ == "__main__":
    unittest.main()
