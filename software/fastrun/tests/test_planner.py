"""planner / geometry / maze のユニットテスト(ハード非依存)。"""

import math
import sys
import unittest
from pathlib import Path

# software/fastrun をimport探索パスに入れる(flat import)。
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from geometry import (
    CELL_MM,
    DIAG_CELL_MM,
    Direction,
    slalom_dir_symbol,
    turn_between,
)
from maze import WallMap
from planner import (
    PlannerConfig,
    _states_to_moves,
    _trapezoid_time_s,
    estimate_time,
    find_path,
    moves_to_segments,
    plan,
    pattern_from_cells,
)
from pattern import Slalom, Straight


class TestGeometry(unittest.TestCase):
    def test_delta_north_is_plus_y(self):
        self.assertEqual(Direction.N.delta, (0, 1))
        self.assertEqual(Direction.E.delta, (1, 0))
        self.assertEqual(Direction.S.delta, (0, -1))
        self.assertEqual(Direction.W.delta, (-1, 0))
        # 斜め
        self.assertEqual(Direction.NE.delta, (1, 1))
        self.assertEqual(Direction.SW.delta, (-1, -1))
        self.assertTrue(Direction.NE.is_diagonal)
        self.assertFalse(Direction.N.is_diagonal)

    def test_turned(self):
        self.assertEqual(Direction.N.turned(1), Direction.NE)  # 右45
        self.assertEqual(Direction.N.turned(2), Direction.E)   # 右90
        self.assertEqual(Direction.N.turned(-1), Direction.NW)  # 左45
        self.assertEqual(Direction.N.turned(-2), Direction.W)  # 左90
        self.assertEqual(Direction.N.turned(4), Direction.S)

    def test_turn_between(self):
        self.assertEqual(turn_between(Direction.N, Direction.N), 0)
        self.assertEqual(turn_between(Direction.N, Direction.NE), 1)  # 右45
        self.assertEqual(turn_between(Direction.N, Direction.E), 2)   # 右90
        self.assertEqual(turn_between(Direction.N, Direction.W), -2)  # 左90
        self.assertEqual(turn_between(Direction.N, Direction.S), 4)   # Uターン
        self.assertEqual(turn_between(Direction.N, Direction.SE), 3)  # 右135
        self.assertEqual(turn_between(Direction.N, Direction.SW), -3)  # 左135
        self.assertEqual(turn_between(Direction.N, Direction.NW), -1)  # 左45

    def test_slalom_symbol(self):
        self.assertEqual(slalom_dir_symbol(1), "R")
        self.assertEqual(slalom_dir_symbol(-1), "L")
        self.assertEqual(slalom_dir_symbol(3), "R")
        self.assertEqual(slalom_dir_symbol(-3), "L")
        with self.assertRaises(ValueError):
            slalom_dir_symbol(0)


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

    def test_can_move_diagonal(self):
        wm = WallMap(4, 4)
        # 開放なら斜め NE へ進める
        self.assertTrue(wm.can_move(0, 0, Direction.NE))
        # 直交の片側に壁があると斜めは通れない
        wm.add_wall(0, 0, Direction.N)
        self.assertFalse(wm.can_move(0, 0, Direction.NE))
        wm2 = WallMap(4, 4)
        wm2.add_wall(0, 0, Direction.E)
        self.assertFalse(wm2.can_move(0, 0, Direction.NE))
        # 外周へ向かう斜めは不可
        self.assertFalse(wm.can_move(0, 0, Direction.SW))
        self.assertFalse(wm2.can_move(3, 3, Direction.NE))

    def test_can_move_diagonal_target_side_wall(self):
        # (1,2) の NE 角を挟む目標セル側の壁((2,3)の南壁=角に面する辺)で塞がる。
        # この壁は出発セル側の壁チェックでは拾えないが、機体が角を切る際に
        # クリップするため斜めを禁止しなければならない(実機検証で発見)。
        wm = WallMap(4, 4)
        wm.add_wall(2, 3, Direction.S)  # (2,3)と(2,2)の境界
        self.assertTrue(wm.can_move(1, 2, Direction.N))    # 北は開いている
        self.assertTrue(wm.can_move(1, 2, Direction.E))    # 東も開いている
        self.assertFalse(wm.can_move(1, 2, Direction.NE))  # それでも斜めは不可

    def test_add_wall_diagonal_raises(self):
        wm = WallMap(4, 4)
        with self.assertRaises(ValueError):
            wm.add_wall(0, 0, Direction.NE)
        with self.assertRaises(ValueError):
            wm.has_wall(0, 0, Direction.SW)


def _diag_blocked_maze() -> WallMap:
    """4×4 の盤面で列0からの斜め(NE)カットだけを塞ぎ、(0,0)N→(3,3)が
    NNN→EEE のカーディナル L字(ターン1回)になるようにした迷路。"""
    wm = WallMap(4, 4)
    for y in range(3):
        wm.add_wall(0, y, Direction.E)
    return wm


class TestFindPath(unittest.TestCase):
    def test_straight_line_no_walls(self):
        wm = WallMap(4, 4)
        # (0,0) 北向き -> (0,3): まっすぐ北へ3セル
        path = find_path(wm, (0, 0), Direction.N, (0, 3))
        cells = [(x, y) for (x, y, _) in path]
        self.assertEqual(cells, [(0, 0), (0, 1), (0, 2), (0, 3)])

    def test_prefers_fewer_turns(self):
        # 斜めカットを塞いだ盤面で (0,0)N -> (3,3)。カーディナル L字
        # (北へ3 → 東へ3)でターンは1回であるべき。
        wm = _diag_blocked_maze()
        path = find_path(wm, (0, 0), Direction.N, (3, 3))
        dirs = [path[i + 1][2] for i in range(len(path) - 1)]
        turns = sum(
            1 for i in range(1, len(dirs)) if dirs[i] != dirs[i - 1]
        )
        # 先頭の向き変更も1回に含める
        first_turn = 1 if dirs and dirs[0] != Direction.N else 0
        self.assertEqual(turns + first_turn, 1)
        # 経路は 6 セル(北3→東3)
        self.assertEqual(len(path), 7)
        self.assertEqual(dirs, [Direction.N] * 3 + [Direction.E] * 3)

    def test_diagonal_used_on_open_board(self):
        # 開放盤面では斜め(45°×N)が最短時間になる
        wm = WallMap(4, 4)
        path = find_path(wm, (0, 0), Direction.N, (3, 3))
        dirs = [path[i + 1][2] for i in range(len(path) - 1)]
        self.assertEqual(dirs, [Direction.NE] * 3)

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
        # 斜めカットを塞いだ盤面で (0,0)N -> (3,3): 直進 → 右スラローム → 直進。
        wm = _diag_blocked_maze()
        segs = plan(wm, (0, 0), Direction.N, (3, 3))
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(len(straights), 2)
        self.assertEqual(slaloms[0].angle_deg, 90.0)
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

    def test_diagonal_plan_has_45deg_slalom(self):
        # 開放盤面 (0,0)N -> (2,2): 45°スラローム → 斜め直進(2セル)。
        wm = WallMap(4, 4)
        segs = plan(wm, (0, 0), Direction.N, (2, 2))
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(len(straights), 1)
        self.assertEqual(slaloms[0].angle_deg, 45.0)
        self.assertEqual(slaloms[0].dir, "R")
        # 斜め2セル - 45°スラロームの接線長 R·tan(22.5°)
        import math
        tangent = 90.0 * math.tan(math.radians(45.0) / 2.0)
        self.assertAlmostEqual(straights[0].distance_mm,
                               2 * DIAG_CELL_MM - tangent)
        self.assertEqual(straights[0].v_start_mmps, 360.0)
        self.assertEqual(straights[0].v_end_mmps, 0.0)

    def test_135deg_turn_segment(self):
        # 手動状態列: (0,0)N→(0,1)N→(0,2)N→(1,0)SE(135°右ターン)。
        moves = _states_to_moves(
            [
                (0, 0, Direction.N),
                (0, 1, Direction.N),
                (0, 2, Direction.N),
                (1, 0, Direction.SE),
            ],
            Direction.N,
        )
        segs = moves_to_segments(moves, PlannerConfig())
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(slaloms[0].angle_deg, 135.0)
        self.assertEqual(slaloms[0].dir, "R")
        # 135°の接線長 R·tan(67.5°)で前後が短縮される
        import math
        tangent = 90.0 * math.tan(math.radians(135.0) / 2.0)
        self.assertAlmostEqual(straights[0].distance_mm, 2 * CELL_MM - tangent)
        self.assertAlmostEqual(straights[1].distance_mm, DIAG_CELL_MM - tangent)

    def test_uturn_is_two_90deg_slaloms(self):
        # 180°Uターンは90°スラローム×2(直進の短縮は各R)。
        moves = _states_to_moves(
            [
                (0, 0, Direction.N),
                (0, 1, Direction.N),
                (0, 0, Direction.S),
            ],
            Direction.N,
        )
        segs = moves_to_segments(moves, PlannerConfig())
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertEqual(len(slaloms), 2)
        self.assertEqual(len(straights), 2)
        self.assertTrue(all(s.angle_deg == 90.0 for s in slaloms))
        # 両端の1セル直進は 180-90 = 90mm
        self.assertAlmostEqual(straights[0].distance_mm, CELL_MM - 90.0)
        self.assertAlmostEqual(straights[1].distance_mm, CELL_MM - 90.0)

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


class TestPatternFromCells(unittest.TestCase):
    def test_pure_straight_two_cells(self):
        # 北へ2セル: 単一 Straight 360mm、発進停止
        segs = pattern_from_cells([(0, 0), (0, 1), (0, 2)], Direction.N)
        self.assertEqual(len(segs), 1)
        s = segs[0]
        self.assertIsInstance(s, Straight)
        self.assertAlmostEqual(s.distance_mm, 2 * CELL_MM)

    def test_l_turn_with_slalom(self):
        # N→E の L字(2セル→2セル): 直進 → 右90° → 直進
        segs = pattern_from_cells(
            [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)], Direction.N
        )
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(slaloms[0].dir, "R")
        self.assertEqual(slaloms[0].angle_deg, 90.0)
        # 前後直進は接線長(90mm)だけ短縮
        self.assertAlmostEqual(straights[0].distance_mm, 2 * CELL_MM - 90.0)
        self.assertAlmostEqual(straights[1].distance_mm, 2 * CELL_MM - 90.0)

    def test_diagonal_move(self):
        # (0,0)→(1,1) は斜め NE: 45°R スラローム → 斜め直進(√2×180mm)
        segs = pattern_from_cells([(0, 0), (1, 1)], Direction.N)
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(slaloms[0].angle_deg, 45.0)
        self.assertEqual(slaloms[0].dir, "R")
        import math
        tangent = 90.0 * math.tan(math.radians(45.0) / 2.0)
        self.assertAlmostEqual(straights[0].distance_mm, DIAG_CELL_MM - tangent)

    def test_initial_turn_respected(self):
        # start_dir=W なのに経路は北: 最初に右90°スラローム(W→N)
        segs = pattern_from_cells([(0, 0), (0, 1)], Direction.W)
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(slaloms[0].dir, "R")
        self.assertEqual(slaloms[0].angle_deg, 90.0)

    def test_non_adjacent_raises(self):
        # (0,0)→(0,2) は1セル飛ばし → エラー
        with self.assertRaises(ValueError):
            pattern_from_cells([(0, 0), (0, 2)], Direction.N)

    def test_verify_loop_cells_close(self):
        # 現行 verify_loop と同じ閉ループマス列。
        cells = [
            (0, 0), (0, 1), (0, 2), (1, 2), (1, 3), (2, 3), (3, 3),
            (3, 2), (2, 2), (1, 2), (0, 2), (0, 1), (0, 0),
        ]
        segs = pattern_from_cells(cells, Direction.N)
        self.assertTrue(any(isinstance(s, Slalom) for s in segs))


VERIFY_LOOP_CELLS = [
    (0, 0), (0, 1), (0, 2), (1, 2), (1, 3), (2, 3), (3, 3),
    (3, 2), (2, 2), (1, 2), (0, 2), (0, 1), (0, 0),
]


class TestDiagonalShortcut(unittest.TestCase):
    """pattern_from_cells の斜めショートカット(45°スラローム+斜め直進)。"""

    def test_staircase_becomes_diagonal(self):
        # 前方階段(N→E,N→E)が「45°スラローム→斜め直進→45°スラローム」になる。
        segs = pattern_from_cells(VERIFY_LOOP_CELLS, Direction.N)
        self.assertEqual(len(segs), 10)
        self.assertEqual(segs[1].angle_deg, 45.0)
        self.assertEqual(segs[1].dir, "R")
        self.assertEqual(segs[3].angle_deg, 45.0)
        self.assertEqual(segs[3].dir, "R")
        T = 90.0 * math.tan(math.radians(45.0) / 2.0)
        # 入口直進 = 2セル − (半セル + T) = 232.72
        self.assertAlmostEqual(segs[0].distance_mm, 2 * CELL_MM - (90.0 + T),
                               places=6)
        # 斜め直進 = 270√2 − 2T = 307.28(直進3セル分の角を斜めに切る)
        self.assertAlmostEqual(segs[2].distance_mm,
                               270.0 * math.sqrt(2.0) - 2.0 * T, places=6)
        # 出口直進 = 2セル − (戻りR90の接線90 + 半セル + T) = 142.72
        self.assertAlmostEqual(segs[4].distance_mm,
                               2 * CELL_MM - 90.0 - (90.0 + T), places=6)
        # 斜め直進は 45°スラローム境界速度
        self.assertEqual(segs[2].v_start_mmps, 360.0)
        self.assertEqual(segs[2].v_end_mmps, 360.0)

    def test_diagonal_disabled_keeps_staircase(self):
        # diagonal=False なら従来通り90°スラローム列(斜め直進は出ない)。
        segs = pattern_from_cells(VERIFY_LOOP_CELLS, Direction.N,
                                  diagonal=False)
        straights = [s for s in segs if isinstance(s, Straight)]
        self.assertNotIn(307.28, [round(s.distance_mm, 2) for s in straights])
        self.assertTrue(all(isinstance(s, Slalom) and s.angle_deg == 90.0
                            for s in segs if isinstance(s, Slalom)))

    def test_diagonal_forward_path_geometry(self):
        # 前方区間(直進232.72 → R45 → 斜め307.28 → R45 → 直進142.72)を
        # トレースすると (540,630) に達する(戻りR90ターン前)。
        from planner import _trace_segments
        segs = pattern_from_cells(VERIFY_LOOP_CELLS, Direction.N)
        pts = _trace_segments(segs[:5], 90.0, 90.0,
                              Direction.N.heading_rad)
        x, y = pts[-1]
        self.assertAlmostEqual(x, 540.0, places=1)
        self.assertAlmostEqual(y, 630.0, places=1)

    def test_diagonal_clears_dev_maze(self):
        # dev迷路(wm)で斜め化すると、斜め直進が残り最小クリアランス≥50mm。
        from dev_maze import build_dev_maze
        from planner import _min_clearance, _trace_segments, _wall_segments_mm
        wm = build_dev_maze()
        segs = pattern_from_cells(VERIFY_LOOP_CELLS, Direction.N, wm=wm)
        T = 90.0 * math.tan(math.radians(45.0) / 2.0)
        diag = [s for s in segs if isinstance(s, Straight)
                and abs(s.distance_mm - (270.0 * math.sqrt(2.0) - 2.0 * T))
                < 1e-3]
        self.assertEqual(len(diag), 1)  # ゲートを通過して斜めが残る
        pts = _trace_segments(segs[:5], 90.0, 90.0, Direction.N.heading_rad)
        self.assertGreaterEqual(_min_clearance(pts, _wall_segments_mm(wm)),
                                50.0)

    def test_k2_staircase_becomes_diagonal(self):
        # N,N,E,N の単一コーナー(k=2: R90→L90)も斜め化できる。
        # (0,0)→(0,1)→(0,2)→(1,2)→(1,3): E(1)→N(1) の角を NE 斜めで切る。
        segs = pattern_from_cells(
            [(0, 0), (0, 1), (0, 2), (1, 2), (1, 3)], Direction.N)
        self.assertEqual(len(segs), 5)
        self.assertEqual(segs[1].angle_deg, 45.0)
        self.assertEqual(segs[1].dir, "R")   # 北→北東
        self.assertEqual(segs[3].angle_deg, 45.0)
        self.assertEqual(segs[3].dir, "L")   # 北東→北
        T = 90.0 * math.tan(math.radians(45.0) / 2.0)
        # 斜め直進 = 1セル斜め(180√2) − 2T = 180.0
        self.assertAlmostEqual(segs[2].distance_mm,
                               DIAG_CELL_MM - 2.0 * T, places=6)
        # 入口 2セル− (90+T) = 232.72、出口 1セル − (90+T) = 52.72
        self.assertAlmostEqual(segs[0].distance_mm, 2 * CELL_MM - (90.0 + T),
                               places=6)
        self.assertAlmostEqual(segs[4].distance_mm, CELL_MM - (90.0 + T),
                               places=6)

    def test_k2_diagonal_reaches_cell_center(self):
        # k=2 斜め化でも (1,3) マス中心 (270,630) に到達する。
        from planner import _trace_segments
        segs = pattern_from_cells(
            [(0, 0), (0, 1), (0, 2), (1, 2), (1, 3)], Direction.N)
        pts = _trace_segments(segs, 90.0, 90.0, Direction.N.heading_rad)
        x, y = pts[-1]
        self.assertAlmostEqual(x, 270.0, places=1)
        self.assertAlmostEqual(y, 630.0, places=1)

    def test_simple_l_turn_not_diagonalized(self):
        # ターン1回のL字は階段でないので斜め化しない(既存挙動を維持)。
        segs = pattern_from_cells(
            [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2)], Direction.N)
        slaloms = [s for s in segs if isinstance(s, Slalom)]
        self.assertEqual(len(slaloms), 1)
        self.assertEqual(slaloms[0].angle_deg, 90.0)


class TestTimeEstimate(unittest.TestCase):
    """Phase 4a: 台形プロファイル・実時間見積もりの精緻化。"""

    def test_trapezoid_time_reaches_cruise(self):
        # 1セル180mm・巡航400mm/s・加減速1000mm/s^2: 加速80mm+減速80mmで
        # 巡航区間20mmがある台形。t = 2*(400/1000) + 20/400 = 0.85s
        t = _trapezoid_time_s(180.0, 400.0, 0.0, 0.0, 1000.0)
        self.assertAlmostEqual(t, 0.85, places=4)

    def test_trapezoid_time_triangle_profile(self):
        # 距離が加速+減速に満たない(20mm): 巡航なし三角プロファイル。
        t = _trapezoid_time_s(20.0, 400.0, 0.0, 0.0, 1000.0)
        # v_peak = sqrt((2*1000*20 + 0 + 0)/2) = sqrt(20000) ≈ 141.4 mm/s
        # t = 141.4/1000 * 2 ≈ 0.2828 s
        self.assertAlmostEqual(t, 0.2828427, places=3)

    def test_trapezoid_time_start_end_nonzero(self):
        # 巡航開始・巡航終了(スラローム境界)の区間は加速・減速を省略する
        # (360 → 400 → 360): t = 0.04 + 巡航分 + 0.04
        t = _trapezoid_time_s(100.0, 400.0, 360.0, 360.0, 1000.0)
        # 加速距離 = (400²-360²)/2000 = 15.2mm、減速も同じ
        d_acc = (400.0**2 - 360.0**2) / 2000.0
        expected = 2 * (400.0 - 360.0) / 1000.0 + (100.0 - 2 * d_acc) / 400.0
        self.assertAlmostEqual(t, expected, places=4)

    def test_edge_time_now_includes_accel(self):
        # 精緻化後: 1セル(180mm)は台形プロファイルで0.85s(巡航一定の0.45sより長い)
        cfg = PlannerConfig()
        straight_t = _straight_time = _trapezoid_time_s(180.0, 400.0, 0.0, 0.0, 1000.0)
        self.assertGreater(straight_t, 0.45)
        self.assertAlmostEqual(straight_t, 0.85, places=4)

    def test_estimate_time_straight_only(self):
        # (0,0)N -> (0,3): 3セル直進(540mm、発進0・停止0)
        wm = WallMap(4, 4)
        segs = plan(wm, (0, 0), Direction.N, (0, 3))
        cfg = PlannerConfig()
        t = estimate_time(segs, cfg)
        expected = _trapezoid_time_s(540.0, 400.0, 0.0, 0.0, 1000.0)
        self.assertAlmostEqual(t, expected, places=4)

    def test_estimate_time_with_slalom(self):
        # 斜めカットを塞いだ盤面で (0,0)N -> (3,3): 直進 → 右90°スラローム → 直進
        wm = _diag_blocked_maze()
        segs = plan(wm, (0, 0), Direction.N, (3, 3))
        cfg = PlannerConfig()
        t = estimate_time(segs, cfg)
        # 手動で期待値を組み立てる
        R = cfg.slalom_radius_mm
        # 直進1: 3セル-90mm=450mm、v_start=0, v_end=360
        s1 = _trapezoid_time_s(3 * CELL_MM - R, 400.0, 0.0, 360.0, 1000.0)
        # スラローム: 90°・R90・360mm/s → 円弧長 141.37mm
        arc = R * math.pi / 2.0
        sl = arc / 360.0
        # 直進2: 3セル-90mm=450mm、v_start=360, v_end=0
        s2 = _trapezoid_time_s(3 * CELL_MM - R, 400.0, 360.0, 0.0, 1000.0)
        self.assertAlmostEqual(t, s1 + sl + s2, places=4)

    def test_estimate_time_diagonal(self):
        # 開放盤面 (0,0)N -> (2,2): 45°スラローム + 斜め直進
        wm = WallMap(4, 4)
        segs = plan(wm, (0, 0), Direction.N, (2, 2))
        cfg = PlannerConfig()
        t = estimate_time(segs, cfg)
        R = cfg.slalom_radius_mm
        tangent = R * math.tan(math.radians(45.0) / 2.0)
        s = _trapezoid_time_s(2 * DIAG_CELL_MM - tangent, 400.0, 360.0, 0.0, 1000.0)
        arc = R * math.radians(45.0)
        self.assertAlmostEqual(t, s + arc / 360.0, places=4)


if __name__ == "__main__":
    unittest.main()
