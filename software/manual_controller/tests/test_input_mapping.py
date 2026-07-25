import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import protocol as proto
from input_mapping import hat_to_events


class TestHatToEvents(unittest.TestCase):
    def test_no_change_emits_nothing(self):
        self.assertEqual(hat_to_events((0, 0), (0, 0)), [])
        self.assertEqual(hat_to_events((1, 0), (1, 0)), [])

    def test_up_press(self):
        self.assertEqual(
            hat_to_events((0, 0), (0, 1)), [(proto.DPAD_UP, proto.ACTION_DOWN)]
        )

    def test_up_release(self):
        self.assertEqual(
            hat_to_events((0, 1), (0, 0)), [(proto.DPAD_UP, proto.ACTION_UP)]
        )

    def test_down_press_and_release(self):
        self.assertEqual(
            hat_to_events((0, 0), (0, -1)), [(proto.DPAD_DOWN, proto.ACTION_DOWN)]
        )
        self.assertEqual(
            hat_to_events((0, -1), (0, 0)), [(proto.DPAD_DOWN, proto.ACTION_UP)]
        )

    def test_right_press_and_release(self):
        self.assertEqual(
            hat_to_events((0, 0), (1, 0)), [(proto.DPAD_RIGHT, proto.ACTION_DOWN)]
        )
        self.assertEqual(
            hat_to_events((1, 0), (0, 0)), [(proto.DPAD_RIGHT, proto.ACTION_UP)]
        )

    def test_left_press_and_release(self):
        self.assertEqual(
            hat_to_events((0, 0), (-1, 0)), [(proto.DPAD_LEFT, proto.ACTION_DOWN)]
        )
        self.assertEqual(
            hat_to_events((-1, 0), (0, 0)), [(proto.DPAD_LEFT, proto.ACTION_UP)]
        )

    def test_diagonal_press_emits_both_axes(self):
        events = hat_to_events((0, 0), (1, 1))
        self.assertEqual(set(events), {
            (proto.DPAD_RIGHT, proto.ACTION_DOWN),
            (proto.DPAD_UP, proto.ACTION_DOWN),
        })

    def test_diagonal_release_emits_both_axes(self):
        events = hat_to_events((1, 1), (0, 0))
        self.assertEqual(set(events), {
            (proto.DPAD_RIGHT, proto.ACTION_UP),
            (proto.DPAD_UP, proto.ACTION_UP),
        })

    def test_direct_flip_emits_release_then_press(self):
        events = hat_to_events((-1, 0), (1, 0))
        self.assertEqual(
            events, [(proto.DPAD_LEFT, proto.ACTION_UP), (proto.DPAD_RIGHT, proto.ACTION_DOWN)]
        )

    def test_y_axis_independent_of_x_axis(self):
        # 右を押したまま上も押す → x側は変化なし、y側だけイベント
        events = hat_to_events((1, 0), (1, 1))
        self.assertEqual(events, [(proto.DPAD_UP, proto.ACTION_DOWN)])


if __name__ == "__main__":
    unittest.main()
