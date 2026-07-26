import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import remote_protocol as proto
from input_mapping import apply_rumble_messages, hat_to_events, process_incoming_lines


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


class FakeJoystick:
    def __init__(self):
        self.rumble_calls = []
        self.stop_calls = 0

    def rumble(self, low, high, duration_ms):
        self.rumble_calls.append((low, high, duration_ms))

    def stop_rumble(self):
        self.stop_calls += 1


class TestProcessIncomingLines(unittest.TestCase):
    def test_single_complete_line(self):
        buf, messages = process_incoming_lines(b"", proto.encode_rumble(100))
        self.assertEqual(buf, b"")
        self.assertEqual(messages, [{"type": "rumble", "duration_ms": 100}])

    def test_partial_line_is_buffered(self):
        chunk = proto.encode_rumble(100)
        partial = chunk[:-1]  # 末尾の改行を欠いた不完全な行
        buf, messages = process_incoming_lines(b"", partial)
        self.assertEqual(buf, partial)
        self.assertEqual(messages, [])

    def test_partial_line_completes_on_next_chunk(self):
        chunk = proto.encode_rumble(100)
        buf, messages = process_incoming_lines(b"", chunk[:-1])
        buf, messages = process_incoming_lines(buf, chunk[-1:])
        self.assertEqual(buf, b"")
        self.assertEqual(messages, [{"type": "rumble", "duration_ms": 100}])

    def test_multiple_lines_in_one_chunk(self):
        chunk = proto.encode_rumble(50) + proto.encode_heartbeat() + proto.encode_rumble(150)
        buf, messages = process_incoming_lines(b"", chunk)
        self.assertEqual(buf, b"")
        self.assertEqual(
            messages,
            [
                {"type": "rumble", "duration_ms": 50},
                {"type": "heartbeat"},
                {"type": "rumble", "duration_ms": 150},
            ],
        )

    def test_garbage_line_is_skipped(self):
        chunk = b"not json\n" + proto.encode_rumble(100)
        buf, messages = process_incoming_lines(b"", chunk)
        self.assertEqual(messages, [{"type": "rumble", "duration_ms": 100}])


class TestApplyRumbleMessages(unittest.TestCase):
    def test_no_rumble_messages_returns_none(self):
        joystick = FakeJoystick()
        result = apply_rumble_messages(joystick, [{"type": "heartbeat"}], now=10.0)
        self.assertIsNone(result)
        self.assertEqual(joystick.rumble_calls, [])

    def test_rumble_message_triggers_rumble_and_returns_stop_time(self):
        joystick = FakeJoystick()
        result = apply_rumble_messages(
            joystick, [{"type": "rumble", "duration_ms": 100}], now=10.0
        )
        self.assertEqual(joystick.rumble_calls, [(1.0, 1.0, 100)])
        self.assertAlmostEqual(result, 10.1)

    def test_multiple_rumble_messages_uses_last(self):
        joystick = FakeJoystick()
        messages = [
            {"type": "rumble", "duration_ms": 50},
            {"type": "rumble", "duration_ms": 200},
        ]
        result = apply_rumble_messages(joystick, messages, now=5.0)
        self.assertEqual(len(joystick.rumble_calls), 2)
        self.assertAlmostEqual(result, 5.2)

    def test_ignores_non_rumble_messages_mixed_in(self):
        joystick = FakeJoystick()
        messages = [
            {"type": "button", "name": "dpad_up", "action": "down"},
            {"type": "rumble", "duration_ms": 100},
        ]
        result = apply_rumble_messages(joystick, messages, now=0.0)
        self.assertEqual(joystick.rumble_calls, [(1.0, 1.0, 100)])
        self.assertAlmostEqual(result, 0.1)


if __name__ == "__main__":
    unittest.main()
