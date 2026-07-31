import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import remote_protocol as proto


class TestEncodeDecodeButtonEvent(unittest.TestCase):
    def test_roundtrip(self):
        raw = proto.encode_button_event(proto.DPAD_UP, proto.ACTION_DOWN)
        self.assertTrue(raw.endswith(b"\n"))
        decoded = proto.decode_line(raw.decode("utf-8"))
        self.assertEqual(decoded, {"type": "button", "name": "dpad_up", "action": "down"})

    def test_unknown_button_raises(self):
        with self.assertRaises(ValueError):
            proto.encode_button_event("start", proto.ACTION_DOWN)

    def test_unknown_action_raises(self):
        with self.assertRaises(ValueError):
            proto.encode_button_event(proto.DPAD_UP, "held")


class TestDecodeLine(unittest.TestCase):
    def test_heartbeat(self):
        raw = proto.encode_heartbeat()
        decoded = proto.decode_line(raw.decode("utf-8"))
        self.assertEqual(decoded, {"type": "heartbeat"})

    def test_garbage_returns_none(self):
        self.assertIsNone(proto.decode_line("not json"))
        self.assertIsNone(proto.decode_line("{"))
        self.assertIsNone(proto.decode_line(""))
        self.assertIsNone(proto.decode_line("   "))

    def test_non_dict_json_returns_none(self):
        self.assertIsNone(proto.decode_line("[1,2,3]"))
        self.assertIsNone(proto.decode_line("42"))

    def test_unknown_type_returns_none(self):
        self.assertIsNone(proto.decode_line('{"type": "ping"}'))

    def test_button_missing_fields_returns_none(self):
        self.assertIsNone(proto.decode_line('{"type": "button", "name": "dpad_up"}'))
        self.assertIsNone(proto.decode_line('{"type": "button", "action": "down"}'))

    def test_button_unknown_name_or_action_returns_none(self):
        self.assertIsNone(
            proto.decode_line('{"type": "button", "name": "start", "action": "down"}')
        )
        self.assertIsNone(
            proto.decode_line('{"type": "button", "name": "dpad_up", "action": "held"}')
        )


class TestEncodeDecodeRumble(unittest.TestCase):
    def test_roundtrip(self):
        raw = proto.encode_rumble(100)
        self.assertTrue(raw.endswith(b"\n"))
        decoded = proto.decode_line(raw.decode("utf-8"))
        self.assertEqual(decoded, {"type": "rumble", "duration_ms": 100})

    def test_missing_duration_returns_none(self):
        self.assertIsNone(proto.decode_line('{"type": "rumble"}'))

    def test_non_numeric_duration_returns_none(self):
        self.assertIsNone(proto.decode_line('{"type": "rumble", "duration_ms": "soon"}'))


if __name__ == "__main__":
    unittest.main()
