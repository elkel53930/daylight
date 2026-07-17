#!/usr/bin/env python3
"""
melodies.py — Every buzzer melody Default UI plays, gathered in one place so
sounds can be tuned without hunting through default_ui.py.

Melody syntax (see ui_client.play() / ui_server's BuzzerManager):
    c d e f g a b   = low notes (524-988 Hz)
    C D E F G A B   = high notes (1048-1976 Hz)
    anything else   = a rest
Each character plays for ~150ms.
"""

# Short click feedback for button presses: a low tone for "move" (L) and a
# higher tone for "decide/execute" (R), so the two are distinguishable by ear.
BUTTON_CLICK_MELODY = {"left": "c", "right": "e"}

# Notification chime played when handing the UI off to a launched application.
APP_LAUNCH_MELODY = "ce"

# Warning sound repeated at intervals while the battery is low.
LOW_BATTERY_MELODY = "cc"
