"""Four virtual controllers play RflyGame split screen, end to end.

Start the game first:
    target/release/rflygame --vpad --auto split=0 --screenshot out.png --after 30
(this opens the real split-screen join screen; nobody is pre-joined).

The script then, using only controller input:
  1. each pad presses A to join (P1..P4);
  2. pad 3 presses X twice: Angle -> Acro -> Beginner (chase camera);
  3. pad 0 drives the menu: D-pad right x4 to pick "Race: ..." then down and A on Start;
  4. after GO (races arm everyone automatically) they fly: climb, then forward with a little roll.
The game prints each pilot's slot, mode, armed state and altitude at --after seconds.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from rflyctl.vpad import VirtualPad, hold_all  # noqa: E402

pads = [VirtualPad(i) for i in range(4)]
hold_all(pads, 1.0)  # connect
for p in pads:  # join in order: P1..P4
    p.press("A")
    hold_all(pads, 0.3)
for _ in range(2):  # P4: Angle -> Acro -> Beginner
    pads[3].press("X")
    hold_all(pads, 0.2)
p1 = pads[0]
for _ in range(int(os.environ.get("VPAD_ACTIVITY", "4"))):
    p1.press("RIGHT")
    hold_all(pads, 0.1)
p1.press("DOWN")
hold_all(pads, 0.2)
p1.press("A")  # Start
hold_all(pads, 4.5)  # race countdown; races arm everyone (except Acro) at GO
for i, p in enumerate(pads):
    p.sticks(throttle=0.6)
hold_all(pads, 2.0)
for i, p in enumerate(pads):
    p.sticks(throttle=0.15, pitch=0.35, roll=(i - 1.5) * 0.1)
hold_all(pads, float(os.environ.get("VPAD_FLY", "20")))
print("vpad test: done sending")
