"""Python model of the Arduino controller.

Mirrors the firmware decision logic so it can be tested without hardware.
All constants below are assumptions: keep them equal to the values in
firmware/trinetra_robot/trinetra_robot.ino.

Wheel command convention: (left, right) in -1.0 .. 1.0.
  left > 0, right < 0  -> rotate clockwise (turn right)
  left < 0, right > 0  -> rotate counter-clockwise (turn left)
"""
from collections import namedtuple

# ---- Tunable constants (match with your .ino) ----
OBSTACLE_CM = 15      # enter AVOID at or below this distance
CLEAR_CM = 25         # leave AVOID at or above this distance
ALIGN_TOL = 15        # |x_offset| allowed before moving forward
SLOW_SIZE = 50        # slow down at or above this target size
STOP_SIZE = 80        # stop at or above this target size
FAILSAFE_MS = 500     # stop motors if no packet for this long

SEARCH_TURN = 0.4
ALIGN_TURN = 0.35
AVOID_TURN = 0.5
FWD_FAST = 0.8
FWD_SLOW = 0.4

Packet = namedtuple("Packet", "detected x_offset size")


def parse_packet(line):
    """Parse 'V,<detected>,<x_offset>,<size>'. Return Packet or None if invalid."""
    parts = line.strip().split(",")
    if len(parts) != 4 or parts[0] != "V":
        return None
    try:
        detected, x_offset, size = int(parts[1]), int(parts[2]), int(parts[3])
    except ValueError:
        return None
    if detected not in (0, 1):
        return None
    return Packet(detected, max(-100, min(100, x_offset)), max(0, min(100, size)))


class Controller:
    """Decision logic: SEARCH, ALIGN, APPROACH, STOP, AVOID (+ FAILSAFE)."""

    def __init__(self):
        self.state = "SEARCH"
        self.pkt = Packet(0, 0, 0)
        self.last_pkt_ms = None
        self.avoiding = False

    def step(self, now_ms, dist_cm, pkt=None):
        """Advance one control tick. Returns (state, left, right)."""
        if pkt is not None:
            self.pkt = pkt
            self.last_pkt_ms = now_ms

        # Failsafe: no recent packet -> stop.
        if self.last_pkt_ms is None or now_ms - self.last_pkt_ms > FAILSAFE_MS:
            self.state = "FAILSAFE"
            return self.state, 0.0, 0.0

        # Obstacle safety has priority over vision (with hysteresis).
        if dist_cm <= OBSTACLE_CM:
            self.avoiding = True
        elif self.avoiding and dist_cm >= CLEAR_CM:
            self.avoiding = False
        if self.avoiding:
            self.state = "AVOID"
            return self.state, AVOID_TURN, -AVOID_TURN

        p = self.pkt
        if not p.detected:
            self.state = "SEARCH"
            return self.state, SEARCH_TURN, -SEARCH_TURN

        if abs(p.x_offset) > ALIGN_TOL:
            self.state = "ALIGN"
            if p.x_offset < 0:      # target on the left -> turn left
                return self.state, -ALIGN_TURN, ALIGN_TURN
            return self.state, ALIGN_TURN, -ALIGN_TURN

        if p.size >= STOP_SIZE:
            self.state = "STOP"
            return self.state, 0.0, 0.0

        self.state = "APPROACH"
        speed = FWD_SLOW if p.size >= SLOW_SIZE else FWD_FAST
        k = p.x_offset / 200.0      # small steering correction
        return self.state, speed * (1 + k), speed * (1 - k)
