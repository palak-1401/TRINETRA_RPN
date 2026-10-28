"""2D physics and sensor model used by run_sim.py and trial_runner.py.

Units: centimetres, seconds, radians. Model assumptions (all approximate):
  * differential-drive robot, circular body
  * camera: 60 degree horizontal field of view, target seen as % of frame width
  * HC-SR04: three rays (-15, 0, +15 degrees), Gaussian noise
  * camera noise and random packet dropouts
"""
import math
import random

from robot_logic import Controller, Packet

ARENA = 300.0
ROBOT_R = 8.0
SENSOR_FWD = 8.0          # sensor/camera sits this far ahead of the centre
WHEEL_BASE = 12.0
V_MAX = 20.0              # cm/s at command 1.0
DT = 0.05                 # 20 Hz control loop
FOV = math.radians(60)
VISION_RANGE = 250.0
TARGET_R = 15.0           # target is 30 cm wide
US_MAX = 400.0
US_NOISE = 0.5
X_NOISE = 2.0
SIZE_NOISE = 1.0
DROPOUT = 0.02
TIMEOUT_S = 60.0


def _ray_circle(ox, oy, dx, dy, cx, cy, r):
    ocx, ocy = cx - ox, cy - oy
    t = ocx * dx + ocy * dy
    if t < 0:
        return None
    perp2 = ocx * ocx + ocy * ocy - t * t
    if perp2 > r * r:
        return None
    return max(0.0, t - math.sqrt(r * r - perp2))


def _ray_walls(ox, oy, dx, dy):
    best = US_MAX
    for d, o, lim in ((dx, ox, ARENA), (dy, oy, ARENA)):
        if abs(d) > 1e-9:
            best = min(best, ((0.0 if d < 0 else lim) - o) / d)
    return max(0.0, best)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class World:
    def __init__(self, seed=None, n_obstacles=2):
        self.rng = random.Random(seed)
        r = self.rng
        self.t = 0.0
        self.x = r.uniform(40, ARENA - 40)
        self.y = r.uniform(40, ARENA - 40)
        self.th = r.uniform(-math.pi, math.pi)
        while True:
            self.tx = r.uniform(40, ARENA - 40)
            self.ty = r.uniform(40, ARENA - 40)
            if math.hypot(self.tx - self.x, self.ty - self.y) > 100:
                break
        self.obstacles = []
        tries = 0
        while len(self.obstacles) < n_obstacles and tries < 500:
            tries += 1
            ox, oy, orad = r.uniform(30, ARENA - 30), r.uniform(30, ARENA - 30), r.uniform(8, 15)
            if (math.hypot(ox - self.x, oy - self.y) > ROBOT_R + orad + 40
                    and math.hypot(ox - self.tx, oy - self.ty) > TARGET_R + orad + 40):
                self.obstacles.append((ox, oy, orad))
        self.ctrl = Controller()
        self.path = [(self.x, self.y)]
        self.state = "SEARCH"

    # --- sensors ---
    def sensor_pos(self):
        return (self.x + SENSOR_FWD * math.cos(self.th),
                self.y + SENSOR_FWD * math.sin(self.th))

    def ultrasonic(self):
        sx, sy = self.sensor_pos()
        best = US_MAX
        for off in (-math.radians(15), 0.0, math.radians(15)):
            a = self.th + off
            dx, dy = math.cos(a), math.sin(a)
            best = min(best, _ray_walls(sx, sy, dx, dy))
            for ox, oy, orad in self.obstacles + [(self.tx, self.ty, TARGET_R)]:
                t = _ray_circle(sx, sy, dx, dy, ox, oy, orad)
                if t is not None:
                    best = min(best, t)
        return max(0.0, min(US_MAX, best + self.rng.gauss(0, US_NOISE)))

    def camera(self):
        sx, sy = self.sensor_pos()
        d = math.hypot(self.tx - sx, self.ty - sy)
        bearing = _wrap(math.atan2(self.ty - sy, self.tx - sx) - self.th)
        if abs(bearing) > FOV / 2 or d > VISION_RANGE or self.rng.random() < DROPOUT:
            return Packet(0, 0, 0)
        ux, uy = (self.tx - sx) / d, (self.ty - sy) / d
        for ox, oy, orad in self.obstacles:          # occlusion
            t = _ray_circle(sx, sy, ux, uy, ox, oy, orad)
            if t is not None and t < d:
                return Packet(0, 0, 0)
        x = -bearing / (FOV / 2) * 100 + self.rng.gauss(0, X_NOISE)
        size = 100 * (2 * TARGET_R) / (2 * d * math.tan(FOV / 2)) + self.rng.gauss(0, SIZE_NOISE)
        return Packet(1, int(round(max(-100, min(100, x)))), int(round(max(0, min(100, size)))))

    # --- dynamics ---
    def collided(self):
        if not (ROBOT_R <= self.x <= ARENA - ROBOT_R and ROBOT_R <= self.y <= ARENA - ROBOT_R):
            return "wall"
        for ox, oy, orad in self.obstacles:
            if math.hypot(self.x - ox, self.y - oy) < ROBOT_R + orad:
                return "obstacle"
        if math.hypot(self.x - self.tx, self.y - self.ty) < ROBOT_R + TARGET_R:
            return "target"
        return None

    def tick(self):
        """One control + physics step. Returns (state, dist_cm, packet)."""
        dist = self.ultrasonic()
        pkt = self.camera()
        self.state, left, right = self.ctrl.step(int(self.t * 1000), dist, pkt)
        vl, vr = left * V_MAX, right * V_MAX
        v, w = (vl + vr) / 2, (vr - vl) / WHEEL_BASE
        self.th = _wrap(self.th + w * DT)
        self.x += v * math.cos(self.th) * DT
        self.y += v * math.sin(self.th) * DT
        self.t += DT
        self.path.append((self.x, self.y))
        return self.state, dist, pkt


def run_episode(seed=None, n_obstacles=2, trace=None):
    """Run until STOP (success), collision, or timeout. Returns a result dict."""
    w = World(seed, n_obstacles)
    result = None
    while w.t < TIMEOUT_S and result is None:
        state, dist, pkt = w.tick()
        if trace is not None:
            trace(w, state, dist, pkt)
        hit = w.collided()
        if hit:
            result = ("collision_" + hit)
        elif state == "STOP":
            result = "success"
    if result is None:
        result = "timeout"
    final_d = math.hypot(w.tx - w.x, w.ty - w.y) - ROBOT_R - TARGET_R
    return {"seed": seed, "result": result, "time_s": round(w.t, 2),
            "final_gap_cm": round(final_d, 1), "world": w}
