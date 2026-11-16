# UAV flight supervisor (SITL-lite safety layer, documented).
#
# Enforces conservative limits (no PX4 on this stack; see feasibility
# report for the platform decision):
#   horizontal speed <= VMAX_XY (1.5 m/s)
#   climb/descent   <= VMAX_Z  (1.0 m/s)
#   horizontal accel<= AMAX    (2.0 m/s^2, via rate-limited commands)
# Modes: TRACK (follow cmd_vel toward waypoint), HOLD (zero + hover),
# STOP (zero wrench/velocity, used on gate fail / DEGRADED).
# Subscribes joint states for odometry-equivalent xyz (+spawn offset).
import math
import time

VMAX_XY = 1.5
VMAX_Z = 1.0
AMAX = 2.0
KP = 1.2


class Supervisor:
    def __init__(self, vmax_xy=None, vmax_z=None):
        # Optional per-case limits (characterization speed factor);
        # defaults preserve frozen behavior exactly.
        self.mode = "HOLD"
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_t = time.time()
        self.vmax_xy = float(vmax_xy) if vmax_xy else VMAX_XY
        self.vmax_z = float(vmax_z) if vmax_z else VMAX_Z

    def set_mode(self, mode):
        assert mode in ("TRACK", "HOLD", "STOP")
        self.mode = mode
        if mode != "TRACK":
            self.last_cmd = (0.0, 0.0, 0.0)

    def track(self, pos, target):
        """Velocity command toward target, limited. Returns (vx,vy,vz)."""
        if self.mode != "TRACK":
            return (0.0, 0.0, 0.0)
        dx, dy, dz = (target[0] - pos[0], target[1] - pos[1],
                      target[2] - pos[2])
        vx, vy, vz = KP * dx, KP * dy, KP * dz
        h = math.hypot(vx, vy)
        if h > self.vmax_xy:
            vx, vy = vx / h * self.vmax_xy, vy / h * self.vmax_xy
        vz = max(-self.vmax_z, min(self.vmax_z, vz))
        # accel limit via slew
        now = time.time()
        dt = max(1e-3, now - self.last_t)
        self.last_t = now
        px, py, pz = self.last_cmd
        mx = AMAX * dt
        vx = px + max(-mx, min(mx, vx - px))
        vy = py + max(-mx, min(mx, vy - py))
        mz = AMAX * dt
        vz = pz + max(-mz, min(mz, vz - pz))
        self.last_cmd = (vx, vy, vz)
        return self.last_cmd


def waypoint_done(pos, target, tol_xy=0.3, tol_z=0.2):
    return math.hypot(pos[0] - target[0], pos[1] - target[1]) < tol_xy \
        and abs(pos[2] - target[2]) < tol_z
