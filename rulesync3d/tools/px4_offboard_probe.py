# Minimal PX4 offboard probe (no px4_ros_com dependency).
# Subscribes VehicleOdometry, publishes OffboardControlMode +
# TrajectorySetpoint, arms via VehicleCommand, runs takeoff -> goto ->
# HOLD (failsafe/land command) sequence. Prints phase transitions.
# Serves final-integration §8 (telemetry/command/mission/HOLD proof).
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import HistoryPolicy


def px4_qos():
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.BEST_EFFORT
    q.durability = DurabilityPolicy.VOLATILE
    q.history = HistoryPolicy.KEEP_LAST
    return q


class Probe(Node):
    def __init__(self, target):
        super().__init__("px4_offboard_probe")
        from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                                  VehicleCommand, VehicleOdometry,
                                  VehicleStatus)
        self._M = (OffboardControlMode, TrajectorySetpoint, VehicleCommand)
        self.odom = None
        self.status = None
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry",
                                 self._on_odom, px4_qos())
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status",
                                 self._on_status, px4_qos())
        self.pub_mode = self.create_publisher(OffboardControlMode,
                                              "/fmu/in/offboard_control_mode",
                                              px4_qos())
        self.pub_sp = self.create_publisher(TrajectorySetpoint,
                                            "/fmu/in/trajectory_setpoint",
                                            px4_qos())
        self.pub_cmd = self.create_publisher(VehicleCommand,
                                             "/fmu/in/vehicle_command",
                                             px4_qos())
        self.target = tuple(float(v) for v in target)

    def _on_odom(self, msg):
        self.odom = (float(msg.position[0]), float(msg.position[1]),
                     float(msg.position[2]))

    def _on_status(self, msg):
        self.status = (int(msg.arming_state), int(msg.nav_state))

    def send_cmd(self, command, p1=0.0, p2=0.0):
        from px4_msgs.msg import VehicleCommand as VC
        m = VC()
        m.command = int(command)
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system, m.target_component = 1, 1
        m.source_system, m.source_component = 1, 1
        m.from_external = True
        self.pub_cmd.publish(m)

    def spin_wait(self, dur, rate=20.0):
        t0 = time.time()
        while time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(1.0 / rate)


def main():
    import sys
    tgt = [float(v) for v in (sys.argv[1:] + [0, 0, -5])[:3]]
    rclpy.init()
    n = Probe(tgt)
    from px4_msgs.msg import OffboardControlMode as OCM
    from px4_msgs.msg import TrajectorySetpoint as TS
    # stream setpoints before arming (offboard requirement)
    for _ in range(50):
        m = OCM()
        m.position, m.velocity, m.acceleration = True, False, False
        m.attitude, m.body_rate = False, False
        n.pub_mode.publish(m)
        s = TS()
        s.position = [0.0, 0.0, -5.0]
        n.pub_sp.publish(s)
        n.spin_wait(0.05)
    print(f"odom={n.odom} status={n.status}", flush=True)
    # arm (400) then offboard mode (1763941? use VEHICLE_CMD_DO_SET_MODE 176
    # with param1=1 custom + param2=6 offboard)
    n.send_cmd(400, p1=1.0)
    n.spin_wait(2.0)
    print(f"after arm: odom={n.odom} status={n.status}", flush=True)
    n.send_cmd(176, p1=1.0, p2=6.0)
    n.spin_wait(2.0)
    print(f"after offboard-mode-cmd: odom={n.odom} status={n.status}",
          flush=True)
    t0 = time.time()
    while time.time() - t0 < 25.0:
        m = OCM()
        m.position = True
        n.pub_mode.publish(m)
        s = TS()
        s.position = list(tgt)
        n.pub_sp.publish(s)
        n.spin_wait(0.05)
    print(f"after goto: odom={n.odom} status={n.status}", flush=True)
    # HOLD via loiter-return? use Navigator HOLD: VehicleCommand DO_HOLD
    # (208?) — PX4 holds on RC loss; explicit: switch to HOLD mode
    # param2=4? (4=AUTO_LOITER). Use AUTO_LOITER as safe hold.
    n.send_cmd(176, p1=1.0, p2=4.0)
    n.spin_wait(5.0)
    print(f"after hold-cmd: odom={n.odom} status={n.status}", flush=True)
    n.destroy_node()
    rclpy.shutdown()


main()
