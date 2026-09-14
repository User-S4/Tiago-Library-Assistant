#!/usr/bin/env python3
"""
Reactive navigation to the target shelf column, ERC 2026 Phase 1.

Deliberately not Nav2. Nav2 needs a map, localisation and costmap tuning; this
task is "turn until you can see the right number, then drive at it until you are
close", which a reactive controller does in a fraction of the setup time.

State machine:
  SEARCH    rotate on the spot until the column detector reports the target
  CENTRE    rotate until the target number sits in the middle of the frame
  APPROACH  drive straight at the shelf until the depth camera says stop
  SETTLE    tilt the head down so the books are in view, then hold still
  ARRIVED   stop, publish zero velocity, announce
  FAILED    give up after search_timeout_sec so a trial cannot hang

Ranging uses the head depth camera, not the base laser. The laser sits low on the
base and repeatedly reported obstacles at around a metre while the shelf was four
metres away, stopping the robot in open floor; whether it was catching the bin
table, the robot's own arms or the shelf underside, it disagreed with reality
every time. The depth camera looks where the robot is going and has matched the
true distance on every check, so it is what the approach trusts. The laser is
kept only as a close-range safety stop, where a spurious short reading costs
nothing but caution.

The approach commits. The number plaque sits above the shelf unit and the camera
has a 56 degree vertical field of view, so the plaque climbs out of frame as the
robot closes in. Losing sight of a column already identified and aimed at is
expected, not a failure.

Subscribes:
  /shelf_column_detector/target_offset                        (Float32)
  /head_front_camera/head_front_camera/depth/image_rect_raw   (Image)
  /scan_front_raw                                             (LaserScan)
Publishes:
  /cmd_vel                             (Twist)
  /head_controller/joint_trajectory    (JointTrajectory)
  ~/state                              (String)
"""

import math

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

SEARCH = "SEARCH"
CENTRE = "CENTRE"
APPROACH = "APPROACH"
SETTLE = "SETTLE"
ARRIVED = "ARRIVED"
FAILED = "FAILED"


class ColumnNavigator(Node):
    def __init__(self):
        super().__init__("column_navigator")

        self.declare_parameter("offset_topic", "/shelf_column_detector/target_offset")
        self.declare_parameter(
            "depth_topic", "/head_front_camera/head_front_camera/depth/image_rect_raw"
        )
        self.declare_parameter("scan_topic", "/scan_front_raw")

        # --- speeds ---------------------------------------------------------
        # Modest on purpose. Collisions cost half a point each. These are
        # simulator speeds; they look slow at a low real-time factor but are
        # correct on a machine running at full speed.
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("centre_angular_speed", 0.35)
        self.declare_parameter("approach_linear_speed", 0.25)

        # --- ranging ----------------------------------------------------------
        # Central patch of the depth image, as a fraction of width and height.
        # Small enough to be "what is straight ahead", large enough to average
        # over shelf structure rather than a single noisy pixel.
        self.declare_parameter("depth_patch_width_frac", 0.25)
        self.declare_parameter("depth_patch_height_frac", 0.40)
        # Laser is advisory only: an emergency stop, not the primary range.
        self.declare_parameter("laser_emergency_stop_m", 0.45)

        # --- tolerances -----------------------------------------------------
        self.declare_parameter("centre_tolerance", 0.06)
        self.declare_parameter("stop_distance_m", 2.3)
        self.declare_parameter("slow_down_distance_m", 2.2)
        self.declare_parameter("min_linear_speed", 0.07)
        self.declare_parameter("offset_timeout_sec", 3.0)
        self.declare_parameter("settle_hold_sec", 3.0)
        self.declare_parameter("search_timeout_sec", 600.0)
        self.declare_parameter("startup_check_sec", 10.0)

        # --- head -------------------------------------------------------------
        # Tilt up while approaching to keep the plaque in shot for longer, then
        # down on arrival so the shelf rows fill the frame.
        self.declare_parameter("head_tilt_approach_rad", 0.25)
        self.declare_parameter("head_tilt_shelf_rad", 0.0)
        # The controller may not be ready the instant this node starts, and a
        # trajectory sent too early is silently dropped.
        self.declare_parameter("head_command_delay_sec", 3.0)
        # Resend periodically: cheap, and covers a dropped command.
        self.declare_parameter("head_resend_period_sec", 2.0)

        self.declare_parameter("control_period_sec", 0.1)
        self.declare_parameter("log_period_sec", 3.0)

        self.state = SEARCH
        self.offset = None
        self.last_offset_time = None
        self.depth_distance = None
        self.laser_min = None
        self.depth_frames = 0
        self.startup_checked = False
        self.head_target = None
        self.last_head_command = None
        self.bridge = CvBridge()

        self.state_entered = self.get_clock().now()
        self.started = self.get_clock().now()
        self.last_log = self.get_clock().now()
        self.announced = set()

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub = self.create_publisher(String, "~/state", 10)
        self.head_pub = self.create_publisher(
            JointTrajectory, "/head_controller/joint_trajectory", 10
        )

        self.create_subscription(
            Float32, self.get_parameter("offset_topic").value, self.on_offset, 10
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.on_depth, 1
        )
        self.create_subscription(
            LaserScan, self.get_parameter("scan_topic").value, self.on_scan, 1
        )

        self.timer = self.create_timer(
            self.get_parameter("control_period_sec").value, self.control_step
        )

        self.get_logger().info(
            "Column navigator ready. Ranging on the depth camera; "
            "laser used only as a close-range safety stop."
        )

    # -------------------------------------------------------------- inputs

    def on_offset(self, msg):
        self.offset = float(msg.data)
        self.last_offset_time = self.get_clock().now()

    def on_depth(self, msg):
        """Distance to whatever fills the middle of the view.

        A low percentile rather than the median: the patch covers shelf openings
        that see straight through to the back panel, and we want the distance to
        the nearest real structure, not an average of near and far.
        """
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception:                              # noqa: BLE001
            return
        self.depth_frames += 1

        h, w = depth.shape[:2]
        pw = self.get_parameter("depth_patch_width_frac").value
        ph = self.get_parameter("depth_patch_height_frac").value
        x0, x1 = int(w * (0.5 - pw / 2)), int(w * (0.5 + pw / 2))
        y0, y1 = int(h * (0.5 - ph / 2)), int(h * (0.5 + ph / 2))

        patch = np.asarray(depth[y0:y1, x0:x1], dtype=np.float32)
        patch = patch[np.isfinite(patch) & (patch > 0.05)]
        if patch.size == 0:
            self.depth_distance = None
            return
        value = float(np.percentile(patch, 20))
        # Gazebo publishes float32 metres, but guard against a mm encoding.
        self.depth_distance = value / 1000.0 if value > 100.0 else value

    def on_scan(self, msg):
        """Nearest return anywhere in the forward half of the sweep.

        Advisory only. Used to refuse to keep driving into something very close,
        not to decide when the shelf has been reached.
        """
        if not msg.ranges:
            return
        forward = (msg.angle_min + msg.angle_max) / 2.0
        half = math.radians(60.0)

        closest = None
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle - forward) > half:
                continue
            if not math.isfinite(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            if closest is None or distance < closest:
                closest = distance
        self.laser_min = closest

    # --------------------------------------------------------------- helpers

    def target_visible(self):
        if self.offset is None or self.last_offset_time is None:
            return False
        age = (self.get_clock().now() - self.last_offset_time).nanoseconds / 1e9
        return age <= self.get_parameter("offset_timeout_sec").value

    def seconds_running(self):
        return (self.get_clock().now() - self.started).nanoseconds / 1e9

    def seconds_in_state(self):
        return (self.get_clock().now() - self.state_entered).nanoseconds / 1e9

    def change_state(self, new_state, reason=""):
        if new_state == self.state:
            return
        self.get_logger().info(
            f"{self.state} -> {new_state}" + (f" ({reason})" if reason else "")
        )
        self.state = new_state
        self.state_entered = self.get_clock().now()

    def publish(self, linear_x=0.0, angular_z=0.0):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_pub.publish(twist)

    def stop(self):
        """Zero velocity. The base holds its last command, so this is not optional."""
        self.publish(0.0, 0.0)

    def request_head_tilt(self, tilt_rad):
        self.head_target = float(tilt_rad)
        self.last_head_command = None       # force an immediate send

    def service_head(self):
        """Send and periodically resend the requested tilt.

        Resending covers the case where the controller was not yet accepting
        trajectories when the first command went out.
        """
        if self.head_target is None:
            return
        if self.seconds_running() < self.get_parameter("head_command_delay_sec").value:
            return
        period = self.get_parameter("head_resend_period_sec").value
        if self.last_head_command is not None:
            age = (self.get_clock().now() - self.last_head_command).nanoseconds / 1e9
            if age < period:
                return
        self.last_head_command = self.get_clock().now()

        message = JointTrajectory()
        message.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, self.head_target]
        point.time_from_start = Duration(sec=1)
        message.points = [point]
        self.head_pub.publish(message)

    def check_startup(self):
        """Fail loudly and early if the simulator is not actually running."""
        if self.startup_checked:
            return
        if self.seconds_running() < self.get_parameter("startup_check_sec").value:
            return
        self.startup_checked = True
        if self.depth_frames == 0:
            self.get_logger().error(
                "No depth images after startup. Is the simulation running? "
                "Launch erc_bringup simulation.launch.py first."
            )
            self.change_state(FAILED, "no sensor data")

    def periodic_log(self):
        period = self.get_parameter("log_period_sec").value
        if (self.get_clock().now() - self.last_log).nanoseconds / 1e9 < period:
            return
        self.last_log = self.get_clock().now()
        offset = f"{self.offset:+.2f}" if self.offset is not None else "none"
        depth = (f"{self.depth_distance:.2f}m"
                 if self.depth_distance is not None else "none")
        laser = f"{self.laser_min:.2f}m" if self.laser_min is not None else "none"
        self.get_logger().info(
            f"[{self.state}] offset {offset} (visible: {self.target_visible()}) "
            f"depth {depth} laser {laser} t+{self.seconds_in_state():.0f}s"
        )

    # ---------------------------------------------------------- control loop

    def control_step(self):
        self.state_pub.publish(String(data=self.state))
        self.periodic_log()
        self.check_startup()

        if self.head_target is None:
            self.request_head_tilt(
                self.get_parameter("head_tilt_approach_rad").value
            )
        self.service_head()

        if (self.state not in (ARRIVED, FAILED)
                and self.seconds_running()
                > self.get_parameter("search_timeout_sec").value):
            self.change_state(FAILED, f"timed out after {self.seconds_running():.0f}s")

        handler = {
            SEARCH: self.do_search,
            CENTRE: self.do_centre,
            APPROACH: self.do_approach,
            SETTLE: self.do_settle,
            ARRIVED: self.do_arrived,
            FAILED: self.do_failed,
        }[self.state]
        handler()

    def do_search(self):
        """Rotate on the spot until the target column comes into view."""
        if self.target_visible():
            self.stop()
            self.change_state(CENTRE, "target acquired")
            return
        self.publish(0.0, self.get_parameter("search_angular_speed").value)

    def do_centre(self):
        if not self.target_visible():
            self.change_state(SEARCH, "lost target")
            return

        if abs(self.offset) <= self.get_parameter("centre_tolerance").value:
            self.stop()
            self.change_state(APPROACH, f"centred at {self.offset:+.2f}")
            return

        # Positive offset means the target is right of centre, so turn right,
        # which is negative angular velocity under REP-103.
        speed = self.get_parameter("centre_angular_speed").value
        if abs(self.offset) < 0.15:
            speed *= 0.4
        self.publish(0.0, -math.copysign(speed, self.offset))

    def do_approach(self):
        """Drive straight until the depth camera says the shelf is close."""
        stop_at = self.get_parameter("stop_distance_m").value

        if self.depth_distance is not None and self.depth_distance <= stop_at:
            self.stop()
            self.change_state(SETTLE, f"reached {self.depth_distance:.2f} m")
            return

        # Safety net only. Something very close that the camera has not seen.
        emergency = self.get_parameter("laser_emergency_stop_m").value
        if self.laser_min is not None and self.laser_min <= emergency:
            self.stop()
            self.change_state(
                SETTLE, f"laser emergency stop at {self.laser_min:.2f} m"
            )
            return

        speed = self.get_parameter("approach_linear_speed").value
        slow_from = self.get_parameter("slow_down_distance_m").value
        if self.depth_distance is not None and self.depth_distance < slow_from:
            # Ease off as the shelf gets close so the stop is gentle rather than
            # a lurch that could nudge the shelving.
            span = max(0.01, slow_from - stop_at)
            scale = (self.depth_distance - stop_at) / span
            speed = max(self.get_parameter("min_linear_speed").value, speed * scale)

        # Gentle correction only while the plaque is still visible. Small gain so
        # the path stays near straight rather than arcing round the shelf.
        angular = -0.25 * self.offset if self.target_visible() else 0.0
        self.publish(speed, angular)

    def do_settle(self):
        """Stop, look down at the shelf, and let the view stabilise."""
        self.stop()
        if SETTLE not in self.announced:
            self.announced.add(SETTLE)
            self.request_head_tilt(self.get_parameter("head_tilt_shelf_rad").value)

        if self.seconds_in_state() >= self.get_parameter("settle_hold_sec").value:
            self.change_state(ARRIVED, "settled at shelf")

    def do_arrived(self):
        self.stop()
        if ARRIVED not in self.announced:
            self.announced.add(ARRIVED)
            depth = (f"{self.depth_distance:.2f} m"
                     if self.depth_distance is not None else "unknown")
            self.get_logger().info(
                f"Arrived at target column, {depth} from shelf by depth camera, "
                f"{self.seconds_running():.1f} s wall clock."
            )

    def do_failed(self):
        self.stop()
        if FAILED not in self.announced:
            self.announced.add(FAILED)
            self.get_logger().warn("Could not reach the target column.")

    def destroy_node(self):
        # Leaving the base with a non-zero command would have it drift into a
        # collision penalty after the node exits.
        self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ColumnNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()