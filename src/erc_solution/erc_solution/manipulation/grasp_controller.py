#!/usr/bin/env python3
import math
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque

import cv2
import numpy as np
import rclpy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


URDF = "/opt/erc_ws/src/erc_description/urdf/tiago_pro.urdf"

BASE = "base_link"
TIP = "gripper_left_grasping_link"

LEFT_ARM = [
    f"arm_left_{i}_joint"
    for i in range(1, 8)
]

RIGHT_ARM = [
    f"arm_right_{i}_joint"
    for i in range(1, 8)
]


# ------------------------------------------------------------
# POSES PROVEN DURING THE SUCCESSFUL PHYSICAL TRIAL
# ------------------------------------------------------------

LEFT_PREGRASP_REF = [
    +0.900514,
    +0.525123,
    -0.691844,
    -1.916996,
    -3.020830,
    -1.487775,
    +0.076853,
]

LEFT_CARRY = [
    +0.822118,
    +1.134464,
    -0.696242,
    -2.141305,
    -2.612429,
    -1.884956,
    +0.506579,
]

RIGHT_TUCK = [
    -0.218457,
    +0.311219,
    +1.502029,
    -2.022216,
    +2.033837,
    +2.575969,
    +1.066115,
]


class GraspController(Node):

    def __init__(self):

        super().__init__("grasp_controller")

        self.bridge = CvBridge()

        self.nav_state = None

        self.book_samples = deque(
            maxlen=30
        )

        # Latest continuously observed target-book position.
        # Used during the final shelf approach.
        self.latest_book_xyz = None
        self.latest_book_time = 0.0

        self.collect_books = False

        # After navigator ARRIVED, the head is still physically
        # finishing its shelf-view trajectory. Do not lock the
        # target until that motion has settled.
        self.book_collect_after = 0.0
        self.book_settle_delay_sec = 4.0

        self.odom = None
        self.start_xy = None

        self.joints = {}

        self.latest_depth = None
        self.depth_scale = 1.0

        self.bin_candidate = None
        self.bin_candidate_time = 0.0
        self.bin_confirm = 0

        self.worker_started = False

        # Startup arm-safety handshake.
        self.startup_tuck_started = False
        self.startup_arms_safe = False
        self.done = False

        self.book_contact_time = 0.0
        self.right_book_time = 0.0
        self.left_book_time = 0.0

        self.book_bin_time = 0.0

        self.unsafe_contact = None


        # --------------------------------------------------------
        # Publishers
        # --------------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # IMPORTANT:
        # public trajectory topic,
        # NOT the old raw action.
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            "/gripper_left_controller/joint_trajectory",
            10
        )

        self.head_pub = self.create_publisher(
            JointTrajectory,
            "/head_controller/joint_trajectory",
            10
        )


        # --------------------------------------------------------
        # Actions
        # --------------------------------------------------------

        self.left_arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_left_controller/follow_joint_trajectory"
        )

        self.right_arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_right_controller/follow_joint_trajectory"
        )

        self.torso_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/torso_controller/follow_joint_trajectory"
        )


        # --------------------------------------------------------
        # Subscribers
        # --------------------------------------------------------

        self.create_subscription(
            String,
            "/column_navigator/state",
            self.on_nav_state,
            10
        )

        self.create_subscription(
            PointStamped,
            "/erc/target_book_point",
            self.on_book_point,
            20
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.on_odom,
            20
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.on_joint_state,
            20
        )

        self.create_subscription(
            Contacts,
            "/contacts",
            self.on_contacts,
            50
        )

        self.create_subscription(
            Contacts,
            "/bin_contacts",
            self.on_contacts,
            50
        )

        self.create_subscription(
            Image,
            "/head_front_camera/head_front_camera/color/image_raw",
            self.on_image,
            1
        )

        self.create_subscription(
            Image,
            "/head_front_camera/head_front_camera/depth/image_rect_raw",
            self.on_depth,
            1
        )


        self.load_urdf()

        self.startup_safe_pub = self.create_publisher(
            Bool,
            "/erc/startup_arms_safe",
            10
        )

        # This timer starts arm folding immediately.
        self.create_timer(
            0.5,
            self.startup_supervisor
        )

        # Existing task supervisor still waits for ARRIVED.
        self.create_timer(
            0.5,
            self.supervisor
        )

        self._final_nav_state_sub = self.create_subscription(
            String,
            "/column_navigator/state",
            self._final_nav_state_callback,
            10,
        )

        self.get_logger().info(
            "AUTONOMOUS grasp controller ready. "
            "Waiting for column_navigator ARRIVED."
        )


    # ==========================================================
    # CALLBACKS
    # ==========================================================

    def startup_supervisor(self):
        """
        Fold both arms before allowing the navigator to move.
        """

        if self.startup_arms_safe:

            # Republish continuously so navigator cannot miss
            # a one-shot message during startup.
            self.startup_safe_pub.publish(
                Bool(data=True)
            )

            return


        if (
            self.startup_tuck_started
            or self.done
        ):
            return


        self.startup_tuck_started = True

        threading.Thread(
            target=self.startup_fold_arms,
            daemon=True
        ).start()


    def startup_fold_arms(self):

        self.phase(
            "STARTUP FOLD BOTH ARMS"
        )


        self.get_logger().info(
            "BASE LOCKED. Folding RIGHT arm."
        )


        # STARTUP_CONTROLLER_WAIT_INSTALLED
        # ROS controllers can take several wall-clock seconds
        # to become ready after solution.launch starts.
        # Keep the base locked while they initialize.
        self.get_logger().info(
            "Waiting for arm controllers to become ready."
        )

        self.stop_base()

        time.sleep(6.0)

        if not self.command_right_arm(
            RIGHT_TUCK
        ):

            return self.fail(
                "Startup right-arm tuck failed"
            )


        self.get_logger().info(
            "RIGHT ARM FOLDED. Folding LEFT arm."
        )


        if not self.command_left_arm(
            LEFT_CARRY
        ):

            return self.fail(
                "Startup left-arm tuck failed"
            )


        self.startup_arms_safe = True


        self.startup_safe_pub.publish(
            Bool(data=True)
        )


        self.get_logger().info(
            "BOTH ARMS FOLDED AND SAFE. "
            "Navigator released."
        )


    def on_nav_state(self, msg):

        old = self.nav_state

        self.nav_state = msg.data

        if (
            self.nav_state == "ARRIVED"
            and old != "ARRIVED"
        ):

            self.book_samples.clear()

            # Do not collect immediately. The navigator has just
            # commanded the head into its shelf-view pose and the
            # camera geometry is still changing for a few seconds.
            self.collect_books = True

            self.book_collect_after = (
                time.time()
                + self.book_settle_delay_sec
            )

            self.get_logger().info(
                "ARRIVED at shelf. "
                f"Waiting {self.book_settle_delay_sec:.1f}s "
                "for head/camera to settle before locking book."
            )


    def on_book_point(self, msg):

        p = msg.point

        xyz = (
            float(p.x),
            float(p.y),
            float(p.z),
        )

        # Always retain the latest visual target.
        self.latest_book_xyz = xyz
        self.latest_book_time = time.time()

        # Initial sample collection used to lock the
        # target before manipulation begins.
        if not self.collect_books:
            return

        # Head/camera settle window. Keep updating
        # latest_book_xyz for diagnostics, but do NOT allow
        # these moving-camera samples into the lock buffer.
        if time.time() < self.book_collect_after:
            self.book_samples.clear()
            return

        if (
            0.35 < p.x < 4.0
            and -0.8 < p.y < 0.8
            and 0.20 < p.z < 1.6
        ):

            self.book_samples.append(
                (
                    float(p.x),
                    float(p.y),
                    float(p.z),
                    time.time()
                )
            )


    def on_odom(self, msg):

        self.odom = msg

        if self.start_xy is None:

            p = msg.pose.pose.position

            self.start_xy = (
                float(p.x),
                float(p.y)
            )


    def on_joint_state(self, msg):

        self.joints = dict(
            zip(
                msg.name,
                msg.position
            )
        )


    def on_depth(self, msg):

        try:

            self.latest_depth = np.asarray(
                self.bridge.imgmsg_to_cv2(
                    msg,
                    desired_encoding="passthrough"
                )
            )

            encoding = str(
                msg.encoding
            ).upper()

            self.depth_scale = (
                0.001
                if (
                    "16U" in encoding
                    or "MONO16" in encoding
                )
                else 1.0
            )

        except Exception:
            pass


    def on_image(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

        except Exception:
            return


        candidate = self.find_red_bin(
            frame
        )


        if candidate is None:

            self.bin_confirm = 0

        else:

            self.bin_candidate = candidate

            self.bin_candidate_time = (
                time.time()
            )

            self.bin_confirm = min(
                20,
                self.bin_confirm + 1
            )


    def on_contacts(self, msg):

        now = time.time()

        for contact in getattr(
            msg,
            "contacts",
            []
        ):

            a = contact.collision1.name
            b = contact.collision2.name

            s = (
                a + " " + b
            ).lower()


            # ------------------------------------------
            # Book / gripper
            # ------------------------------------------

            if (
                "gripper_left" in s
                and "book_" in s
            ):

                self.book_contact_time = now

                if "fingertip_right" in s:

                    self.right_book_time = now

                if "fingertip_left" in s:

                    self.left_book_time = now


            # ------------------------------------------
            # Placement confirmation
            # ------------------------------------------

            if (
                "book_" in s
                and "collection_bin" in s
            ):

                self.book_bin_time = now


            # ------------------------------------------
            # Robot collision
            # ------------------------------------------

            if (
                "tiago_pro" in s
                and (
                    "erc_table" in s
                    or "collection_bin" in s
                    or "shelf" in s
                    or "wall_" in s
                )
            ):

                # Log the FIRST unsafe collision so we know
                # exactly what touched what.
                if self.unsafe_contact is None:
                    self.get_logger().warning(
                        f"UNSAFE CONTACT: {a} <-> {b}"
                    )

                self.unsafe_contact = (
                    a,
                    b,
                    now
                )


    # ==========================================================
    # START GATING
    # ==========================================================

    def _final_nav_state_callback(self, msg):
        # Both subscriptions must preserve the arrival/settling logic.
        self.on_nav_state(msg)


    def supervisor(self):
        """
        FINAL submission start gate.

        Preferred trigger:
            column_navigator reports ARRIVED.

        Reliable fallback:
            target book is fresh, close to the shelf-stop
            distance, and stationary for four consecutive
            detections.

        This prevents a ROS navigation-state topic problem from
        leaving the robot permanently stopped at the shelf.
        """

        if self.worker_started or self.done:
            return

        if self.latest_book_xyz is None:
            return

        now = time.time()

        # Wait for the same valid samples required by run_sequence.
        if not self.startup_arms_safe or not self.collect_books:
            return

        if now < self.book_collect_after:
            return

        # Leave a small freshness margin before the worker starts.
        ready_samples = [
            sample for sample in list(self.book_samples)
            if 0.0 <= now - sample[3] < 2.5
        ]
        if len(ready_samples) < 8:
            if now - getattr(self, "_sample_wait_log_time", 0.0) >= 5.0:
                self._sample_wait_log_time = now
                self.get_logger().info(
                    f"WAITING FOR BOOK LOCK: {len(ready_samples)}/8 "
                    "valid samples within 2.5s"
                )
            return

        # Latest detector point must be live.
        if now - self.latest_book_time > 5.0:
            return

        nav_arrived = (
            getattr(self, "nav_state", None) == "ARRIVED"
        )

        # ----------------------------------------------------
        # Independent arrival fallback.
        #
        # book_samples entries are:
        #     (x, y, z, timestamp)
        #
        # While approaching, x changes continuously.
        # Once stopped at the shelf it becomes nearly constant.
        # ----------------------------------------------------

        fresh = [
            p
            for p in self.book_samples
            if now - p[3] < 10.0
        ]

        stable_at_shelf = False

        if len(fresh) >= 4:

            recent = fresh[-4:]

            xs = [float(p[0]) for p in recent]
            ys = [float(p[1]) for p in recent]
            zs = [float(p[2]) for p in recent]

            mean_x = sum(xs) / len(xs)

            stable_xyz = (
                (max(xs) - min(xs)) <= 0.030
                and
                (max(ys) - min(ys)) <= 0.035
                and
                (max(zs) - min(zs)) <= 0.035
            )

            # All successful ARRIVED tests have placed the
            # detected book comfortably below 2.60 m.
            near_shelf_stop = (
                1.80 <= mean_x <= 2.60
            )

            stable_at_shelf = (
                stable_xyz
                and near_shelf_stop
            )

        if not nav_arrived and not stable_at_shelf:
            return

        # Lock before thread creation so the timer can never
        # start manipulation twice.
        self.worker_started = True

        bx, by, bz = self.latest_book_xyz

        reason = (
            "NAV ARRIVED"
            if nav_arrived
            else "STABLE BOOK FALLBACK"
        )

        self.get_logger().info(
            f"FINAL START ({reason}): "
            f"book x={bx:.3f} "
            f"y={by:.3f} "
            f"z={bz:.3f}. "
            "Starting manipulation NOW."
        )

        threading.Thread(
            target=self.run_sequence,
            daemon=True
        ).start()


    def phase(self, text):

        self.get_logger().info(
            f"=== {text} ==="
        )


    def recent_book_contact(
        self,
        age=2.0
    ):

        return (
            time.time()
            - self.book_contact_time
            < age
        )


    def recent_bin_contact(
        self,
        age=2.0
    ):

        return (
            time.time()
            - self.book_bin_time
            < age
        )


    def stop_base(self):

        zero = Twist()

        for _ in range(6):

            self.cmd_pub.publish(
                zero
            )

            time.sleep(
                0.03
            )


    def wait_future(
        self,
        future,
        timeout
    ):

        end = (
            time.time()
            + timeout
        )

        while (
            rclpy.ok()
            and not future.done()
            and time.time() < end
        ):

            time.sleep(
                0.05
            )

        return future.done()


    # ==========================================================
    # JOINT COMMANDS
    # ==========================================================

    def action_trajectory(
        self,
        client,
        joint_names,
        positions,
        duration,
        timeout=350.0
    ):

        if not client.wait_for_server(
            timeout_sec=10.0
        ):
            return False

        trajectory = JointTrajectory()

        trajectory.joint_names = list(
            joint_names
        )

        point = JointTrajectoryPoint()

        point.positions = [
            float(v)
            for v in positions
        ]

        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int(
                (
                    duration
                    - int(duration)
                )
                * 1e9
            )
        )

        trajectory.points = [
            point
        ]

        goal = FollowJointTrajectory.Goal()

        goal.trajectory = trajectory

        future = client.send_goal_async(
            goal
        )

        if not self.wait_future(
            future,
            20.0
        ):
            return False

        handle = future.result()

        if (
            handle is None
            or not handle.accepted
        ):
            return False

        result_future = (
            handle.get_result_async()
        )

        # --------------------------------------------------
        # Slow Gazebo may take a long time to report the
        # action result even after the joints have physically
        # arrived. Watch /joint_states directly as well.
        # --------------------------------------------------

        targets = dict(
            zip(
                joint_names,
                positions
            )
        )

        stable_since = None

        deadline = (
            time.time()
            + timeout
        )

        while (
            rclpy.ok()
            and time.time() < deadline
        ):

            if result_future.done():
                return True

            have_all = all(
                name in self.joints
                for name in joint_names
            )

            if have_all:

                reached = True

                for name, target in targets.items():

                    current = self.joints[
                        name
                    ]

                    tolerance = (
                        0.008
                        if name == "torso_lift_joint"
                        else 0.035
                    )

                    if abs(
                        current - target
                    ) > tolerance:

                        reached = False
                        break

                if reached:

                    if stable_since is None:
                        stable_since = time.time()

                    elif (
                        time.time()
                        - stable_since
                        > 0.8
                    ):

                        # It has physically reached the target.
                        # Stop waiting for a delayed Gazebo
                        # action-result message.
                        handle.cancel_goal_async()

                        return True

                else:

                    stable_since = None

            time.sleep(
                0.05
            )

        handle.cancel_goal_async()

        return False


    def command_left_arm(
        self,
        q,
        duration=10.0
    ):

        return self.action_trajectory(
            self.left_arm_client,
            LEFT_ARM,
            q,
            duration
        )


    def command_right_arm(
        self,
        q,
        duration=8.0
    ):

        return self.action_trajectory(
            self.right_arm_client,
            RIGHT_ARM,
            q,
            duration
        )


    def command_torso(
        self,
        z,
        duration=8.0
    ):

        return self.action_trajectory(
            self.torso_client,
            [
                "torso_lift_joint"
            ],
            [
                z
            ],
            duration
        )


    def command_gripper(
        self,
        value,
        duration=5.0,
        timeout=180.0
    ):

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            "gripper_left_finger_joint"
        ]


        point = JointTrajectoryPoint()

        point.positions = [
            float(value)
        ]

        point.time_from_start = Duration(
            sec=int(duration)
        )

        trajectory.points = [
            point
        ]


        for _ in range(3):

            self.gripper_pub.publish(
                trajectory
            )

            time.sleep(
                0.1
            )


        end = (
            time.time()
            + timeout
        )


        while (
            rclpy.ok()
            and time.time() < end
        ):

            current = self.joints.get(
                "gripper_left_finger_joint"
            )


            if current is not None:

                if abs(
                    current
                    - value
                ) < 0.0025:

                    return True


                if (
                    value <= 0.001
                    and current <= 0.003
                ):

                    return True


                if (
                    value >= 0.068
                    and current >= 0.067
                ):

                    return True


            time.sleep(
                0.08
            )


        return False


    def command_head(
        self,
        tilt=0.0
    ):

        trajectory = JointTrajectory()

        trajectory.joint_names = [
            "head_1_joint",
            "head_2_joint"
        ]


        p = JointTrajectoryPoint()

        p.positions = [
            0.0,
            float(tilt)
        ]

        p.time_from_start = Duration(
            sec=2
        )

        trajectory.points = [
            p
        ]


        for _ in range(3):

            self.head_pub.publish(
                trajectory
            )

            time.sleep(
                0.1
            )


    # ==========================================================
    # BASE HELPERS
    # ==========================================================

    def current_xy(self):

        if self.odom is None:
            return None


        p = (
            self.odom
            .pose.pose.position
        )

        return (
            float(p.x),
            float(p.y)
        )


    def current_yaw(self):

        q = (
            self.odom
            .pose.pose.orientation
        )

        return math.atan2(
            2.0 * (
                q.w * q.z
                + q.x * q.y
            ),
            1.0 - 2.0 * (
                q.y * q.y
                + q.z * q.z
            )
        )


    def move_distance(
        self,
        distance,
        speed,
        stop_on_book=False,
        timeout=700.0
    ):

        start = self.current_xy()

        if start is None:

            return "NO_ODOM"


        self.unsafe_contact = None

        end = (
            time.time()
            + timeout
        )


        while (
            rclpy.ok()
            and time.time() < end
        ):

            xy = self.current_xy()

            moved = math.hypot(
                xy[0] - start[0],
                xy[1] - start[1]
            )


            if (
                stop_on_book
                and time.time()
                - self.book_contact_time
                < 0.4
            ):

                self.stop_base()

                return "BOOK"


            if self.unsafe_contact is not None:

                self.stop_base()

                return "BAD"


            if moved >= abs(
                distance
            ):

                self.stop_base()

                return "OK"


            remaining = (
                abs(distance)
                - moved
            )


            magnitude = min(
                abs(speed),
                max(
                    0.008,
                    0.25 * remaining
                )
            )


            command = Twist()

            command.linear.x = (
                math.copysign(
                    magnitude,
                    distance
                )
            )


            self.cmd_pub.publish(
                command
            )

            time.sleep(
                0.05
            )


        self.stop_base()

        return "TIMEOUT"


    def approach_book_visually(
        self,
        locked_xyz,
        stop_x=0.82,
        timeout=900.0
    ):
        """
        Approach the locked book with the ARM FROZEN.

        The live target-book point is expressed in base_link,
        so its X value naturally falls as the robot approaches.

        Stop conditions:
          - deliberate gripper/book contact
          - book reaches stop_x
          - maximum expected travel reached
          - collision
          - lost visual target
        """

        locked_x, locked_y, locked_z = locked_xyz

        start_xy = self.current_xy()

        if start_xy is None:
            return "NO_ODOM"

        # Safety travel limit based on the initial visual range.
        expected = max(
            0.0,
            locked_x - stop_x
        )

        max_travel = min(
            1.75,
            expected + 0.12
        )

        self.get_logger().info(
            f"VISUAL APPROACH: "
            f"book_start_x={locked_x:.3f}, "
            f"stop_x={stop_x:.3f}, "
            f"expected_travel={expected:.3f}, "
            f"max_travel={max_travel:.3f}"
        )

        self.unsafe_contact = None

        lost_since = None

        end = time.time() + timeout

        while (
            rclpy.ok()
            and time.time() < end
        ):

            # ----------------------------------------------
            # Collision
            # ----------------------------------------------

            if self.unsafe_contact is not None:

                self.stop_base()

                return "BAD"


            # ----------------------------------------------
            # Intended contact
            # ----------------------------------------------

            if (
                time.time()
                - self.book_contact_time
                < 0.35
            ):

                self.stop_base()

                return "BOOK"


            # ----------------------------------------------
            # Travel limit
            # ----------------------------------------------

            xy = self.current_xy()

            travelled = math.hypot(
                xy[0] - start_xy[0],
                xy[1] - start_xy[1]
            )

            if travelled >= max_travel:

                self.stop_base()

                return "TRAVEL_LIMIT"


            # ----------------------------------------------
            # Fresh visual point
            # ----------------------------------------------

            fresh = (
                self.latest_book_xyz is not None
                and time.time()
                - self.latest_book_time
                < 2.0
            )

            if not fresh:

                self.stop_base()

                if lost_since is None:
                    lost_since = time.time()

                if (
                    time.time()
                    - lost_since
                    > 5.0
                ):
                    return "TARGET_LOST"

                time.sleep(
                    0.05
                )

                continue

            lost_since = None


            bx, by, bz = (
                self.latest_book_xyz
            )


            # Ignore a sudden switch to a different red book.
            if abs(
                bz - locked_z
            ) > 0.18:

                self.stop_base()

                time.sleep(
                    0.05
                )

                continue


            # ----------------------------------------------
            # Desired pre-contact distance achieved
            # ----------------------------------------------

            if bx <= stop_x:

                self.stop_base()

                self.get_logger().info(
                    f"VISUAL APPROACH COMPLETE: "
                    f"book_x={bx:.3f}, "
                    f"travel={travelled:.3f}"
                )

                return "READY"


            # ----------------------------------------------
            # Speed scheduling
            # ----------------------------------------------

            gap = bx - stop_x


            if gap > 0.80:
                speed = 0.050

            elif gap > 0.40:
                speed = 0.035

            elif gap > 0.18:
                speed = 0.022

            else:
                speed = 0.012


            command = Twist()

            command.linear.x = speed

            # Very small lateral correction only.
            #
            # The arm target was generated using the locked
            # lateral Y coordinate, so large corrections here
            # would invalidate that pregrasp pose.
            lateral_error = (
                by - locked_y
            )

            command.linear.y = max(
                -0.008,
                min(
                    0.008,
                    0.08 * lateral_error
                )
            )


            self.cmd_pub.publish(
                command
            )

            time.sleep(
                0.05
            )


        self.stop_base()

        return "TIMEOUT"


    def return_to_start(
        self,
        timeout=700.0
    ):

        self.unsafe_contact = None

        end = (
            time.time()
            + timeout
        )


        while (
            rclpy.ok()
            and time.time() < end
        ):

            xy = self.current_xy()


            if xy is None:

                time.sleep(
                    0.1
                )

                continue


            ex = (
                self.start_xy[0]
                - xy[0]
            )

            ey = (
                self.start_xy[1]
                - xy[1]
            )


            distance = math.hypot(
                ex,
                ey
            )


            if distance < 0.15:

                self.stop_base()

                return True


            yaw = self.current_yaw()

            c = math.cos(
                yaw
            )

            s = math.sin(
                yaw
            )


            vx = (
                c * ex
                + s * ey
            )

            vy = (
                -s * ex
                + c * ey
            )


            command = Twist()

            command.linear.x = max(
                -0.045,
                min(
                    0.045,
                    0.35 * vx
                )
            )

            command.linear.y = max(
                -0.045,
                min(
                    0.045,
                    0.35 * vy
                )
            )


            self.cmd_pub.publish(
                command
            )


            if self.unsafe_contact is not None:

                self.stop_base()

                return False


            if not self.recent_book_contact(
                2.0
            ):

                self.stop_base()

                return False


            time.sleep(
                0.05
            )


        self.stop_base()

        return False


    # ==========================================================
    # URDF + IK
    # ==========================================================

    def load_urdf(self):

        root = ET.parse(
            URDF
        ).getroot()


        self.urdf = {}
        self.child_joint = {}


        def xyz(text):

            return np.array(
                [
                    float(x)
                    for x in (
                        text
                        or "0 0 0"
                    ).split()
                ]
            )


        for joint in root.findall(
            "joint"
        ):

            name = joint.attrib[
                "name"
            ]

            parent = (
                joint.find(
                    "parent"
                ).attrib[
                    "link"
                ]
            )

            child = (
                joint.find(
                    "child"
                ).attrib[
                    "link"
                ]
            )


            origin = np.eye(
                4
            )


            o = joint.find(
                "origin"
            )


            if o is not None:

                origin[:3, 3] = xyz(
                    o.attrib.get(
                        "xyz"
                    )
                )

                origin[:3, :3] = (
                    Rotation.from_euler(
                        "xyz",
                        xyz(
                            o.attrib.get(
                                "rpy"
                            )
                        )
                    ).as_matrix()
                )


            axis_element = joint.find(
                "axis"
            )


            axis = (
                xyz(
                    axis_element.attrib.get(
                        "xyz",
                        "1 0 0"
                    )
                )
                if axis_element is not None
                else np.array(
                    [
                        1.0,
                        0.0,
                        0.0
                    ]
                )
            )


            limit = joint.find(
                "limit"
            )


            lower = (
                float(
                    limit.attrib[
                        "lower"
                    ]
                )
                if (
                    limit is not None
                    and "lower"
                    in limit.attrib
                )
                else None
            )


            upper = (
                float(
                    limit.attrib[
                        "upper"
                    ]
                )
                if (
                    limit is not None
                    and "upper"
                    in limit.attrib
                )
                else None
            )


            self.urdf[name] = {
                "type": joint.attrib.get(
                    "type",
                    "fixed"
                ),
                "parent": parent,
                "axis": axis,
                "origin": origin,
                "lower": lower,
                "upper": upper,
            }


            self.child_joint[
                child
            ] = name


    def joint_motion(
        self,
        joint,
        q
    ):

        transform = np.eye(
            4
        )


        axis = np.array(
            joint["axis"],
            dtype=float
        )


        magnitude = np.linalg.norm(
            axis
        )


        if magnitude > 0.0:

            axis /= magnitude


        if joint["type"] in (
            "revolute",
            "continuous"
        ):

            transform[:3, :3] = (
                Rotation.from_rotvec(
                    axis * q
                ).as_matrix()
            )


        elif joint["type"] == "prismatic":

            transform[:3, 3] = (
                axis * q
            )


        return transform


    def fk(self, q):

        names = [
            "torso_lift_joint"
        ] + LEFT_ARM


        values = dict(
            zip(
                names,
                q
            )
        )


        chain = []

        link = TIP


        while link != BASE:

            name = self.child_joint[
                link
            ]

            chain.append(
                name
            )

            link = self.urdf[
                name
            ][
                "parent"
            ]


        transform = np.eye(
            4
        )


        for name in reversed(
            chain
        ):

            joint = self.urdf[
                name
            ]

            transform = (
                transform
                @ joint[
                    "origin"
                ]
            )


            if name in values:

                transform = (
                    transform
                    @ self.joint_motion(
                        joint,
                        values[name]
                    )
                )


        return transform


    def solve_ik(
        self,
        target_xyz,
        reference_arm,
        torso_seed=0.0,
        fixed_torso=None
    ):

        names = [
            "torso_lift_joint"
        ] + LEFT_ARM


        seed = np.array(
            [
                torso_seed
            ]
            + list(
                reference_arm
            ),
            dtype=float
        )


        reference_rotation = (
            self.fk(
                seed
            )[:3, :3]
        )


        lower = []
        upper = []


        for index, name in enumerate(
            names
        ):

            joint = self.urdf[
                name
            ]


            lo = (
                joint[
                    "lower"
                ]
                if joint[
                    "lower"
                ] is not None
                else -math.pi
            )

            hi = (
                joint[
                    "upper"
                ]
                if joint[
                    "upper"
                ] is not None
                else math.pi
            )


            if (
                index == 0
                and fixed_torso is not None
            ):

                lo = fixed_torso

                hi = (
                    fixed_torso
                    + 1e-8
                )

                seed[0] = (
                    fixed_torso
                )


            lower.append(
                lo
            )

            upper.append(
                hi
            )


        lower = np.array(
            lower
        )

        upper = np.array(
            upper
        )


        seed = np.clip(
            seed,
            lower,
            upper
        )


        target = np.array(
            target_xyz,
            dtype=float
        )


        def residual(q):

            transform = self.fk(
                q
            )


            position_error = (
                transform[:3, 3]
                - target
            )


            orientation_error = (
                Rotation.from_matrix(
                    reference_rotation.T
                    @ transform[:3, :3]
                ).as_rotvec()
            )


            return np.concatenate(
                [
                    14.0
                    * position_error,

                    2.2
                    * orientation_error,

                    0.035
                    * (
                        q
                        - seed
                    )
                ]
            )


        solution = least_squares(
            residual,
            seed,
            bounds=(
                lower,
                upper
            ),
            max_nfev=2500
        ).x


        error = float(
            np.linalg.norm(
                self.fk(
                    solution
                )[:3, 3]
                - target
            )
        )


        return (
            solution,
            error
        )


    # ==========================================================
    # RED BIN VISION
    # ==========================================================

    def depth_at(
        self,
        cx,
        cy,
        radius=10
    ):

        if self.latest_depth is None:

            return None


        h, w = (
            self.latest_depth.shape[:2]
        )


        patch = np.asarray(
            self.latest_depth[
                max(
                    0,
                    cy - radius
                ):
                min(
                    h,
                    cy + radius + 1
                ),
                max(
                    0,
                    cx - radius
                ):
                min(
                    w,
                    cx + radius + 1
                )
            ],
            dtype=np.float32
        ).reshape(
            -1
        )


        patch = patch[
            np.isfinite(
                patch
            )
            & (
                patch > 0
            )
        ]


        patch = (
            patch
            * self.depth_scale
        )


        patch = patch[
            (
                patch > 0.05
            )
            & (
                patch < 10.0
            )
        ]


        if patch.size == 0:

            return None


        return float(
            np.median(
                patch
            )
        )


    def find_red_bin(
        self,
        frame
    ):

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )


        mask1 = cv2.inRange(
            hsv,
            (
                0,
                110,
                60
            ),
            (
                10,
                255,
                255
            )
        )


        mask2 = cv2.inRange(
            hsv,
            (
                168,
                110,
                60
            ),
            (
                179,
                255,
                255
            )
        )


        mask = cv2.bitwise_or(
            mask1,
            mask2
        )


        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones(
                (
                    5,
                    5
                ),
                np.uint8
            )
        )


        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )


        h, w = frame.shape[:2]

        candidates = []


        for contour in contours:

            area = float(
                cv2.contourArea(
                    contour
                )
            )


            # Manual run:
            # held red book ~3754 px
            # real red bin ~16369 px
            #
            # This rejects the held book.
            if area < 7000.0:

                continue


            x, y, cw, ch = (
                cv2.boundingRect(
                    contour
                )
            )


            cx = (
                x
                + cw // 2
            )

            cy = (
                y
                + ch // 2
            )


            depth = self.depth_at(
                cx,
                cy,
                max(
                    6,
                    min(
                        18,
                        cw // 8
                    )
                )
            )


            if (
                depth is None
                or not (
                    0.70
                    <= depth
                    <= 2.60
                )
            ):

                continue


            offset = (
                (
                    cx
                    - w / 2.0
                )
                / (
                    w / 2.0
                )
            )


            candidates.append(
                (
                    area,
                    offset,
                    depth
                )
            )


        if not candidates:

            return None


        area, offset, depth = max(
            candidates,
            key=lambda value: value[0]
        )


        return {
            "area": area,
            "offset": offset,
            "depth": depth,
        }


    def visual_bin_approach(
        self
    ):

        self.command_head(
            0.0
        )


        self.bin_confirm = 0
        self.unsafe_contact = None


        # --------------------------------------------------
        # SEARCH
        # --------------------------------------------------

        self.phase(
            "VISUAL BIN SEARCH"
        )


        end = (
            time.time()
            + 600.0
        )


        while (
            rclpy.ok()
            and time.time() < end
        ):

            if (
                self.bin_candidate
                and time.time()
                - self.bin_candidate_time
                < 1.5
                and self.bin_confirm >= 3
            ):

                self.stop_base()

                c = (
                    self.bin_candidate
                )

                self.get_logger().info(
                    f"BIN FOUND: "
                    f"area={c['area']:.0f} "
                    f"depth={c['depth']:.2f} "
                    f"offset={c['offset']:+.2f}"
                )

                break


            command = Twist()

            command.angular.z = 0.12

            self.cmd_pub.publish(
                command
            )

            time.sleep(
                0.05
            )


        else:

            self.stop_base()

            return False


        # --------------------------------------------------
        # CENTER
        # --------------------------------------------------

        self.phase(
            "CENTER BIN"
        )


        end = (
            time.time()
            + 300.0
        )


        while (
            rclpy.ok()
            and time.time() < end
        ):

            if (
                not self.bin_candidate
                or time.time()
                - self.bin_candidate_time
                > 2.0
            ):

                self.stop_base()

                time.sleep(
                    0.1
                )

                continue


            offset = (
                self.bin_candidate[
                    "offset"
                ]
            )


            if abs(
                offset
            ) < 0.07:

                self.stop_base()

                break


            command = Twist()

            command.angular.z = (
                -math.copysign(
                    (
                        0.06
                        if abs(
                            offset
                        ) > 0.15
                        else 0.025
                    ),
                    offset
                )
            )


            self.cmd_pub.publish(
                command
            )


            if self.unsafe_contact is not None:

                self.stop_base()

                return False


            time.sleep(
                0.05
            )


        else:

            self.stop_base()

            return False


        # --------------------------------------------------
        # APPROACH
        # --------------------------------------------------

        self.phase(
            "APPROACH BIN"
        )


        self.unsafe_contact = None


        end = (
            time.time()
            + 500.0
        )


        while (
            rclpy.ok()
            and time.time() < end
        ):

            if (
                not self.bin_candidate
                or time.time()
                - self.bin_candidate_time
                > 2.5
            ):

                self.stop_base()

                time.sleep(
                    0.1
                )

                continue


            if self.recent_bin_contact(
                0.5
            ):

                self.stop_base()

                return True


            if (
                self.bin_candidate[
                    "depth"
                ]
                <= 0.82
            ):

                self.stop_base()

                return True


            command = Twist()


            command.linear.x = (
                0.022
                if self.bin_candidate[
                    "depth"
                ] > 1.0
                else 0.012
            )


            command.angular.z = max(
                -0.05,
                min(
                    0.05,
                    -0.12
                    * self.bin_candidate[
                        "offset"
                    ]
                )
            )


            self.cmd_pub.publish(
                command
            )


            if self.unsafe_contact is not None:

                self.stop_base()

                return False


            time.sleep(
                0.05
            )


        self.stop_base()

        return False


    # ==========================================================
    # FINAL WRIST RELEASE
    # ==========================================================

    def wrist_release(self):

        if self.recent_bin_contact(
            1.5
        ):

            return True


        now = time.time()


        if (
            now
            - self.right_book_time
            < 1.0
        ):

            support = "RIGHT"


        elif (
            now
            - self.left_book_time
            < 1.0
        ):

            support = "LEFT"


        else:

            time.sleep(
                5.0
            )

            return self.recent_bin_contact(
                2.0
            )


        q = [
            self.joints.get(
                joint
            )
            for joint in LEFT_ARM
        ]


        if any(
            value is None
            for value in q
        ):

            return False


        # From the proven manual release geometry:
        # negative J7 lowers the right finger,
        # positive J7 lowers the left finger.
        delta = (
            -0.65
            if support == "RIGHT"
            else +0.65
        )


        target = list(
            q
        )


        target[-1] += (
            delta
        )


        joint7 = self.urdf[
            "arm_left_7_joint"
        ]


        if (
            joint7[
                "lower"
            ] is not None
        ):

            target[-1] = max(
                joint7[
                    "lower"
                ] + 0.01,
                target[-1]
            )


        if (
            joint7[
                "upper"
            ] is not None
        ):

            target[-1] = min(
                joint7[
                    "upper"
                ] - 0.01,
                target[-1]
            )


        if not self.left_arm_client.wait_for_server(
            timeout_sec=10.0
        ):

            return False


        trajectory = JointTrajectory()

        trajectory.joint_names = (
            LEFT_ARM
        )


        point = JointTrajectoryPoint()

        point.positions = target

        point.time_from_start = Duration(
            sec=20
        )


        trajectory.points = [
            point
        ]


        goal = FollowJointTrajectory.Goal()

        goal.trajectory = trajectory


        future = (
            self.left_arm_client
            .send_goal_async(
                goal
            )
        )


        if not self.wait_future(
            future,
            20.0
        ):

            return False


        handle = future.result()


        if (
            handle is None
            or not handle.accepted
        ):

            return False


        result_future = (
            handle.get_result_async()
        )


        self.unsafe_contact = None

        start = time.time()


        while (
            rclpy.ok()
            and time.time()
            - start
            < 300.0
        ):

            if self.recent_bin_contact(
                0.8
            ):

                handle.cancel_goal_async()

                return True


            if self.unsafe_contact is not None:

                handle.cancel_goal_async()

                return False


            if result_future.done():

                break


            time.sleep(
                0.05
            )


        time.sleep(
            5.0
        )


        return self.recent_bin_contact(
            2.0
        )


    # ==========================================================
    # FAILURE
    # ==========================================================

    def fail(
        self,
        reason
    ):

        self.stop_base()

        self.done = True

        self.get_logger().error(
            "AUTONOMOUS FAILED: "
            + str(
                reason
            )
        )


    # ==========================================================
    # FULL AUTONOMOUS SEQUENCE
    # ==========================================================

    def run_sequence(self):


        try:

            # --------------------------------------------------
            # LOCK BOOK
            # --------------------------------------------------

            self.phase(
                "LOCK TARGET BOOK"
            )


            fresh = np.array(
                [
                    [
                        p[0],
                        p[1],
                        p[2]
                    ]
                    for p in self.book_samples
                    if time.time()
                    - p[3]
                    < 3.0
                ],
                dtype=float
            )


            if len(
                fresh
            ) < 8:

                return self.fail(
                    "Not enough fresh book samples"
                )


            bx, by, bz = np.median(
                fresh,
                axis=0
            )


            self.collect_books = False


            self.get_logger().info(
                f"Locked book: "
                f"x={bx:.3f} "
                f"y={by:.3f} "
                f"z={bz:.3f}"
            )


            # --------------------------------------------------
            # PREPARE
            # --------------------------------------------------

            self.phase(
                "TUCK RIGHT ARM"
            )


            if not self.command_right_arm(
                RIGHT_TUCK
            ):

                return self.fail(
                    "Right arm tuck failed"
                )


            self.phase(
                "OPEN LEFT GRIPPER"
            )


            if not self.command_gripper(
                0.069
            ):

                return self.fail(
                    "Could not open left gripper"
                )


            # --------------------------------------------------
            # ROW-ADAPTIVE PREGRASP
            # --------------------------------------------------

            self.phase(
                "ROW ADAPTIVE PREGRASP"
            )


            # Proven manual geometry:
            # gripper approximately x=0.72 m from base.
            #
            # The robot remains far from the shelf while
            # establishing this pose; the BASE then performs
            # the long approach with the arm frozen.

            pregrasp_x = 0.72

            target = [
                pregrasp_x,

                max(
                    -0.30,
                    min(
                        0.32,
                        by
                    )
                ),

                max(
                    0.48,
                    min(
                        1.28,
                        bz + 0.058
                    )
                )
            ]

            self.get_logger().info(
                f"PREGRASP GEOMETRY: "
                f"book=({bx:.3f},{by:.3f},{bz:.3f}) "
                f"gripper_x={pregrasp_x:.3f}"
            )


            torso_seed = max(
                0.0,
                min(
                    0.35,
                    target[2]
                    - 0.90
                )
            )


            solution, error = (
                self.solve_ik(
                    target,
                    LEFT_PREGRASP_REF,
                    torso_seed
                )
            )


            self.get_logger().info(
                f"PREGRASP IK: "
                f"error={error:.4f} "
                f"torso={solution[0]:.3f}"
            )


            if error > 0.015:

                return self.fail(
                    "Pregrasp IK error too large"
                )


            if not self.command_torso(
                solution[0]
            ):

                return self.fail(
                    "Pregrasp torso command failed"
                )


            if not self.command_left_arm(
                solution[1:]
            ):

                return self.fail(
                    "Pregrasp arm command failed"
                )


            # --------------------------------------------------
            # INSERT BASE UNTIL BOOK CONTACT
            # --------------------------------------------------

            self.phase(
                "VISUAL BASE APPROACH TO BOOK"
            )

            # Arm is now frozen in the safe pregrasp pose.
            #
            # Use the live 3D book point to bring the book
            # from ~2.2 m down to ~0.82 m in base_link.

            result = self.approach_book_visually(
                (
                    bx,
                    by,
                    bz
                ),
                # Stop early. We will perform a fresh
                # camera-based arm alignment before contact.
                stop_x=1.00,
                timeout=900.0
            )


            if result == "BAD":

                return self.fail(
                    "Collision during visual shelf approach"
                )


            if result == "TARGET_LOST":

                return self.fail(
                    "Lost target book during visual approach"
                )


            if result not in (
                "READY",
                "BOOK",
                "TIMEOUT_OK"
            ):

                return self.fail(
                    f"Visual book approach failed: {result}"
                )


            # ==================================================
            # FINAL LIVE BOOK REALIGNMENT
            #
            # The head camera and perspective can change during
            # the long base approach. Do NOT use the old book Z
            # for the final grasp.
            #
            # Re-read the current target book XYZ and solve the
            # arm/torso pose again immediately before insertion.
            # ==================================================

            if result != "BOOK":

                self.phase(
                    "FINAL LIVE BOOK REALIGN"
                )


                # Give perception a short moment to update after
                # the base has stopped.
                time.sleep(
                    1.5
                )


                if (
                    self.latest_book_xyz is None
                    or time.time()
                    - self.latest_book_time
                    > 3.0
                ):

                    return self.fail(
                        "No fresh target book for final alignment"
                    )


                fbx, fby, fbz = (
                    self.latest_book_xyz
                )


                self.get_logger().info(
                    f"FINAL BOOK XYZ: "
                    f"x={fbx:.3f} "
                    f"y={fby:.3f} "
                    f"z={fbz:.3f}"
                )


                # Keep the grasp frame at the proven ~0.72 m
                # forward reach, but update lateral position and
                # HEIGHT from the current camera measurement.
                final_target = [
                    0.72,

                    max(
                        -0.30,
                        min(
                            0.32,
                            fby
                        )
                    ),

                    max(
                        0.48,
                        min(
                            1.28,
                            fbz + 0.058
                        )
                    )
                ]


                final_torso_seed = max(
                    0.0,
                    min(
                        0.35,
                        final_target[2]
                        - 0.90
                    )
                )


                final_solution, final_error = (
                    self.solve_ik(
                        final_target,
                        LEFT_PREGRASP_REF,
                        final_torso_seed
                    )
                )


                self.get_logger().info(
                    f"FINAL REALIGN IK: "
                    f"error={final_error:.4f} "
                    f"torso={final_solution[0]:.3f}"
                )


                if final_error > 0.015:

                    return self.fail(
                        "Final book realign IK error too large"
                    )


                if not self.command_torso(
                    final_solution[0]
                ):

                    return self.fail(
                        "Final torso realign failed"
                    )


                if not self.command_left_arm(
                    final_solution[1:]
                ):

                    return self.fail(
                        "Final left-arm realign failed"
                    )


                # ------------------------------------------------
                # Refresh target ONCE MORE after moving the arm.
                # ------------------------------------------------

                time.sleep(
                    1.0
                )


                if (
                    self.latest_book_xyz is not None
                    and time.time()
                    - self.latest_book_time
                    < 3.0
                ):

                    fbx, fby, fbz = (
                        self.latest_book_xyz
                    )


                self.phase(
                    "FINE BOOK CONTACT SEARCH"
                )


                # ==================================================
                # SAFE FINAL GRASP APPROACH
                #
                # Previous trial collided when book_x reached
                # roughly 0.744 m.
                #
                # Do NOT intentionally drive all the way until
                # contact. Stop while the target book is still
                # about 0.77 m from base_link, then close the
                # gripper around it.
                # ==================================================

                # ==================================================
                # LIVE CAMERA-GUIDED FINAL CREEP
                #
                # Do not use a fixed odometry distance here.
                # Previous run stopped too early at book_x ~0.865.
                #
                # Creep until the LIVE book point reaches 0.79 m.
                # Previous shelf collision happened around 0.744 m,
                # so 0.79 leaves useful safety margin.
                # ==================================================

                self.get_logger().info("CREEP_FIX_V3: timeout=900s, travel_limit=0.25m")
                creep_progress_log = 0.0
                safe_book_x = 0.79
                self.phase('LIVE FINAL CREEP TO BOOK')
                self.get_logger().info(f'LIVE CREEP START: target_book_x={safe_book_x:.3f}')
                creep_start_xy = self.current_xy()
                if creep_start_xy is None:
                    return self.fail('No odometry for final creep')
                self.unsafe_contact = None
                creep_deadline = time.time() + 900.0
                result = None
                while rclpy.ok() and time.time() < creep_deadline:
                    if self.unsafe_contact is not None:
                        self.stop_base()
                        return self.fail('Collision during live final creep')
                    if time.time() - creep_progress_log >= 5.0:
                        creep_progress_log = time.time()
                        self.get_logger().info(f'CREEP PROGRESS: book={self.latest_book_xyz}; target x=0.790')
                    if self.recent_book_contact(0.35):
                        self.stop_base()
                        result = 'BOOK'
                        self.get_logger().info('BOOK CONTACT during live creep.')
                        break
                    xy = self.current_xy()
                    if xy is None:
                        self.stop_base()
                        return self.fail('Lost odometry during final creep')
                    moved = math.hypot(xy[0] - creep_start_xy[0], xy[1] - creep_start_xy[1])
                    if moved >= 0.25:
                        self.stop_base()
                        return self.fail('Final creep exceeded 25 cm safety limit')
                    fresh_book = self.latest_book_xyz is not None and time.time() - self.latest_book_time < 2.0
                    if not fresh_book:
                        self.stop_base()
                        time.sleep(0.05)
                        continue
                    (live_x, live_y, live_z) = self.latest_book_xyz
                    if live_x <= safe_book_x:
                        self.stop_base()
                        result = 'OK'
                        self.get_logger().info(f'LIVE GRASP POSITION REACHED: book_x={live_x:.3f}, book_y={live_y:.3f}, travel={moved:.3f}')
                        break
                    command = Twist()
                    command.linear.x = 0.003
                    self.cmd_pub.publish(command)
                    time.sleep(0.05)
                self.stop_base()
                if result not in ('OK', 'BOOK'):
                    return self.fail('Live final creep timed out')
                if result == 'BOOK':
                    self.get_logger().info('Target book already touching gripper. Closing now.')
                else:
                    self.get_logger().info('SAFE LIVE GRASP POSITION REACHED. Closing gripper now.')

                # Absolute stop before gripper closure.
                self.stop_base()

                time.sleep(
                    0.5
                )


            # --------------------------------------------------
            # CLOSE
            # --------------------------------------------------

            self.phase(
                "CLOSE GRIPPER"
            )


            if not self.command_gripper(
                0.0
            ):

                return self.fail(
                    "Gripper close failed"
                )


            time.sleep(
                2.0
            )


            if not self.recent_book_contact():

                return self.fail(
                    "Book not held after closing"
                )


            # --------------------------------------------------
            # EXTRACT WITH ARM FROZEN
            # --------------------------------------------------

            self.phase(
                "EXTRACT BOOK"
            )


            result = self.move_distance(
                -0.42,
                0.035
            )


            if result != "OK":

                return self.fail(
                    f"Extraction failed: {result}"
                )


            if not self.recent_book_contact():

                return self.fail(
                    "Book lost during extraction"
                )


            # --------------------------------------------------
            # CARRY
            # --------------------------------------------------

            self.phase(
                "CARRY POSE"
            )


            if not self.command_torso(
                0.0
            ):

                return self.fail(
                    "Torso reset failed"
                )


            if not self.command_left_arm(
                LEFT_CARRY
            ):

                return self.fail(
                    "Carry pose failed"
                )


            # --------------------------------------------------
            # RETURN
            # --------------------------------------------------

            self.phase(
                "RETURN TO START"
            )


            if not self.return_to_start():

                return self.fail(
                    "Could not return to start with book"
                )


            # --------------------------------------------------
            # HIGH TABLE-SAFE BIN POSE
            # --------------------------------------------------

            self.phase(
                "HIGH BIN PLACEMENT PREP"
            )


            if not self.command_torso(
                0.24,
                12.0
            ):

                return self.fail(
                    "Could not raise torso for bin"
                )


            placement_solution, placement_error = (
                self.solve_ik(
                    [
                        0.88,
                        0.10,
                        1.15
                    ],
                    LEFT_PREGRASP_REF,
                    0.24,
                    fixed_torso=0.24
                )
            )


            self.get_logger().info(
                f"PLACEMENT IK: "
                f"error={placement_error:.4f}"
            )


            if placement_error > 0.018:

                return self.fail(
                    "Placement IK error too large"
                )


            if not self.command_left_arm(
                placement_solution[1:],
                12.0
            ):

                return self.fail(
                    "Placement arm pose failed"
                )


            # --------------------------------------------------
            # VISUAL BIN DETECTION + APPROACH
            # --------------------------------------------------

            if not self.visual_bin_approach():

                return self.fail(
                    "Visual bin approach failed"
                )


            # --------------------------------------------------
            # OPEN ABOVE BIN
            # --------------------------------------------------

            self.phase(
                "OPEN ABOVE BIN"
            )


            self.unsafe_contact = None


            if not self.command_gripper(
                0.069
            ):

                return self.fail(
                    "Could not open above bin"
                )


            time.sleep(
                5.0
            )


            # --------------------------------------------------
            # WRIST RELEASE IF NEEDED
            # --------------------------------------------------

            if not self.recent_bin_contact():

                self.phase(
                    "WRIST RELEASE"
                )


                if not self.wrist_release():

                    return self.fail(
                        "Book did not reach bin"
                    )


            # --------------------------------------------------
            # FINAL CONFIRMATION
            # --------------------------------------------------

            if not self.recent_bin_contact():

                return self.fail(
                    "Final book-bin contact missing"
                )


            self.stop_base()

            self.done = True


            self.phase(
                "SUCCESS"
            )


            self.get_logger().info(
                "AUTONOMOUS RUN COMPLETE: "
                "book contacted collection bin."
            )


        except Exception as exc:

            self.fail(
                f"{type(exc).__name__}: {exc}"
            )


    def destroy_node(self):

        self.stop_base()

        super().destroy_node()


def main(args=None):

    rclpy.init(
        args=args
    )

    node = GraspController()

    try:

        rclpy.spin(
            node
        )

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
