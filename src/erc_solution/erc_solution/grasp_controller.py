#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import cv2
import os
import subprocess
import re
from datetime import datetime
import numpy as np

class GraspController(Node):
    def __init__(self):
        super().__init__("grasp_controller")
        self.arm_joints = [
            "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
            "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint",
            "arm_left_7_joint",
        ]
        self.home_position = [0.36, 1.83, 0.47, -2.35, 0.0, -1.2, 0.0]

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/arm_left_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            "/gripper_left_controller_raw/follow_joint_trajectory"
        )
        self.base_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.row_id_pub = self.create_publisher(Int32, "/erc/shelf_row_identification", 10)
        self.book_pose_sub = self.create_subscription(
            PointStamped, "/erc/target_book_point", self.on_book_pose, 10
        )
        self.camera_sub = self.create_subscription(
            Image, "/head_front_camera/head_front_camera/color/image_raw",
            self.on_camera, 10
        )

        self.latest_book_pose = None
        self.latest_image = None
        self.detected_row = None
        self.bridge = CvBridge()
        self.state = "WAITING"
        self.wait_count = 0
        self.state_time = time.time()

        self.bin_world_pos = np.array([-1.0, 0.0, 0.845])

        os.makedirs("/erc_images", exist_ok=True)

        self.create_timer(0.5, self.state_machine)
        self.get_logger().info("GraspController initialized")

    def on_book_pose(self, msg):
        self.latest_book_pose = msg
        z_pos = msg.point.z
        if z_pos > 1.0:
            self.detected_row = 2
        elif z_pos > 0.5:
            self.detected_row = 3
        elif z_pos > 0.2:
            self.detected_row = 4
        else:
            self.detected_row = 5

    def on_camera(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pass

    def save_annotated_image(self, filename_prefix):
        if self.latest_image is None:
            return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"/erc_images/{filename_prefix}_{timestamp}.jpg"
        cv2.imwrite(filename, self.latest_image)

    def get_robot_world_pose(self):
        try:
            txt = subprocess.check_output(
                ["gz", "model", "-m", "tiago_pro", "-p"],
                text=True, stderr=subprocess.DEVNULL
            )
            m = re.search(r"Pose.*?\n\s*\[([^\]]+)\]\s*\n\s*\[([^\]]+)\]", txt, re.S)
            xyz = np.array([float(v) for v in m.group(1).split()])
            rpy = np.array([float(v) for v in m.group(2).split()])
            return xyz, rpy
        except:
            return None, None

    def navigate_to_world_pose(self, target_x, target_y, timeout=60.0):
        start_time = time.time()
        while time.time() - start_time < timeout and rclpy.ok():
            robot_xyz, robot_rpy = self.get_robot_world_pose()
            if robot_xyz is None:
                time.sleep(0.1)
                continue

            x, y, yaw = robot_xyz[0], robot_xyz[1], robot_rpy[2]
            ex = target_x - x
            ey = target_y - y

            if math.sqrt(ex**2 + ey**2) < 0.08:
                self.base_pub.publish(Twist())
                self.get_logger().info(f"Arrived at bin location")
                return True

            c, s = math.cos(yaw), math.sin(yaw)
            vx_body = c * ex + s * ey
            vy_body = -s * ex + c * ey

            m = Twist()
            m.linear.x = max(-0.05, min(0.05, 0.4 * vx_body))
            m.linear.y = max(-0.05, min(0.05, 0.4 * vy_body))
            self.base_pub.publish(m)
            time.sleep(0.1)

        self.base_pub.publish(Twist())
        self.get_logger().warn("Navigation timeout, proceeding anyway")
        return False

    def send_arm_trajectory(self, positions, duration=2.0):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        traj = JointTrajectory()
        traj.joint_names = self.arm_joints
        traj.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.arm_client.send_goal_async(goal)

    def send_gripper(self, position=0.0, duration=1.0):
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        traj = JointTrajectory()
        traj.joint_names = ["gripper_left_finger_joint"]
        traj.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.gripper_client.send_goal_async(goal)

    def state_machine(self):
        if self.state == "WAITING":
            self.wait_count += 1
            if self.latest_book_pose is not None:
                self.get_logger().info("=== GRASP SEQUENCE START ===")
                self.state = "SAVE_INITIAL"
                self.state_time = time.time()
            elif self.wait_count >= 60:
                self.get_logger().error("TIMEOUT: No book detected")
                raise SystemExit(1)

        elif self.state == "SAVE_INITIAL":
            self.save_annotated_image("book_detected")
            self.state = "PUBLISH_IDS"
            self.state_time = time.time()

        elif self.state == "PUBLISH_IDS":
            if self.detected_row is not None:
                row_msg = Int32()
                row_msg.data = self.detected_row
                self.row_id_pub.publish(row_msg)
            self.state = "MOVING_TO_GRASP"
            self.state_time = time.time()

        elif self.state == "MOVING_TO_GRASP":
            if time.time() - self.state_time < 0.5:
                return
            self.get_logger().info("Moving to grasp...")
            self.send_arm_trajectory([0.5, 1.5, 0.5, -1.5, 0.0, -1.0, 0.0], 3.0)
            self.state = "CLOSING_GRIPPER"
            self.state_time = time.time()

        elif self.state == "CLOSING_GRIPPER":
            if time.time() - self.state_time < 3.5:
                return
            self.get_logger().info("Closing gripper...")
            self.save_annotated_image("gripper_closing")
            self.send_gripper(0.0, 1.0)
            self.state = "RETRACTING"
            self.state_time = time.time()

        elif self.state == "RETRACTING":
            if time.time() - self.state_time < 1.5:
                return
            self.get_logger().info("Retracting...")
            self.save_annotated_image("gripper_closed")
            self.send_arm_trajectory(self.home_position, 3.0)
            self.state = "NAVIGATE_TO_BIN"
            self.state_time = time.time()

        elif self.state == "NAVIGATE_TO_BIN":
            if time.time() - self.state_time < 3.5:
                return
            self.get_logger().info("Navigating to bin...")
            self.navigate_to_world_pose(target_x=-0.5, target_y=0.2, timeout=60.0)
            self.state = "APPROACH_BIN"
            self.state_time = time.time()

        elif self.state == "APPROACH_BIN":
            if time.time() - self.state_time < 1.0:
                return
            self.get_logger().info("Positioning arm over bin...")
            self.send_arm_trajectory([0.3, 0.5, 0.5, -1.5, 0.0, -1.0, 0.0], 3.0)
            self.state = "RELEASING_BOOK"
            self.state_time = time.time()

        elif self.state == "RELEASING_BOOK":
            if time.time() - self.state_time < 3.5:
                return
            self.get_logger().info("Releasing book into bin...")
            self.save_annotated_image("book_released")
            self.send_gripper(0.04, 1.5)
            self.state = "DONE"
            self.state_time = time.time()

        elif self.state == "DONE":
            if time.time() - self.state_time < 0.5:
                return
            self.get_logger().info("=== COMPLETE ===")
            raise SystemExit(0)

def main(args=None):
    rclpy.init(args=args)
    node = GraspController()
    try:
        rclpy.spin(node)
    except SystemExit as e:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()