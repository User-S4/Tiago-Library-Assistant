#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class GraspController(Node):
    def __init__(self):
        super().__init__('grasp_controller')
        self.arm_side = 'left'
        self.arm_joints = [
            'arm_left_1_joint', 'arm_left_2_joint', 'arm_left_3_joint',
            'arm_left_4_joint', 'arm_left_5_joint', 'arm_left_6_joint',
            'arm_left_7_joint',
        ]
        self.home_position = [0.36, 1.83, 0.47, -2.35, 0.0, -1.2, 0.0]

        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_left_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_left_controller_raw/follow_joint_trajectory'
        )

        self.row_id_pub = self.create_publisher(Int32, '/erc/shelf_row_identification', 10)
        self.col_id_pub = self.create_publisher(Int32, '/erc/shelf_column_identification', 10)
        self.book_pose_sub = self.create_subscription(
            PointStamped, '/erc/target_book_point', self.on_book_pose, 10
        )
        self.camera_sub = self.create_subscription(
            Image, '/head_front_camera/head_front_camera/color/image_raw', 
            self.on_camera, 10
        )

        self.latest_book_pose = None
        self.latest_image = None
        self.detected_row = None
        self.detected_col = None
        self.bridge = CvBridge()
        self.state = 'WAITING'
        self.wait_count = 0
        self.state_time = time.time()
        
        # Create output directory for images
        os.makedirs('/erc_images', exist_ok=True)
        
        self.create_timer(0.5, self.state_machine)
        self.get_logger().info('GraspController initialized')

    def detect_row_from_height(self, z_pos):
        """Detect which row based on Z position (height from base)."""
        if z_pos > 1.0:
            return 2
        elif z_pos > 0.5:
            return 3
        elif z_pos > 0.2:
            return 4
        else:
            return 5

    def detect_column_from_position(self, x_pos):
        """Detect which column based on X position (left-right from robot)."""
        # Assuming columns 1-4 across the shelf
        if x_pos < 1.5:
            return 1
        elif x_pos < 2.5:
            return 2
        elif x_pos < 3.5:
            return 3
        else:
            return 4

    def on_book_pose(self, msg):
        self.latest_book_pose = msg
        row = self.detect_row_from_height(msg.point.z)
        col = self.detect_column_from_position(msg.point.x)
        self.detected_row = row
        self.detected_col = col
        self.get_logger().info(
            f'Book: x={msg.point.x:.2f} y={msg.point.y:.2f} z={msg.point.z:.2f} (Row {row}, Col {col})'
        )

    def on_camera(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pass

    def save_annotated_image(self, filename_prefix):
        """Save current camera image with timestamp."""
        if self.latest_image is None:
            return
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        filename = f'/erc_images/{filename_prefix}_{timestamp}.jpg'
        cv2.imwrite(filename, self.latest_image)
        self.get_logger().info(f'Saved: {filename}')

    def send_arm_trajectory(self, positions, duration=2.0):
        if not self.arm_client.server_is_ready():
            return False
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
        return True

    def send_gripper(self, position=0.0, duration=1.0):
        if not self.gripper_client.server_is_ready():
            return False
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)
        traj = JointTrajectory()
        traj.joint_names = ['gripper_left_finger_joint']
        traj.points = [point]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.gripper_client.send_goal_async(goal)
        return True

    def state_machine(self):
        if self.state == 'WAITING':
            self.wait_count += 1
            if self.latest_book_pose is not None:
                self.get_logger().info('=== GRASP SEQUENCE START ===')
                self.state = 'SAVE_INITIAL'
                self.state_time = time.time()
            elif self.wait_count >= 60:
                self.get_logger().error('TIMEOUT: No book detected')
                self.state = 'DONE'

        elif self.state == 'SAVE_INITIAL':
            self.save_annotated_image('book_detected')
            self.state = 'PUBLISH_IDS'
            self.state_time = time.time()

        elif self.state == 'PUBLISH_IDS':
            if self.detected_row is not None:
                row_msg = Int32()
                row_msg.data = self.detected_row
                self.row_id_pub.publish(row_msg)
            if self.detected_col is not None:
                col_msg = Int32()
                col_msg.data = self.detected_col
                self.col_id_pub.publish(col_msg)
                self.get_logger().info(f'Published: Row {self.detected_row}, Col {self.detected_col}')
            self.state = 'MOVING_TO_GRASP'
            self.state_time = time.time()

        elif self.state == 'MOVING_TO_GRASP':
            if time.time() - self.state_time < 0.5:
                return
            self.get_logger().info('Moving to grasp...')
            self.send_arm_trajectory([0.5, 1.5, 0.5, -1.5, 0.0, -1.0, 0.0], 3.0)
            self.state = 'CLOSING_GRIPPER'
            self.state_time = time.time()

        elif self.state == 'CLOSING_GRIPPER':
            if time.time() - self.state_time < 3.5:
                return
            self.get_logger().info('Closing gripper...')
            self.save_annotated_image('gripper_closing')
            self.send_gripper(0.0, 1.0)
            self.state = 'RETRACTING'
            self.state_time = time.time()

        elif self.state == 'RETRACTING':
            if time.time() - self.state_time < 1.5:
                return
            self.get_logger().info('Retracting...')
            self.save_annotated_image('gripper_closed')
            self.send_arm_trajectory(self.home_position, 3.0)
            self.state = 'OPENING_GRIPPER'
            self.state_time = time.time()

        elif self.state == 'OPENING_GRIPPER':
            if time.time() - self.state_time < 3.5:
                return
            self.get_logger().info('Opening gripper...')
            self.save_annotated_image('placing')
            self.send_gripper(0.04, 1.0)
            self.state = 'DONE'
            self.state_time = time.time()

        elif self.state == 'DONE':
            if time.time() - self.state_time < 1.5:
                return
            self.get_logger().info('=== COMPLETE ===')
            raise SystemExit(0)

def main(args=None):
    rclpy.init(args=args)
    node = GraspController()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
