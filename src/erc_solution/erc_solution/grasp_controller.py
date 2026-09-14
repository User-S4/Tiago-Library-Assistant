#!/usr/bin/env python3

import statistics
from collections import deque

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32, String


class GraspController(Node):

    def __init__(self):
        super().__init__('grasp_controller')

        self.points = deque(maxlen=10)

        self.row = None
        self.nav_state = None

        self.create_subscription(
            PointStamped,
            '/erc/target_book_point',
            self.on_book,
            10
        )

        self.create_subscription(
            Int32,
            '/erc/shelf_row_identification',
            self.on_row,
            10
        )

        self.create_subscription(
            String,
            '/column_navigator/state',
            self.on_nav_state,
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.report
        )

        self.get_logger().info(
            'Grasp controller ready — READ ONLY, robot will not move.'
        )

    def on_book(self, msg):

        # We only want target positions expressed in base_link.
        if msg.header.frame_id not in ('base_link', '/base_link'):
            self.get_logger().warning(
                f'Ignoring target in frame {msg.header.frame_id}'
            )
            return

        self.points.append((
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z)
        ))

    def on_row(self, msg):
        self.row = int(msg.data)

    def on_nav_state(self, msg):
        self.nav_state = msg.data

    def report(self):

        if not self.points:
            self.get_logger().info(
                f'Waiting for target... row={self.row} '
                f'nav={self.nav_state}'
            )
            return

        xs = [p[0] for p in self.points]
        ys = [p[1] for p in self.points]
        zs = [p[2] for p in self.points]

        x = statistics.median(xs)
        y = statistics.median(ys)
        z = statistics.median(zs)

        self.get_logger().info(
            '\n'
            '========== VISION GRASP TARGET ==========\n'
            f'nav state : {self.nav_state}\n'
            f'row       : {self.row}\n'
            f'samples   : {len(self.points)}\n'
            f'x         : {x:.4f} m\n'
            f'y         : {y:+.4f} m\n'
            f'z         : {z:.4f} m\n'
            '========================================='
        )


def main(args=None):
    rclpy.init(args=args)

    node = GraspController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
