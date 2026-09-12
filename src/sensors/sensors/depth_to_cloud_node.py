#!/usr/bin/env python3
"""Depth-camera point cloud for the ERC head RGBD sensor.

The Gazebo ``rgbd_camera`` advertises ``/head_front_camera/depth/points`` but
never actually fills it, so RViz's DepthCloud and any downstream node get an
empty depth cloud. This node rebuilds it the standard way: back-project every
valid pixel of the float32 depth image through the camera intrinsics and colour
it with the aligned RGB image, publishing a normal ``sensor_msgs/PointCloud2``.

Depth and colour cameras share intrinsics (fx=fy≈348, cx=320, cy=240) and sit
~1.5 cm apart, so pixel (u, v) pairs directly with negligible parallax at
shelf range. Output is in the depth optical frame, ready for RViz PointCloud2.
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
]
_FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class DepthToCloud(Node):

    def __init__(self):
        super().__init__('depth_to_cloud')
        self.depth_topic = self.declare_parameter(
            'depth_topic', '/head_front_camera/head_front_camera/depth/image_rect_raw').value
        self.info_topic = self.declare_parameter(
            'info_topic', '/head_front_camera/head_front_camera/depth/camera_info').value
        self.color_topic = self.declare_parameter(
            'color_topic', '/head_front_camera/head_front_camera/color/image_raw').value
        self.cloud_topic = self.declare_parameter(
            'cloud_topic', '/head_front_camera/head_front_camera/depth/color/points').value
        self.fov_topic = self.declare_parameter(
            'fov_topic', '/head_front_camera/depth/fov').value
        # how far out to draw the FoV frustum lines (m)
        self.fov_range = float(self.declare_parameter('fov_range', 4.0).value)
        self.stride = int(self.declare_parameter('stride', 2).value)
        self.min_range = float(self.declare_parameter('min_range', 0.2).value)
        self.max_range = float(self.declare_parameter('max_range', 8.0).value)

        # How to colour the cloud. A real depth camera outputs geometry only, so
        # the /depth/points cloud is NOT RGB-textured — it is shown coloured by
        # distance (like the RealSense Viewer's depth stream). Options:
        #   'depth' (default) — turbo colour-map over [color_min, color_max] metres
        #   'rgb'             — the aligned colour image (the fused RGBD product,
        #                       which a real sensor exposes only on /depth/color/points)
        #   'none'            — no colour field, XYZ only (RViz colours it by axis)
        self.color_source = str(
            self.declare_parameter('color_source', 'depth').value).lower()
        self.color_min = float(self.declare_parameter('color_min', 0.3).value)
        self.color_max = float(self.declare_parameter('color_max', 5.0).value)

        # ── RealSense D435-like noise model (applied to the depth image) ──
        # A perfect sim depth is the giveaway that a cloud is fake. A stereo depth
        # camera's error is dominated by sub-pixel disparity: converting depth to
        # disparity d = f*B/Z, quantising d, adding disparity noise and converting
        # back reproduces both the characteristic range-dependent noise (grows ~Z^2)
        # and the tell-tale quantisation "shells". On top of that, boundaries lose
        # points (occlusion / flying pixels) and a few pixels drop at random.
        self.rs_noise = bool(self.declare_parameter('realsense_noise', True).value)
        self.baseline = float(self.declare_parameter('baseline', 0.05).value)      # m
        self.subpixel = int(self.declare_parameter('subpixel_bits', 8).value)      # 1/N px steps
        self.disp_sigma = float(self.declare_parameter('disparity_sigma', 0.09).value)  # px
        self.edge_thresh = float(self.declare_parameter('edge_hole_thresh', 0.04).value)  # m
        self.random_drop = float(self.declare_parameter('random_drop', 0.015).value)  # fraction
        self.rng = np.random.default_rng()

        self.bridge = CvBridge()
        self.k = None                # 3x3 intrinsics
        self.color = None            # latest rgb8 image (H, W, 3)

        self.pub = self.create_publisher(
            PointCloud2, self.cloud_topic, qos_profile_sensor_data)
        self.pub_fov = self.create_publisher(Marker, self.fov_topic, 10)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, 10)
        self.create_subscription(Image, self.color_topic, self.on_color,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            f"depth_to_cloud: {self.depth_topic} + {self.color_topic} "
            f"-> {self.cloud_topic} (stride={self.stride})")

    def on_info(self, msg: CameraInfo):
        self.k = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def on_color(self, msg: Image):
        try:
            self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"color cv_bridge failed: {e}")

    def on_depth(self, msg: Image):
        if self.k is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"depth cv_bridge failed: {e}")
            return

        depth = np.asarray(depth, dtype=np.float32)
        if self.rs_noise:
            depth = self._apply_realsense_noise(depth)
        h, w = depth.shape[:2]
        self.pub_fov.publish(self._fov_marker(w, h, msg.header.frame_id))
        s = max(1, self.stride)
        d = depth[::s, ::s]
        vv, uu = np.mgrid[0:h:s, 0:w:s]
        valid = (np.isfinite(d) & (d > self.min_range) & (d < self.max_range))
        z = d[valid].astype(np.float32)
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        if self.color_source == 'none':
            self.pub.publish(self._make_cloud_xyz(x, y, z, msg.header))
            return

        color = self.color
        if (self.color_source == 'rgb' and color is not None
                and color.shape[0] == h and color.shape[1] == w):
            csub = color[::s, ::s][valid]  # N x 3 (r, g, b)
            r = csub[:, 0].astype(np.uint32)
            g = csub[:, 1].astype(np.uint32)
            b = csub[:, 2].astype(np.uint32)
            rgb = (r << 16) | (g << 8) | b
        else:  # 'depth' — colour by distance (real depth-camera style)
            rgb = self._depth_colormap(z)

        self.pub.publish(self._make_cloud(x, y, z, rgb, msg.header))

    def _fov_marker(self, w, h, frame):
        """LINE_LIST frustum showing the depth camera's field of view.

        Rays from the optical origin to the four image corners at ``fov_range``,
        plus the rectangle where they cut that plane — a wireframe of exactly the
        volume the point cloud can cover.
        """
        fx, fy = self.k[0, 0], self.k[1, 1]
        cx, cy = self.k[0, 2], self.k[1, 2]
        d = self.fov_range

        def corner(u, v):
            return Point(x=float((u - cx) * d / fx),
                         y=float((v - cy) * d / fy), z=float(d))

        o = Point(x=0.0, y=0.0, z=0.0)
        tl, tr, br, bl = corner(0, 0), corner(w, 0), corner(w, h), corner(0, h)

        m = Marker()
        m.header.frame_id = frame  # stamp 0 -> latest TF (no flicker)
        m.ns = 'depth_fov'
        m.id = 0
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.01  # line width (m)
        m.color = ColorRGBA(r=0.1, g=0.9, b=1.0, a=0.8)  # cyan
        m.pose.orientation.w = 1.0
        m.points = [o, tl, o, tr, o, br, o, bl,          # four edges from origin
                    tl, tr, tr, br, br, bl, bl, tl]      # far-plane rectangle
        m.lifetime = Duration(sec=0, nanosec=0)
        return m

    def _depth_colormap(self, z):
        """Turbo colour-map of range → packed RGB (how a depth stream is shown)."""
        span = max(1e-6, self.color_max - self.color_min)
        t = np.clip((z - self.color_min) / span, 0.0, 1.0)
        idx = (t * 255.0).astype(np.uint8).reshape(-1, 1)
        bgr = cv2.applyColorMap(idx, cv2.COLORMAP_TURBO).reshape(-1, 3)
        r = bgr[:, 2].astype(np.uint32)
        g = bgr[:, 1].astype(np.uint32)
        b = bgr[:, 0].astype(np.uint32)
        return (r << 16) | (g << 8) | b

    def _apply_realsense_noise(self, depth):
        """Turn a clean sim depth image into a RealSense-D435-like one.

        Steps (all vectorised): map depth Z to stereo disparity d = f*B/Z, quantise
        d to sub-pixel steps (the "shells"), add Gaussian disparity noise (→ error
        that grows ~Z^2), map back to Z, then punch holes at depth discontinuities
        (occlusion / flying pixels) and a small random fraction everywhere.
        """
        f = float(self.k[0, 0])           # depth focal length in px (current res)
        b = self.baseline
        valid = np.isfinite(depth) & (depth > 0.0)
        out = np.full_like(depth, np.inf)

        z = depth[valid]
        if z.size:
            disp = f * b / z
            if self.subpixel > 0:
                disp = np.round(disp * self.subpixel) / self.subpixel
            if self.disp_sigma > 0:
                disp = disp + self.rng.normal(0.0, self.disp_sigma, disp.shape)
            disp = np.maximum(disp, 1e-3)
            out[valid] = (f * b / disp).astype(np.float32)

        # edge holes: drop pixels straddling a depth step (finite-difference grad)
        if self.edge_thresh > 0:
            safe = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
            gx = np.abs(np.diff(safe, axis=1, prepend=safe[:, :1]))
            gy = np.abs(np.diff(safe, axis=0, prepend=safe[:1, :]))
            out[(gx > self.edge_thresh) | (gy > self.edge_thresh)] = np.inf

        # sprinkle of random invalid pixels
        if self.random_drop > 0:
            out[self.rng.random(out.shape) < self.random_drop] = np.inf
        return out

    @staticmethod
    def _make_cloud(x, y, z, rgb, header):
        n = int(z.shape[0])
        structured = np.zeros(
            n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
        structured['x'] = x
        structured['y'] = y
        structured['z'] = z
        structured['rgb'] = rgb
        msg = PointCloud2()
        msg.header = header  # depth optical frame + sensor stamp
        msg.height = 1
        msg.width = n
        msg.fields = _FIELDS
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        msg.data = structured.tobytes()
        return msg

    @staticmethod
    def _make_cloud_xyz(x, y, z, header):
        """Geometry-only cloud (no colour field) — a raw depth camera's output."""
        n = int(z.shape[0])
        structured = np.zeros(
            n, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
        structured['x'] = x
        structured['y'] = y
        structured['z'] = z
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = n
        msg.fields = _FIELDS_XYZ
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * n
        msg.is_dense = True
        msg.data = structured.tobytes()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = DepthToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
