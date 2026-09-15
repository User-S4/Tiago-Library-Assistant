#!/usr/bin/env python3
"""
Book colour + shelf row + 3D target-book detection for ERC 2026 Phase 1.

Preserves the existing row detector and adds:

  /erc/target_book_point  (geometry_msgs/PointStamped)

The target point is expressed in base_link.

RGB and depth optical frames are parallel but their origins are separated by
15 mm along optical X. The target RGB pixel is therefore registered into the
depth image before the depth value is sampled.

The final depth-camera XYZ point is transformed dynamically into base_link
using TF2, so head movement does not require hard-coded base/camera geometry.
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformListener


# OpenCV hue is 0-179.
COLOUR_RANGES = {
    "red": [
        ((0, 110, 60), (10, 255, 255)),
        ((168, 110, 60), (179, 255, 255)),
    ],
    "green": [
        ((38, 70, 40), (88, 255, 255)),
    ],
    "blue": [
        ((98, 110, 40), (132, 255, 255)),
    ],
    "yellow": [
        ((20, 110, 110), (35, 255, 255)),
    ],
}

DRAW_BGR = {
    "red": (0, 0, 255),
    "green": (0, 170, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 200, 200),
}

FIRST_BOOK_ROW = 1
BOOKS_PER_COLUMN = 4


class BookRowDetector(Node):

    def __init__(self):
        super().__init__("book_colour_detector")

        # -------------------------------------------------------------- parameters

        self.declare_parameter("book_colour", "red")

        self.declare_parameter(
            "image_topic",
            "/head_front_camera/head_front_camera/color/image_raw",
        )

        self.declare_parameter(
            "depth_topic",
            "/head_front_camera/head_front_camera/depth/image_rect_raw",
        )

        self.declare_parameter(
            "color_info_topic",
            "/head_front_camera/head_front_camera/color/camera_info",
        )

        self.declare_parameter(
            "depth_info_topic",
            "/head_front_camera/head_front_camera/depth/camera_info",
        )

        self.declare_parameter("image_dir", "/erc_images")

        # Blob filters.
        self.declare_parameter("min_area", 8)
        self.declare_parameter("min_aspect_ratio", 1.0)
        self.declare_parameter("max_width_frac", 0.10)

        # Column splitting.
        self.declare_parameter("column_gap_ratio", 2.2)
        self.declare_parameter("min_column_gap_frac", 0.02)
        self.declare_parameter("require_full_column", True)
        self.declare_parameter("first_book_row", FIRST_BOOK_ROW)

        self.declare_parameter("min_depth_m", 0.0)
        self.declare_parameter("max_depth_m", 0.0)
        self.declare_parameter("save_period_sec", 2.0)

        # TF/output frame.
        self.declare_parameter("base_frame", "base_link")

        # Measured with:
        #
        # tf2_echo head_front_camera_color_optical_frame
        #          head_front_camera_depth_optical_frame
        #
        # Depth optical origin is +0.015 m in colour optical X.
        self.declare_parameter("depth_origin_in_color_x_m", 0.015)

        self.colour = str(
            self.get_parameter("book_colour").value
        ).lower()

        if self.colour not in COLOUR_RANGES:
            raise ValueError(
                f"book_colour must be one of "
                f"{sorted(COLOUR_RANGES)}, got '{self.colour}'"
            )

        self.image_dir = self.get_parameter("image_dir").value
        os.makedirs(self.image_dir, exist_ok=True)

        # --------------------------------------------------------------- state

        self.bridge = CvBridge()

        self.latest_depth = None
        self.latest_depth_scale = 1.0
        self.latest_depth_frame = (
            "head_front_camera_depth_optical_frame"
        )

        self.color_info = None
        self.depth_info = None

        self.frames_seen = 0
        self.frames_full_column = 0
        self.row_histogram = {}
        self.last_save = None

        self.last_xyz_log_ns = 0

        # --------------------------------------------------------------- TF

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        # --------------------------------------------------------------- publishers

        self.row_pub = self.create_publisher(
            Int32,
            "/erc/shelf_row_identification",
            10,
        )

        self.point_pub = self.create_publisher(
            PointStamped,
            "/erc/target_book_point",
            10,
        )

        self.annotated_pub = self.create_publisher(
            Image,
            "~/annotated",
            1,
        )

        self.mask_pub = self.create_publisher(
            Image,
            "~/mask",
            1,
        )

        # ------------------------------------------------------------- subscriptions

        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.on_image,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self.on_depth,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter("color_info_topic").value,
            self.on_color_info,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_info_topic").value,
            self.on_depth_info,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Target colour {self.colour}. "
            f"Rows inferred from book order, "
            f"{BOOKS_PER_COLUMN} books per column starting at row "
            f"{self.get_parameter('first_book_row').value}."
        )

        self.get_logger().info(
            "3D target output enabled on /erc/target_book_point"
        )

    # ================================================================= camera info

    def on_color_info(self, msg):
        self.color_info = msg

    def on_depth_info(self, msg):
        self.depth_info = msg

    # ====================================================================== depth

    def on_depth(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )

            self.latest_depth = depth
            self.latest_depth_frame = (
                msg.header.frame_id
                or "head_front_camera_depth_optical_frame"
            )

            encoding = str(msg.encoding).upper()

            if "16U" in encoding or "MONO16" in encoding:
                self.latest_depth_scale = 0.001
            else:
                self.latest_depth_scale = 1.0

        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Could not convert depth frame: {exc}"
            )
            self.latest_depth = None

    def depth_values_in_window(self, u, v, radius_x, radius_y):
        """Return valid depths in metres around a depth-image pixel."""

        depth = self.latest_depth

        if depth is None:
            return np.empty((0,), dtype=np.float32)

        dh, dw = depth.shape[:2]

        cx = int(round(float(u)))
        cy = int(round(float(v)))

        x0 = max(0, cx - int(radius_x))
        x1 = min(dw, cx + int(radius_x) + 1)

        y0 = max(0, cy - int(radius_y))
        y1 = min(dh, cy + int(radius_y) + 1)

        if x1 <= x0 or y1 <= y0:
            return np.empty((0,), dtype=np.float32)

        values = np.asarray(
            depth[y0:y1, x0:x1],
            dtype=np.float32,
        ).reshape(-1)

        values = values[np.isfinite(values)]
        values = values[values > 0.0]

        if values.size == 0:
            return values

        values = values * float(self.latest_depth_scale)

        # Generic sanity range.
        values = values[
            (values > 0.05) &
            (values < 20.0)
        ]

        if values.size == 0:
            return values

        lo = float(
            self.get_parameter("min_depth_m").value
        )

        hi = float(
            self.get_parameter("max_depth_m").value
        )

        if lo > 0.0:
            values = values[values >= lo]

        if hi > 0.0:
            values = values[values <= hi]

        return values

    def median_depth_window(self, u, v, radius_x=1, radius_y=2):
        values = self.depth_values_in_window(
            u,
            v,
            radius_x,
            radius_y,
        )

        if values.size == 0:
            return None

        return float(np.median(values))

    def foreground_depth_window(self, u, v, radius_x=8, radius_y=5):
        """
        Rough foreground depth.

        The lower percentile biases the estimate toward the book front rather
        than the shelf directly behind it.
        """

        values = self.depth_values_in_window(
            u,
            v,
            radius_x,
            radius_y,
        )

        if values.size == 0:
            return None

        return float(np.percentile(values, 20.0))

    def depth_at(self, x, y, w, h):
        """
        Existing bounding-box depth helper.

        Kept for compatibility with the original row detector/debug labels.
        """

        depth = self.latest_depth

        if depth is None:
            return None

        dh, dw = depth.shape[:2]

        x0 = max(0, int(x))
        x1 = min(dw, int(x + w))

        y0 = max(0, int(y))
        y1 = min(dh, int(y + h))

        if x1 <= x0 or y1 <= y0:
            return None

        patch = np.asarray(
            depth[y0:y1, x0:x1],
            dtype=np.float32,
        )

        patch = patch[
            np.isfinite(patch) &
            (patch > 0.0)
        ]

        if patch.size == 0:
            return None

        value = float(np.median(patch))

        return (
            value / 1000.0
            if value > 100.0
            else value
        )

    # =========================================================== RGB/depth mapping

    @staticmethod
    def camera_intrinsics(info):
        if info is None:
            return None

        if len(info.k) != 9:
            return None

        fx = float(info.k[0])
        fy = float(info.k[4])
        cx = float(info.k[2])
        cy = float(info.k[5])

        if fx <= 0.0 or fy <= 0.0:
            return None

        return fx, fy, cx, cy

    def register_rgb_pixel_to_depth(self, u_rgb, v_rgb, depth_m):
        """
        Map one colour-image pixel to the depth image.

        Measured fixed transform:
            color_optical -> depth_optical
            translation = [+0.015, 0, 0]
            rotation = identity

        Therefore:
            X_depth = X_color - baseline_x
        """

        c = self.camera_intrinsics(self.color_info)
        d = self.camera_intrinsics(self.depth_info)

        if c is None or d is None:
            return None

        if depth_m is None or depth_m <= 0.0:
            return None

        fx_c, fy_c, cx_c, cy_c = c
        fx_d, fy_d, cx_d, cy_d = d

        baseline_x = float(
            self.get_parameter(
                "depth_origin_in_color_x_m"
            ).value
        )

        xn_color = (
            float(u_rgb) - cx_c
        ) / fx_c

        yn_color = (
            float(v_rgb) - cy_c
        ) / fy_c

        xn_depth = xn_color - baseline_x / depth_m

        u_depth = fx_d * xn_depth + cx_d
        v_depth = fy_d * yn_color + cy_d

        return u_depth, v_depth

    def registered_book_depth(self, book):
        """
        Estimate depth for the RGB target book.

        1. Get a rough foreground depth near the RGB centre.
        2. Use it to compute the stereo pixel shift.
        3. Sample a small patch at the registered depth pixel.
        4. Refine once using the newly measured depth.
        """

        u_rgb = float(book["cx"])
        v_rgb = float(book["cy"])

        _, _, _, h = book["box"]

        rough = self.foreground_depth_window(
            u_rgb,
            v_rgb,
            radius_x=8,
            radius_y=max(
                2,
                min(6, int(h // 5)),
            ),
        )

        if rough is None:
            return None

        best_depth = rough
        best_uv = None

        # Two iterations are enough because the baseline is only 15 mm.
        for _ in range(2):

            mapped = self.register_rgb_pixel_to_depth(
                u_rgb,
                v_rgb,
                best_depth,
            )

            if mapped is None:
                return None

            u_depth, v_depth = mapped

            # Test the calculated pixel and its immediate neighbours.
            # The nearest valid median is normally the book front.
            candidates = []

            for du in (-1.0, 0.0, 1.0):
                value = self.median_depth_window(
                    u_depth + du,
                    v_depth,
                    radius_x=1,
                    radius_y=max(
                        1,
                        min(3, int(h // 8)),
                    ),
                )

                if value is not None:
                    candidates.append(
                        (
                            value,
                            u_depth + du,
                            v_depth,
                        )
                    )

            if not candidates:
                return None

            candidates.sort(
                key=lambda item: item[0]
            )

            best_depth, best_u, best_v = candidates[0]
            best_uv = (best_u, best_v)

        if best_uv is None:
            return None

        return (
            float(best_depth),
            float(best_uv[0]),
            float(best_uv[1]),
        )

    # ================================================================ 3D geometry

    def depth_pixel_to_xyz(self, u_depth, v_depth, depth_m):
        """Back-project a depth-image pixel into the depth optical frame."""

        intrinsics = self.camera_intrinsics(
            self.depth_info
        )

        if intrinsics is None:
            return None

        fx, fy, cx, cy = intrinsics

        z = float(depth_m)
        x = (
            (float(u_depth) - cx)
            * z
            / fx
        )

        y = (
            (float(v_depth) - cy)
            * z
            / fy
        )

        return np.array(
            [x, y, z],
            dtype=np.float64,
        )

    @staticmethod
    def rotate_by_quaternion(vector, qx, qy, qz, qw):
        """Rotate a 3-vector using quaternion x,y,z,w."""

        q_norm = np.sqrt(
            qx * qx +
            qy * qy +
            qz * qz +
            qw * qw
        )

        if q_norm <= 1e-12:
            return vector.copy()

        qx /= q_norm
        qy /= q_norm
        qz /= q_norm
        qw /= q_norm

        xx = qx * qx
        yy = qy * qy
        zz = qz * qz

        xy = qx * qy
        xz = qx * qz
        yz = qy * qz

        wx = qw * qx
        wy = qw * qy
        wz = qw * qz

        rotation = np.array([
            [
                1.0 - 2.0 * (yy + zz),
                2.0 * (xy - wz),
                2.0 * (xz + wy),
            ],
            [
                2.0 * (xy + wz),
                1.0 - 2.0 * (xx + zz),
                2.0 * (yz - wx),
            ],
            [
                2.0 * (xz - wy),
                2.0 * (yz + wx),
                1.0 - 2.0 * (xx + yy),
            ],
        ])

        return rotation @ vector

    def transform_depth_xyz_to_base(self, xyz_depth):
        """Transform a depth-frame XYZ point into base_link."""

        target_frame = str(
            self.get_parameter("base_frame").value
        )

        source_frame = self.latest_depth_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )

        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"TF {source_frame} -> {target_frame} "
                f"not available: {exc}"
            )
            return None

        t = transform.transform.translation
        q = transform.transform.rotation

        rotated = self.rotate_by_quaternion(
            xyz_depth,
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )

        result = rotated + np.array(
            [
                float(t.x),
                float(t.y),
                float(t.z),
            ],
            dtype=np.float64,
        )

        return result

    def estimate_target_point(self, book, image_msg):
        """Return PointStamped for the detected target book."""

        registered = self.registered_book_depth(
            book
        )

        if registered is None:
            return None

        depth_m, u_depth, v_depth = registered

        xyz_depth = self.depth_pixel_to_xyz(
            u_depth,
            v_depth,
            depth_m,
        )

        if xyz_depth is None:
            return None

        xyz_base = self.transform_depth_xyz_to_base(
            xyz_depth
        )

        if xyz_base is None:
            return None

        point = PointStamped()

        point.header.stamp = image_msg.header.stamp
        point.header.frame_id = str(
            self.get_parameter("base_frame").value
        )

        point.point.x = float(xyz_base[0])
        point.point.y = float(xyz_base[1])
        point.point.z = float(xyz_base[2])

        book["registered_depth"] = depth_m
        book["u_depth"] = u_depth
        book["v_depth"] = v_depth
        book["xyz_base"] = xyz_base

        return point

    # ================================================================ colour blobs

    @staticmethod
    def mask_for(hsv, colour):
        mask = None

        for lo, hi in COLOUR_RANGES[colour]:
            part = cv2.inRange(
                hsv,
                np.array(lo),
                np.array(hi),
            )

            mask = (
                part
                if mask is None
                else cv2.bitwise_or(mask, part)
            )

        return cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((3, 1), np.uint8),
        )

    def find_books(self, hsv, image_width):

        min_area = self.get_parameter(
            "min_area"
        ).value

        min_aspect = self.get_parameter(
            "min_aspect_ratio"
        ).value

        max_width = (
            image_width
            * self.get_parameter(
                "max_width_frac"
            ).value
        )

        books = []

        for colour in COLOUR_RANGES:

            mask = self.mask_for(
                hsv,
                colour,
            )

            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            for contour in contours:

                x, y, w, h = cv2.boundingRect(
                    contour
                )

                area = w * h

                if (
                    area < min_area
                    or w == 0
                    or w > max_width
                ):
                    continue

                if (
                    h / float(w)
                ) < min_aspect:
                    continue

                books.append({
                    "colour": colour,
                    "box": (x, y, w, h),
                    "cx": x + w / 2.0,
                    "cy": y + h / 2.0,
                })

        return books

    # ================================================================ column logic

    def split_into_columns(self, books, image_width):

        if len(books) < 2:
            return [books] if books else []

        by_x = sorted(
            books,
            key=lambda b: b["cx"],
        )

        gaps = [
            by_x[i + 1]["cx"]
            - by_x[i]["cx"]
            for i in range(
                len(by_x) - 1
            )
        ]

        positive = [
            gap
            for gap in gaps
            if gap > 0.5
        ]

        typical = (
            float(np.median(positive))
            if positive
            else 1.0
        )

        threshold = max(
            typical
            * self.get_parameter(
                "column_gap_ratio"
            ).value,
            image_width
            * self.get_parameter(
                "min_column_gap_frac"
            ).value,
        )

        columns = []
        current = [by_x[0]]

        for book, gap in zip(
            by_x[1:],
            gaps,
        ):

            if gap > threshold:
                columns.append(current)
                current = [book]

            else:
                current.append(book)

        columns.append(current)

        return columns

    def choose_column(self, columns, image_width):

        if not columns:
            return None

        centre = image_width / 2.0

        def distance(column):
            mean_x = (
                sum(
                    b["cx"]
                    for b in column
                )
                / len(column)
            )
            return abs(
                mean_x - centre
            )

        # Navigation has already centred the requested shelf column.
        # Stay locked to the column nearest the image centre.
        #
        # Do NOT switch to another column merely because that neighbouring
        # column happens to have all four books visible. At close range the
        # target column can be partially clipped by the camera FOV.
        return min(
            columns,
            key=distance,
        )

    def assign_rows(self, column):

        first = self.get_parameter(
            "first_book_row"
        ).value

        ordered = sorted(
            column,
            key=lambda b: b["cy"],
        )

        for index, book in enumerate(
            ordered
        ):
            book["row"] = (
                first + index
            )

        return ordered

    def depth_ok(self, distance):

        if distance is None:
            return True

        lo = self.get_parameter(
            "min_depth_m"
        ).value

        hi = self.get_parameter(
            "max_depth_m"
        ).value

        if lo > 0.0 and distance < lo:
            return False

        if hi > 0.0 and distance > hi:
            return False

        return True

    # ==================================================================== callback

    def on_image(self, msg):

        self.frames_seen += 1

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

        except Exception as exc:
            self.get_logger().warn(
                f"Could not convert frame: {exc}"
            )
            return


        width = frame.shape[1]

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV,
        )


        # ------------------------------------------------------------
        # Detect every coloured book spine visible in the frame.
        # ------------------------------------------------------------

        books = self.find_books(
            hsv,
            width,
        )


        for book in books:

            book["depth"] = self.depth_at(
                *book["box"]
            )


        books = [
            b
            for b in books
            if self.depth_ok(
                b["depth"]
            )
        ]


        # Keep the old column split only for debug drawing.
        # It is NO LONGER used to choose the target.
        columns = self.split_into_columns(
            books,
            width,
        )


        chosen = None
        target_row = None
        target_book = None
        target_point = None


        # ============================================================
        # TARGET SELECTION
        #
        # Navigation has already centred the requested shelf column.
        # Therefore the target-colour book belonging to that shelf
        # column is the target-colour blob nearest image centre.
        #
        # This avoids grouping books by X, which is unreliable because
        # ERC randomises each book horizontally inside its shelf row.
        # ============================================================

        colour_books = [
            b
            for b in books
            if b["colour"] == self.colour
        ]


        if colour_books:

            image_centre = (
                width / 2.0
            )


            target_book = min(
                colour_books,
                key=lambda b: abs(
                    b["cx"]
                    - image_centre
                )
            )


            centre_error = (
                target_book["cx"]
                - image_centre
            )


            self.get_logger().info(
                f"CENTER-LOCK {self.colour}: "
                f"pixel=({target_book['cx']:.1f},"
                f"{target_book['cy']:.1f}) "
                f"dx={centre_error:+.1f}px",
                throttle_duration_sec=2.0,
            )


            # --------------------------------------------------------
            # Determine which old debug column contains this target.
            # This affects drawing only.
            # --------------------------------------------------------

            for column in columns:

                if any(
                    b is target_book
                    for b in column
                ):

                    chosen = column
                    break


            # ========================================================
            # ROW IDENTIFICATION
            #
            # All books lie on four fixed horizontal shelf rows.
            # Cluster the vertical centres of ALL visible coloured
            # books into those four row levels.
            # ========================================================

            if len(books) >= 4:

                y_values = np.array(
                    [
                        [float(b["cy"])]
                        for b in books
                    ],
                    dtype=np.float32,
                )


                criteria = (
                    cv2.TERM_CRITERIA_EPS
                    + cv2.TERM_CRITERIA_MAX_ITER,
                    50,
                    0.1,
                )


                _compactness, _labels, centres = cv2.kmeans(
                    y_values,
                    4,
                    None,
                    criteria,
                    10,
                    cv2.KMEANS_PP_CENTERS,
                )


                row_centres = sorted(
                    float(c[0])
                    for c in centres
                )


                nearest_index = min(
                    range(4),
                    key=lambda i: abs(
                        target_book["cy"]
                        - row_centres[i]
                    ),
                )


                first_row = int(
                    self.get_parameter(
                        "first_book_row"
                    ).value
                )


                target_row = (
                    first_row
                    + nearest_index
                )


                target_book["row"] = (
                    target_row
                )


                self.get_logger().info(
                    "ROW LEVELS: "
                    + ", ".join(
                        f"{v:.1f}"
                        for v in row_centres
                    )
                    + f" | target row={target_row}",
                    throttle_duration_sec=2.0,
                )


        # ============================================================
        # OFFICIAL ROW TOPIC
        # ============================================================

        if target_row is not None:

            self.row_histogram[
                target_row
            ] = (
                self.row_histogram.get(
                    target_row,
                    0,
                )
                + 1
            )


            self.row_pub.publish(
                Int32(
                    data=int(
                        target_row
                    )
                )
            )


        # ============================================================
        # 3D TARGET POINT
        # ============================================================

        if target_book is not None:

            target_point = (
                self.estimate_target_point(
                    target_book,
                    msg,
                )
            )


            if target_point is not None:

                self.point_pub.publish(
                    target_point
                )


                now_ns = (
                    self.get_clock()
                    .now()
                    .nanoseconds
                )


                if (
                    now_ns
                    - self.last_xyz_log_ns
                    > 1_000_000_000
                ):

                    self.last_xyz_log_ns = (
                        now_ns
                    )


                    depth_m = (
                        target_book.get(
                            "registered_depth"
                        )
                    )


                    self.get_logger().info(
                        f"TARGET {self.colour}: "
                        f"row={target_row} | "
                        f"RGB=({target_book['cx']:.1f},"
                        f"{target_book['cy']:.1f}) | "
                        f"depth={depth_m:.3f} m | "
                        f"base XYZ="
                        f"({target_point.point.x:.3f}, "
                        f"{target_point.point.y:.3f}, "
                        f"{target_point.point.z:.3f})"
                    )


        # ============================================================
        # DEBUG / SCORING IMAGE
        # ============================================================

        annotated = self.draw(
            frame,
            columns,
            chosen,
            target_row,
            target_book,
            target_point,
        )


        self.publish_debug(
            annotated,
            self.mask_for(
                hsv,
                self.colour,
            ),
        )


        self.maybe_save(
            annotated,
            target_row is not None,
        )

    # ======================================================================= draw

    def draw(
        self,
        frame,
        columns,
        chosen,
        target_row,
        target_book,
        target_point,
    ):

        annotated = frame.copy()

        for column in columns:

            is_chosen = (
                chosen is not None
                and column is chosen
            )

            for book in column:

                x, y, w, h = book["box"]

                colour = DRAW_BGR[
                    book["colour"]
                ]

                if not is_chosen:
                    colour = tuple(
                        int(c * 0.4)
                        for c in colour
                    )

                cv2.rectangle(
                    annotated,
                    (x, y),
                    (x + w, y + h),
                    colour,
                    2 if is_chosen else 1,
                )

                if is_chosen:

                    label = (
                        f"r"
                        f"{book.get('row', '?')}"
                    )

                    if (
                        book.get("depth")
                        is not None
                    ):
                        label += (
                            f" "
                            f"{book['depth']:.1f}m"
                        )

                    cv2.putText(
                        annotated,
                        label,
                        (
                            x,
                            max(12, y - 4),
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        colour,
                        1,
                        cv2.LINE_AA,
                    )

        if chosen:

            xs = [
                b["box"][0]
                for b in chosen
            ]

            xe = [
                b["box"][0]
                + b["box"][2]
                for b in chosen
            ]

            ys = [
                b["box"][1]
                for b in chosen
            ]

            ye = [
                b["box"][1]
                + b["box"][3]
                for b in chosen
            ]

            cv2.rectangle(
                annotated,
                (
                    min(xs) - 8,
                    min(ys) - 8,
                ),
                (
                    max(xe) + 8,
                    max(ye) + 8,
                ),
                (255, 255, 255),
                1,
            )

        # Target crosshair.
        if target_book is not None:

            u = int(
                round(
                    target_book["cx"]
                )
            )

            v = int(
                round(
                    target_book["cy"]
                )
            )

            cv2.drawMarker(
                annotated,
                (u, v),
                (255, 255, 255),
                cv2.MARKER_CROSS,
                14,
                2,
            )

        found = (
            len(chosen)
            if chosen
            else 0
        )

        sizes = (
            "/".join(
                str(len(c))
                for c in columns
            )
            if columns
            else "-"
        )

        full_pct = (
            100.0
            * self.frames_full_column
            / self.frames_seen
            if self.frames_seen
            else 0.0
        )

        cv2.putText(
            annotated,
            f"{self.colour} -> row "
            f"{target_row if target_row else '?'} | "
            f"chosen {found}/{BOOKS_PER_COLUMN} | "
            f"columns {sizes}",
            (6, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        cv2.putText(
            annotated,
            f"full column in "
            f"{full_pct:.0f}% of "
            f"{self.frames_seen} frames",
            (6, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

        if target_point is not None:

            xyz_text = (
                f"XYZ "
                f"{target_point.point.x:.2f}, "
                f"{target_point.point.y:.2f}, "
                f"{target_point.point.z:.2f} m"
            )

            cv2.putText(
                annotated,
                xyz_text,
                (6, 48),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        return annotated

    # ================================================================ debug output

    def publish_debug(self, annotated, mask):

        try:
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    annotated,
                    "bgr8",
                )
            )

            self.mask_pub.publish(
                self.bridge.cv2_to_imgmsg(
                    mask,
                    "mono8",
                )
            )

        except Exception:  # noqa: BLE001
            pass

    def maybe_save(self, annotated, had_target):

        if not had_target:
            return

        now = self.get_clock().now()

        period = self.get_parameter(
            "save_period_sec"
        ).value

        if self.last_save is not None:

            if (
                now - self.last_save
            ).nanoseconds < period * 1e9:
                return

        self.last_save = now

        stamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S_%f"
        )[:-3]

        path = os.path.join(
            self.image_dir,
            f"book_{self.colour}_{stamp}.png",
        )

        try:
            cv2.imwrite(
                path,
                annotated,
            )

        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"Could not save {path}: {exc}"
            )

    # =================================================================== shutdown

    def destroy_node(self):

        if self.frames_seen:

            self.get_logger().info(
                f"Full column visible in "
                f"{100.0 * self.frames_full_column / self.frames_seen:.0f}% "
                f"of {self.frames_seen} frames"
            )

        if self.row_histogram:

            rows = ", ".join(
                f"row {k}: {v}"
                for k, v
                in sorted(
                    self.row_histogram.items()
                )
            )

            self.get_logger().info(
                f"{self.colour} rows published: "
                f"{rows}"
            )

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = BookRowDetector()

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
