#!/usr/bin/env python3
"""
Book colour + shelf row detection for ERC 2026 Phase 1.

Rows are inferred from the books themselves rather than from the shelf boards.

The competition guarantees each column holds exactly four books, one of each
colour, one per row, occupying rows 2 to 5 of a six-row unit. That makes the
books their own row markers: find all four in a column, sort them top to bottom,
and their rows are 2, 3, 4, 5 by definition.

This beats detecting the boards. Boards are thin, low contrast, and appear and
disappear with viewing angle; a single missed board shifts every row below it.
Books are large, saturated, and already detected reliably.

Columns are separated by looking for gaps in the horizontal spacing rather than
by growing clusters outward from each book. Growing clusters chains: book A is
near B, B is near C, and the whole shelf merges into one column of twenty. Real
column boundaries show up as spacings several times larger than the jitter within
a column, so splitting at the large gaps recovers them regardless of how far away
the shelf is or how many columns are in frame.

Publishes:
  /erc/shelf_row_identification  (Int32)   row 2-5 of the target book
  ~/annotated                    (Image)   debug view with all books and rows
  ~/mask                         (Image)   binary mask of the target colour
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# OpenCV hue runs 0-179, not 0-359. Red straddles the wraparound, so it needs two
# ranges OR'd together or you lose half the red pixels.
COLOUR_RANGES = {
    "red": [((0, 110, 60), (10, 255, 255)), ((168, 110, 60), (179, 255, 255))],
    "green": [((38, 70, 40), (88, 255, 255))],
    "blue": [((98, 110, 40), (132, 255, 255))],
    "yellow": [((20, 110, 110), (35, 255, 255))],
}

DRAW_BGR = {
    "red": (0, 0, 255),
    "green": (0, 170, 0),
    "blue": (255, 0, 0),
    "yellow": (0, 200, 200),
}

# Books occupy rows 2-5 of the six-row unit, top to bottom.
FIRST_BOOK_ROW = 2
BOOKS_PER_COLUMN = 4


class BookRowDetector(Node):
    def __init__(self):
        super().__init__("book_colour_detector")

        self.declare_parameter("book_colour", "red")
        self.declare_parameter(
            "image_topic", "/head_front_camera/head_front_camera/color/image_raw"
        )
        self.declare_parameter(
            "depth_topic", "/head_front_camera/head_front_camera/depth/image_rect_raw"
        )
        self.declare_parameter("image_dir", "erc_images")

        # --- blob filters -------------------------------------------------
        self.declare_parameter("min_area", 8)
        self.declare_parameter("min_aspect_ratio", 1.0)   # height / width
        self.declare_parameter("max_width_frac", 0.10)    # of image width

        # --- column splitting -------------------------------------------------
        # A gap this many times the typical within-column spacing marks a column
        # boundary. Scale-free, so it works at any distance from the shelf.
        self.declare_parameter("column_gap_ratio", 2.2)
        # Floor on the gap, as a fraction of image width, so tiny jitter between
        # two vertically stacked books never reads as a boundary.
        self.declare_parameter("min_column_gap_frac", 0.02)
        # Publish only when the chosen column shows the full set. A partial
        # column means books are hidden and the numbering would be a guess.
        self.declare_parameter("require_full_column", True)
        self.declare_parameter("first_book_row", FIRST_BOOK_ROW)

        self.declare_parameter("min_depth_m", 0.0)
        self.declare_parameter("max_depth_m", 0.0)
        self.declare_parameter("save_period_sec", 2.0)

        self.colour = str(self.get_parameter("book_colour").value).lower()
        if self.colour not in COLOUR_RANGES:
            raise ValueError(
                f"book_colour must be one of {sorted(COLOUR_RANGES)}, got '{self.colour}'"
            )

        self.image_dir = self.get_parameter("image_dir").value
        os.makedirs(self.image_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.latest_depth = None

        self.row_pub = self.create_publisher(Int32, "/erc/shelf_row_identification", 10)
        self.annotated_pub = self.create_publisher(Image, "~/annotated", 1)
        self.mask_pub = self.create_publisher(Image, "~/mask", 1)

        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, 1
        )
        self.create_subscription(
            Image, self.get_parameter("depth_topic").value, self.on_depth, 1
        )

        self.frames_seen = 0
        self.frames_full_column = 0
        self.row_histogram = {}
        self.last_save = None

        self.get_logger().info(
            f"Target colour {self.colour}. Rows inferred from book order, "
            f"{BOOKS_PER_COLUMN} books per column starting at row "
            f"{self.get_parameter('first_book_row').value}."
        )

    # ------------------------------------------------------------------ depth

    def on_depth(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception:                              # noqa: BLE001
            self.latest_depth = None

    def depth_at(self, x, y, w, h):
        """Median depth in metres over a detection, or None when unavailable."""
        depth = self.latest_depth
        if depth is None:
            return None
        dh, dw = depth.shape[:2]
        x0, x1 = max(0, x), min(dw, x + w)
        y0, y1 = max(0, y), min(dh, y + h)
        if x1 <= x0 or y1 <= y0:
            return None
        patch = np.asarray(depth[y0:y1, x0:x1], dtype=np.float32)
        patch = patch[np.isfinite(patch) & (patch > 0.0)]
        if patch.size == 0:
            return None
        value = float(np.median(patch))
        return value / 1000.0 if value > 100.0 else value

    # ------------------------------------------------------------ colour blobs

    @staticmethod
    def mask_for(hsv, colour):
        mask = None
        for lo, hi in COLOUR_RANGES[colour]:
            part = cv2.inRange(hsv, np.array(lo), np.array(hi))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        # Close vertical gaps in a spine without destroying thin blobs. A square
        # kernel would erase books only 2-3 pixels wide.
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 1), np.uint8))

    def find_books(self, hsv, image_width):
        min_area = self.get_parameter("min_area").value
        min_aspect = self.get_parameter("min_aspect_ratio").value
        max_width = image_width * self.get_parameter("max_width_frac").value

        books = []
        for colour in COLOUR_RANGES:
            mask = self.mask_for(hsv, colour)
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for c in contours:
                x, y, w, h = cv2.boundingRect(c)
                area = w * h
                if area < min_area or w == 0 or w > max_width:
                    continue
                if (h / float(w)) < min_aspect:
                    continue
                books.append({
                    "colour": colour,
                    "box": (x, y, w, h),
                    "cx": x + w / 2.0,
                    "cy": y + h / 2.0,
                })
        return books

    # ---------------------------------------------------------- column logic

    def split_into_columns(self, books, image_width):
        """Split books into columns at unusually large horizontal gaps.

        Within a column, book centres differ only by a few pixels of perspective
        jitter. Between columns the step is much larger. Comparing each gap to
        the median gap finds the boundaries without needing to know the distance
        to the shelf or how many columns are visible.
        """
        if len(books) < 2:
            return [books] if books else []

        by_x = sorted(books, key=lambda b: b["cx"])
        gaps = [by_x[i + 1]["cx"] - by_x[i]["cx"] for i in range(len(by_x) - 1)]

        positive = [g for g in gaps if g > 0.5]
        typical = float(np.median(positive)) if positive else 1.0
        threshold = max(
            typical * self.get_parameter("column_gap_ratio").value,
            image_width * self.get_parameter("min_column_gap_frac").value,
        )

        columns, current = [], [by_x[0]]
        for book, gap in zip(by_x[1:], gaps):
            if gap > threshold:
                columns.append(current)
                current = [book]
            else:
                current.append(book)
        columns.append(current)
        return columns

    def choose_column(self, columns, image_width):
        """The column nearest frame centre: the robot centred on its target.

        Prefers complete columns, since a four-book column near the centre beats
        a stray detection that happens to sit closer in.
        """
        if not columns:
            return None
        centre = image_width / 2.0

        def distance(column):
            mean_x = sum(b["cx"] for b in column) / len(column)
            return abs(mean_x - centre)

        complete = [c for c in columns if len(c) == BOOKS_PER_COLUMN]
        return min(complete or columns, key=distance)

    def assign_rows(self, column):
        """Sort top to bottom and number from first_book_row downward."""
        first = self.get_parameter("first_book_row").value
        ordered = sorted(column, key=lambda b: b["cy"])
        for index, book in enumerate(ordered):
            book["row"] = first + index
        return ordered

    def depth_ok(self, distance):
        if distance is None:
            return True                    # no depth yet: do not reject on it
        lo = self.get_parameter("min_depth_m").value
        hi = self.get_parameter("max_depth_m").value
        if lo > 0.0 and distance < lo:
            return False
        if hi > 0.0 and distance > hi:
            return False
        return True

    # ---------------------------------------------------------------- callback

    def on_image(self, msg):
        self.frames_seen += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f"Could not convert frame: {exc}")
            return

        width = frame.shape[1]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        books = self.find_books(hsv, width)
        for book in books:
            book["depth"] = self.depth_at(*book["box"])
        books = [b for b in books if self.depth_ok(b["depth"])]

        columns = self.split_into_columns(books, width)
        chosen = self.choose_column(columns, width)

        target_row = None
        if chosen:
            ordered = self.assign_rows(chosen)
            complete = len(ordered) == BOOKS_PER_COLUMN
            if complete:
                self.frames_full_column += 1
            if complete or not self.get_parameter("require_full_column").value:
                for book in ordered:
                    if book["colour"] == self.colour:
                        target_row = book["row"]
                        break

        if target_row is not None:
            self.row_histogram[target_row] = self.row_histogram.get(target_row, 0) + 1
            self.row_pub.publish(Int32(data=int(target_row)))

        annotated = self.draw(frame, columns, chosen, target_row)
        self.publish_debug(annotated, self.mask_for(hsv, self.colour))
        self.maybe_save(annotated, target_row is not None)

    def draw(self, frame, columns, chosen, target_row):
        annotated = frame.copy()

        for column in columns:
            is_chosen = chosen is not None and column is chosen
            for book in column:
                x, y, w, h = book["box"]
                colour = DRAW_BGR[book["colour"]]
                if not is_chosen:
                    # Dim the columns we are not reading, so the chosen one is
                    # unmistakable in the saved images.
                    colour = tuple(int(c * 0.4) for c in colour)
                cv2.rectangle(annotated, (x, y), (x + w, y + h), colour,
                              2 if is_chosen else 1)
                if is_chosen:
                    label = f"r{book.get('row', '?')}"
                    if book.get("depth") is not None:
                        label += f" {book['depth']:.1f}m"
                    cv2.putText(
                        annotated, label, (x, max(12, y - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1, cv2.LINE_AA,
                    )

        if chosen:
            xs = [b["box"][0] for b in chosen]
            xe = [b["box"][0] + b["box"][2] for b in chosen]
            ys = [b["box"][1] for b in chosen]
            ye = [b["box"][1] + b["box"][3] for b in chosen]
            cv2.rectangle(annotated, (min(xs) - 8, min(ys) - 8),
                          (max(xe) + 8, max(ye) + 8), (255, 255, 255), 1)

        found = len(chosen) if chosen else 0
        sizes = "/".join(str(len(c)) for c in columns) if columns else "-"
        full_pct = (100.0 * self.frames_full_column / self.frames_seen
                    if self.frames_seen else 0.0)
        cv2.putText(
            annotated,
            f"{self.colour} -> row {target_row if target_row else '?'} | "
            f"chosen {found}/{BOOKS_PER_COLUMN} | columns {sizes}",
            (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"full column in {full_pct:.0f}% of {self.frames_seen} frames",
            (6, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA,
        )
        return annotated

    def publish_debug(self, annotated, mask):
        try:
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        except Exception:                              # noqa: BLE001
            pass

    def maybe_save(self, annotated, had_target):
        if not had_target:
            return
        now = self.get_clock().now()
        period = self.get_parameter("save_period_sec").value
        if self.last_save is not None:
            if (now - self.last_save).nanoseconds < period * 1e9:
                return
        self.last_save = now

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        path = os.path.join(self.image_dir, f"book_{self.colour}_{stamp}.png")
        try:
            cv2.imwrite(path, annotated)
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f"Could not save {path}: {exc}")

    def destroy_node(self):
        # Printed on Ctrl+C. Report-ready row identification statistics.
        if self.frames_seen:
            self.get_logger().info(
                f"Full column visible in "
                f"{100.0 * self.frames_full_column / self.frames_seen:.0f}% "
                f"of {self.frames_seen} frames"
            )
        if self.row_histogram:
            rows = ", ".join(
                f"row {k}: {v}" for k, v in sorted(self.row_histogram.items())
            )
            self.get_logger().info(f"{self.colour} rows published: {rows}")
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