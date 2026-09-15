#!/usr/bin/env python3
"""
Shelf column number recognition for ERC 2026 Phase 1.

The five shelf columns each carry a number plaque above them, and the numbers are
shuffled on every simulation load. This node reads them and reports where the
target column actually is.

Recognition is template matching against the competition's own digit textures
(erc_description/models/number_marker/textures/*.png) rather than OCR. Those are
the exact images the simulator renders, so matching is both faster and more
reliable than any general text recogniser at this resolution.

Two filters keep coloured book spines from being mistaken for digits, which
matters because a distant book can out-score a distant plaque on shape alone:
  1. Saturation. Plaque digits are black; books are saturated colour.
  2. Plaque row. Real markers sit in a horizontal line above the shelf unit, so
     candidates far from that line are dropped.

Publishes:
  /erc/shelf_column_identification  (Int32)   the target column, once confidently seen
  ~/target_offset                   (Float32) target's horizontal position in frame,
                                              -1.0 far left, 0.0 centred, +1.0 far right.
                                              Navigation can steer on this directly.
  ~/annotated                       (Image)   debug view with boxes and scores
  ~/digit_mask                      (Image)   binary mask of candidate digit pixels

Frame-rate agnostic: processes whatever arrives, assumes no particular camera rate.
"""

import os
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32

TEXTURE_DIR = "/opt/erc_ws/src/erc_description/models/number_marker/textures"
VALID_DIGITS = (1, 2, 3, 4, 5)

MATCH_BGR = (0, 200, 0)
TARGET_BGR = (0, 140, 255)
WEAK_BGR = (140, 140, 140)


class ShelfColumnDetector(Node):
    def __init__(self):
        super().__init__("shelf_column_detector")

        self.declare_parameter("shelf_column_number", 1)
        self.declare_parameter(
            "image_topic", "/head_front_camera/head_front_camera/color/image_raw"
        )
        self.declare_parameter("texture_dir", TEXTURE_DIR)
        self.declare_parameter("image_dir", "/erc_images")

        # --- digit segmentation -------------------------------------------
        # Plaque digits are near black on a light grey plaque.
        self.declare_parameter("dark_threshold", 90)
        # A digit is achromatic. Book spines are strongly coloured, and at this
        # resolution a distant book can out-score a distant plaque on shape alone,
        # so this filter is what stops the wrong column being chosen.
        self.declare_parameter("max_saturation", 60)
        self.declare_parameter("min_digit_height_px", 8)
        self.declare_parameter("max_digit_height_px", 120)
        self.declare_parameter("min_digit_aspect", 0.6)   # height / width
        self.declare_parameter("max_digit_aspect", 3.0)
        # Markers sit above the shelves, so ignore the lower part of the frame.
        # 1.0 searches the whole image.
        self.declare_parameter("search_top_frac", 0.75)

        # --- plaque row -----------------------------------------------------
        # The five markers form a horizontal band. Once two or more candidates
        # agree on a height, anything well outside that band is not a marker.
        self.declare_parameter("use_plaque_row", True)
        self.declare_parameter("plaque_row_tolerance_frac", 0.12)   # of image height

        # --- matching -------------------------------------------------------
        self.declare_parameter("min_match_score", 0.45)
        # Publish only after the same digit wins this many frames running, which
        # stops a single bad frame from reporting the wrong column.
        self.declare_parameter("confirm_frames", 5)

        self.declare_parameter("save_period_sec", 3.0)

        self.target = int(self.get_parameter("shelf_column_number").value)
        if self.target not in VALID_DIGITS:
            raise ValueError(f"shelf_column_number must be 1-5, got {self.target}")

        self.image_dir = self.get_parameter("image_dir").value
        os.makedirs(self.image_dir, exist_ok=True)

        self.templates = self.load_templates()
        if not self.templates:
            raise RuntimeError(
                f"No digit templates found in {self.get_parameter('texture_dir').value}"
            )

        self.bridge = CvBridge()
        self.column_pub = self.create_publisher(
            Int32, "/erc/shelf_column_identification", 10
        )
        self.offset_pub = self.create_publisher(Float32, "~/target_offset", 10)
        self.annotated_pub = self.create_publisher(Image, "~/annotated", 1)
        self.mask_pub = self.create_publisher(Image, "~/digit_mask", 1)

        self.create_subscription(
            Image, self.get_parameter("image_topic").value, self.on_image, 1
        )

        self.frames_seen = 0
        self.frames_with_target = 0
        self.frames_all_five = 0
        self.consecutive_target = 0
        self.confirmed = False
        self.last_save = None
        self.digit_sightings = {}

        self.get_logger().info(
            f"Looking for shelf column {self.target}. "
            f"Loaded templates: {sorted(self.templates)}"
        )

    # ----------------------------------------------------------- templates

    def load_templates(self):
        """Load each digit texture as a normalised binary image.

        Stored as white-digit-on-black so matching compares ink coverage,
        independent of how the plaque happens to be lit in the scene.
        """
        directory = self.get_parameter("texture_dir").value
        templates = {}
        for digit in VALID_DIGITS:
            path = os.path.join(directory, f"{digit}.png")
            image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                self.get_logger().warn(f"Could not read template {path}")
                continue
            _, binary = cv2.threshold(
                image, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
            )
            binary = self.tight_crop(binary)
            if binary is not None:
                templates[digit] = binary
        return templates

    @staticmethod
    def tight_crop(binary):
        """Trim a binary image to the bounding box of its white pixels."""
        coords = cv2.findNonZero(binary)
        if coords is None:
            return None
        x, y, w, h = cv2.boundingRect(coords)
        if w == 0 or h == 0:
            return None
        return binary[y:y + h, x:x + w]

    # --------------------------------------------------------- segmentation

    def find_digit_candidates(self, gray, hsv):
        """Dark, achromatic, digit-shaped blobs in the upper part of the frame."""
        h, w = gray.shape
        limit = int(h * self.get_parameter("search_top_frac").value)
        dark_threshold = self.get_parameter("dark_threshold").value
        max_sat = self.get_parameter("max_saturation").value

        mask = np.zeros_like(gray)
        _, dark = cv2.threshold(gray[:limit], dark_threshold, 255, cv2.THRESH_BINARY_INV)
        # Drop anything with real colour in it: that is a book, not a digit.
        achromatic = (hsv[:limit, :, 1] <= max_sat).astype(np.uint8) * 255
        mask[:limit] = cv2.bitwise_and(dark, achromatic)
        # Reconnect strokes broken by rendering, without merging separate digits.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((2, 2), np.uint8))

        min_h = self.get_parameter("min_digit_height_px").value
        max_h = self.get_parameter("max_digit_height_px").value
        min_aspect = self.get_parameter("min_digit_aspect").value
        max_aspect = self.get_parameter("max_digit_aspect").value

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)
            if ch < min_h or ch > max_h or cw == 0:
                continue
            aspect = ch / float(cw)
            if aspect < min_aspect or aspect > max_aspect:
                continue
            # Ignore blobs touching the frame edge: usually clipped scenery.
            if x <= 0 or y <= 0 or (x + cw) >= w:
                continue
            candidates.append((x, y, cw, ch))
        return candidates, mask

    def filter_to_plaque_row(self, results, image_height):
        """Keep only candidates lying in the dominant horizontal band.

        The five markers sit in a line above the shelf unit. Grouping by height
        and keeping the largest group discards stragglers elsewhere in the frame.
        """
        if not self.get_parameter("use_plaque_row").value or len(results) < 2:
            return results, None

        tolerance = image_height * self.get_parameter("plaque_row_tolerance_frac").value
        centres = [(box[1] + box[3] / 2.0, i) for i, (box, _, _) in enumerate(results)]

        best_group, best_centre = [], None
        for centre, _ in centres:
            group = [i for c, i in centres if abs(c - centre) <= tolerance]
            # Prefer the biggest band; break ties towards the higher one, since
            # markers sit above everything else in the scene.
            if len(group) > len(best_group) or (
                len(group) == len(best_group)
                and best_centre is not None
                and centre < best_centre
            ):
                best_group, best_centre = group, centre

        return [results[i] for i in best_group], best_centre

    # ------------------------------------------------------------ matching

    def classify(self, mask, box):
        """Best (digit, score) for one candidate, comparing ink overlap.

        Both candidate and template are reduced to the same small binary grid, so
        the comparison is scale invariant and tolerates the mild perspective skew
        you get when viewing the shelf from an angle.
        """
        x, y, w, h = box
        crop = mask[y:y + h, x:x + w]
        crop = self.tight_crop(crop)
        if crop is None:
            return None, 0.0

        grid = (24, 32)     # width, height
        candidate = cv2.resize(crop, grid, interpolation=cv2.INTER_AREA)
        candidate = (candidate > 127).astype(np.float32)

        best_digit, best_score = None, 0.0
        for digit, template in self.templates.items():
            resized = cv2.resize(template, grid, interpolation=cv2.INTER_AREA)
            resized = (resized > 127).astype(np.float32)
            # Intersection over union of the ink: 1.0 is a perfect overlap.
            intersection = float(np.sum(candidate * resized))
            union = float(np.sum(np.maximum(candidate, resized)))
            score = intersection / union if union > 0 else 0.0
            if score > best_score:
                best_digit, best_score = digit, score
        return best_digit, best_score

    # ------------------------------------------------------------ callback

    def on_image(self, msg):
        self.frames_seen += 1
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"Could not convert frame: {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        candidates, mask = self.find_digit_candidates(gray, hsv)
        min_score = self.get_parameter("min_match_score").value

        results = []
        for box in candidates:
            digit, score = self.classify(mask, box)
            if digit is not None and score >= min_score:
                results.append((box, digit, score))

        in_row, row_centre = self.filter_to_plaque_row(results, frame.shape[0])
        dropped = [r for r in results if r not in in_row]

        # Keep the strongest match per digit: a digit cannot appear twice.
        best_per_digit = {}
        for box, digit, score in in_row:
            if digit not in best_per_digit or score > best_per_digit[digit][1]:
                best_per_digit[digit] = (box, score)

        for digit in best_per_digit:
            self.digit_sightings[digit] = self.digit_sightings.get(digit, 0) + 1
        if len(best_per_digit) == 5:
            self.frames_all_five += 1

        target_hit = best_per_digit.get(self.target)
        if target_hit is not None:
            self.frames_with_target += 1
            self.consecutive_target += 1
            self.publish_target(target_hit[0], frame.shape[1])
        else:
            self.consecutive_target = 0

        annotated = self.draw(frame, best_per_digit, dropped, row_centre)
        self.publish_debug(annotated, mask)
        self.maybe_save(annotated, target_hit is not None)

    def publish_target(self, box, image_width):
        x, _y, w, _h = box
        centre = x + w / 2.0
        offset = (centre - image_width / 2.0) / (image_width / 2.0)
        self.offset_pub.publish(Float32(data=float(offset)))

        needed = self.get_parameter("confirm_frames").value
        if self.consecutive_target >= needed and not self.confirmed:
            self.column_pub.publish(Int32(data=self.target))
            self.confirmed = True
            self.get_logger().info(
                f"Column {self.target} confirmed after {needed} frames, "
                f"offset {offset:+.2f}"
            )
        elif self.consecutive_target >= needed:
            self.column_pub.publish(Int32(data=self.target))

    def draw(self, frame, best_per_digit, dropped, row_centre):
        annotated = frame.copy()
        width = annotated.shape[1]

        if row_centre is not None:
            cv2.line(annotated, (0, int(row_centre)), (width, int(row_centre)),
                     (90, 90, 90), 1)

        for (x, y, w, h), _digit, _score in dropped:
            cv2.rectangle(annotated, (x, y), (x + w, y + h), WEAK_BGR, 1)

        target_box = best_per_digit.get(self.target, (None, None))[0]
        for digit, (box, score) in sorted(best_per_digit.items()):
            x, y, w, h = box
            is_target = box == target_box
            colour = TARGET_BGR if is_target else MATCH_BGR
            cv2.rectangle(annotated, (x, y), (x + w, y + h), colour,
                          2 if is_target else 1)
            cv2.putText(
                annotated, f"{digit} {score:.2f}", (x, max(10, y - 3)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1, cv2.LINE_AA,
            )

        seen = " ".join(
            f"{d}:{best_per_digit[d][1]:.2f}" for d in sorted(best_per_digit)
        ) or "none"
        rate = (100.0 * self.frames_with_target / self.frames_seen
                if self.frames_seen else 0.0)
        all_five = (100.0 * self.frames_all_five / self.frames_seen
                    if self.frames_seen else 0.0)
        status = "CONFIRMED" if self.confirmed else f"{self.consecutive_target} in a row"

        cv2.putText(
            annotated, f"target {self.target} | {status} | seen: {seen}",
            (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            f"target in {rate:.0f}% | all five in {all_five:.0f}% | "
            f"n={self.frames_seen} | {len(dropped)} off-row",
            (6, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA,
        )
        return annotated

    def publish_debug(self, annotated, mask):
        try:
            self.annotated_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "mono8"))
        except Exception:                               # noqa: BLE001
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
        path = os.path.join(self.image_dir, f"column_{self.target}_{stamp}.png")
        try:
            cv2.imwrite(path, annotated)
        except Exception as exc:                        # noqa: BLE001
            self.get_logger().warn(f"Could not save {path}: {exc}")

    def destroy_node(self):
        # Printed on Ctrl+C. Report-ready recognition statistics.
        if self.frames_seen:
            self.get_logger().info(
                f"Column {self.target}: visible in "
                f"{100.0 * self.frames_with_target / self.frames_seen:.0f}%, "
                f"all five digits read in "
                f"{100.0 * self.frames_all_five / self.frames_seen:.0f}% "
                f"of {self.frames_seen} frames"
            )
        if self.digit_sightings:
            seen = ", ".join(
                f"{d}: {n}" for d, n in sorted(self.digit_sightings.items())
            )
            self.get_logger().info(f"Digit sightings: {seen}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ShelfColumnDetector()
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