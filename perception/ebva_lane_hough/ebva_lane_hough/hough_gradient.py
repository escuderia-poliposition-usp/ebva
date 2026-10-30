import math

import cv2
import numpy as np


class LaneDetector:

    def __init__(
        self,
        smoothing=0.30,
        min_confidence=0.35,
        max_missing_frames=5,
    ):
        self.smoothing = float(smoothing)
        self.min_confidence = float(min_confidence)
        self.max_missing_frames = int(max_missing_frames)

        self.left_coeff = None
        self.right_coeff = None

        self.missing_frames = 0

    def preprocess(self, image):
        """
        Robust lane segmentation.

        Uses:
          - grayscale gradient
          - HLS lightness
          - saturation

        Returns binary lane mask.
        """

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

        hls = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HLS
        )

        lightness = hls[:, :, 1]
        saturation = hls[:, :, 2]

        # Gradient in horizontal direction.
        sobel_x = cv2.Sobel(
            gray,
            cv2.CV_64F,
            1,
            0,
            ksize=3,
        )

        abs_sobel = np.absolute(sobel_x)

        if abs_sobel.max() > 0:
            scaled_sobel = np.uint8(
                255 * abs_sobel / abs_sobel.max()
            )
        else:
            scaled_sobel = np.zeros_like(gray)

        gradient_mask = (
            (scaled_sobel >= 25)
            & (scaled_sobel <= 255)
        )

        # Bright lane markings.
        bright_mask = lightness >= 150

        # Saturated lane markings can survive illumination changes.
        saturation_mask = saturation >= 70

        combined = (
            gradient_mask
            | bright_mask
            | saturation_mask
        )

        # Remove small isolated noise.
        binary = np.uint8(combined) * 255

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        binary = cv2.morphologyEx(
            binary,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=1,
        )

        return binary

    def region_of_interest(self, image):
        """
        Trapezoidal ROI.

        The exact values remain configurable through the class
        constants because camera mounting can differ.
        """

        height, width = image.shape[:2]

        mask = np.zeros_like(image)

        polygon = np.array(
            [[
                (int(0.03 * width), height),
                (int(0.97 * width), height),
                (int(0.66 * width), int(0.46 * height)),
                (int(0.34 * width), int(0.46 * height)),
            ]],
            dtype=np.int32,
        )

        cv2.fillPoly(
            mask,
            polygon,
            255,
        )

        return cv2.bitwise_and(
            image,
            mask,
        )

    def detect_lines(self, image):
        binary = self.preprocess(image)

        roi = self.region_of_interest(
            binary
        )

        lines = cv2.HoughLinesP(
            roi,
            rho=1.0,
            theta=np.pi / 180.0,
            threshold=30,
            minLineLength=25,
            maxLineGap=80,
        )

        return lines, binary, roi

    def separate_lanes(
        self,
        lines,
        image_width,
        image_height,
    ):
        left = []
        right = []

        if lines is None:
            return left, right

        center_x = image_width * 0.5

        for line in lines:
            x1, y1, x2, y2 = line[0]

            dx = x2 - x1
            dy = y2 - y1

            if abs(dx) < 2:
                continue

            slope = dy / float(dx)

            # Reject almost-horizontal structures.
            if abs(slope) < 0.25:
                continue

            length = math.hypot(
                dx,
                dy,
            )

            if length < 25.0:
                continue

            midpoint_x = (
                x1 + x2
            ) * 0.5

            # Image y increases downwards.
            # Left lane normally slopes up-right.
            if slope < 0 and midpoint_x < center_x:
                left.append(
                    (x1, y1, x2, y2, length)
                )

            # Right lane normally slopes down-right.
            elif slope > 0 and midpoint_x > center_x:
                right.append(
                    (x1, y1, x2, y2, length)
                )

        return left, right

    def fit_lane(self, lines):
        """
        Fit x = a*y² + b*y + c.

        Weighted by segment length.
        """

        if not lines:
            return None

        xs = []
        ys = []
        weights = []

        for x1, y1, x2, y2, length in lines:
            xs.extend([x1, x2])
            ys.extend([y1, y2])

            weight = max(
                float(length),
                1.0,
            )

            weights.extend(
                [weight, weight]
            )

        if len(xs) < 4:
            return None

        try:
            coeff = np.polyfit(
                np.asarray(ys, dtype=np.float64),
                np.asarray(xs, dtype=np.float64),
                2,
                w=np.asarray(
                    weights,
                    dtype=np.float64,
                ),
            )

            if not np.all(
                np.isfinite(coeff)
            ):
                return None

            return coeff

        except (
            np.linalg.LinAlgError,
            ValueError,
        ):
            return None

    def smooth_coefficients(
        self,
        current,
        previous,
    ):
        if current is None:
            return previous

        if previous is None:
            return current.copy()

        alpha = self.smoothing

        return (
            alpha * current
            + (1.0 - alpha) * previous
        )

    def lane_geometry_valid(
        self,
        left_coeff,
        right_coeff,
        image_width,
        image_height,
    ):
        if (
            left_coeff is None
            or right_coeff is None
        ):
            return False

        # Validate lane width at multiple depths.
        y_values = np.array(
            [
                image_height * 0.60,
                image_height * 0.70,
                image_height * 0.80,
                image_height * 0.90,
            ],
            dtype=np.float64,
        )

        left_x = np.polyval(
            left_coeff,
            y_values,
        )

        right_x = np.polyval(
            right_coeff,
            y_values,
        )

        widths = (
            right_x - left_x
        )

        # Lane must remain ordered.
        if np.any(widths <= 0):
            print(f"[GEOVALID] FAIL widths<=0 widths={widths}", flush=True)
            return False

        # Reject physically absurd image widths.
        min_width = image_width * 0.15
        max_width = image_width * 0.95

        if np.any(
            widths < min_width
        ):
            print(f"[GEOVALID] FAIL too_narrow widths={widths} min_width={min_width}", flush=True)
            return False

        if np.any(
            widths > max_width
        ):
            print(f"[GEOVALID] FAIL too_wide widths={widths} max_width={max_width}", flush=True)
            return False

        # Width should not change wildly over the ROI.
        mean_width = np.mean(widths)

        if mean_width <= 1.0:
            return False

        relative_variation = (
            np.max(widths)
            - np.min(widths)
        ) / mean_width

        if relative_variation > 0.70:
            print(f"[GEOVALID] FAIL variation={relative_variation:.2f} widths={widths}", flush=True)
            return False

        print(f"[GEOVALID] PASS widths={widths}", flush=True)
        return True

    def compute_confidence(
        self,
        left_lines,
        right_lines,
        geometry_valid,
        image_width,
        image_height,
    ):
        if not geometry_valid:
            return 0.0

        left_count = len(left_lines)
        right_count = len(right_lines)

        # More supporting segments → higher confidence,
        # saturating quickly.
        left_score = min(
            left_count / 8.0,
            1.0,
        )

        right_score = min(
            right_count / 8.0,
            1.0,
        )

        balance = min(
            left_score,
            right_score,
        )

        # Confidence starts conservative.
        confidence = (
            0.20
            + 0.80 * balance
        )

        return float(
            np.clip(
                confidence,
                0.0,
                1.0,
            )
        )

    def compute_lane_center(
        self,
        left_coeff,
        right_coeff,
        image_width,
        image_height,
    ):
        if (
            left_coeff is None
            or right_coeff is None
        ):
            return None

        # Reference point toward the lower image.
        y_eval = image_height * 0.78

        left_x = np.polyval(
            left_coeff,
            y_eval,
        )

        right_x = np.polyval(
            right_coeff,
            y_eval,
        )

        lane_center = (
            left_x + right_x
        ) * 0.5

        image_center = (
            image_width * 0.5
        )

        # Positive means vehicle/image center is
        # to the LEFT of lane center.
        #
        # This preserves the Stanley convention used
        # in our controller.
        lateral_error = (
            image_center
            - lane_center
        ) / image_width

        left_derivative = np.polyder(
            left_coeff
        )

        right_derivative = np.polyder(
            right_coeff
        )

        left_slope = np.polyval(
            left_derivative,
            y_eval,
        )

        right_slope = np.polyval(
            right_derivative,
            y_eval,
        )

        center_slope = (
            left_slope
            + right_slope
        ) * 0.5

        heading_error = math.atan(
            center_slope
        )

        return (
            float(lane_center),
            float(lateral_error),
            float(heading_error),
        )

    def process(self, image):
        height, width = image.shape[:2]

        lines, binary, roi = (
            self.detect_lines(image)
        )

        left_lines, right_lines = (
            self.separate_lanes(
                lines,
                width,
                height,
            )
        )
        print(f'[DEBUG] left={len(left_lines)} right={len(right_lines)} total_lines={0 if lines is None else len(lines)}', flush=True)

        left_current = self.fit_lane(
            left_lines
        )

        right_current = self.fit_lane(
            right_lines
        )

        geometry_valid = (
            self.lane_geometry_valid(
                left_current,
                right_current,
                width,
                height,
            )
        )

        confidence = (
            self.compute_confidence(
                left_lines,
                right_lines,
                geometry_valid,
                width,
                height,
            )
        )

        if geometry_valid:
            self.left_coeff = (
                self.smooth_coefficients(
                    left_current,
                    self.left_coeff,
                )
            )

            self.right_coeff = (
                self.smooth_coefficients(
                    right_current,
                    self.right_coeff,
                )
            )

            self.missing_frames = 0

        else:
            self.missing_frames += 1

        # Short-term temporal hold.
        # We do NOT declare a lane permanently valid;
        # this is only for brief perception dropouts.
        usable_geometry = (
            self.left_coeff is not None
            and self.right_coeff is not None
            and self.missing_frames
            <= self.max_missing_frames
        )

        if usable_geometry:
            geometry = self.compute_lane_center(
                self.left_coeff,
                self.right_coeff,
                width,
                height,
            )
        else:
            geometry = None

        detected = (
            geometry is not None
            and confidence >= self.min_confidence
        )

        lane_center = None
        lateral_error = None
        heading_error = None

        if geometry is not None:
            (
                lane_center,
                lateral_error,
                heading_error,
            ) = geometry

        debug_image = image.copy()

        # ROI outline.
        roi_mask = np.zeros_like(
            debug_image
        )

        polygon = np.array(
            [[
                (int(0.03 * width), height),
                (int(0.97 * width), height),
                (int(0.66 * width), int(0.46 * height)),
                (int(0.34 * width), int(0.46 * height)),
            ]],
            dtype=np.int32,
        )

        cv2.polylines(
            roi_mask,
            polygon,
            True,
            (255, 255, 0),
            2,
        )

        debug_image = cv2.addWeighted(
            debug_image,
            1.0,
            roi_mask,
            0.5,
            0.0,
        )

        # Raw detected line segments.
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]

                cv2.line(
                    debug_image,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 255),
                    1,
                )

        # Draw fitted lanes.
        if (
            self.left_coeff is not None
            and self.right_coeff is not None
        ):
            y_draw = np.linspace(
                height * 0.46,
                height,
                60,
            )

            left_x = np.polyval(
                self.left_coeff,
                y_draw,
            )

            right_x = np.polyval(
                self.right_coeff,
                y_draw,
            )

            for i in range(
                len(y_draw) - 1
            ):
                lx1 = int(left_x[i])
                ly1 = int(y_draw[i])
                lx2 = int(left_x[i + 1])
                ly2 = int(y_draw[i + 1])

                rx1 = int(right_x[i])
                ry1 = int(y_draw[i])
                rx2 = int(right_x[i + 1])
                ry2 = int(y_draw[i + 1])

                cv2.line(
                    debug_image,
                    (lx1, ly1),
                    (lx2, ly2),
                    (0, 255, 0),
                    3,
                )

                cv2.line(
                    debug_image,
                    (rx1, ry1),
                    (rx2, ry2),
                    (0, 255, 0),
                    3,
                )

        # Lane center.
        if lane_center is not None:
            y_eval = int(
                height * 0.78
            )

            cv2.circle(
                debug_image,
                (
                    int(lane_center),
                    y_eval,
                ),
                8,
                (0, 0, 255),
                -1,
            )

            cv2.line(
                debug_image,
                (
                    width // 2,
                    y_eval - 35,
                ),
                (
                    width // 2,
                    y_eval + 35,
                ),
                (255, 0, 0),
                2,
            )

        # Status text.
        status = (
            "TRACKING"
            if detected
            else "LOST/DEGRADED"
        )

        cv2.putText(
            debug_image,
            f"{status}  conf={confidence:.2f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        return (
            detected,
            lateral_error,
            heading_error,
            confidence,
            debug_image,
        )
