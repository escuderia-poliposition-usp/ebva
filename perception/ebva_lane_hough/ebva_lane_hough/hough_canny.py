import cv2
import numpy as np


class LaneDetector:

    def __init__(self):
        self.left_coeff = None
        self.right_coeff = None

    def preprocess(self, image):
        """Preprocesamiento de la imagen."""

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(
            gray,
            (5, 5),
            0
        )

        edges = cv2.Canny(
            blur,
            50,
            150
        )

        return edges

    def region_of_interest(self, image):
        """Selecciona la región de interés."""

        height, width = image.shape[:2]

        mask = np.zeros_like(image)

        polygon = np.array([
            [
                (int(0.05 * width), height),
                (int(0.95 * width), height),
                (int(0.60 * width), int(0.50 * height)),
                (int(0.40 * width), int(0.50 * height))
            ]
        ], dtype=np.int32)

        cv2.fillPoly(
            mask,
            polygon,
            255
        )

        return cv2.bitwise_and(
            image,
            mask
        )

    def detect_lines(self, image):
        """Detecta líneas de carril."""

        edges = self.preprocess(image)

        roi = self.region_of_interest(
            edges
        )

        lines = cv2.HoughLinesP(
            roi,
            rho=1,
            theta=np.pi / 180,
            threshold=40,
            minLineLength=30,
            maxLineGap=50
        )

        return lines

    def separate_lanes(self, lines, image_width):
        """Separa líneas izquierdas y derechas."""

        left_lines = []
        right_lines = []

        if lines is None:
            return left_lines, right_lines

        center_x = image_width / 2.0

        for line in lines:

            x1, y1, x2, y2 = line[0]

            dx = x2 - x1
            dy = y2 - y1

            if abs(dx) < 1:
                continue

            slope = dy / dx

            if abs(slope) < 0.3:
                continue

            midpoint = (x1 + x2) / 2.0

            if slope < 0 and midpoint < center_x:

                left_lines.append(
                    (x1, y1, x2, y2)
                )

            elif slope > 0 and midpoint > center_x:

                right_lines.append(
                    (x1, y1, x2, y2)
                )

        return left_lines, right_lines

    def fit_lane(self, lines):
        """Ajusta x = a*y² + b*y + c."""

        if len(lines) == 0:
            return None

        points_x = []
        points_y = []

        for x1, y1, x2, y2 in lines:

            points_x.extend([
                x1,
                x2
            ])

            points_y.extend([
                y1,
                y2
            ])

        if len(points_x) < 3:
            return None

        try:

            coefficients = np.polyfit(
                np.array(points_y),
                np.array(points_x),
                2
            )

            return coefficients

        except np.linalg.LinAlgError:

            return None

    def compute_lane_center(
        self,
        left_coeff,
        right_coeff,
        image_width,
        image_height
    ):
        """Calcula centro del carril y errores."""

        if (
            left_coeff is None
            or right_coeff is None
        ):
            return None, None, None

        y_eval = image_height * 0.75

        left_x = np.polyval(
            left_coeff,
            y_eval
        )

        right_x = np.polyval(
            right_coeff,
            y_eval
        )

        lane_center = (
            left_x + right_x
        ) / 2.0

        image_center = (
            image_width / 2.0
        )

        lateral_error = (
            image_center - lane_center
        ) / image_width

        left_derivative = np.polyder(
            left_coeff
        )

        right_derivative = np.polyder(
            right_coeff
        )

        left_slope = np.polyval(
            left_derivative,
            y_eval
        )

        right_slope = np.polyval(
            right_derivative,
            y_eval
        )

        center_slope = (
            left_slope + right_slope
        ) / 2.0

        heading_error = np.arctan(
            center_slope
        )

        return (
            lane_center,
            lateral_error,
            heading_error
        )

    def process(self, image):
        """Procesa una imagen completa."""

        height, width = image.shape[:2]

        lines = self.detect_lines(
            image
        )

        left_lines, right_lines = (
            self.separate_lanes(
                lines,
                width
            )
        )

        left_coeff = self.fit_lane(
            left_lines
        )

        right_coeff = self.fit_lane(
            right_lines
        )

        (
            lane_center,
            lateral_error,
            heading_error
        ) = self.compute_lane_center(
            left_coeff,
            right_coeff,
            width,
            height
        )

        detection_available = (
            left_coeff is not None
            and right_coeff is not None
        )

        debug_image = image.copy()

        if lines is not None:

            for line in lines:

                x1, y1, x2, y2 = line[0]

                cv2.line(
                    debug_image,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

        if lane_center is not None:

            y_eval = int(
                height * 0.75
            )

            cv2.circle(
                debug_image,
                (
                    int(lane_center),
                    y_eval
                ),
                8,
                (0, 0, 255),
                -1
            )

            cv2.line(
                debug_image,
                (
                    width // 2,
                    y_eval - 30
                ),
                (
                    width // 2,
                    y_eval + 30
                ),
                (255, 0, 0),
                2
            )

        return (
            detection_available,
            lateral_error,
            heading_error,
            debug_image
        )
