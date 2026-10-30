import cv2
import numpy as np
from ebva_lane_hough.hough_gradient import LaneDetector


def main():



    detector = LaneDetector()


    # Crear una imagen de prueba
    image = np.zeros(
        (480, 640, 3),
        dtype=np.uint8
    )


    # Dibujar dos líneas simulando un carril
    cv2.line(
        image,
        (180, 480),
        (300, 240),
        (255, 255, 255),
        8
    )

    cv2.line(
        image,
        (460, 480),
        (340, 240),
        (255, 255, 255),
        8
    )


    (
        detected,
        lateral_error,
        heading_error,
        debug_image
    ) = detector.process(image)


    print("================================")
    print("   TEST LANE DETECTOR")
    print("================================")

    print(
        "Lane detected:",
        detected
    )

    print(
        "Lateral error:",
        lateral_error
    )

    print(
        "Heading error:",
        heading_error
    )

    print("================================")


    cv2.imwrite(
        "lane_detector_test.png",
        debug_image
    )


if __name__ == '__main__':
    main()
