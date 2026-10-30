def main():
    import cv2
    import numpy as np
    from ebva_lane_hough.hough_gradient import LaneDetector
    from ebva_lane_hough.stanley_controller import StanleyController
    detector = LaneDetector()
    controller = StanleyController(k_stanley=1.0, max_steering=0.45)
    vehicle_speed = 10.0
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.line(image, (180, 480), (300, 240), (255, 255, 255), 8)
    cv2.line(image, (460, 480), (340, 240), (255, 255, 255), 8)
    (detected, lateral_error, heading_error, debug_image) = detector.process(image)
    print()
    print('========================================')
    print('       TEST COMPLETO LANE KEEPING')
    print('========================================')
    print('Lane detected:', detected)
    print('Lateral error:', lateral_error)
    print('Heading error:', heading_error)
    if detected:
        steering_angle = controller.compute_steering(lateral_error, heading_error, vehicle_speed)
        print('Vehicle speed:', vehicle_speed)
        print('Steering angle:', steering_angle)
    else:
        steering_angle = 0.0
        print('Lane lost -> STOP')
    print('========================================')
    print()
