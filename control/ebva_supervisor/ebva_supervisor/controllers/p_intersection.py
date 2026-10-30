class PIntersectionController:
    def __init__(self, kp=0.8, base_throttle=0.25, max_steering=1.0):
        self.kp = kp
        self.base_throttle = base_throttle
        self.max_steering = max_steering

    def control(self, state):
        """
        Controller for intersections.

        Expected state fields:
        - position: 'left', 'middle', or 'right'
        - track_type: usually 'intersection'

        Output:
        - steering in [-1, 1]
        - throttle in [-1, 1]
        """

        position = state.get('position', 'middle')

        if position == 'left':
            error = 1.0
        elif position == 'right':
            error = -1.0
        else:
            error = 0.0

        steering = self.kp * error
        steering = max(-self.max_steering, min(self.max_steering, steering))

        throttle = self.base_throttle

        return steering, throttle