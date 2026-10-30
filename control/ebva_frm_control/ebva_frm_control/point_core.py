"""Point-sample FRM law: preserved sample differences, no ROS or physical units."""

CONTRACT = "frm_point_sample_v1"
INDICES = (540, 900, 180, 720, 360)


class FrmPointCore:
    def __init__(self):
        self.reset()

    def reset(self):
        self.e1_1 = self.e2_1 = self.int_e1 = self.int_e2 = self.distance_front_1 = 0.
        self.steer_value, self.throttle_value = 0., .06

    def step(self, ranges):
        K1 = 0.6
        K3 = 4.0
        K5 = 0.0
        K2 = 0.02
        K4 = 0.3
        K6 = 0.0
        distance_front = ranges[540]
        d_m90 = ranges[180]
        d_90 = ranges[900]
        d_m30 = ranges[360]
        d_30 = ranges[720]
        e1 = d_90 - d_m90
        e2 = d_30 - d_m30
        if e1 > 10.0:
            e1 = 10.0
        elif e1 < -10.0:
            e1 = -10.0
        if e2 > 10.0:
            e2 = 10.0
        elif e2 < -10.0:
            e2 = -10.0
        de1 = e1 - self.e1_1
        de2 = e2 - self.e2_1
        alpha = 0.2
        self.int_e1 = (1 - alpha) * self.int_e1 + alpha * e1
        self.int_e2 = (1 - alpha) * self.int_e2 + alpha * e2
        int1 = self.int_e1
        int2 = self.int_e2
        self.e1_1 = e1
        self.e2_1 = e2
        d_yaw = K1 * e1 + K3 * de1 + K5 * int1 + K2 * e2 + K4 * de2 + K6 * int2
        D_YAW_MAX = 1.0
        D_YAW_MIN = -1.0
        if d_yaw > D_YAW_MAX:
            d_yaw = D_YAW_MAX
        elif d_yaw < -D_YAW_MAX:
            d_yaw = -D_YAW_MAX
        self.steer_value = d_yaw
        kp = 0.03
        kd = 0.01
        a_max = 1.0
        if distance_front > 10:
            distance_front = 10
        d_distance = distance_front - self.distance_front_1
        a_frenagem = kp * distance_front + kd * d_distance
        self.distance_front_1 = distance_front
        if a_frenagem < -a_max:
            a_frenagem = -a_max
        elif a_frenagem > a_max:
            a_frenagem = a_max
        self.throttle_value = a_frenagem
        return self.steer_value, self.throttle_value
