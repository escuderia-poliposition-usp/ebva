"""Pure FTG/PD method with the selected original gains and sample recurrences."""
from dataclasses import dataclass
from types import SimpleNamespace
import copy
import numpy as np
from .gap_geometry import GapGeometry

CONTRACT="ftg_pd_sample_v1"


@dataclass
class Scan:
    ranges: list
    angle_min: float
    angle_max: float
    angle_increment: float
    range_max: float


class FtgPdCore(GapGeometry):
    def __init__(self):
        self.reset()

    def reset(self):
        self.carDiameter = 0.8
        self.tolMaskDisparities = 0.01
        self.disparityTreshold = 0.05
        self.timeToCollision = 30
        self.SafeLateralDistance = 0.1
        self.steeringSaturation = 60
        self.minSpeedFactor = 1 / 0.4
        self.safeCurveExitRatio = 1 / 3
        self.angleOld = 0
        self.thetaOld = 0
        self.deltaThetaMax = np.radians(20)
        self.speedOld = 0
        self.maxDeltaSpeed = 0.02
        self.maxThrotle = 0.12
        self.Kp_steer = 1
        self.Td = 0.2
        self.N = 10
        self.Ts = 1 / 40
        self.ud_1 = 0
        self.steeringMeasurment_1 = 0
        self.lidar_msg=None
        self.steering_msg=SimpleNamespace(data=0.)
        self.steering_output=self.throttle_output=0.

    def step(self, scan, feedback):
        self.lidar_msg=self.lidarPreprocessing(copy.deepcopy(scan))
        self.steering_msg.data=feedback
        self.followTheGap()
        return float(self.steering_output),float(self.throttle_output)

    def getLidarBeamIndexByAngle(self, angle_degree):
        angle = np.radians(angle_degree)
        angle_min = self.lidar_msg.angle_min
        angle_max = self.lidar_msg.angle_max
        angle_increment = self.lidar_msg.angle_increment
        if angle >= angle_min and angle <= angle_max:
            beams_number = int((angle_max - angle_min) / angle_increment)
            index = int(beams_number * (angle - angle_min) / (angle_max - angle_min))
            return index
        else:
            return -1

    def lidarPreprocessing(self, lidar: 'LaserScan'):
        for i in range(len(lidar.ranges)):
            if lidar.ranges[i] != lidar.ranges[i] or lidar.ranges[i] == float('inf') or lidar.ranges[i] == float('-inf'):
                lidar.ranges[i] = lidar.range_max
        return lidar

    def getLidarRangeByAngle(self, angle_degree, lidar_ranges):
        index = self.getLidarBeamIndexByAngle(angle_degree)
        return lidar_ranges[index]

    def getAngleFromLidarBeamIndex(self, index, lidar: 'LaserScan'):
        return lidar.angle_min + lidar.angle_increment * index

    def mask_disparities(self, disp_index, lidar: 'LaserScan'):
        if lidar.ranges[disp_index] < lidar.ranges[disp_index + 1]:
            alpha = np.arctan((self.carDiameter / 2 + self.tolMaskDisparities) / lidar.ranges[disp_index])
            numberMaskedLidarBeams = int(np.ceil(alpha / lidar.angle_increment))
            for i in range(disp_index, disp_index + numberMaskedLidarBeams):
                if i >= len(lidar.ranges):
                    break
                lidar.ranges[i] = lidar.ranges[disp_index]
        else:
            alpha = np.arctan((self.carDiameter / 2 + self.tolMaskDisparities) / lidar.ranges[disp_index + 1])
            numberMaskedLidarBeams = int(np.ceil(alpha / lidar.angle_increment))
            for i in range(max(0, disp_index - numberMaskedLidarBeams), disp_index):
                if i >= len(lidar.ranges):
                    break
                lidar.ranges[i] = lidar.ranges[disp_index]
        return lidar

    def thereIsLargeGapFunction(self, ranges, range_max):
        result_indices = self.find_consecutive_samples(arr=ranges, threshold=0.9 * range_max, nsamp=10)
        if result_indices:
            return (True, result_indices)
        else:
            return (False, [])

    def getAngleAndDistanceFarthest(self, masked_ranges, lidar: 'LaserScan'):
        for i in range(len(masked_ranges)):
            angle = self.getAngleFromLidarBeamIndex(i, lidar)
            if abs(angle) > np.radians(90):
                masked_ranges[i] = 0
        (thereIsLargeGap, largeGapListIndexes) = self.thereIsLargeGapFunction(masked_ranges, lidar.range_max)
        if not thereIsLargeGap:
            index_of_max = np.argmax(np.array(masked_ranges))
            maxRange = np.max(masked_ranges)
            angleOfMaxRange = self.getAngleFromLidarBeamIndex(index=index_of_max, lidar=lidar)
        else:
            middle_index = len(largeGapListIndexes) // 2
            index = largeGapListIndexes[middle_index]
            maxRange = masked_ranges[index]
            angleOfMaxRange = self.getAngleFromLidarBeamIndex(index=index, lidar=lidar)
        if isinstance(angleOfMaxRange, float) and (not (angleOfMaxRange != angleOfMaxRange or angleOfMaxRange == float('inf') or angleOfMaxRange == float('-inf'))):
            angleOfMaxRange = angleOfMaxRange
        else:
            angleOfMaxRange = 0.0
        return (angleOfMaxRange, maxRange)

    def getLateralMinDistance(self, lidar: 'LaserScan'):
        rightDistances = []
        leftDistances = []
        robustnessRange = [85, 95]
        for angle_degree in range(-robustnessRange[0], -robustnessRange[1], -1):
            range_at_angle = self.getLidarRangeByAngle(angle_degree, lidar.ranges)
            if range_at_angle is not None:
                leftDistances.append(range_at_angle * abs(np.sin(np.radians(angle_degree))))
        for angle_degree in range(robustnessRange[0], robustnessRange[1]):
            range_at_angle = self.getLidarRangeByAngle(angle_degree, lidar.ranges)
            if range_at_angle is not None:
                rightDistances.append(range_at_angle * abs(np.sin(np.radians(angle_degree))))
        if leftDistances and rightDistances:
            meanRightDistance = np.mean(rightDistances)
            meanLeftDistance = np.mean(leftDistances)
            return min(meanRightDistance, meanLeftDistance)
        else:
            return 0.0

    def steerIsSafe(self, lidar: 'LaserScan'):
        minLateralDistance = self.getLateralMinDistance(lidar)
        if minLateralDistance > self.SafeLateralDistance:
            return True
        else:
            return False

    def safeCurveExitFactorFunction(self, angle):
        return 1.0

    def setSafeMaxSpeed(self, distance, angle):
        speedBasedOnDistance = distance / self.timeToCollision
        steeringFactor = 1 - abs(np.degrees(angle)) / (self.minSpeedFactor * self.steeringSaturation)
        safeCurveExitFactor = self.safeCurveExitFactorFunction(angle)
        speed = safeCurveExitFactor * steeringFactor * speedBasedOnDistance
        if speed - self.speedOld > self.maxDeltaSpeed:
            speed = self.speedOld + self.maxDeltaSpeed
            self.speedOld = speed
        else:
            self.speedOld = speed
        return speed

    def steer_command(self, steerAngle):
        steeringMeasurment = self.steering_msg.data
        steeringReference = steerAngle
        error = steeringReference - steeringMeasurment
        up = self.Kp_steer * error
        ud = self.Td / (self.Td + self.N * self.Ts) * self.ud_1 - self.Kp_steer * self.Td / (self.Td + self.N * self.Ts) * (steeringMeasurment - self.steeringMeasurment_1)
        steeringControl = up + ud
        self.steering_output = steeringControl
        self.ud_1 = ud
        self.steeringMeasurment_1 = steeringMeasurment

    def publish_control(self, steer, throttle):
        max_steer = np.radians(self.steeringSaturation)
        min_steer = -max_steer
        steer = max(min(steer, max_steer), min_steer) * 1.0
        deltaTheta = steer - self.thetaOld
        sign = np.sign(deltaTheta)
        if abs(deltaTheta) > self.deltaThetaMax:
            steer = self.thetaOld + sign * self.deltaThetaMax
            self.thetaOld = steer
        else:
            self.thetaOld = steer
        self.steer_command(steer)
        if throttle > self.maxThrotle:
            throttle = self.maxThrotle
        self.throttle_output = throttle

    def followTheGap(self):
        lidar = self.lidar_msg
        disparities = self.findDisparities(lidar)
        masked_lidar = lidar
        for i in range(len(disparities)):
            masked_lidar = self.mask_disparities(disp_index=disparities[i], lidar=masked_lidar)
        masked_ranges = masked_lidar.ranges
        (angleFarthest, distanceFarthest) = self.getAngleAndDistanceFarthest(masked_ranges, lidar)
        if not self.steerIsSafe(lidar):
            angleFarthest = 0
        max_speed = self.setSafeMaxSpeed(distanceFarthest, angleFarthest)
        self.publish_control(steer=angleFarthest, throttle=max_speed)
