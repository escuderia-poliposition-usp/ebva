from .gap_geometry import GapGeometry
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import copy
import numpy as np


from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan



class FollowTheGap(GapGeometry, Node):

    #Class Constructor
    def __init__(self):
        super().__init__('follow_the_gap')
        #Publishers
            #Publisher 1: Steering control
        self.steer_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.laser_debug_publisher = self.create_publisher(LaserScan, '/ebva/debug/lidar', 10)
        self.goal_debug_publisher = self.create_publisher(LaserScan, '/ebva/debug/goal', 10)
        #Subscribers
            #Subscriber 1: Get LiDAR information
        #self.subscription = self.create_subscription(
        #    LaserScan,
        #    '/autodrive/f1tenth_1/lidar',
        #    self.lidar_callback,
        #    10
        #)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
            #Subscriber 2: Get Steering information
        self.subscription = self.create_subscription(
            Float32,
            '/autodrive/f1tenth_1/steering',
            self.steering_callback,
            10
        )
        
        #Other definitions
        self.subscription  # prevent unused variable warning
        self.lidar_msg=LaserScan()
        self.steering_msg=Float32()

        self.LaserScan_direction=LaserScan()

        #Fitted parameters of the Follow the gap problem
        self.carDiameter= 0.8#     #cm
        self.tolMaskDisparities=0.01   #cm
        self.disparityTreshold=0.05 #cm
        self.timeToCollision = 50#30# s
        self.SafeLateralDistance = 0.1 #cm
        self.steeringSaturation=60 #degrees
        self.minSpeedFactor = 1/0.4 #0.5 #5->20%reduction # values > 1
        self.safeCurveExitRatio = 1/3 # in [0,1] ensure safe exit from curve (do not accelerate until going fully straight)
        self.angleOld=0
        self.thetaOld=0
        self.deltaThetaMax=np.radians(20) #180 means no limit in steering ratio
        self.speedOld=0
        self.maxDeltaSpeed=0.02 #2%
        self.maxThrotle=0.12 #% (experimentaly more than this cause crash)
            #Steeirng Control
        self.Kp_steer=1
        self.Td=0.2
        self.N=10
        self.Ts=1/40 #s
        self.ud_1=0
        self.steeringMeasurment_1=0
        

    #Define usefull methods    
    def getLidarBeamIndexByAngle(self,angle_degree):
        angle=np.radians(angle_degree)
        angle_min=self.lidar_msg.angle_min #[rad]
        angle_max=self.lidar_msg.angle_max #[rad]
        angle_increment=self.lidar_msg.angle_increment #[rad]
        if angle>=angle_min and angle<=angle_max:
            beams_number=int((angle_max-angle_min)/angle_increment)
            index=int(beams_number*(angle-angle_min)/(angle_max-angle_min))
            return index
        else:
            return -1
    
    def lidarPreprocessing(self, lidar: 'LaserScan'):
        # Remove inf entries in lidar range
        for i in range(len(lidar.ranges)):
            if lidar.ranges[i] != lidar.ranges[i] or lidar.ranges[i] == float('inf') or lidar.ranges[i] == float('-inf'):
                #lidar.ranges[i] = max_val
                lidar.ranges[i] = lidar.range_max
        return lidar


    def getLidarRangeByAngle(self,angle_degree,lidar_ranges):
        index=self.getLidarBeamIndexByAngle(angle_degree)
        return lidar_ranges[index]

    def getAngleFromLidarBeamIndex(self,index,lidar: 'LaserScan'):
        return lidar.angle_min + lidar.angle_increment*index


    def mask_disparities(self,disp_index,lidar: 'LaserScan'):
        if lidar.ranges[disp_index]<lidar.ranges[disp_index+1]:# mask to the right
            alpha=np.arctan((self.carDiameter/2+self.tolMaskDisparities)/lidar.ranges[disp_index])
            numberMaskedLidarBeams=int(np.ceil(alpha/lidar.angle_increment))
            for i in range(disp_index, disp_index+numberMaskedLidarBeams):
                if i >= len(lidar.ranges):
                    break
                lidar.ranges[i]=lidar.ranges[disp_index]

        else:# mask to the left
            alpha=np.arctan((self.carDiameter/2+self.tolMaskDisparities)/lidar.ranges[disp_index+1])
            numberMaskedLidarBeams=int(np.ceil(alpha/lidar.angle_increment))
            for i in range(disp_index-numberMaskedLidarBeams, disp_index):
                if i >= len(lidar.ranges):
                    break
                lidar.ranges[i]=lidar.ranges[disp_index]

        return lidar
    

    def thereIsLargeGapFunction(self,ranges,range_max):
        result_indices=self.find_consecutive_samples(arr=ranges, threshold=0.9*range_max, nsamp=10)#Distance between 2 lidar beams is 0.04m -> then 40cm is 10 laser beams
        if result_indices: #check if not empty
            return True, result_indices
        else:
            return False, []

    #def thereIsLargeGap(self,ranges,range_max):
    #    numberOfConsecutiveLargeMeasures=0
    #    oldIsLarge=True
    #    largeMeasurmentsIndex=[]
    #    for i in range(len(ranges)):
    #        if ranges>0.9*range_max and oldIsLarge:
    #            numberOfConsecutiveLargeMeasures=numberOfConsecutiveLargeMeasures+1
    #            largeMeasurmentsIndex.append(i)
    #            oldIsLarge=True
    #        else:
    #            oldIsLarge=False
            
        

    def getAngleAndDistanceFarthest(self,masked_ranges, lidar: 'LaserScan'):
        #Consider only leaser beams from -90 to 90 degrees
        for i in range(len(masked_ranges)):
            angle=self.getAngleFromLidarBeamIndex(i,lidar)
            if abs(angle)>np.radians(90):
                masked_ranges[i]=0 
        index_out=0

        thereIsLargeGap,largeGapListIndexes =self.thereIsLargeGapFunction(masked_ranges,lidar.range_max)
        if not thereIsLargeGap:
            index_of_max = np.argmax(np.array(masked_ranges))
            # Get the maximum value
            #maxRange = np.max(np.array(masked_ranges))
            maxRange = np.max(masked_ranges)
            #Get angle associated to maximum value
            angleOfMaxRange = self.getAngleFromLidarBeamIndex(index=index_of_max,lidar=lidar)
            index_out=index_of_max

        else:
            middle_index = len(largeGapListIndexes) // 2
            index=largeGapListIndexes[middle_index]
            maxRange = masked_ranges[index]
            angleOfMaxRange = self.getAngleFromLidarBeamIndex(index=index,lidar=lidar)
            index_out=index



        #Treat invalid values
        if isinstance(angleOfMaxRange, float) and not (angleOfMaxRange != angleOfMaxRange or angleOfMaxRange == float('inf') or angleOfMaxRange == float('-inf')):
            angleOfMaxRange=angleOfMaxRange
        else:
            angleOfMaxRange=0.0
    
        #if isinstance(maxRange, float) and not (maxRange != maxRange or maxRange == float('inf') or maxRange == float('-inf')):
        #    maxRange=maxRange
        #else:
        #    maxRange=1.0

        return angleOfMaxRange, maxRange, index_out

    def getLateralMinDistance(self, lidar: 'LaserScan'):
        rightDistances = []
        leftDistances = []
        robustnessRange = [85, 95]  # Considering angles near lateral beams (-90 and 90 degrees)
        
        # For the left side (negative angles)
        for angle_degree in range(-robustnessRange[0], -robustnessRange[1], -1):  # Step -1 to move leftward
            range_at_angle = self.getLidarRangeByAngle(angle_degree, lidar.ranges)
            if range_at_angle is not None:
                leftDistances.append(range_at_angle * abs(np.sin(np.radians(angle_degree))))

        # For the right side (positive angles)
        for angle_degree in range(robustnessRange[0], robustnessRange[1]):
            range_at_angle = self.getLidarRangeByAngle(angle_degree, lidar.ranges)
            if range_at_angle is not None:
                rightDistances.append(range_at_angle * abs(np.sin(np.radians(angle_degree))))

        # Check if there are valid distances for both sides
        if leftDistances and rightDistances:
            meanRightDistance = np.mean(rightDistances)
            meanLeftDistance = np.mean(leftDistances)
            
            # Return the smaller of the two means
            return min(meanRightDistance, meanLeftDistance)
        else:
            # Handle case where no valid data is found on one side
            return 0.0


    def steerIsSafe(self,lidar: 'LaserScan'):
        minLateralDistance = self.getLateralMinDistance(lidar)
        if minLateralDistance>self.SafeLateralDistance:
            return True
        else:
            return False
    
    def safeCurveExitFactorFunction(self,angle):
        #if abs(self.angleOld-angle)<np.radians(self.steeringSaturation*self.safeCurveExitRatio):
        #    self.angleOld=angle #update delayed value
        #    return 0.5
        #else:
        #    self.angleOld=angle #update delayed value
        #    return 1.0
        return 1.0


    def setSafeMaxSpeed(self, distance, angle):
        speedBasedOnDistance=distance / self.timeToCollision
        steeringFactor=1-abs(np.degrees(angle))/(self.minSpeedFactor*self.steeringSaturation) #reduce speed if is steering
        safeCurveExitFactor=self.safeCurveExitFactorFunction(angle)
        speed=safeCurveExitFactor*steeringFactor*speedBasedOnDistance

        #Limit throtle ratio 
        if speed-self.speedOld>self.maxDeltaSpeed:
            speed=self.speedOld+self.maxDeltaSpeed
            self.speedOld=speed
        else:
            self.speedOld=speed
        #print("Speed",speed)
        return speed


    def steer_command(self,steerAngle):
        #    #Control steerAngle as a reference (PD control based on Controle Aplicado Bruno/Gabriel pag 85)
        #steeringMeasurment=self.steering_msg.data
        #steeringReference=steerAngle
        #error=steeringReference-steeringMeasurment
        #up = self.Kp_steer*error #proportional control effort
        #ud = (self.Td/(self.Td+self.N*self.Ts))*self.ud_1 - (self.Kp_steer*self.Td)/(self.Td+self.N*self.Ts)*(steeringMeasurment-self.steeringMeasurment_1)
        #steeringControl = up+ud
        #    #Publish to vehicle
        #self.steer_publisher.publish(Float32(data=steeringControl))
        #    #Update the delayed
        #self.ud_1=ud
        #self.steeringMeasurment_1=steeringMeasurment
            #ALTERNATIVE REAL VEHICLE CONTROL
        self.steer_publisher.publish(Float32(data=steerAngle))

    def publish_control(self, steer, throttle):
       
        #Saturate steering u=theta
        max_steer = np.radians(self.steeringSaturation)  
        min_steer = - max_steer  

        # Saturate steer value between -15 and 15 degrees (in radians)
        steer = max(min(steer, max_steer), min_steer)*1.0 #cast to Float

        #Saturate ratio of steering u=delta theta
        deltaTheta=steer-self.thetaOld
        sign=np.sign(deltaTheta)
        if abs(deltaTheta)>self.deltaThetaMax:
            steer=self.thetaOld+sign*self.deltaThetaMax
            self.thetaOld=steer
        else:
            self.thetaOld=steer

        #Publish command
        self.steer_command(steer)

        #Saturate Max Throttle
        if throttle>self.maxThrotle:
            throttle=self.maxThrotle

        self.throttle_publisher.publish(Float32(data=throttle))

        


        self.get_logger().info(
            f"FTG: Throttle={throttle:.3f} %, "
            f"Steering={steer:.3f} rad"
        )

        


    #Define follow the gap method
    def followTheGap(self):
        #1 Acquire the LIDAR data
        lidar=self.lidar_msg
        #2 Find the disparities in LIDAR data
        disparities=self.findDisparities(lidar)
        #3 For each disparity:
        masked_lidar=lidar
        for i in range(len(disparities)):
            masked_lidar=self.mask_disparities(
                disp_index=disparities[i],
                lidar=masked_lidar)
        masked_ranges=masked_lidar.ranges
        #DEBUG
        self.laser_debug_publisher.publish(masked_lidar)

        #4 Search the new filtered distances for the farthest
        angleFarthest, distanceFarthest, index_out  = self.getAngleAndDistanceFarthest(masked_ranges,lidar)
        

        #5 Steer is safe?
        #if not self.steerIsSafe(lidar):#5.1 This is ok steer?
        #    angleFarthest = 0



        # DEBUG 
        goal_lidar = copy.copy(masked_lidar)
        #goal_lidar.ranges = [goal_lidar.ranges[i] if i == self.getLidarBeamIndexByAngle(angleFarthest) else 0.0 for i in range(len(goal_lidar.ranges))]
        for i in range(len(goal_lidar.ranges)):
            if not i == index_out:
                goal_lidar.ranges[i]=0.0
        self.goal_debug_publisher.publish(goal_lidar)
        
        #6 Set the speed based on the farthest distance
        max_speed = self.setSafeMaxSpeed(distanceFarthest,angleFarthest)

        #7 Send control to vehicle
        self.publish_control(steer=angleFarthest, throttle=max_speed)
        
    #Subscribers ROS Callbacks
        #LiDAR Callback
    def lidar_callback(self, msg):
        msg = self.lidarPreprocessing(msg)
        self.lidar_msg=msg
        self.followTheGap()

    def steering_callback(self, msg):
        self.steering_msg=msg
    

def main(args=None):
    rclpy.init(args=args)

    follow_the_gap = FollowTheGap()

    try:
        rclpy.spin(follow_the_gap)
    except KeyboardInterrupt:
        pass #This will handle Ctrl+C

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    follow_the_gap.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
        
        
        

        

