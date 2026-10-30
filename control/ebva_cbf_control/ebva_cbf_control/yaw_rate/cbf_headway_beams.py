from ..model_kernels.yaw_rate import cbf_filter as pure_cbf_filter
from ..qp import qp_hildreth
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import copy
import numpy as np


from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry



class CBF(Node):

    #Class Constructor
    def __init__(self):
        super().__init__('cbf')
        #Publishers
            #Publisher 1: Steering control
        self.steer_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', 10)
        self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', 10)
        self.laser_debug_publisher = self.create_publisher(LaserScan, '/ebva/debug/lidar', 10)
        self.goal_debug_publisher = self.create_publisher(LaserScan, '/ebva/debug/goal', 10)
        #Subscribers

            #Subscriber 1: Get LiDAR information
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
            #Subscriber 2: Get Throttle information
        self.throttle_sub = self.create_subscription(
            Float32,
            '/a_no',
            self.throttle_callback,
            10
        )
            #Subscriber 3: Get Steering information
        self.steer_sub = self.create_subscription(
            Float32,
            '/w_no',
            self.steering_callback,
            10
        )
            #Subscriber 4: Get longitudinal speed through VESC odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        #Other definitions
        
        self.lidar_msg=LaserScan()
        self.steering_msg=Float32()
        self.throttle_msg=Float32()
        self.LaserScan_direction=LaserScan()

        self.v = 0.0  # linear speed in x (longitudinal vehicle direction)
        self.theta_steer=0.0


        self.steeringSaturation = 120 #degrees
        self.thetaOld=0
        self.deltaThetaMax=np.radians(180) #180 means no limit in steering ratio
        self.speedOld=0


        self.maxThrotle=0.24 #% (experimentaly more than this cause crash)
            #Steeirng Control
        self.Ts=1/40 #s

        self.L=0.33 #[m] Wheelbase (distance from rear axis to the front axis)

        

    #=============================
    # LiDAR Processing Methods
    # ============================  
      
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

    
    #=============================
    # Send commands Methods
    # ============================  

    def yawrate_to_servo(self,w, v, L):
        """
        Convert yaw rate command to steering servo command.

        w : yaw rate [rad/s]
        v : longitudinal speed [m/s]
        L : wheelbase [m]
        """

        delta_max = np.pi / 6.0     # 30 deg
        v_min = 0.1                 # regularization

        # Regularized inversion
        v_eff = max(abs(v), v_min) * np.sign(v if v != 0 else 1.0)
        delta = np.arctan((L / v_eff) * w)

        # Saturate steering angle
        #delta = np.clip(delta, -delta_max, delta_max)
        delta = np.clip(delta, -delta_max, delta_max)

        # Map δ → servo
        servo = 0.5 + 0.35 * (delta / delta_max)
        servo = np.clip(servo, 0.15, 0.85)

        return servo, delta



    def steer_command(self,steerAngle):
        self.steer_publisher.publish(Float32(data=steerAngle))

    def publish_control(self, steer, throttle):
       
        #Saturate steering u=theta
        max_steer = np.radians(self.steeringSaturation)  
        min_steer = - max_steer  

        # Saturate steer value 
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

        # self.get_logger().info(
        #     f"FTG: Throttle={throttle:.3f} %, "
        #     f"Steering={steer:.3f} rad"
        # )

    #=============================
    # QP Hildreth Solver Code
    # ============================   
    qp_hildreth = staticmethod(qp_hildreth)
 
         

    #=============================
    # CBF Code
    # ============================      
    cbf_filter = pure_cbf_filter

    
    #=============================
    # ROS2 Subscribe Callbacks
    # ============================        

    def odom_callback(self, msg: Odometry):
        # Linear velocity along x (in child_frame_id, typically base_link)
        self.v = msg.twist.twist.linear.x

        #LiDAR Callback
    def lidar_callback(self, msg):
        msg = self.lidarPreprocessing(msg)
        self.lidar_msg=msg
        
    def steering_callback(self, msg):
        self.steering_msg=msg

    def throttle_callback(self, msg):
        #1) acceleration
        self.throttle_msg=msg
        a=self.throttle_msg.data
        
        #2) angular speed
        w=self.steering_msg.data

        #3) nominal control
        u_nominal=np.array([a,w])
        
        #4) LiDAR msg extraction
        if len(self.lidar_msg.ranges) == 0:
            return #avoid crashing due to empty LiDAR msg
        scan_ranges = np.array(self.lidar_msg.ranges)
        angle_min = self.lidar_msg.angle_min
        angle_increment = self.lidar_msg.angle_increment
        scan_angles = angle_min + np.arange(scan_ranges.size) * angle_increment

        #5) Crop LiDAR info in [-pi/2,+pi/2]
        # Desired angular window
        angle_low  = -np.pi / 2
        angle_high =  np.pi / 2
        # Boolean mask
        mask = (scan_angles >= angle_low) & (scan_angles <= angle_high)
        # Crop both arrays consistently
        scan_angles_cropped = scan_angles[mask]
        scan_ranges_cropped = scan_ranges[mask]


        #6) Speed from VESC odometry
        v=self.v
        #7) Apply CBF Filter
        [u_opt, h]=self.cbf_filter(v, scan_ranges_cropped, scan_angles_cropped, u_nominal) 
        #8) Send filtered control to vehicle
        a=u_opt[0]
        w=u_opt[1]

        self.get_logger().info(
            f"FTG: Throttle={a:.3f} %, "
            f"Steering={w:.3f} rad/s"
        )

        #9) Transform w to delta steering angle, but in the servo values [0.15, 0.85]
        servo_cmd, delta_cmd = self.yawrate_to_servo(w,v,self.L)
        
        #10) Publish commands
        #self.publish_control(steer=self.theta_steer, throttle=a)
        self.publish_control(steer=delta_cmd , throttle=a)
        

#=============================
# MAIN FUNCTION
# ============================       

def main(args=None):
    rclpy.init(args=args)

    cbf = CBF()

    try:
        rclpy.spin(cbf)
    except KeyboardInterrupt:
        pass #This will handle Ctrl+C

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cbf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
        
        
        

        

