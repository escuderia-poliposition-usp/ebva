

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import copy
import numpy as np


from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan



class ScanFilter(Node):

    #Class Constructor
    def __init__(self):
        super().__init__('scan_filter')
        input_topic=self.declare_parameter('input_topic','/scan').value
        output_topic=self.declare_parameter('output_topic','/escuderia/scan').value
        #Publishers
            #Publisher 1: Steering control
        self.scan_escuderia_publisher = self.create_publisher(LaserScan, output_topic, 10)
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
            input_topic,
            self.lidar_callback,
            10
        )

        
        #Other definitions
        self.subscription  # prevent unused variable warning
        self.lidar_msg=LaserScan()


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
        max_val = max(val for val in lidar.ranges if val != float('inf') and val != float('-inf'))
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


    
    def find_consecutive_samples(self,arr, threshold, nsamp):
        """
        Finds consecutive samples in the array that are greater than or equal to the threshold.
        
        Parameters:
        - arr (numpy array): The array of samples to search.
        - threshold (float): The threshold value.
        - nsamp (int): The minimum number of consecutive samples required.
        
        Returns:
        - result_indices (list): List of indices of the consecutive samples that meet the condition.
        """
        result_indices = []  # This will store the result indices
        count = 0  # To count consecutive samples >= threshold
        start_index = None  # To store the starting index of a valid sequence

        # Loop through the array
        for i in range(len(arr)):
            if arr[i] >= threshold:  # If the sample meets the threshold condition
                if count == 0:
                    start_index = i  # Mark the start of a new sequence
                count += 1
            else:
                count = 0  # Reset count if the sample is below threshold
            
            # If we have found enough consecutive samples
            if count >= nsamp:
                result_indices.extend(range(start_index, i + 1))  # Add the indices to result list
                count = 0  # Reset count to avoid overlapping sequences

        return result_indices

    def thereIsLargeGapFunction(self,ranges,range_max):
        result_indices=self.find_consecutive_samples(arr=ranges, threshold=0.9*range_max, nsamp=10)#Distance between 2 lidar beams is 0.04m -> then 40cm is 10 laser beams
        if result_indices: #check if not empty
            return True, result_indices
        else:
            return False, []
        
    def is_in_angle_range(self,angle_degree,angle_range):
        angle_range_min=angle_range[0]
        angle_range_max=angle_range[1]
        if angle_degree<angle_range_min or angle_degree>angle_range_max:
            return False
        else:
            return True
        
        
        
    def exclude_vehicle_in_scan(self,lidar: 'LaserScan'):

        angle_range=[-90,90] #degrees -110,110
        
        for idx in range(len(lidar.ranges)):
            angle_rad=self.getAngleFromLidarBeamIndex(idx,lidar)
            angle_degree=np.rad2deg(angle_rad)
            if not self.is_in_angle_range(angle_degree,angle_range):
                self.lidar_msg.ranges[idx]=0.0       

        
    #Subscribers ROS Callbacks
        #LiDAR Callback
    def lidar_callback(self, msg):
        #1) remove outliers [-inf,+inf]->[0,10]
        msg = self.lidarPreprocessing(msg)
        self.lidar_msg=msg

        #2) limit lidar range to exclude the measurment of the vehcile    
        self.exclude_vehicle_in_scan(self.lidar_msg)
        
        #3) publish escuderia scan
        self.scan_escuderia_publisher.publish(self.lidar_msg)
    

def main(args=None):
    rclpy.init(args=args)

    scan_filter = ScanFilter()

    try:
        rclpy.spin(scan_filter)
    except KeyboardInterrupt:
        pass #This will handle Ctrl+C

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
        
        
        

        

