"""Exact equivalent kernels shared by three FTG implementations."""
class GapGeometry:
    def findDisparities(self,lidar: 'LaserScan'):
            ranges = lidar.ranges
            disparities = []
            for i in range(len(ranges)-1):
                if abs(ranges[i+1]-ranges[i])>self.disparityTreshold:
                    disparities.append(i)
            return disparities

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

