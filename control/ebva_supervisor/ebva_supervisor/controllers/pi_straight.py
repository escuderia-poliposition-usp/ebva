from .base_controller import BaseController

class PIStraightController(BaseController):
    def __init__(self, kp=1.0, ki=0.0):
        super().__init__()
        self.kp = kp
        self.ki = ki
        self.integral_error = 0.0
        
        self.max_steer = 1.0       # Steering limit - NEED TO BE SET
        self.max_throttle = 1.0    # Maximum throttle
        self.windup_limit = 5.0    # Limit for the integral accumulation

    def control(self, state_data) -> tuple:
        # Extract split state data (with safe default values)
        track_prob = state_data.get('track_prob', 1.0)
        track_entropy = state_data.get('track_entropy', 0.0)
        
        position = state_data.get('position', 'middle')
        pos_prob = state_data.get('pos_prob', 1.0)
        pos_entropy = state_data.get('pos_entropy', 0.0)
        
        # 1. Lateral Error Modeling
        lateral_error = 0.0
        
        # Convention: 
        # Positive steering ( > 0 ) turns the wheel to the RIGHT
        # Negative steering ( < 0 ) turns the wheel to the LEFT
        
        if position == 'left':
            lateral_error = 1.0 * pos_prob
            
        elif position == 'right':
            lateral_error = -1.0 * pos_prob
            
        elif position == 'middle':
            lateral_error = 0.0
            
        # Optional Smoothing: Dampen correction if position entropy is high
        # This prevents the steering wheel from shaking if the network is confused
        lateral_error *= max(0.0, 1.0 - pos_entropy)
            
        # 2. PI Calculation
        
        # Proportional Term
        p_term = self.kp * lateral_error
        
        # Integral Term (Error accumulation over time)
        self.integral_error += lateral_error
        
        # ANTI-WINDUP: Prevents the I term from exploding if the car stays off-center for too long
        if self.integral_error > self.windup_limit:
            self.integral_error = self.windup_limit
        elif self.integral_error < -self.windup_limit:
            self.integral_error = -self.windup_limit
            
        # Reset the accumulation if the car has stabilized confidently in the middle
        if position == 'middle' and pos_prob > 0.8: # Adjust the threshold as needed
            self.integral_error = 0.0
            
        i_term = self.ki * self.integral_error
        
        # 3. Final Composition and Saturation (Clipping)
        steering = p_term + i_term
        
        # Ensure the command does not exceed the mechanical limit of the car
        steering = max(min(steering, self.max_steer), -self.max_steer)
        
        # 4. Throttle Control
        
        # Throttle is always set to maximum as requested
        throttle = self.max_throttle
        
        return float(steering), float(throttle)