from .base_controller import BaseController

class PCurveController(BaseController):
    def __init__(self, kp=1.5, curvature_gain=0.3):
        super().__init__()
        self.kp = kp
        self.curvature_gain = curvature_gain 
        self.max_steer = 1.0
        self.base_throttle = 0.6
        self.min_throttle = 0.3
        self.default_curve_steer = 0.5 

    def control(self, state_data) -> tuple:
        # Track Classification
        track_type = state_data.get('track_type', 'left_curve')
        track_prob = state_data.get('track_prob', 1.0)
        track_entropy = state_data.get('track_entropy', 0.0)
        
        # Position Classification
        position = state_data.get('position', 'middle')
        pos_prob = state_data.get('pos_prob', 1.0)
        pos_entropy = state_data.get('pos_entropy', 0.0)
        
        # Physical Curvature
        curvature_inv_r = state_data.get('curvature', 0.0)
        
        # Feed-Forward (Kinematic Steering based on R^-1)
        feed_forward = 0.0
        
        if curvature_inv_r > 0.0:
            ff_magnitude = self.curvature_gain * curvature_inv_r
        else:
            ff_magnitude = self.default_curve_steer

        # Apply direction
        if track_type == 'right_curve':
            feed_forward = ff_magnitude
        elif track_type == 'left_curve':
            feed_forward = -ff_magnitude

        # Safety Check: If the network is unsure about being in a curve, 
        # we scale down the base steering to avoid throwing the car off-track.
        feed_forward *= track_prob


        # Lateral Error Modeling (Proportional Correction)
        lateral_error = 0.0
        
        # We only apply lateral correction based on how confident the POSITION network is.
        if position == 'left':
            lateral_error = 1.0 * pos_prob
        elif position == 'right':
            lateral_error = -1.0 * pos_prob
        elif position == 'middle':
            lateral_error = 0.0
            
        # Optional Smoothing: High position entropy means messy data. 
        # Multiplying by (1 - pos_entropy) weakens the correction when confused, 
        # preventing sudden jerks of the steering wheel.
        lateral_error *= max(0.0, 1.0 - pos_entropy)
            
        p_term = self.kp * lateral_error
        
        # Final steering = Base Steer + Correction
        steering = feed_forward + p_term
        steering = max(min(steering, self.max_steer), -self.max_steer)
        
        # Dynamic Throttle Control
        throttle = self.base_throttle
        
        # Slow down drastically if:
        # A. The curve is extremely sharp -  Set trhreshold based on track testing and tuning
        # B. The track network is confused (might not be a curve at all)
        # C. The position network is confused (we are losing lane tracking)
        if (curvature_inv_r > 1.0 or 
            track_entropy > 0.6 or 
            pos_entropy > 0.6 or 
            track_prob < 0.5):
            
            throttle = self.min_throttle
            
        return float(steering), float(throttle)