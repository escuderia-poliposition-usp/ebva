from .base_controller import BaseController

class WallController(BaseController):
    def __init__(self):
        super().__init__()
        
        # State variables to manage the "rewind" maneuver
        self.is_recovering = False
        self.recovery_sequence = []
        self.recovery_index = 0
        
        # Fallback parameters in case history is empty or runs out
        self.default_reverse_throttle = -0.5
        self.fallback_steer = 0.0

    def control(self, state_data) -> tuple:
        history = state_data.get('command_history', [])
        
        # 1. Initialization of the Reversal Maneuver
        # If this is the first tick we are in 'walls' mode, snapshot the history
        if not self.is_recovering:
            self.is_recovering = True
            
            # We reverse the history list so index 0 is the step right before the crash
            self.recovery_sequence = list(reversed(history))
            self.recovery_index = 0

        # 2. Replay the History Backwards
        if self.recovery_index < len(self.recovery_sequence):
            # Fetch the past command
            past_cmd = self.recovery_sequence[self.recovery_index]
            self.recovery_index += 1
            
            past_steer = past_cmd.get('steer', 0.0)
            past_throttle = past_cmd.get('throttle', 0.0)
            
            # To retrace the exact path backward:
            # We keep the steering EXACTLY the same, but we invert the throttle to go backward.
            # We use -abs() to guarantee it goes backward, even if the past throttle was negative.
            reverted_steer = past_steer
            reverted_throttle = -abs(past_throttle)
            
            return float(reverted_steer), float(reverted_throttle)
            
        else:
            # 3. Fallback (If history runs out but we are still stuck)
            # Just back up straight at a safe speed
            return float(self.fallback_steer), float(self.default_reverse_throttle)

    def reset_recovery(self):
        """
        The Supervisor MUST call this method when the car is no longer stuck.
        This resets the controller so it's ready for the next time it hits a wall.
        """
        self.is_recovering = False
        self.recovery_sequence = []
        self.recovery_index = 0