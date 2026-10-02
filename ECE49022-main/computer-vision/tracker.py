"""Short-term target persistence for computer vision.

This tracker keeps the robot from dropping a target immediately after one missed
frame, which makes the vision subsystem more stable before data reaches fusion.
"""

import time

class TargetTracker:
    def __init__(self, max_lost_time=3.0): 
        self.max_lost_time = max_lost_time
        self.last_seen_time = None

    def update_and_lock(self, target_info):
        current_time = time.time()
        if target_info is not None:
            self.last_seen_time = current_time
            return target_info
        else:
            if self.last_seen_time is None:
                return None            
            duration = current_time - self.last_seen_time
            if duration < self.max_lost_time:
                return "wait_state"
            else:
                return None
