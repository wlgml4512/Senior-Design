"""Fusion-side state holder for recognized voice commands.

This file does not capture audio itself; instead, it remembers whether follow is
enabled, whether emergency stop is latched, and what speed scaling should be
applied before motion reaches the motor layer.
"""

DEFAULT_SPEED_MULTIPLIER = 1.0
MIN_SPEED_MULTIPLIER = 0.4
MAX_SPEED_MULTIPLIER = 1.6
SPEED_STEP = 0.2


class VoiceCommandController:
    def __init__(self, follow_enabled=False):
        # When voice control is enabled, fusion stays idle until it receives a
        # navigation-level "follow" command from the voice subsystem.
        self.follow_active = bool(follow_enabled)
        self.emergency_stop_active = False
        self.speed_multiplier = DEFAULT_SPEED_MULTIPLIER
        self.last_command = None

    def apply_command(self, event):
        if event is None or not event.get("recognized"):
            return None

        command = event["command"]
        action = command.get("action")
        if action not in {"follow", "reduce_speed", "increase_speed", "emergency_stop", "resume"}:
            return None

        # Keep the last accepted command for debugging and operator feedback.
        self.last_command = {
            "timestamp": event["timestamp"],
            "text": event["text"],
            "action": action,
            "source": event["source"],
        }

        if action == "follow":
            self.follow_active = True
            self.emergency_stop_active = False
            return "follow_enabled"

        if action == "reduce_speed":
            self.speed_multiplier = max(
                MIN_SPEED_MULTIPLIER,
                self.speed_multiplier - SPEED_STEP,
            )
            return "speed_reduced"

        if action == "increase_speed":
            self.speed_multiplier = min(
                MAX_SPEED_MULTIPLIER,
                self.speed_multiplier + SPEED_STEP,
            )
            return "speed_increased"

        if action == "emergency_stop":
            self.emergency_stop_active = True
            return "emergency_stop"

        if action == "resume":
            self.emergency_stop_active = False
            return "resumed"

        return None

    def is_follow_active(self):
        return self.follow_active

    def is_emergency_stop_active(self):
        return self.emergency_stop_active

    def apply_speed_modifier(self, motion_cmd):
        if motion_cmd is None:
            motion_cmd = {"linear": 0.0, "angular": 0.0, "stop": True, "reason": "no_motion"}
        else:
            motion_cmd = dict(motion_cmd)

        # Voice speed modifiers only affect forward speed. Steering and safety
        # overrides remain owned by the planner / motion arbiter.
        motion_cmd["linear"] *= self.speed_multiplier
        return motion_cmd

    def snapshot(self):
        return {
            "motion_enabled": self.follow_active and not self.emergency_stop_active,
            "follow_active": self.follow_active,
            "emergency_stop_active": self.emergency_stop_active,
            "speed_multiplier": self.speed_multiplier,
            "last_command": self.last_command,
        }
