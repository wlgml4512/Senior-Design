"""Priority rules for turning planned motion into final fused motion.

This module is relevant because it decides which subsystem wins when commands
conflict, enforcing ATLAS's safety order before anything reaches the drivetrain.
"""

def arbitrate_motion(planned_motion, voice_controller):
    """
    Priority order:
    1. Voice emergency stop
    2. Voice follow gate
    3. Planner-issued obstacle stop/avoidance
    4. Default CV-driven tracking / waiting / lost-target behavior
    5. Low-priority voice speed modifiers layered onto safe motion
    """
    if planned_motion is None:
        # Keep the arbiter total-order and branch-free even if an upstream layer
        # fails to produce a command for this cycle.
        planned_motion = {"linear": 0.0, "angular": 0.0, "stop": True, "reason": "no_motion"}
    else:
        planned_motion = dict(planned_motion)

    if voice_controller.is_emergency_stop_active():
        # Stop/resume voice commands now flow through fusion, so emergency stop
        # must latch here before any CV or ultrasonic motion is applied.
        final_motion = {
            "linear": 0.0,
            "angular": 0.0,
            "stop": True,
            "reason": "voice_emergency_stop",
            "priority_source": "voice_emergency",
        }
    elif not voice_controller.is_follow_active():
        # With voice control enabled, fusion should stay idle until the
        # navigation subsystem explicitly requests following behavior.
        final_motion = {
            "linear": 0.0,
            "angular": 0.0,
            "stop": True,
            "reason": "voice_waiting_for_follow",
            "priority_source": "voice_follow_gate",
        }
    elif planned_motion.get("reason") == "front_obstacle":
        # The planner already decided safety needs to override tracking, so
        # preserve that command exactly instead of scaling it with voice input.
        final_motion = dict(planned_motion)
        final_motion["priority_source"] = "ultrasonic_obstacle"
    else:
        # Only low-priority voice modifiers, such as speed up / slow down, are
        # allowed to adjust the default CV-driven motion.
        final_motion = voice_controller.apply_speed_modifier(planned_motion)
        final_motion["priority_source"] = "cv_default"

    # Expose the voice-controller state in the output so logs explain why a
    # command was chosen on a given loop iteration.
    final_motion["voice"] = voice_controller.snapshot()
    return final_motion
