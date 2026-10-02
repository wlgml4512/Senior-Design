"""Motion planner for following and obstacle response.

This file translates fused target and obstacle state into linear and angular
motion commands, so it is the behavior layer that determines how ATLAS should
move before motor mixing happens.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlannerConfig:
    follow_distance: float = 1.0
    follow_distance_tolerance: float = 0.10
    target_min_distance: float = 0.35
    stop_distance: float = 0.35
    side_clearance: float = 0.30
    angle_deadband_deg: float = 4.0
    angle_gain: float = 0.04
    avoidance_gain: float = 1.2
    linear_speed_gain: float = 0.8
    max_linear_speed: float = 0.6
    max_angular_speed: float = 1.0
    hold_linear_scale: float = 0.35
    hold_max_linear_speed: float = 0.20


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def compute_motion(target_state, obstacle_state, planner_config=None):
    """
    target_state: dict with keys "distance", "angle", and tracking metadata
    obstacle_state: dict with front/left/right obstacle distances
    """
    planner_config = planner_config or PlannerConfig()

    if obstacle_state is None:
        obstacle_state = {
            "left_distance": None,
            "right_distance": None,
            "front_distance": None,
        }

    obstacle_distance = obstacle_state["front_distance"]
    left_distance = obstacle_state["left_distance"]
    right_distance = obstacle_state["right_distance"]

    # Side clearance pushes the robot away from nearby obstacles even while it is tracking.
    left_avoidance = (
        0.0
        if left_distance is None or left_distance >= planner_config.side_clearance
        else (planner_config.side_clearance - left_distance) / planner_config.side_clearance
    )
    right_avoidance = (
        0.0
        if right_distance is None or right_distance >= planner_config.side_clearance
        else (planner_config.side_clearance - right_distance) / planner_config.side_clearance
    )
    # A tighter obstacle on the left should push the robot right (negative
    # angular), while a tighter obstacle on the right should push it left.
    avoidance_angular = planner_config.avoidance_gain * (right_avoidance - left_avoidance)

    # A close obstacle in front overrides target following and forces a stop/turn behavior.
    if obstacle_distance is not None and obstacle_distance < planner_config.stop_distance:
        if left_distance is None and right_distance is None:
            turn_direction = 0.0
        elif right_distance is None or (left_distance is not None and left_distance > right_distance):
            turn_direction = planner_config.max_angular_speed
        else:
            turn_direction = -planner_config.max_angular_speed
        return {"linear": 0.0, "angular": turn_direction, "stop": True, "reason": "front_obstacle"}

    if target_state is None:
        stop_cmd = abs(avoidance_angular) < 1e-6
        return {
            "linear": 0.0,
            "angular": _clamp(avoidance_angular, -planner_config.max_angular_speed, planner_config.max_angular_speed),
            "stop": stop_cmd,
            "reason": "obstacle_monitoring" if not stop_cmd else "target_lost",
        }

    target_distance = target_state["distance"]
    target_angle = target_state["angle"]
    tracking_state = target_state.get("tracking_state", "tracking")

    if tracking_state == "lost":
        return {"linear": 0.0, "angular": 0.0, "stop": True, "reason": "target_lost"}

    if target_angle is None or target_distance is None:
        return {"linear": 0.0, "angular": avoidance_angular, "stop": True, "reason": "target_invalid"}

    # Ignore very small target-angle errors to avoid constant steering chatter.
    if abs(target_angle) <= planner_config.angle_deadband_deg:
        tracking_angular = 0.0
    else:
        tracking_angular = planner_config.angle_gain * target_angle

    angular_cmd = _clamp(
        tracking_angular + avoidance_angular,
        -planner_config.max_angular_speed,
        planner_config.max_angular_speed,
    )

    # During the tracker's waiting state, hold position but keep turning toward the last target.
    if tracking_state == "waiting":
        return {"linear": 0.0, "angular": angular_cmd, "stop": False, "reason": "target_waiting"}

    if target_distance <= planner_config.target_min_distance:
        return {"linear": 0.0, "angular": angular_cmd, "stop": True, "reason": "target_too_close"}

    # Forward speed is proportional to how far outside the configured follow band the target is.
    distance_error = target_distance - planner_config.follow_distance
    if abs(distance_error) <= planner_config.follow_distance_tolerance:
        linear_cmd = 0.0
    else:
        linear_cmd = _clamp(
            planner_config.linear_speed_gain * distance_error,
            0.0,
            planner_config.max_linear_speed,
        )

    if tracking_state == "holding":
        held_linear_cmd = min(
            linear_cmd * planner_config.hold_linear_scale,
            planner_config.hold_max_linear_speed,
        )
        stop_cmd = held_linear_cmd == 0.0 and abs(angular_cmd) < 1e-6
        return {
            "linear": held_linear_cmd,
            "angular": angular_cmd,
            "stop": stop_cmd,
            "reason": "holding_last_target",
        }

    return {"linear": linear_cmd, "angular": angular_cmd, "stop": False, "reason": "tracking_target"}
