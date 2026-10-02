"""Normalization and short-term state holding for fusion inputs.

This module converts raw CV and ultrasonic snapshots into the `target_state` and
`obstacle_state` structures used by planning, so it is the handoff point
between subsystem adapters and motion logic.
"""

from dataclasses import dataclass
import time


@dataclass(frozen=True)
class FusionConfig:
    vision_hold_timeout_s: float = 1.5


def _hold_age_s(last_target, now):
    if last_target is None:
        return None

    last_seen_at = last_target.get("last_seen_at")
    if last_seen_at is None:
        return None

    return max(0.0, now - last_seen_at)


def _can_hold_last_target(last_target, now, fusion_config):
    hold_age = _hold_age_s(last_target, now)
    if hold_age is None:
        return False
    return hold_age <= fusion_config.vision_hold_timeout_s


def _build_held_target(last_target, vision_state, now):
    held_target = dict(last_target)
    held_target["tracking_state"] = "holding"
    held_target["vision_state"] = vision_state
    held_target["is_live"] = False
    held_target["is_held"] = True
    held_target["hold_age_s"] = _hold_age_s(last_target, now)
    return held_target


def fuse_data(cv_data, ultrasonic_data, last_target, fusion_config=None, now=None):
    fusion_config = fusion_config or FusionConfig()
    now = time.time() if now is None else now

    if cv_data is None:
        cv_data = {
            "tracking_state": "disabled",
            "target_id": None,
            "angle": None,
            "distance": None,
            "is_live": False,
            "received_at": None,
        }

    if ultrasonic_data is None:
        ultrasonic_data = {
            "timestamp": None,
            "received_at": None,
            "left_distance": None,
            "right_distance": None,
            "front_distance": None,
        }

    tracking_state = cv_data["tracking_state"]
    angle = cv_data["angle"]
    distance = cv_data["distance"]
    left_distance = ultrasonic_data["left_distance"]
    right_distance = ultrasonic_data["right_distance"]
    front_distance = ultrasonic_data["front_distance"]

    # The fusion layer only trusts CV when geometry is available; otherwise it can
    # temporarily reuse the previous target during brief vision interruptions.
    if angle is None or distance is None:
        if tracking_state in {"waiting", "stale", "lost"} and _can_hold_last_target(last_target, now, fusion_config):
            target_state = _build_held_target(last_target, tracking_state, now)
        else:
            target_state = None
    else:
        target_state = {
            "target_id": cv_data["target_id"],
            "angle": angle,
            "distance": distance,
            "timestamp": cv_data.get("timestamp"),
            "last_seen_at": cv_data.get("timestamp") or cv_data.get("received_at") or now,
            "tracking_state": tracking_state,
            "vision_state": tracking_state,
            "is_live": cv_data["is_live"],
            "is_held": False,
            "hold_age_s": 0.0,
        }

    # Preserve the most recent target geometry, but reflect the current tracker state.
    if target_state is not None and not target_state.get("is_held", False) and tracking_state != "tracking":
        target_state = dict(target_state)
        target_state["tracking_state"] = tracking_state
        target_state["vision_state"] = tracking_state
        target_state["is_live"] = cv_data["is_live"]

    # The planner expects a normalized obstacle dictionary even if sensors are disabled.
    obstacle_state = {
        "received_at": ultrasonic_data.get("received_at"),
        "left_distance": left_distance,
        "right_distance": right_distance,
        "front_distance": front_distance,
    }

    return target_state, obstacle_state
