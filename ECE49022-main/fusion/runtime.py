"""Shared runtime configuration for fusion runs.

This module keeps command-line and test-time tuning separate from the control
logic itself, which makes it the single place where loop rate, tracing, and
planner scaling are bundled for the rest of fusion.
"""

from dataclasses import dataclass
from pathlib import Path

from fusion import FusionConfig
from planner import PlannerConfig


@dataclass
class FusionRuntimeConfig:
    # These settings are runtime/test knobs, not robot-behavior logic, so they
    # live separately from the planner and subsystem interfaces.
    loop_hz: float = 10.0
    max_steps: int | None = None
    trace_path: Path | None = None
    arm_motors: bool = False
    follow_distance_m: float = 1.0
    follow_distance_tolerance_m: float = 0.10
    target_min_distance_m: float = 0.35
    obstacle_stop_distance_m: float = 0.35
    side_clearance_m: float = 0.30
    vision_hold_timeout_s: float = 1.5
    linear_speed_gain: float = 0.8
    max_linear_speed_mps: float = 0.6
    max_angular_speed_rad_s: float = 1.0
    hold_linear_scale: float = 0.35
    hold_max_linear_speed_mps: float = 0.20
    turn_mix_gain: float = 0.75
    motor_command_deadband: float = 0.05

    @property
    def dt(self):
        return 1.0 / self.loop_hz

    def planner_config(self):
        return PlannerConfig(
            follow_distance=self.follow_distance_m,
            follow_distance_tolerance=self.follow_distance_tolerance_m,
            target_min_distance=self.target_min_distance_m,
            stop_distance=self.obstacle_stop_distance_m,
            side_clearance=self.side_clearance_m,
            linear_speed_gain=self.linear_speed_gain,
            max_linear_speed=self.max_linear_speed_mps,
            max_angular_speed=self.max_angular_speed_rad_s,
            hold_linear_scale=self.hold_linear_scale,
            hold_max_linear_speed=self.hold_max_linear_speed_mps,
        )

    def fusion_config(self):
        return FusionConfig(vision_hold_timeout_s=self.vision_hold_timeout_s)
