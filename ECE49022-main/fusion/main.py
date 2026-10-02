"""Command-line entry point for the fusion control loop.

This file is the operator-facing launcher for the robot's software stack: it
collects runtime flags, builds the shared config object, and starts the main
decision loop.
"""

import argparse
from pathlib import Path

from control_loop import run_control_loop
from runtime import FusionRuntimeConfig


def _parse_args():
    # Keep the CLI thin; all runtime behavior lives in the control loop.
    parser = argparse.ArgumentParser(description="Run the fusion control loop.")
    parser.add_argument(
        "--cv",
        choices=("on", "off"),
        default="on",
        help="Enable or disable the computer-vision subsystem.",
    )
    parser.add_argument(
        "--ultrasonic",
        choices=("on", "off"),
        default="on",
        help="Enable or disable the ultrasonic subsystem.",
    )
    parser.add_argument(
        "--voice",
        choices=("on", "off"),
        default="off",
        help="Enable or disable the voice-command subsystem.",
    )
    parser.add_argument(
        "--start-cv",
        action="store_true",
        help="Launch computer-vision/main.py automatically before the control loop starts.",
    )
    parser.add_argument(
        "--start-voice",
        action="store_true",
        help="Launch voice-recognition/voice_controller.py before the control loop starts.",
    )
    parser.add_argument(
        "--arm-motors",
        action="store_true",
        help="Allow fusion to send commands to the embedded motor backend.",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        help="Stop automatically after this many control-loop iterations.",
    )
    parser.add_argument(
        "--trace-file",
        type=Path,
        help="Write one JSON trace snapshot per control-loop iteration.",
    )
    parser.add_argument(
        "--follow-distance",
        type=float,
        default=1.0,
        help="Desired person-following distance in meters.",
    )
    parser.add_argument(
        "--follow-tolerance",
        type=float,
        default=0.10,
        help="Acceptable deadband around the follow distance in meters.",
    )
    parser.add_argument(
        "--target-min-distance",
        type=float,
        default=0.35,
        help="Minimum distance to the tracked person before stopping forward motion.",
    )
    parser.add_argument(
        "--stop-distance",
        type=float,
        default=0.35,
        help="Obstacle stop buffer in meters for ultrasonic avoidance.",
    )
    parser.add_argument(
        "--vision-hold-timeout",
        type=float,
        default=1.5,
        help="How long fusion should reuse the last known target during brief vision loss.",
    )
    parser.add_argument(
        "--linear-speed-gain",
        type=float,
        default=0.8,
        help="Forward-speed gain applied to follow-distance error before motor normalization.",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=0.6,
        help="Maximum forward planner speed in m/s that maps to full motor command.",
    )
    parser.add_argument(
        "--max-angular-speed",
        type=float,
        default=1.0,
        help="Maximum angular planner speed in rad/s that maps to full turn command.",
    )
    parser.add_argument(
        "--hold-linear-scale",
        type=float,
        default=0.35,
        help="Scale factor applied to forward speed while reusing the last known target.",
    )
    parser.add_argument(
        "--hold-max-linear-speed",
        type=float,
        default=0.20,
        help="Maximum forward speed in m/s while holding the last known target.",
    )
    parser.add_argument(
        "--turn-mix-gain",
        type=float,
        default=0.75,
        help="How strongly angular commands split left/right motor output.",
    )
    parser.add_argument(
        "--motor-deadband",
        type=float,
        default=0.05,
        help="Minimum normalized motor command magnitude before a side is stopped.",
    )
    return parser.parse_args()

if __name__ == "__main__":
    args = _parse_args()
    runtime_config = FusionRuntimeConfig(
        max_steps=args.max_steps,
        trace_path=args.trace_file,
        arm_motors=args.arm_motors,
        follow_distance_m=args.follow_distance,
        follow_distance_tolerance_m=args.follow_tolerance,
        target_min_distance_m=args.target_min_distance,
        obstacle_stop_distance_m=args.stop_distance,
        vision_hold_timeout_s=args.vision_hold_timeout,
        linear_speed_gain=args.linear_speed_gain,
        max_linear_speed_mps=args.max_linear_speed,
        max_angular_speed_rad_s=args.max_angular_speed,
        hold_linear_scale=args.hold_linear_scale,
        hold_max_linear_speed_mps=args.hold_max_linear_speed,
        turn_mix_gain=args.turn_mix_gain,
        motor_command_deadband=args.motor_deadband,
    )
    # Convert the user-facing on/off flags into booleans for the runtime code.
    run_control_loop(
        use_cv=args.cv == "on",
        use_ultrasonic=args.ultrasonic == "on",
        use_voice=args.voice == "on",
        start_cv=args.start_cv,
        start_voice=args.start_voice,
        runtime_config=runtime_config,
    )
