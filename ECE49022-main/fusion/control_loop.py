"""Main runtime loop for fused robot decision-making.

This file is the central coordinator for ATLAS at runtime: it reads every
subsystem input, computes the final motion command, reports what happened, and
optionally records traces for later analysis.
"""

import json
import time
from pathlib import Path

from interfaces.cv_interface import CV_LOG_PATH, get_cv_data, reset_cv_stream, start_cv_process
from interfaces.ultrasonic_interface import get_ultrasonic_data
from interfaces.motor_interface import (
    configure_motor_mapping,
    send_motion_command,
    set_motor_output_enabled,
)
from interfaces.voice_interface import (
    VOICE_COMMAND_LOG_PATH,
    VOICE_EMBEDDED_COMMAND_LOG_PATH,
    get_voice_commands,
    reset_voice_stream,
    start_voice_recognizer,
    stop_voice_recognizer,
)

from fusion import fuse_data
from motion_priority import arbitrate_motion
from planner import compute_motion
from runtime import FusionRuntimeConfig
from voice_control import VoiceCommandController

LOOP_HZ = 10


def _disabled_cv_data():
    # Match the normal CV interface shape so downstream code can stay branch-free.
    return {
        "timestamp": None,
        "received_at": None,
        "tracking_state": "disabled",
        "target_id": None,
        "angle": None,
        "distance": None,
        "is_live": False,
    }


def _disabled_ultrasonic_data():
    # Use the same obstacle schema even when ultrasonic sensing is disabled.
    return {
        "timestamp": None,
        "received_at": None,
        "left_distance": None,
        "right_distance": None,
        "front_distance": None,
    }


def _make_json_safe(value):
    if isinstance(value, dict):
        return {key: _make_json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_make_json_safe(item) for item in value]
    if isinstance(value, Path):
        return str(value)
    return value


def _append_trace(trace_path, snapshot):
    if trace_path is None:
        return

    # Save one JSON object per cycle so runs can be inspected or replayed later
    # without relying on terminal output.
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    with trace_path.open("a", encoding="utf-8") as trace_file:
        trace_file.write(json.dumps(_make_json_safe(snapshot)) + "\n")


def _format_scalar(value, suffix="", precision=2, signed=False):
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, float):
        number = f"{value:+.{precision}f}" if signed else f"{value:.{precision}f}"
        return f"{number}{suffix}"
    if isinstance(value, int):
        number = f"{value:+d}" if signed else str(value)
        return f"{number}{suffix}"
    return str(value)


def _format_age(timestamp):
    if timestamp is None:
        return "n/a"
    return f"{max(0.0, time.time() - timestamp):.2f}s ago"


def _max_timestamp(*timestamps):
    present = [timestamp for timestamp in timestamps if timestamp is not None]
    return max(present) if present else None


def _print_runtime_config(runtime_config, use_cv, use_ultrasonic, use_voice, start_cv, start_voice):
    print("Runtime configuration:")
    print(f"  CV: enabled={use_cv} start_process={start_cv}")
    print(f"  Ultrasonic: enabled={use_ultrasonic}")
    print(f"  Voice: enabled={use_voice} start_process={start_voice}")
    print(f"  Loop: hz={runtime_config.loop_hz} dt={runtime_config.dt:.2f}s")
    print(
        "  Planner: "
        f"follow_distance={runtime_config.follow_distance_m:.2f} m "
        f"tolerance={runtime_config.follow_distance_tolerance_m:.2f} m "
        f"target_min={runtime_config.target_min_distance_m:.2f} m "
        f"stop_distance={runtime_config.obstacle_stop_distance_m:.2f} m"
    )
    print(
        "  Motion scaling: "
        f"linear_gain={runtime_config.linear_speed_gain:.2f} "
        f"max_linear={runtime_config.max_linear_speed_mps:.2f} m/s "
        f"max_angular={runtime_config.max_angular_speed_rad_s:.2f} rad/s "
        f"hold_scale={runtime_config.hold_linear_scale:.2f} "
        f"hold_max={runtime_config.hold_max_linear_speed_mps:.2f} m/s "
        f"turn_mix={runtime_config.turn_mix_gain:.2f} "
        f"deadband={runtime_config.motor_command_deadband:.2f}"
    )
    print(f"  Vision hold timeout: {runtime_config.vision_hold_timeout_s:.2f}s")
    print(f"  Motors armed: {runtime_config.arm_motors}")
    print(f"  Max steps: {runtime_config.max_steps if runtime_config.max_steps is not None else 'none'}")
    print(f"  Trace file: {runtime_config.trace_path if runtime_config.trace_path is not None else 'none'}")


def _print_voice_event(event, result):
    status = "accepted" if event.get("recognized") else "ignored"
    action = (event.get("command") or {}).get("action", "unrecognized")
    print(
        f"[VOICE CMD] {status} | text='{event.get('text', '')}' | "
        f"action={action} | result={result} | "
        f"confidence={_format_scalar(event.get('confidence'))} | source={event.get('source', 'unknown')}"
    )


def _print_step_summary(step_count, snapshot):
    cv = snapshot["cv"]
    ultrasonic = snapshot["ultrasonic"]
    voice_state = snapshot["voice_state"]
    voice_results = snapshot["voice_results"]
    target_state = snapshot["target_state"]
    planned_motion = snapshot["planned_motion"]
    motion = snapshot["motion"]
    motor_report = snapshot["motor_report"]
    obstacle_state = snapshot["obstacle_state"]
    timing = snapshot["timing"]

    print()
    print(f"=== Fusion Step {step_count} ===")
    print(
        "[CV] "
        f"state={cv['tracking_state']} | live={_format_scalar(cv['is_live'])} | "
        f"target_id={_format_scalar(cv['target_id'])} | "
        f"angle={_format_scalar(cv['angle'], ' deg', signed=True)} | "
        f"distance={_format_scalar(cv['distance'], ' m')} | "
        f"updated={_format_age(cv['timestamp'])}"
    )
    print(
        "[ULTRASONIC] "
        f"left={_format_scalar(ultrasonic['left_distance'], ' m')} | "
        f"front={_format_scalar(ultrasonic['front_distance'], ' m')} | "
        f"right={_format_scalar(ultrasonic['right_distance'], ' m')}"
    )
    print(
        "[VOICE] "
        f"follow_active={_format_scalar(voice_state['follow_active'])} | "
        f"emergency_stop={_format_scalar(voice_state.get('emergency_stop_active', False))} | "
        f"speed_multiplier={_format_scalar(voice_state['speed_multiplier'])} | "
        f"last_command={voice_state['last_command']['action'] if voice_state['last_command'] else 'none'}"
    )
    if voice_results:
        for item in voice_results:
            event = item["event"]
            action = (event.get("command") or {}).get("action", "unrecognized")
            print(
                "  [VOICE EVENT] "
                f"text='{event.get('text', '')}' | action={action} | "
                f"result={item['result']} | source={event.get('source', 'unknown')}"
            )
    else:
        print("  [VOICE EVENT] no new voice commands this cycle")

    if target_state is None:
        target_summary = "none"
    else:
        target_summary = (
            f"id={_format_scalar(target_state.get('target_id'))}, "
            f"state={target_state.get('tracking_state', 'unknown')}, "
            f"angle={_format_scalar(target_state.get('angle'), ' deg', signed=True)}, "
            f"distance={_format_scalar(target_state.get('distance'), ' m')}"
        )
    print(f"[FUSION] target={target_summary}")
    print(
        "[FUSION] "
        f"obstacle_front={_format_scalar(obstacle_state['front_distance'], ' m')} | "
        f"planned_reason={planned_motion.get('reason')} | "
        f"planned_linear={_format_scalar(planned_motion.get('linear'), ' m/s')} | "
        f"planned_angular={_format_scalar(planned_motion.get('angular'), ' rad/s', signed=True)}"
    )
    print(
        "[MOTION] "
        f"priority={motion.get('priority_source')} | reason={motion.get('reason')} | "
        f"linear={_format_scalar(motion.get('linear'), ' m/s')} | "
        f"angular={_format_scalar(motion.get('angular'), ' rad/s', signed=True)} | "
        f"stop={_format_scalar(motion.get('stop'))}"
    )
    print(
        "[MOTOR] "
        f"backend={motor_report['backend_report']['backend']} | "
        f"left={_format_scalar(motor_report['left_motor'])} | "
        f"right={_format_scalar(motor_report['right_motor'])} | "
        f"linear_norm={_format_scalar(motor_report['linear_normalized'])} | "
        f"angular_norm={_format_scalar(motor_report['angular_normalized'], signed=True)}"
    )
    print(
        "[TIMING] "
        f"sensor_to_fusion={_format_scalar(timing.get('input_to_fused_output_latency_s'), ' s', precision=3)} | "
        f"sensor_to_motor={_format_scalar(timing.get('input_to_motor_command_latency_s'), ' s', precision=3)} | "
        f"cycle={_format_scalar(timing.get('cycle_elapsed_s'), ' s', precision=3)} | "
        f"loop_hz={_format_scalar(timing.get('loop_frequency_hz'), precision=2)} | "
        f"deadline_miss={_format_scalar(timing.get('deadline_miss_s'), ' s', precision=3)}"
    )


def run_control_step(
    cv_data,
    ultrasonic_data,
    voice_events,
    last_target,
    voice_controller,
    planner_config=None,
    fusion_config=None,
):
    # This helper mirrors one loop iteration so the core fusion logic can be
    # exercised in short tests without running the whole infinite loop.
    for event in voice_events:
        voice_controller.apply_command(event)

    decision_started_at = time.time()
    decision_started_perf = time.perf_counter()
    target_state, obstacle_state = fuse_data(
        cv_data,
        ultrasonic_data,
        last_target,
        fusion_config=fusion_config,
        now=decision_started_at,
    )
    fused_output_generated_at = time.time()

    if target_state is not None and not target_state.get("is_held", False):
        next_last_target = target_state
    else:
        next_last_target = last_target

    planned_motion = compute_motion(target_state, obstacle_state, planner_config=planner_config)
    motion_cmd = arbitrate_motion(planned_motion, voice_controller)
    motor_report = send_motion_command(motion_cmd)
    motor_command_generated_at = time.time()
    latest_input_received_at = _max_timestamp(
        (
            cv_data.get("timestamp") or cv_data.get("received_at")
            if cv_data is not None
            else None
        ),
        (
            ultrasonic_data.get("timestamp") or ultrasonic_data.get("received_at")
            if ultrasonic_data is not None
            else None
        ),
    )

    return {
        "target_state": target_state,
        "obstacle_state": obstacle_state,
        "planned_motion": planned_motion,
        "motion": motion_cmd,
        "motor_report": motor_report,
        "voice_state": voice_controller.snapshot(),
        "last_target": next_last_target,
        "timing": {
            "decision_started_at": decision_started_at,
            "fused_output_generated_at": fused_output_generated_at,
            "motor_command_generated_at": motor_command_generated_at,
            "decision_elapsed_s": time.perf_counter() - decision_started_perf,
            "input_to_fused_output_latency_s": (
                None
                if latest_input_received_at is None
                else max(0.0, fused_output_generated_at - latest_input_received_at)
            ),
            "input_to_motor_command_latency_s": (
                None
                if latest_input_received_at is None
                else max(0.0, motor_command_generated_at - latest_input_received_at)
            ),
        },
    }


def run_control_loop(
    use_cv=True,
    use_ultrasonic=True,
    use_voice=False,
    start_cv=False,
    start_voice=False,
    runtime_config=None,
):
    runtime_config = runtime_config or FusionRuntimeConfig(loop_hz=LOOP_HZ)
    # Keep motor arming under runtime control instead of tying it to whether the
    # embedded backend happens to be importable on this machine.
    set_motor_output_enabled(runtime_config.arm_motors)
    print("Starting navigation control loop...")
    print("Press Ctrl+C to stop the fusion loop.")
    _print_runtime_config(
        runtime_config=runtime_config,
        use_cv=use_cv,
        use_ultrasonic=use_ultrasonic,
        use_voice=use_voice,
        start_cv=start_cv,
        start_voice=start_voice,
    )

    # Store the most recent usable target so a short CV dropout does not immediately
    # erase the follow target.
    last_target = None
    voice_controller = VoiceCommandController(follow_enabled=not use_voice)
    if use_cv:
        if start_cv:
            print(f"CV enabled: starting computer vision and reading live data from {CV_LOG_PATH}")
        else:
            print(f"CV enabled: waiting for an external computer vision process to update {CV_LOG_PATH}")
    else:
        print("CV disabled.")

    if use_ultrasonic:
        print("Ultrasonic enabled.")
    else:
        print("Ultrasonic disabled.")

    if use_voice:
        if start_voice:
            print(
                "Voice enabled: starting voice_controller.py and monitoring voice command logs "
                f"in {VOICE_COMMAND_LOG_PATH} and {VOICE_EMBEDDED_COMMAND_LOG_PATH}"
            )
        else:
            print(
                "Voice enabled: waiting for voice commands in "
                f"{VOICE_COMMAND_LOG_PATH} and {VOICE_EMBEDDED_COMMAND_LOG_PATH}"
            )
    else:
        print("Voice disabled.")

    if runtime_config.arm_motors:
        print("Motor output armed.")
    else:
        print("Motor output disarmed: commands will be computed and logged but not sent to hardware.")

    if use_cv:
        # Start reading from the end of the current log so old detections are ignored.
        reset_cv_stream()
    if use_voice:
        reset_voice_stream()
        if not voice_controller.is_follow_active():
            print("Voice navigation is idle until a 'follow' command is received.")
    cv_process = start_cv_process() if use_cv and start_cv else None
    voice_recognizer = start_voice_recognizer() if use_voice and start_voice else None
    cv_process_reported_dead = False
    voice_process_reported_dead = False
    no_cv_data_reported = False
    step_count = 0
    previous_cycle_started_at = None
    planner_config = runtime_config.planner_config()
    fusion_config = runtime_config.fusion_config()
    configure_motor_mapping(
        max_linear_speed_mps=runtime_config.max_linear_speed_mps,
        max_angular_speed_rad_s=runtime_config.max_angular_speed_rad_s,
        turn_mix_gain=runtime_config.turn_mix_gain,
        min_command_deadband=runtime_config.motor_command_deadband,
    )

    try:
        while True:
            cycle_started_at = time.time()
            cycle_started_perf = time.perf_counter()

            if cv_process is not None and cv_process.poll() is not None and not cv_process_reported_dead:
                print(f"Computer vision process exited with code {cv_process.returncode}.")
                cv_process_reported_dead = True
            if (
                voice_recognizer is not None
                and hasattr(voice_recognizer, "poll")
                and voice_recognizer.poll() is not None
                and not voice_process_reported_dead
            ):
                print(f"Voice controller process exited with code {voice_recognizer.returncode}.")
                voice_process_reported_dead = True

            # 1. Read inputs
            cv_data = get_cv_data() if use_cv else _disabled_cv_data()
            ultrasonic_data = get_ultrasonic_data() if use_ultrasonic else _disabled_ultrasonic_data()
            voice_events = get_voice_commands() if use_voice else []

            if use_cv and cv_data["timestamp"] is None and not no_cv_data_reported:
                print(
                    "No live CV data received yet. "
                    "Make sure computer-vision/main.py is running, the camera opened successfully, "
                    "and the ArUco target is actually being detected."
                )
                no_cv_data_reported = True
            elif use_cv and cv_data["timestamp"] is not None and no_cv_data_reported:
                print("Live CV data received.")
                no_cv_data_reported = False

            if use_voice and start_voice and voice_recognizer is None and not voice_process_reported_dead:
                print(
                    "Voice controller could not be started. "
                    "Fusion will keep running and continue to accept commands from the voice command logs."
                )
                voice_process_reported_dead = True

            voice_results = []
            for event in voice_events:
                voice_result = voice_controller.apply_command(event)
                voice_results.append({"event": event, "result": voice_result})
                _print_voice_event(event, voice_result)

            step_result = run_control_step(
                cv_data=cv_data,
                ultrasonic_data=ultrasonic_data,
                voice_events=[],
                last_target=last_target,
                voice_controller=voice_controller,
                planner_config=planner_config,
                fusion_config=fusion_config,
            )
            last_target = step_result["last_target"]
            target_state = step_result["target_state"]
            obstacle_state = step_result["obstacle_state"]
            planned_motion = step_result["planned_motion"]
            motion_cmd = step_result["motion"]
            motor_report = step_result["motor_report"]
            timing = dict(step_result["timing"])
            loop_period_s = (
                None if previous_cycle_started_at is None else cycle_started_at - previous_cycle_started_at
            )
            loop_frequency_hz = None if loop_period_s in (None, 0) else 1.0 / loop_period_s
            cycle_elapsed_s = time.perf_counter() - cycle_started_perf
            timing.update(
                {
                    "cycle_started_at": cycle_started_at,
                    "cycle_elapsed_s": cycle_elapsed_s,
                    "loop_period_s": loop_period_s,
                    "loop_frequency_hz": loop_frequency_hz,
                    "deadline_miss_s": max(0.0, cycle_elapsed_s - runtime_config.dt),
                }
            )

            # Debug print
            snapshot = {
                "step": step_count,
                "cv": cv_data,
                "ultrasonic": ultrasonic_data,
                "voice_commands": voice_events,
                "voice_results": voice_results,
                "target_state": target_state,
                "obstacle_state": obstacle_state,
                "planned_motion": planned_motion,
                "voice_state": voice_controller.snapshot(),
                "motion": motion_cmd,
                "motor_report": motor_report,
                "timing": timing,
            }
            _print_step_summary(step_count, snapshot)
            active_cycle_elapsed_s = time.perf_counter() - cycle_started_perf
            sleep_duration_s = max(0, runtime_config.dt - active_cycle_elapsed_s)
            timing.update(
                {
                    "cycle_elapsed_s": active_cycle_elapsed_s,
                    "cycle_completed_at": time.time(),
                    "sleep_duration_s": sleep_duration_s,
                    "deadline_miss_s": max(0.0, active_cycle_elapsed_s - runtime_config.dt),
                }
            )
            snapshot["timing"] = timing
            # Persist the full decision record for each loop when tracing is
            # enabled, which makes on-robot debugging much easier.
            _append_trace(runtime_config.trace_path, snapshot)
            step_count += 1
            previous_cycle_started_at = cycle_started_at

            if runtime_config.max_steps is not None and step_count >= runtime_config.max_steps:
                print(f"Reached max_steps={runtime_config.max_steps}. Stopping fusion loop.")
                break

            # Keep the loop close to the requested update rate.
            time.sleep(sleep_duration_s)
    except KeyboardInterrupt:
        print("\nStopping navigation control loop...")
    finally:
        # Always publish one last stop command before shutting down.
        send_motion_command({"linear": 0.0, "angular": 0.0, "stop": True})
        set_motor_output_enabled(False)
        stop_voice_recognizer(voice_recognizer)
        if cv_process is not None and cv_process.poll() is None:
            cv_process.terminate()
            try:
                cv_process.wait(timeout=2.0)
            except Exception:
                cv_process.kill()
        print("Fusion shutdown complete.")
