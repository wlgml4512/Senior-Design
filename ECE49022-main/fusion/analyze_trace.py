"""Post-run trace analysis for the fusion subsystem.

This script turns JSONL traces from the control loop into operator-facing
latency, tracking, obstacle-response, and loop-timing summaries, which makes it
the main offline validation tool for fusion behavior.
"""

import argparse
import json
import math
import statistics
from pathlib import Path


PASS = "PASS"
FAIL = "FAIL"
INSUFFICIENT = "INSUFFICIENT_DATA"


def _safe_get(mapping, *keys, default=None):
    value = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _sign(value, eps=1e-6):
    if value is None or abs(value) <= eps:
        return 0
    return 1 if value > 0 else -1


def _mean(values):
    return statistics.fmean(values) if values else None


def _stddev(values):
    if len(values) < 2:
        return 0.0 if values else None
    return statistics.pstdev(values)


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    index = (len(ordered) - 1) * percentile
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    lower_value = ordered[lower]
    upper_value = ordered[upper]
    weight = index - lower
    return lower_value + (upper_value - lower_value) * weight


def _format_seconds(value):
    if value is None:
        return "n/a"
    return f"{value * 1000.0:.1f} ms"


def _format_hz(value):
    if value is None:
        return "n/a"
    return f"{value:.2f} Hz"


def _format_meters(value):
    if value is None:
        return "n/a"
    return f"{value:.3f} m"


def _load_snapshots(trace_path):
    snapshots = []
    with trace_path.open("r", encoding="utf-8") as trace_file:
        for line_number, line in enumerate(trace_file, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                snapshot = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON on line {line_number}: {exc}") from exc
            snapshot["_line_number"] = line_number
            snapshots.append(snapshot)
    return snapshots


def _split_sessions(snapshots, session_gap_s):
    if not snapshots:
        return []

    sessions = []
    current_session = [snapshots[0]]
    previous = snapshots[0]

    for snapshot in snapshots[1:]:
        previous_step = previous.get("step")
        current_step = snapshot.get("step")
        previous_start = _safe_get(previous, "timing", "cycle_started_at")
        current_start = _safe_get(snapshot, "timing", "cycle_started_at")

        new_session = False
        if (
            previous_step is not None
            and current_step is not None
            and current_step == 0
            and previous_step != 0
        ):
            new_session = True
        elif (
            previous_start is not None
            and current_start is not None
            and current_start < previous_start
        ):
            new_session = True
        elif (
            previous_start is not None
            and current_start is not None
            and current_start - previous_start > session_gap_s
            and current_step == 0
        ):
            new_session = True

        if new_session:
            sessions.append(current_session)
            current_session = [snapshot]
        else:
            current_session.append(snapshot)

        previous = snapshot

    sessions.append(current_session)
    return sessions


def _result(status, summary, metrics=None, notes=None):
    return {
        "status": status,
        "summary": summary,
        "metrics": metrics or {},
        "notes": notes or [],
    }


def _analyze_real_time_fusion(samples, args):
    fusion_latencies = [
        _safe_get(sample, "timing", "input_to_fused_output_latency_s")
        for sample in samples
    ]
    fusion_latencies = [value for value in fusion_latencies if value is not None]

    motor_latencies = [
        _safe_get(sample, "timing", "input_to_motor_command_latency_s")
        for sample in samples
    ]
    motor_latencies = [value for value in motor_latencies if value is not None]

    if not fusion_latencies:
        return _result(
            INSUFFICIENT,
            "No sensor-receipt timestamps were present in this trace.",
            notes=[
                "Run with live CV and/or ultrasonic inputs so the trace includes sensor-to-fusion timing.",
            ],
        )

    max_fusion = max(fusion_latencies)
    max_motor = max(motor_latencies) if motor_latencies else None
    status = PASS
    if max_fusion > args.max_fusion_latency_s:
        status = FAIL
    if max_motor is not None and max_motor > args.max_motor_latency_s:
        status = FAIL

    return _result(
        status,
        "Measured end-to-end latency from sensor receipt through fusion and motor command generation.",
        metrics={
            "samples": len(fusion_latencies),
            "avg_sensor_to_fusion_s": _mean(fusion_latencies),
            "p95_sensor_to_fusion_s": _percentile(fusion_latencies, 0.95),
            "max_sensor_to_fusion_s": max_fusion,
            "avg_sensor_to_motor_s": _mean(motor_latencies),
            "p95_sensor_to_motor_s": _percentile(motor_latencies, 0.95),
            "max_sensor_to_motor_s": max_motor,
        },
    )


def _eligible_follow_samples(samples, stop_distance):
    eligible = []
    motion_enabled_count = 0
    tracked_or_held_count = 0

    for sample in samples:
        voice_state = sample.get("voice_state") or {}
        if not voice_state.get("motion_enabled", True):
            continue
        motion_enabled_count += 1

        target_state = sample.get("target_state")
        if target_state is not None:
            tracked_or_held_count += 1

        front_distance = _safe_get(sample, "obstacle_state", "front_distance")
        if front_distance is not None and front_distance < stop_distance:
            continue
        if target_state is None:
            continue
        eligible.append(sample)

    availability_ratio = None
    if motion_enabled_count > 0:
        availability_ratio = tracked_or_held_count / motion_enabled_count

    return eligible, availability_ratio


def _analyze_dynamic_following(samples, args):
    eligible, availability_ratio = _eligible_follow_samples(samples, args.stop_distance_m)
    if len(eligible) < args.min_follow_samples:
        return _result(
            INSUFFICIENT,
            "Not enough target-following samples without obstacle overrides.",
            metrics={
                "eligible_samples": len(eligible),
                "target_availability_ratio": availability_ratio,
            },
        )

    heading_cases = []
    heading_aligned = 0
    unnecessary_stops = 0

    for sample in eligible:
        target_angle = _safe_get(sample, "target_state", "angle")
        motion_angular = _safe_get(sample, "motion", "angular", default=0.0)
        motion_stop = bool(_safe_get(sample, "motion", "stop", default=False))

        if target_angle is not None and abs(target_angle) >= args.heading_deadband_deg:
            heading_cases.append((target_angle, motion_angular))
            if _sign(target_angle) == _sign(motion_angular):
                heading_aligned += 1

        target_distance = _safe_get(sample, "target_state", "distance")
        if (
            motion_stop
            and target_distance is not None
            and target_distance > args.target_min_distance_m
        ):
            unnecessary_stops += 1

    oscillation_cases = 0
    oscillation_flips = 0
    previous_angle = None
    previous_angular_cmd = None
    for sample in eligible:
        target_angle = _safe_get(sample, "target_state", "angle")
        motion_angular = _safe_get(sample, "motion", "angular", default=0.0)
        if (
            target_angle is None
            or abs(target_angle) < args.heading_deadband_deg
            or abs(motion_angular) < args.oscillation_min_angular
        ):
            previous_angle = target_angle
            previous_angular_cmd = motion_angular
            continue

        if (
            previous_angle is not None
            and abs(previous_angle) >= args.heading_deadband_deg
            and previous_angular_cmd is not None
            and abs(previous_angular_cmd) >= args.oscillation_min_angular
            and _sign(previous_angle) == _sign(target_angle)
        ):
            oscillation_cases += 1
            if _sign(previous_angular_cmd) != _sign(motion_angular):
                oscillation_flips += 1

        previous_angle = target_angle
        previous_angular_cmd = motion_angular

    heading_alignment_ratio = None
    if heading_cases:
        heading_alignment_ratio = heading_aligned / len(heading_cases)

    unnecessary_stop_ratio = unnecessary_stops / len(eligible)
    oscillation_ratio = None
    if oscillation_cases > 0:
        oscillation_ratio = oscillation_flips / oscillation_cases

    status = PASS
    if availability_ratio is not None and availability_ratio < args.min_target_availability_ratio:
        status = FAIL
    if heading_alignment_ratio is not None and heading_alignment_ratio < args.min_heading_alignment_ratio:
        status = FAIL
    if unnecessary_stop_ratio > args.max_unnecessary_stop_ratio:
        status = FAIL
    if oscillation_ratio is not None and oscillation_ratio > args.max_oscillation_ratio:
        status = FAIL

    return _result(
        status,
        "Heuristic follow-quality check based on heading corrections, stop behavior, and oscillation.",
        metrics={
            "eligible_samples": len(eligible),
            "target_availability_ratio": availability_ratio,
            "heading_cases": len(heading_cases),
            "heading_alignment_ratio": heading_alignment_ratio,
            "unnecessary_stop_ratio": unnecessary_stop_ratio,
            "oscillation_cases": oscillation_cases,
            "oscillation_ratio": oscillation_ratio,
        },
        notes=[
            "This spec still benefits from external trajectory or odometry data. The script uses trace-based heuristics only.",
        ],
    )


def _analyze_follow_distance(samples, args):
    distances = []
    for sample in samples:
        if not _safe_get(sample, "voice_state", "motion_enabled", default=True):
            continue
        target_state = sample.get("target_state")
        if target_state is None:
            continue
        front_distance = _safe_get(sample, "obstacle_state", "front_distance")
        if front_distance is not None and front_distance < args.stop_distance_m:
            continue
        target_distance = target_state.get("distance")
        if target_distance is None:
            continue
        distances.append(target_distance)

    if len(distances) < args.min_follow_samples:
        return _result(
            INSUFFICIENT,
            "Not enough target-distance samples to judge steady-state following distance.",
            metrics={"samples": len(distances)},
        )

    errors = [distance - args.follow_distance_m for distance in distances]
    abs_errors = [abs(error) for error in errors]
    within_tolerance = sum(1 for error in abs_errors if error <= args.follow_tolerance_m)
    within_tolerance_ratio = within_tolerance / len(abs_errors)
    mean_abs_error = _mean(abs_errors)

    status = PASS
    if mean_abs_error is not None and mean_abs_error > args.follow_tolerance_m:
        status = FAIL
    if within_tolerance_ratio < args.min_within_tolerance_ratio:
        status = FAIL

    return _result(
        status,
        "Compared logged target distance against the configured following distance.",
        metrics={
            "samples": len(distances),
            "mean_error_m": _mean(errors),
            "mean_abs_error_m": mean_abs_error,
            "stddev_distance_m": _stddev(distances),
            "p95_abs_error_m": _percentile(abs_errors, 0.95),
            "within_tolerance_ratio": within_tolerance_ratio,
        },
    )


def _analyze_obstacle_response(samples, args):
    events = []

    for index, sample in enumerate(samples):
        front_distance = _safe_get(sample, "obstacle_state", "front_distance")
        previous_front_distance = None
        if index > 0:
            previous_front_distance = _safe_get(samples[index - 1], "obstacle_state", "front_distance")

        if front_distance is None or front_distance >= args.stop_distance_m:
            continue
        if previous_front_distance is not None and previous_front_distance < args.stop_distance_m:
            continue

        detection_time = _safe_get(sample, "timing", "cycle_started_at")
        response_time = None
        unsafe_forward_samples = 0

        for future in samples[index:]:
            future_front = _safe_get(future, "obstacle_state", "front_distance")
            future_time = _safe_get(future, "timing", "cycle_started_at")
            motion_reason = _safe_get(future, "motion", "reason")
            motion_stop = bool(_safe_get(future, "motion", "stop", default=False))
            motion_linear = _safe_get(future, "motion", "linear", default=0.0)

            if future_front is not None and future_front < args.stop_distance_m and motion_linear > args.forward_motion_epsilon:
                unsafe_forward_samples += 1

            if motion_reason == "front_obstacle" or (motion_stop and motion_linear <= args.forward_motion_epsilon):
                if detection_time is not None and future_time is not None:
                    response_time = max(0.0, future_time - detection_time)
                else:
                    response_time = 0.0
                break

        events.append(
            {
                "detection_time": detection_time,
                "response_time_s": response_time,
                "unsafe_forward_samples": unsafe_forward_samples,
            }
        )

    if not events:
        return _result(
            INSUFFICIENT,
            "No obstacle-entry events were found in this trace.",
            notes=[
                "You need a run where front ultrasonic distance crosses below the stop threshold.",
            ],
        )

    response_times = [event["response_time_s"] for event in events if event["response_time_s"] is not None]
    max_response = max(response_times) if response_times else None
    total_unsafe_forward = sum(event["unsafe_forward_samples"] for event in events)

    status = PASS
    if max_response is None or max_response > args.max_obstacle_response_s:
        status = FAIL
    if total_unsafe_forward > 0:
        status = FAIL

    notes = [
        "Distance traveled after detection cannot be computed from the current trace alone because no odometry or integrated wheel displacement is logged.",
    ]

    return _result(
        status,
        "Measured how quickly the controller switched into obstacle-stop behavior after ultrasonic detection.",
        metrics={
            "events": len(events),
            "avg_response_s": _mean(response_times),
            "max_response_s": max_response,
            "unsafe_forward_samples": total_unsafe_forward,
        },
        notes=notes,
    )


def _analyze_loop_timing(samples, args):
    loop_hz = [_safe_get(sample, "timing", "loop_frequency_hz") for sample in samples]
    loop_hz = [value for value in loop_hz if value is not None]
    cycle_times = [_safe_get(sample, "timing", "cycle_elapsed_s") for sample in samples]
    cycle_times = [value for value in cycle_times if value is not None]
    deadline_misses = [_safe_get(sample, "timing", "deadline_miss_s") for sample in samples]
    deadline_misses = [value for value in deadline_misses if value is not None]
    dropped_cycles = [
        value for value in loop_hz if value < args.min_loop_hz * args.dropped_cycle_ratio_threshold
    ]

    if len(loop_hz) < args.min_timing_samples:
        return _result(
            INSUFFICIENT,
            "Not enough loop-timing samples were captured to assess control-loop consistency.",
            metrics={"samples": len(loop_hz)},
        )

    avg_loop_hz = _mean(loop_hz)
    max_deadline_miss = max(deadline_misses) if deadline_misses else None
    max_cycle_time = max(cycle_times) if cycle_times else None

    status = PASS
    if avg_loop_hz is not None and avg_loop_hz < args.min_loop_hz:
        status = FAIL
    if max_deadline_miss is not None and max_deadline_miss > args.max_deadline_miss_s:
        status = FAIL

    return _result(
        status,
        "Summarized the observed control-loop period, frequency, and missed-deadline behavior.",
        metrics={
            "samples": len(loop_hz),
            "avg_loop_hz": avg_loop_hz,
            "min_loop_hz": min(loop_hz) if loop_hz else None,
            "max_loop_hz": max(loop_hz) if loop_hz else None,
            "avg_cycle_elapsed_s": _mean(cycle_times),
            "max_cycle_elapsed_s": max_cycle_time,
            "max_deadline_miss_s": max_deadline_miss,
            "dropped_cycle_count": len(dropped_cycles),
        },
    )


def _analyze_vision_loss(samples, args):
    events = []

    for index, sample in enumerate(samples):
        if index == 0:
            continue

        previous_target = samples[index - 1].get("target_state")
        current_cv_state = _safe_get(sample, "cv", "tracking_state")

        if previous_target is None or previous_target.get("is_held", False):
            continue
        if current_cv_state not in {"waiting", "lost", "stale"}:
            continue

        current_target = sample.get("target_state")
        event = {
            "hold_started": current_target is not None and current_target.get("tracking_state") == "holding",
            "immediate_stop": bool(_safe_get(sample, "motion", "stop", default=False))
            and abs(_safe_get(sample, "motion", "linear", default=0.0)) <= args.forward_motion_epsilon
            and abs(_safe_get(sample, "motion", "angular", default=0.0)) <= args.angular_motion_epsilon,
            "hold_duration_s": 0.0,
            "reacquired": False,
        }

        start_time = _safe_get(sample, "timing", "cycle_started_at")
        last_hold_time = start_time

        for future in samples[index:]:
            future_time = _safe_get(future, "timing", "cycle_started_at")
            future_target = future.get("target_state")
            future_cv_state = _safe_get(future, "cv", "tracking_state")

            if future_target is not None and future_target.get("tracking_state") == "holding":
                last_hold_time = future_time

            if future_cv_state == "tracking" and future_target is not None:
                event["reacquired"] = True
                break

            if future_target is None and future_time is not None:
                break

        if start_time is not None and last_hold_time is not None:
            event["hold_duration_s"] = max(0.0, last_hold_time - start_time)

        events.append(event)

    if not events:
        return _result(
            INSUFFICIENT,
            "No vision-loss transitions were found in this trace.",
            notes=[
                "Run a test where the target is briefly blocked or removed from view.",
            ],
        )

    hold_started_ratio = sum(1 for event in events if event["hold_started"]) / len(events)
    immediate_stop_ratio = sum(1 for event in events if event["immediate_stop"]) / len(events)
    hold_durations = [event["hold_duration_s"] for event in events]
    reacquire_count = sum(1 for event in events if event["reacquired"])
    recovery_ratio = reacquire_count / len(events)
    avg_hold = _mean(hold_durations)

    status = PASS
    if hold_started_ratio < 1.0:
        status = FAIL
    if immediate_stop_ratio > args.max_immediate_stop_ratio:
        status = FAIL
    if avg_hold is not None and avg_hold < args.min_vision_hold_s:
        status = FAIL
    if avg_hold is not None and avg_hold > args.max_vision_hold_s:
        status = FAIL

    return _result(
        status,
        "Checked whether the controller preserved a short-lived held target during brief vision interruptions.",
        metrics={
            "events": len(events),
            "hold_started_ratio": hold_started_ratio,
            "immediate_stop_ratio": immediate_stop_ratio,
            "avg_hold_duration_s": avg_hold,
            "max_hold_duration_s": max(hold_durations) if hold_durations else None,
            "recovery_ratio": recovery_ratio,
        },
        notes=[
            "Recovery ratio only proves reacquisition when the trace contains a later return to tracking.",
        ],
    )


def _build_parser():
    parser = argparse.ArgumentParser(description="Summarize a fusion JSONL trace into spec-oriented metrics.")
    parser.add_argument("trace_file", type=Path, help="Path to the JSONL trace emitted by fusion/main.py --trace-file.")
    parser.add_argument(
        "--session-gap-s",
        type=float,
        default=2.0,
        help="Gap threshold used to split a trace file into multiple sessions.",
    )
    parser.add_argument(
        "--session-index",
        type=int,
        default=-1,
        help="Which detected session to analyze. Default -1 means the most recent session.",
    )
    parser.add_argument("--follow-distance-m", type=float, default=1.0)
    parser.add_argument("--follow-tolerance-m", type=float, default=0.10)
    parser.add_argument("--target-min-distance-m", type=float, default=0.35)
    parser.add_argument("--stop-distance-m", type=float, default=0.35)
    parser.add_argument("--max-fusion-latency-s", type=float, default=0.10)
    parser.add_argument("--max-motor-latency-s", type=float, default=0.10)
    parser.add_argument("--max-obstacle-response-s", type=float, default=0.10)
    parser.add_argument("--min-loop-hz", type=float, default=10.0)
    parser.add_argument("--max-deadline-miss-s", type=float, default=0.02)
    parser.add_argument("--min-vision-hold-s", type=float, default=1.0)
    parser.add_argument("--max-vision-hold-s", type=float, default=2.0)
    parser.add_argument("--heading-deadband-deg", type=float, default=4.0)
    parser.add_argument("--oscillation-min-angular", type=float, default=0.10)
    parser.add_argument("--forward-motion-epsilon", type=float, default=0.01)
    parser.add_argument("--angular-motion-epsilon", type=float, default=0.05)
    parser.add_argument("--min-follow-samples", type=int, default=10)
    parser.add_argument("--min-timing-samples", type=int, default=5)
    parser.add_argument("--min-heading-alignment-ratio", type=float, default=0.85)
    parser.add_argument("--max-unnecessary-stop-ratio", type=float, default=0.10)
    parser.add_argument("--max-oscillation-ratio", type=float, default=0.35)
    parser.add_argument("--min-target-availability-ratio", type=float, default=0.70)
    parser.add_argument("--min-within-tolerance-ratio", type=float, default=0.70)
    parser.add_argument("--dropped-cycle-ratio-threshold", type=float, default=0.70)
    parser.add_argument("--max-immediate-stop-ratio", type=float, default=0.0)
    return parser


def _session_summary(session, session_number, session_count):
    first_time = _safe_get(session[0], "timing", "cycle_started_at")
    last_time = _safe_get(session[-1], "timing", "cycle_started_at")
    duration = None
    if first_time is not None and last_time is not None:
        duration = max(0.0, last_time - first_time)
    return {
        "session_number": session_number,
        "session_count": session_count,
        "samples": len(session),
        "duration_s": duration,
        "first_step": session[0].get("step"),
        "last_step": session[-1].get("step"),
    }


def _print_metric(key, value):
    if key.endswith("_s"):
        pretty = _format_seconds(value)
    elif key.endswith("_hz"):
        pretty = _format_hz(value)
    elif key.endswith("_m"):
        pretty = _format_meters(value)
    elif key.endswith("_ratio"):
        pretty = "n/a" if value is None else f"{value:.2%}"
    else:
        pretty = "n/a" if value is None else str(value)
    print(f"    {key}: {pretty}")


def main():
    parser = _build_parser()
    args = parser.parse_args()

    snapshots = _load_snapshots(args.trace_file)
    if not snapshots:
        raise SystemExit("Trace file is empty.")

    sessions = _split_sessions(snapshots, args.session_gap_s)
    try:
        session = sessions[args.session_index]
    except IndexError as exc:
        raise SystemExit(f"Session index {args.session_index} is out of range for {len(sessions)} detected sessions.") from exc

    summary = _session_summary(
        session,
        session_number=(args.session_index if args.session_index >= 0 else len(sessions) + args.session_index) + 1,
        session_count=len(sessions),
    )

    analyses = [
        ("1. Real-time Fusion", _analyze_real_time_fusion(session, args)),
        ("2. Dynamic Path Following", _analyze_dynamic_following(session, args)),
        ("3. Follow Distance", _analyze_follow_distance(session, args)),
        ("4. Obstacle Response", _analyze_obstacle_response(session, args)),
        ("5. Control Loop Timing", _analyze_loop_timing(session, args)),
        ("6. Vision-Loss Handling", _analyze_vision_loss(session, args)),
    ]

    print(f"Trace file: {args.trace_file}")
    print(
        "Session: "
        f"{summary['session_number']}/{summary['session_count']} | "
        f"samples={summary['samples']} | "
        f"duration={_format_seconds(summary['duration_s'])} | "
        f"steps={summary['first_step']}..{summary['last_step']}"
    )
    print()

    status_counts = {PASS: 0, FAIL: 0, INSUFFICIENT: 0}

    for title, result in analyses:
        status_counts[result["status"]] += 1
        print(f"{title}: {result['status']}")
        print(f"  {result['summary']}")
        for key, value in result["metrics"].items():
            _print_metric(key, value)
        for note in result["notes"]:
            print(f"  Note: {note}")
        print()

    print(
        "Summary: "
        f"{PASS}={status_counts[PASS]} | "
        f"{FAIL}={status_counts[FAIL]} | "
        f"{INSUFFICIENT}={status_counts[INSUFFICIENT]}"
    )


if __name__ == "__main__":
    main()
