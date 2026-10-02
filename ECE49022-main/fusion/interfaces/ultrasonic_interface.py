"""Ultrasonic-sensor interface for fusion.

This adapter hides the embedded module shape and always returns distances in
meters with a consistent left/right/front schema.
"""

from pathlib import Path
import importlib
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _load_ultrasonic_sensor():
    """Import the embedded ultrasonic module only when a reading is needed."""
    # Import lazily so fusion can still run on machines without GPIO hardware.
    try:
        return importlib.import_module("embedded.ultrasonic_sensor")
    except Exception:
        return None


def _cm_to_m(distance_cm):
    """Convert embedded centimeter readings into meters for fusion."""
    if distance_cm is None or distance_cm < 0:
        return None
    return distance_cm / 100.0


def _read_distance(reader, *args):
    """Call an ultrasonic reader and convert its result into meters."""
    if reader is None:
        return None
    try:
        return _cm_to_m(reader(*args))
    except Exception:
        return None


def _read_named_distance(ultrasonic_sensor, function_name):
    """Read one named distance function if that sensor entry exists."""
    if ultrasonic_sensor is None:
        return None
    reader = getattr(ultrasonic_sensor, function_name, None)
    return _read_distance(reader)


def _get_sensor_pin_pair(ultrasonic_sensor, *pin_name_pairs):
    """Resolve one sensor's trigger/echo pins from the embedded module."""
    if ultrasonic_sensor is None:
        return None

    for trig_name, echo_name in pin_name_pairs:
        trig_pin = getattr(ultrasonic_sensor, trig_name, None)
        echo_pin = getattr(ultrasonic_sensor, echo_name, None)
        if trig_pin is not None and echo_pin is not None:
            return trig_pin, echo_pin

    return None


def _read_sensor_pair_distance(ultrasonic_sensor, *pin_name_pairs):
    """Read distance from a generic get_distance(trig, echo) style API."""
    if ultrasonic_sensor is None:
        return None

    reader = getattr(ultrasonic_sensor, "get_distance", None)
    sensor_pair = _get_sensor_pin_pair(ultrasonic_sensor, *pin_name_pairs)
    if reader is None or sensor_pair is None:
        return None

    trig_pin, echo_pin = sensor_pair
    return _read_distance(reader, trig_pin, echo_pin)


def get_ultrasonic_data():
    """Return the latest normalized ultrasonic snapshot.

    If the embedded subsystem only exposes one front-facing helper, fusion uses
    that directly. If separate left/right helpers exist, fusion keeps both and
    also derives a conservative front distance from the closer of the two.
    """
    ultrasonic_sensor = _load_ultrasonic_sensor()
    left_distance_m = _read_named_distance(ultrasonic_sensor, "sensor_left_distance")
    right_distance_m = _read_named_distance(ultrasonic_sensor, "sensor_right_distance")

    # Support the newer embedded API shape where one shared get_distance(trig, echo)
    # helper is paired with per-sensor pin constants instead of dedicated reader
    # functions.
    if left_distance_m is None:
        left_distance_m = _read_sensor_pair_distance(
            ultrasonic_sensor,
            ("LEFT_TRIG", "LEFT_ECHO"),
            ("TRIG_LEFT", "ECHO_LEFT"),
            ("TRIG1", "ECHO1"),
        )
    if right_distance_m is None:
        right_distance_m = _read_sensor_pair_distance(
            ultrasonic_sensor,
            ("RIGHT_TRIG", "RIGHT_ECHO"),
            ("TRIG_RIGHT", "ECHO_RIGHT"),
            ("TRIG2", "ECHO2"),
        )

    # Fall back to a single front-facing reading if distinct left/right sensor
    # functions are not available in the embedded module yet.
    if left_distance_m is None and right_distance_m is None:
        front_reader = getattr(ultrasonic_sensor, "get_distance", None) if ultrasonic_sensor is not None else None
        front_distance_m = _read_distance(front_reader)
    else:
        # If both sides exist, treat the closest one as the effective front obstacle.
        front_candidates = [distance for distance in (left_distance_m, right_distance_m) if distance is not None]
        front_distance_m = min(front_candidates) if front_candidates else None
    measurement_completed_at = time.time()

    return {
        # For synchronous ultrasonic reads, this timestamp is a reasonable proxy
        # for when the sample was actually measured.
        "timestamp": measurement_completed_at,
        "received_at": measurement_completed_at,
        "left_distance": left_distance_m,
        "right_distance": right_distance_m,
        "front_distance": front_distance_m,
    }
