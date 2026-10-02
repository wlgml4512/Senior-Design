"""Computer-vision interface for fusion.

Fusion does not talk to the vision code through direct function calls. Instead,
it tails the CV subsystem's log file and converts each recognized log line into
one consistent dictionary schema for the rest of the control loop.
"""

from datetime import datetime
from pathlib import Path
import re
import subprocess
import sys
import time


REPO_ROOT = Path(__file__).resolve().parents[2]
CV_DIR = REPO_ROOT / "computer-vision"
CV_LOG_PATH = CV_DIR / "log.txt"
CV_MAIN_PATH = CV_DIR / "main.py"

# Fusion reads the vision subsystem indirectly by tailing its log output.
INFO_PATTERN = re.compile(
    r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) "
    r"\[(?P<level>[A-Z]+)\] ID: (?P<target_id>\d+) \| "
    r"Angle: (?P<angle>[+-]?\d+(?:\.\d+)?) deg \| "
    r"Dist: (?P<distance>\d+(?:\.\d+)?) m$"
)
WAITING_PATTERN = re.compile(
    r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) "
    r"\[(?P<level>[A-Z]+)\] waiting for target$"
)
LOST_PATTERN = re.compile(
    r"^(?P<timestamp>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}) "
    r"\[(?P<level>[A-Z]+)\] lost target$"
)

STALE_TIMEOUT_S = 0.5

_cv_log_position = 0
_last_cv_data = {
    "timestamp": None,
    "tracking_state": "lost",
    "target_id": None,
    "angle": None,
    "distance": None,
    "is_live": False,
}


def _empty_cv_data(tracking_state="lost"):
    """Return the canonical "no target" snapshot used by fusion."""
    # Centralize the "no usable target" shape so every caller sees the same
    # schema whether CV is absent, stale, waiting, or lost.
    return {
        "timestamp": None,
        "received_at": None,
        "tracking_state": tracking_state,
        "target_id": None,
        "angle": None,
        "distance": None,
        "is_live": False,
    }


def _parse_timestamp(timestamp_str):
    """Convert the CV logger timestamp into Unix seconds."""
    dt = datetime.strptime(timestamp_str, "%Y-%m-%d %H:%M:%S,%f")
    return dt.timestamp()


def _parse_cv_line(line):
    """Parse one CV log line into structured target-tracking state."""
    line = line.strip()
    if not line:
        return None

    # These regexes convert the vision process' log lines into structured state.
    info_match = INFO_PATTERN.match(line)
    if info_match:
        return {
            "timestamp": _parse_timestamp(info_match.group("timestamp")),
            "tracking_state": "tracking",
            "target_id": int(info_match.group("target_id")),
            "angle": float(info_match.group("angle")),
            "distance": float(info_match.group("distance")),
            "is_live": True,
        }

    waiting_match = WAITING_PATTERN.match(line)
    if waiting_match:
        return {
            "timestamp": _parse_timestamp(waiting_match.group("timestamp")),
            "tracking_state": "waiting",
            "target_id": None,
            "angle": None,
            "distance": None,
            "is_live": True,
        }

    lost_match = LOST_PATTERN.match(line)
    if lost_match:
        return {
            "timestamp": _parse_timestamp(lost_match.group("timestamp")),
            "tracking_state": "lost",
            "target_id": None,
            "angle": None,
            "distance": None,
            "is_live": True,
        }

    return None


def _mark_stale_if_needed(cv_data):
    """Re-label cached CV data as stale when the log stops updating."""
    if cv_data["timestamp"] is None:
        return cv_data

    # If the vision process stops writing, expose that explicitly as "stale".
    age = time.time() - cv_data["timestamp"]
    if age <= STALE_TIMEOUT_S:
        return cv_data

    stale_data = _empty_cv_data("stale")
    stale_data["timestamp"] = cv_data["timestamp"]
    return stale_data


def reset_cv_stream():
    """Start reading the CV log from the current end of file."""
    global _cv_log_position, _last_cv_data

    _last_cv_data = _empty_cv_data("lost")
    if CV_LOG_PATH.exists():
        # Ignore old log history and only consume lines written after fusion starts.
        _cv_log_position = CV_LOG_PATH.stat().st_size
    else:
        _cv_log_position = 0


def get_cv_data():
    """Return the latest normalized CV snapshot for the current loop step."""
    global _cv_log_position, _last_cv_data

    if not CV_LOG_PATH.exists():
        # If the log has not been created yet, keep returning the last known CV
        # state so the rest of fusion does not need a special startup case.
        cv_snapshot = _mark_stale_if_needed(dict(_last_cv_data))
        cv_snapshot["received_at"] = time.time()
        return cv_snapshot

    current_size = CV_LOG_PATH.stat().st_size
    if current_size < _cv_log_position:
        # If the CV process rotated or truncated the log, restart from the
        # beginning of the new file instead of silently missing fresh entries.
        _cv_log_position = 0

    with CV_LOG_PATH.open("r", encoding="utf-8") as log_file:
        log_file.seek(_cv_log_position)
        for line in log_file:
            parsed = _parse_cv_line(line)
            if parsed is None:
                # Ignore unrelated log lines so fusion only reacts to the
                # structured messages that encode tracker state.
                continue

            if parsed["tracking_state"] == "tracking":
                # A fully tracked target replaces the cached state completely.
                _last_cv_data = parsed
            elif parsed["tracking_state"] == "waiting":
                # Waiting means "reuse the last good target, but mark it as not freshly seen."
                waiting_state = dict(_last_cv_data)
                waiting_state.update(
                    {
                        "timestamp": parsed["timestamp"],
                        "tracking_state": "waiting",
                        "is_live": True,
                    }
                )
                _last_cv_data = waiting_state
            else:
                # "lost" explicitly clears the target geometry instead of
                # letting the old target linger indefinitely.
                _last_cv_data = parsed

        _cv_log_position = log_file.tell()

    cv_snapshot = _mark_stale_if_needed(dict(_last_cv_data))
    cv_snapshot["received_at"] = time.time()
    return cv_snapshot


def start_cv_process():
    """Launch the CV subsystem as a child process when fusion manages it."""
    if not CV_MAIN_PATH.exists():
        return None

    # Launch the vision script as a sibling process when fusion is asked to manage it.
    return subprocess.Popen(
        [sys.executable, str(CV_MAIN_PATH)],
        cwd=str(CV_DIR),
    )
