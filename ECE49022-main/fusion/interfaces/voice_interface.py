"""Voice-command interface for fusion.

This module merges three voice paths into one event stream:
1. In-process text injected directly into fusion.
2. JSONL commands written to the navigation voice bridge.
3. JSONL commands written to the embedded voice bridge.

Everything is normalized into a shared event schema before the control loop
sees it, so motion arbitration does not need to care where the command came
from.
"""

from collections import deque
from pathlib import Path
import importlib.util
import json
import subprocess
import sys
import threading
import time
import uuid


REPO_ROOT = Path(__file__).resolve().parents[2]
VOICE_DIR = REPO_ROOT / "voice-recognition"
VOICE_COMMAND_LOG_PATH = VOICE_DIR / "commands.jsonl"
VOICE_EMBEDDED_COMMAND_LOG_PATH = VOICE_DIR / "embedded-commands.jsonl"
COMMAND_PARSER_PATH = VOICE_DIR / "command_parser.py"
SPEECH_TO_TEXT_PATH = VOICE_DIR / "speech_to_text.py"
VOICE_CONTROLLER_PATH = VOICE_DIR / "voice_controller.py"
FUSION_VOICE_ACTIONS = {
    "follow",
    "reduce_speed",
    "increase_speed",
    "emergency_stop",
    "resume",
}

_voice_log_position = 0
_embedded_voice_log_position = 0
_pending_events = deque()
_voice_lock = threading.Lock()
_parser_instance = None
_seen_command_ids = set()


def _load_module(module_name, module_path):
    """Load a Python file by path without requiring it to be import-installed."""
    if not module_path.exists():
        return None

    try:
        spec = importlib.util.spec_from_file_location(module_name, module_path)
    except Exception:
        return None

    if spec is None or spec.loader is None:
        return None

    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception:
        return None

    return module


def _get_command_parser():
    """Return the shared voice parser instance, creating it on first use."""
    global _parser_instance

    if _parser_instance is not None:
        return _parser_instance

    # Load the voice-recognition parser lazily so fusion can still start on
    # machines that do not have the full speech stack installed.
    parser_module = _load_module("voice_command_parser", COMMAND_PARSER_PATH)
    if parser_module is None or not hasattr(parser_module, "CommandParser"):
        return None

    _parser_instance = parser_module.CommandParser()
    return _parser_instance


def _build_voice_event(text, source):
    """Convert raw spoken text into fusion's normalized voice-event shape."""
    parser = _get_command_parser()
    command = None
    confidence = 0.0

    if parser is not None:
        # Normalize raw recognized text into the command/action format that the
        # fusion control loop understands.
        command, confidence = parser.parse(text)

    return {
        "timestamp": time.time(),
        "text": text,
        "command": command,
        "confidence": confidence,
        "recognized": command is not None,
        "source": source,
        "params": {},
        "command_id": None,
    }


def _build_voice_event_from_command(command, source, text="", timestamp=None, params=None, command_id=None):
    """Build a voice event when the action was already resolved upstream."""
    # This path handles the newer JSONL schema where voice-recognition already
    # resolved the final subsystem/action pair before fusion reads it.
    return {
        "timestamp": time.time() if timestamp is None else timestamp,
        "text": text,
        "command": command,
        "confidence": 1.0 if command is not None else 0.0,
        "recognized": command is not None,
        "source": source,
        "params": params or {},
        "command_id": command_id,
    }


def _enqueue_event(event):
    """Queue an in-process voice event until the next control-loop read."""
    with _voice_lock:
        # In-process callbacks push into a queue so the control loop can consume
        # voice input on its normal timing boundary.
        _pending_events.append(event)


def _drain_queue():
    """Atomically drain queued in-process voice events."""
    drained = []
    with _voice_lock:
        while _pending_events:
            drained.append(_pending_events.popleft())
    return drained


def _parse_log_line(line):
    """Parse one line from either voice JSONL bridge into a voice event."""
    line = line.strip()
    if not line:
        return None

    try:
        payload = json.loads(line)
    except json.JSONDecodeError:
        # Plain-text fallback makes the bridge easy to use from scripts or
        # manual terminal testing without requiring JSON formatting.
        payload = {"text": line}

    source = str(payload.get("source", "external_log"))
    command_id = payload.get("command_id")

    if command_id is not None:
        command_id = str(command_id)
        # Deduplicate by command_id so a repeated line or replayed file entry
        # does not trigger the same action twice.
        if command_id in _seen_command_ids:
            return None

    # Prefer the explicit wire-format command if the voice subsystem already
    # resolved the action, and fall back to raw text parsing for older writers.
    subsystem = payload.get("subsystem")
    action = payload.get("action")
    params = payload.get("params", {})
    text = str(payload.get("text", "")).strip()

    if subsystem is not None and action is not None:
        action_text = text if text else str(action)
        command = {
            "subsystem": str(subsystem),
            "action": str(action),
        }
        event = _build_voice_event_from_command(
            command,
            source=source,
            text=action_text,
            params=params if isinstance(params, dict) else {},
            command_id=command_id,
        )
    else:
        if not text:
            return None
        event = _build_voice_event(text, source)
        event["command_id"] = command_id
        event["params"] = params if isinstance(params, dict) else {}

    if "timestamp" in payload:
        try:
            # Preserve the writer's timestamp when available so downstream
            # latency reporting reflects when the command was emitted.
            event["timestamp"] = float(payload["timestamp"])
        except (TypeError, ValueError):
            pass

    # Fusion accepts all supported voice actions and translates them into the
    # fused drivetrain behavior, regardless of which log file carried them.
    command = event.get("command") or {}
    allowed = command.get("action") in FUSION_VOICE_ACTIONS
    if not allowed:
        return None

    if command_id is not None:
        _seen_command_ids.add(command_id)

    return event


def reset_voice_stream():
    """Ignore old bridge traffic and start reading from the current file ends."""
    global _voice_log_position, _embedded_voice_log_position, _seen_command_ids

    with _voice_lock:
        _pending_events.clear()
    _seen_command_ids.clear()

    if VOICE_COMMAND_LOG_PATH.exists():
        # Mirror the CV log behavior: ignore old commands that were written
        # before the current fusion run started.
        _voice_log_position = VOICE_COMMAND_LOG_PATH.stat().st_size
    else:
        _voice_log_position = 0

    if VOICE_EMBEDDED_COMMAND_LOG_PATH.exists():
        # Read only fresh embedded voice commands for the current fusion run.
        _embedded_voice_log_position = VOICE_EMBEDDED_COMMAND_LOG_PATH.stat().st_size
    else:
        _embedded_voice_log_position = 0


def ingest_voice_text(text, source="in_process"):
    """Inject raw text directly into fusion without writing a bridge file."""
    event = _build_voice_event(str(text).strip(), source)
    if not event["text"]:
        return None

    _enqueue_event(event)
    return event


def publish_voice_text(text, source="manual_cli"):
    """Parse text and publish the resulting command into the proper JSONL log."""
    event = _build_voice_event(str(text).strip(), source)
    if not event["text"]:
        return None

    command = event.get("command")
    if command is None:
        # Keep the helper tolerant of unrecognized phrases during manual tests.
        return event

    command_id = str(uuid.uuid4())
    event["command_id"] = command_id

    if command.get("subsystem") == "embedded":
        output_path = VOICE_EMBEDDED_COMMAND_LOG_PATH
    else:
        # Navigation-targeted commands share the normal fusion command bridge.
        output_path = VOICE_COMMAND_LOG_PATH

    # Persist the command so an external writer can communicate with fusion
    # even when it is running in a separate process.
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("a", encoding="utf-8") as log_file:
        log_file.write(
            json.dumps(
                {
                    "timestamp": event["timestamp"],
                    "source": event["source"],
                    "subsystem": command["subsystem"],
                    "action": command["action"],
                    # Match the schema documented by the voice team.
                    "params": event.get("params", {}),
                    "command_id": command_id,
                }
            )
            + "\n"
        )

    return event


def get_voice_commands():
    """Return all new voice events since the previous call."""
    global _voice_log_position, _embedded_voice_log_position

    # Merge in-process callback events and file-based events into one stream so
    # the control loop can treat both voice paths the same way.
    events = _drain_queue()

    for command_path, position_name in (
        (VOICE_COMMAND_LOG_PATH, "_voice_log_position"),
        (VOICE_EMBEDDED_COMMAND_LOG_PATH, "_embedded_voice_log_position"),
    ):
        if not command_path.exists():
            continue

        current_size = command_path.stat().st_size
        current_position = globals()[position_name]
        if current_size < current_position:
            # If a log is truncated or recreated, restart from the beginning of
            # the new file so fresh commands are not skipped.
            current_position = 0

        with command_path.open("r", encoding="utf-8") as log_file:
            log_file.seek(current_position)
            for line in log_file:
                event = _parse_log_line(line)
                if event is not None:
                    events.append(event)
            globals()[position_name] = log_file.tell()

    return events


def start_voice_recognizer():
    """Launch the standalone voice subsystem as a child process."""
    if not VOICE_CONTROLLER_PATH.exists():
        return None

    try:
        # Launch the full voice-control stack so wake word, STT, parsing, and
        # JSONL command publishing behave the same as the standalone subsystem.
        recognizer = subprocess.Popen(
            [sys.executable, str(VOICE_CONTROLLER_PATH)],
            cwd=str(VOICE_DIR),
        )
    except Exception:
        return None

    return recognizer


def stop_voice_recognizer(recognizer):
    """Best-effort shutdown for either a subprocess or custom recognizer object."""
    if recognizer is None:
        return

    if hasattr(recognizer, "terminate") and hasattr(recognizer, "poll"):
        try:
            if recognizer.poll() is None:
                recognizer.terminate()
                recognizer.wait(timeout=2.0)
        except Exception:
            try:
                recognizer.kill()
            except Exception:
                pass
        return

    try:
        recognizer.stop()
    except Exception:
        pass
