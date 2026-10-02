"""JSONL command transport from voice control into the rest of ATLAS.

This module is relevant because it is the bridge that publishes recognized
voice actions to fusion and embedded consumers without requiring direct process
coupling or a separate network protocol.
"""

import json
import logging
import os
import threading
import time
import uuid

logger = logging.getLogger(__name__)

# Paths to the JSONL files. Resolved relative to this file's directory
# so they work regardless of the working directory at launch.
_DEFAULT_COMMANDS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "commands.jsonl"
)
_DEFAULT_EMBEDDED_COMMANDS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "embedded-commands.jsonl"
)


class CommandInterface:
    """Sends voice commands to the Navigation and Embedded subsystems.

    Navigation integration:
        Atomically appends one JSON line per command to ``commands_file``
        (default: ``voice-recognition/commands.jsonl``). Fusion's voice-command
        bridge tails that file and updates ``voice_state`` in real time.

    Embedded integration:
        Atomically appends one JSON line per command to ``embedded_commands_file``
        (default: ``voice-recognition/embedded-commands.jsonl``). Fusion tails
        that file so embedded-style voice actions, such as emergency stop and
        resume, still flow through the unified fusion-to-motor path.
    """

    VALID_SUBSYSTEMS = {"navigation", "embedded"}

    def __init__(
        self,
        commands_file: str = _DEFAULT_COMMANDS_FILE,
        embedded_commands_file: str = _DEFAULT_EMBEDDED_COMMANDS_FILE,
    ):
        self._commands_file = commands_file
        self._embedded_commands_file = embedded_commands_file
        self._lock = threading.Lock()
        logger.info(f"CommandInterface ready. Nav JSONL:      {self._commands_file}")
        logger.info(f"CommandInterface ready. Embedded JSONL: {self._embedded_commands_file}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def send_command(
        self,
        subsystem: str,
        action: str,
        params: dict = None,
    ) -> bool:
        """Send a command to the specified subsystem.

        Args:
            subsystem: ``"navigation"`` or ``"embedded"``.
            action:    Action string, e.g. ``"follow"``, ``"emergency_stop"``.
            params:    Optional extra parameters (dict). Defaults to ``{}``.

        Returns:
            ``True`` on success, ``False`` on failure.
        """
        if subsystem not in self.VALID_SUBSYSTEMS:
            logger.error(f"Unknown subsystem: '{subsystem}'")
            return False

        message = {
            "timestamp": time.time(),
            "source": "voice_control",
            "subsystem": subsystem,
            "action": action,
            "params": params or {},
            "command_id": str(uuid.uuid4()),
        }

        if subsystem == "navigation":
            return self._send_navigation(message)
        else:
            return self._send_embedded(message)

    # ------------------------------------------------------------------
    # Transport implementations
    # ------------------------------------------------------------------

    def _send_navigation(self, message: dict) -> bool:
        """Append a JSON line to commands.jsonl for fusion to consume."""
        line = json.dumps(message)
        try:
            with self._lock:
                with open(self._commands_file, "a") as f:
                    f.write(line + "\n")
                    f.flush()
                    os.fsync(f.fileno())
            logger.info(
                f"→ navigation | action='{message['action']}' "
                f"id={message['command_id'][:8]}"
            )
            return True
        except OSError as e:
            logger.error(f"Failed to write to {self._commands_file}: {e}")
            return False

    def _send_embedded(self, message: dict) -> bool:
        """Append a JSON line to embedded-commands.jsonl for the embedded controller.

        The embedded controller process tails this file and dispatches commands
        to the Motor/HX711 drivers (lgpio) running on the same Raspberry Pi.
        """
        line = json.dumps(message)
        try:
            with self._lock:
                with open(self._embedded_commands_file, "a") as f:
                    f.write(line + "\n")
                    f.flush()
                    os.fsync(f.fileno())
            logger.info(
                f"→ embedded | action='{message['action']}' "
                f"id={message['command_id'][:8]}"
            )
            return True
        except OSError as e:
            logger.error(f"Failed to write to {self._embedded_commands_file}: {e}")
            return False
