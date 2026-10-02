"""Top-level state machine for the standalone voice subsystem.

This file is relevant because it coordinates wake-word detection, speech-to-text,
command parsing, user feedback, and JSONL command dispatch, making it the main
entry point for voice-driven robot control.
"""

import logging
import signal
import socket
import sys
import threading
import time
from enum import Enum

from command_interface import CommandInterface
from command_parser import CommandParser
from feedback_controller import FeedbackController
from runtime_support import load_keys_module
from speech_to_text import SpeechRecognizer
from wake_word import WakeWordDetector


class VoiceControlState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    EXECUTING = "executing"
    ERROR = "error"


class VoiceController:

    FALSE_POSITIVE_TIMEOUT = 5.0   # seconds before declaring a wake word a false positive
    ERROR_RECOVERY_DELAY = 2.0     # seconds in ERROR state before returning to IDLE
    STT_SILENCE_TIMEOUT = 4.0      # passed to SpeechRecognizer
    MIN_CONFIDENCE = 0.8           # minimum parser confidence to accept a command

    def __init__(self):
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
            handlers=[
                logging.StreamHandler(),
                logging.FileHandler("voice_control.log"),
            ],
        )
        self.logger = logging.getLogger(__name__)

        self.state = VoiceControlState.IDLE
        self.state_lock = threading.Lock()
        self.running = False

        self._false_positive_timer = None
        self._stt_active = False
        # Guard so only the first STT callback per listen session is processed
        self._command_processing = False
        self._keys = load_keys_module()

        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

        self._init_feedback()
        self._init_wake_word()
        self._init_stt()
        self.parser = CommandParser()
        self.command_interface = CommandInterface()

        if self._keys is None:
            self.logger.warning(
                "keys.py was not found at the repo root. Wake word and speech recognition will stay disabled."
            )
        if self.wake_word_detector is None:
            self.logger.warning("Wake word detector unavailable. Spoken commands will not be captured on this machine.")

        self.logger.info("VoiceController Initialized.")

    # --- Initialization helpers ---

    def _init_feedback(self):
        try:
            self.feedback = FeedbackController()
            self.logger.info("Feedback controller ready.")
        except Exception as e:
            self.logger.error(f"Feedback init failed (audio/LED unavailable): {e}")
            self.feedback = None

    def _init_wake_word(self):
        try:
            pico_key = getattr(self._keys, "pico_key", None) if self._keys is not None else None
            if not pico_key:
                raise RuntimeError("Missing pico_key in repo-root keys.py")

            # --- Atlas wake word (Raspberry Pi only) ---
            # Uncomment the block below and comment out the porcupine block
            # when deploying to Raspberry Pi. The .ppn model file is
            # platform-specific and will not load on macOS/x86.
            #
            # import os
            # _PPN_PATH = os.path.join(
            #     os.path.dirname(os.path.abspath(__file__)),
            #     "Atlas_en_raspberry-pi_v4_0_0.ppn",
            # )
            # self.wake_word_detector = WakeWordDetector(
            #     access_key=pico_key,
            #     keyword_paths=[_PPN_PATH],
            #     sensitivities=[0.5],
            #     detection_callback=self.on_wake_word_detected,
            # )

            # --- Porcupine wake word (cross-platform fallback for dev/testing) ---
            self.wake_word_detector = WakeWordDetector(
                access_key=pico_key,
                keywords=["porcupine"],
                sensitivities=[0.5],
                detection_callback=self.on_wake_word_detected,
            )
        except Exception as e:
            self.logger.error(f"Wake word detector init failed: {e}")
            self.wake_word_detector = None

    def _init_stt(self):
        # SpeechRecognizer is re-created each listen session; this just validates imports
        self.stt_recognizer = None

    # --- Public API ---

    def start(self):
        """Start the voice control subsystem (blocks until stopped)."""
        self.logger.info("Voice controller starting...")
        self.running = True
        self._start_wake_word_detection()

        try:
            while self.running:
                time.sleep(0.01)
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt.")
        finally:
            self.cleanup()

    def stop(self):
        """Signal the main loop to exit."""
        self.running = False

    def get_state(self) -> VoiceControlState:
        with self.state_lock:
            return self.state

    # --- Callbacks (called from component threads) ---

    def on_wake_word_detected(self, keyword: str):
        """Triggered by WakeWordDetector when the wake word is heard.

        NOTE: This is called from inside a PyAudio stream callback. Stopping the
        stream here would deadlock (PyAudio can't stop while the callback is running).
        Mic handoff and STT startup are deferred to a daemon thread so the callback
        returns immediately.
        """
        with self.state_lock:
            if self.state != VoiceControlState.IDLE:
                self.logger.debug(f"Wake word ignored in state={self.state.value}.")
                return

        self.logger.info(f"Wake word detected: '{keyword}'")
        self._command_processing = False
        self.transition_to(VoiceControlState.LISTENING)

        if self.feedback:
            self.feedback.play_wake_word_detected()
            self.feedback.show_wake_word()

        # Timeout guard: if no speech arrives, treat as false positive
        self._false_positive_timer = threading.Timer(
            self.FALSE_POSITIVE_TIMEOUT, self._on_false_positive_timeout
        )
        self._false_positive_timer.daemon = True
        self._false_positive_timer.start()

        # Defer mic handoff to a thread — cannot call stop_stream() from
        # inside a PyAudio callback (deadlock).
        def _handoff():
            self._stop_wake_word_detection()
            self._start_stt()

        threading.Thread(target=_handoff, daemon=True).start()

    def on_speech_recognized(self, text: str):
        """Triggered by SpeechRecognizer when an utterance is converted to text."""
        # Only handle one utterance per listen session
        if self._command_processing:
            return
        with self.state_lock:
            if self.state != VoiceControlState.LISTENING:
                return

        self._command_processing = True

        if self._false_positive_timer:
            self._false_positive_timer.cancel()
            self._false_positive_timer = None

        if not text or not text.strip():
            self.logger.warning("STT returned empty text.")
            self._handle_error()
            return

        self.logger.info(f"Speech recognized: '{text}'")
        self.transition_to(VoiceControlState.PROCESSING)

        # Stop STT asynchronously (stop() blocks on join, would deadlock here)
        self._stop_stt_async()

        command, confidence = self.parser.parse(text)

        if command and confidence >= self.MIN_CONFIDENCE:
            self.logger.info(
                f"Command accepted: {command} (confidence={confidence:.2f})"
            )
            self.transition_to(VoiceControlState.EXECUTING)
            if self.feedback:
                self.feedback.play_command_recognized()
                self.feedback.show_command_recognized()
            self._execute_command(command)
        else:
            self.logger.warning(
                f"Command not recognized: '{text}' (confidence={confidence:.2f})"
            )
            self._handle_error()

    def on_command_executed(self, success: bool):
        """Called when a command dispatch completes (success or failure)."""
        if success:
            self.logger.info("Command executed successfully.")
            self._return_to_idle(delay=1.5)  # Allow green LED pattern to finish
        else:
            self.logger.warning("Command execution failed.")
            self._handle_error()

    # --- State machine ---

    def transition_to(self, new_state: VoiceControlState):
        """Thread-safe state transition with logging."""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
        self.logger.info(f"State: {old_state.value} → {new_state.value}")

    # --- Internal helpers ---

    def _start_wake_word_detection(self):
        if self.wake_word_detector:
            try:
                self.wake_word_detector.start()
                self.logger.info("Listening for wake word...")
            except Exception as e:
                self.logger.error(f"Failed to start wake word detection: {e}")
        else:
            self.logger.warning("Wake word detection is disabled; no spoken commands can be registered.")

    def _stop_wake_word_detection(self):
        if self.wake_word_detector:
            try:
                self.wake_word_detector.stop()
            except Exception as e:
                self.logger.error(f"Failed to stop wake word detection: {e}")

    def _start_stt(self):
        if not self._check_network():
            self.logger.error("No network - Azure STT unavailable.")
            self._handle_error()
            return

        try:
            # Re-create each session so internal flags start fresh
            self.stt_recognizer = SpeechRecognizer(
                silence_timeout=self.STT_SILENCE_TIMEOUT,
                utterance_callback=self.on_speech_recognized,
            )
            self.stt_recognizer.start()
            self._stt_active = True
            self.logger.info("STT listening...")
        except Exception as e:
            self.logger.error(f"Failed to start STT: {e}")
            self._handle_error()

    def _stop_stt_async(self):
        """Stop STT in a daemon thread to avoid blocking the STT callback thread."""
        def _stop():
            if self.stt_recognizer and self._stt_active:
                try:
                    self.stt_recognizer.stop()
                    self._stt_active = False
                    self.logger.debug("STT stopped.")
                except Exception as e:
                    self.logger.error(f"STT stop error: {e}")

        threading.Thread(target=_stop, daemon=True).start()

    def _execute_command(self, command: dict):
        """Route command to the appropriate subsystem via CommandInterface."""
        subsystem = command.get("subsystem")
        action = command.get("action")
        success = self.command_interface.send_command(subsystem, action)
        self.on_command_executed(success)

    def _handle_error(self):
        """Transition to ERROR, play feedback, then schedule return to IDLE."""
        self.transition_to(VoiceControlState.ERROR)
        if self.feedback:
            self.feedback.play_error()
            self.feedback.show_error()
        self._return_to_idle(delay=self.ERROR_RECOVERY_DELAY)

    def _return_to_idle(self, delay: float = 0.0):
        """Return to IDLE (optionally after a delay) and restart wake word detection."""
        def _go():
            if delay > 0:
                time.sleep(delay)
            if self.feedback:
                try:
                    self.feedback.lights_off()
                except Exception:
                    pass
            self.transition_to(VoiceControlState.IDLE)
            self._start_wake_word_detection()

        threading.Thread(target=_go, daemon=True).start()

    def _on_false_positive_timeout(self):
        """No speech arrived within FALSE_POSITIVE_TIMEOUT after wake word."""
        self.logger.warning("No speech after wake word — treating as false positive.")
        self._stop_stt_async()
        self._handle_error()

    def _check_network(self) -> bool:
        try:
            socket.create_connection(("8.8.8.8", 53), timeout=3)
            return True
        except OSError:
            return False

    def _signal_handler(self, sig, frame):
        self.logger.info(f"Shutdown signal received (sig={sig}).")
        self.running = False
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """Release all resources in a best-effort, exception-safe manner."""
        self.logger.info("Cleaning up...")

        if self._false_positive_timer:
            self._false_positive_timer.cancel()

        try:
            if self.wake_word_detector:
                self.wake_word_detector.cleanup()
        except Exception as e:
            self.logger.error(f"Wake word cleanup error: {e}")

        try:
            if self.stt_recognizer and self._stt_active:
                self.stt_recognizer.stop()
                self._stt_active = False
        except Exception as e:
            self.logger.error(f"STT cleanup error: {e}")

        try:
            if self.feedback:
                self.feedback.lights_off()
        except Exception as e:
            self.logger.error(f"Feedback cleanup error: {e}")

        self.logger.info("Cleanup complete.")


if __name__ == "__main__":
    controller = VoiceController()
    controller.start()
