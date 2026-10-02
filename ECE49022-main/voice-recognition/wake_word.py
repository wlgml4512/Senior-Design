"""Wake-word detection wrapper for the standalone voice subsystem.

This file is relevant because it is responsible for the first trigger in the
spoken-command pipeline, deciding when the robot should hand off from passive
listening to active speech recognition.
"""

from pathlib import Path
import struct
from typing import Callable, Optional
import time

try:
    import pyaudio
except ModuleNotFoundError:
    pyaudio = None

try:
    import pvporcupine
except ModuleNotFoundError:
    pvporcupine = None

from runtime_support import load_keys_module


_KEYS = load_keys_module()

class WakeWordDetector:

    def __init__(
        self,
        access_key: str,
        keyword_paths: Optional[list] = None,
        keywords: Optional[list] = None,
        sensitivities: Optional[list] = None,
        detection_callback: Optional[Callable] = None
    ):
        if pyaudio is None:
            raise RuntimeError(
                "PyAudio is not installed. Install voice-recognition/requirements.txt."
            )
        if pvporcupine is None:
            raise RuntimeError(
                "pvporcupine is not installed. Install voice-recognition/requirements.txt."
            )
        if not access_key:
            raise RuntimeError(
                "Porcupine access key is missing. Add pico_key to repo-root keys.py."
            )
        
        self.access_key = access_key
        self.detection_callback = detection_callback
        self.is_running = False

        # use custom keywords if provided, otherwise use default
        if keyword_paths:
            self.keywords = [Path(path).stem for path in keyword_paths]
            self.porcupine = pvporcupine.create(
                access_key=access_key,
                keyword_paths=keyword_paths,
                sensitivities=sensitivities or [0.5] * len(keyword_paths)
            )
        elif keywords:
            self.porcupine = pvporcupine.create(
                access_key=access_key,
                keywords=keywords,
                sensitivities=sensitivities or [0.5] * len(keywords)
            )
            self.keywords = keywords
        else:
            print("No keyword detected - using 'porcupine' as default wake word.")
            self.porcupine = pvporcupine.create(
                access_key=access_key,
                keywords=["porcupine"],
                sensitivities=[0.5]
            )
            self.keywords = ["porcupine"]
        
        # audio settings
        self.sample_rate = self.porcupine.sample_rate
        self.frame_length = self.porcupine.frame_length
        self.audio_stream = None
        self.pa = None

        print("Wake word detector initialized.")
        print(f"Sample rate: {self.sample_rate} Hz")
        print(f"Frame length: {self.frame_length}")
        print(f"Keywords: {self.keywords}")

    def _close_audio_resources(self):
        """Best-effort close for PyAudio resources used by either run mode."""
        if self.audio_stream is not None:
            try:
                if self.audio_stream.is_active():
                    self.audio_stream.stop_stream()
            except Exception:
                pass
            try:
                self.audio_stream.close()
            except Exception:
                pass
            self.audio_stream = None

        if self.pa is not None:
            try:
                self.pa.terminate()
            except Exception:
                pass
            self.pa = None

    def audio_callback(self, in_data, frame_count, time_info, status):
        
        if status:
            print(f"Audio stream status: {status}")

        # audio to pcm
        pcm = struct.unpack_from("h" * self.frame_length, in_data)

        # process frame
        keyword_index = self.porcupine.process(pcm)

        # check if wake word detected
        if keyword_index >= 0:
            detected_keyword = self.keywords[keyword_index]
            print(f"Wake word detected: {detected_keyword}")

            if self.detection_callback:
                self.detection_callback(detected_keyword)
        
        return (in_data, pyaudio.paContinue)
    
    def start(self):
        """
        Non blocking start.
        """

        self.is_running = True
        self.pa = pyaudio.PyAudio()
        self.audio_stream = self.pa.open(
            rate=self.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.frame_length,
            stream_callback=self.audio_callback
        )

        self.audio_stream.start_stream()
        print("Wake word detection started. Listening...")

    def start_blocking(self):
        """
        Blocking start.
        """

        self.is_running = True
        self.pa = pyaudio.PyAudio()

        self.audio_stream = self.pa.open(
            rate=self.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.frame_length,
        )

        print("Wake word detection started (blocking mode). Listening...")

        try:
            while self.is_running:
                # read audio frame
                pcm = self.audio_stream.read(self.frame_length, exception_on_overflow=False)
                pcm = struct.unpack_from("h" * self.frame_length, pcm)
                
                # process with Porcupine
                keyword_index = self.porcupine.process(pcm)
                
                if keyword_index >= 0:
                    detected_keyword = self.keywords[keyword_index]
                    print(f"Wake word detected: {detected_keyword}")
                    
                    if self.detection_callback:
                        self.detection_callback(detected_keyword)
        
        except KeyboardInterrupt:
            print("\nStopping wake word detection...")
        finally:
            self.is_running = False
            self._close_audio_resources()


    def stop(self):
        if not self.is_running and self.audio_stream is None and self.pa is None:
            return

        self.is_running = False
        self._close_audio_resources()

        print("Wake word detection stopped.")

    def cleanup(self):
        self.stop()
        if self.porcupine:
            self.porcupine.delete()
            self.porcupine = None

if __name__ == "__main__":

    if _KEYS is None or not getattr(_KEYS, "pico_key", None):
        raise RuntimeError("Porcupine access key is missing. Add pico_key to repo-root keys.py.")

    ACCESS_KEY = _KEYS.pico_key

    def on_wake_word_detected(keyword):
        print(f"WAKE WORD TRIGGERED: {keyword}")
        print(f"Timestamp: {time.strftime('%H:%M:%S')}")

    # --- Atlas wake word (Raspberry Pi only) ---
    # The .ppn model is platform-specific and will not load on macOS/x86.
    
    # import os
    # _PPN_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
    #                          "Atlas_en_raspberry-pi_v4_0_0.ppn")
    # detector = WakeWordDetector(
    #     access_key=ACCESS_KEY,
    #     keyword_paths=[_PPN_PATH],
    #     sensitivities=[0.5],
    #     detection_callback=on_wake_word_detected,
    # )

    # --- Porcupine wake word (cross-platform fallback for dev/testing) ---
    detector = WakeWordDetector(
        access_key=ACCESS_KEY,
        keywords=["porcupine"],
        sensitivities=[0.5],
        detection_callback=on_wake_word_detected
    )

    try:
        detector.start_blocking()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        detector.cleanup()

    
