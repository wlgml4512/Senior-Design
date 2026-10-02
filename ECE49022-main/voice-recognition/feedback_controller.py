"""Audio and LED feedback helpers for voice interactions.

This file is relevant because it turns internal voice-controller state changes
into human-facing tones and light patterns, making the subsystem easier to use
and debug on the robot.
"""

import threading
import time

try:
    import numpy as np
except ModuleNotFoundError:
    np = None

try:
    import pygame
except ModuleNotFoundError:
    pygame = None

try:
    from usb_pixel_ring_v2 import PixelRing
    import usb.core
except ModuleNotFoundError:
    PixelRing = None
    usb = None

class FeedbackController:

    def __init__(self):
        missing = []
        if pygame is None:
            missing.append("pygame")
        if np is None:
            missing.append("numpy")
        if missing:
            raise RuntimeError(
                "Feedback controller dependencies are unavailable: "
                + ", ".join(missing)
            )

        pygame.mixer.init(frequency=22050, size=-16, channels=1)
        dev = usb.core.find(idVendor=0x2886, idProduct=0x0018) if PixelRing and usb else None
        if dev:
            self.pixel_ring = PixelRing(dev)
        else:
            self.pixel_ring = None

        self.current_pattern = None
        self.pattern_start_time = None
        self.pattern_duration = None
        self._audio_timer = None

    def generate_tone(self, frequency, duration_ms):
        """
        Generate simple tone.
        """
        sample_rate = 22050
        n_samples = int(sample_rate * duration_ms / 1000)

        t = np.linspace(0, duration_ms/1000, n_samples)
        wave = np.sin(2 * np.pi * frequency * t)

        # convert to 16 bit
        wave = (wave * 32767).astype(np.int16)

        # stereo sound (duplicate for both channels)
        #stereo = np.column_stack((wave, wave))

        return pygame.sndarray.make_sound(wave)
    
    def play_wake_word_detected(self):
        """
        Two ascending beeps for wake word detection feedback. Non-blocking.
        """
        tone1 = self.generate_tone(1000, 50)
        tone2 = self.generate_tone(1500, 50)
        tone1.play()
        self._audio_timer = threading.Timer(0.1, tone2.play)
        self._audio_timer.daemon = True
        self._audio_timer.start()

    def play_command_recognized(self):
        """
        Pleasant feedback tone for when voice command is recognized. Non-blocking.
        """
        tone = self.generate_tone(1000, 200)
        tone.play()

    def play_error(self):
        """
        Descending feedback tone for errors. Non-blocking.
        """
        tone1 = self.generate_tone(500, 200)
        tone2 = self.generate_tone(300, 200)
        tone1.play()
        self._audio_timer = threading.Timer(0.25, tone2.play)
        self._audio_timer.daemon = True
        self._audio_timer.start()

    def show_wake_word(self, duration: float = 2.0):
        """
        LED pattern for wake word detection. Auto-off after duration seconds.
        """
        if self.pixel_ring:
            self.pixel_ring.set_color_palette(0x0000FF, 0x0000FF)  # all blue
            self.pixel_ring.spin()
        self.current_pattern = 'wake_word'
        self.pattern_start_time = time.time()
        self.pattern_duration = duration

    def show_command_recognized(self, duration: float = 1.5):
        """
        LED pattern for command recognized. Auto-off after duration seconds.
        """
        if self.pixel_ring:
            self.pixel_ring.set_color_palette(0x000000, 0x00FF00)  # all green
            self.pixel_ring.think()
        self.current_pattern = 'command_recognized'
        self.pattern_start_time = time.time()
        self.pattern_duration = duration

    def show_error(self, duration: float = 2.0):
        """
        LED pattern for error. Auto-off after duration seconds.
        """
        if self.pixel_ring:
            self.pixel_ring.set_color_palette(0xFF0000, 0x000000)  # all red
            self.pixel_ring.think()
        self.current_pattern = 'error'
        self.pattern_start_time = time.time()
        self.pattern_duration = duration

    def update(self):
        """
        Call periodically in the main control loop to manage timed LED patterns.
        Turns off LEDs automatically when the pattern duration has elapsed.
        """
        if self.current_pattern and self.pattern_start_time:
            elapsed = time.time() - self.pattern_start_time
            if elapsed >= self.pattern_duration:
                self.lights_off()
                self.current_pattern = None
                self.pattern_start_time = None
                self.pattern_duration = None

    def interrupt(self):
        """
        Immediately stop the current LED pattern and cancel any pending audio timer.
        """
        if self._audio_timer is not None:
            self._audio_timer.cancel()
            self._audio_timer = None
        self.lights_off()
        self.current_pattern = None
        self.pattern_start_time = None
        self.pattern_duration = None

    def lights_off(self):
        if self.pixel_ring:
            self.pixel_ring.off()


if __name__ == "__main__":
    feedback = FeedbackController()

    print("Testing wake word detected...")
    feedback.play_wake_word_detected()
    feedback.show_wake_word()
    time.sleep(3)
    feedback.lights_off()

    pygame.time.wait(2000)

    print("Testing command recognized...")
    feedback.play_command_recognized()
    feedback.show_command_recognized()
    time.sleep(3)
    feedback.lights_off()

    pygame.time.wait(2000)

    print("Testing error tone...")
    feedback.play_error()
    feedback.show_error()
    time.sleep(3)
    feedback.lights_off()
