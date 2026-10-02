"""Motor and encoder drivers for the Raspberry Pi drivetrain."""

import json
import os
import threading
import time

import lgpio


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VOICE_COMMANDS_FILE = os.path.join(REPO_ROOT, "voice-recognition", "embedded-commands.jsonl")

DEFAULT_PWM_FREQUENCY_HZ = 1000
ENCODER_COUNTS_PER_REV = 9600

# Robot drivetrain wiring.
LEFT_MOTOR_PWM_PIN = 13
LEFT_MOTOR_DIR_PIN = 24
LEFT_ENCODER_A_PIN = 16
LEFT_ENCODER_B_PIN = 7

RIGHT_MOTOR_PWM_PIN = 12
RIGHT_MOTOR_DIR_PIN = 26
RIGHT_ENCODER_A_PIN = 21
RIGHT_ENCODER_B_PIN = 20


def open_gpiochip(chip_index=0):
    """Open one lgpio chip handle."""
    return lgpio.gpiochip_open(chip_index)


def close_gpiochip(handle):
    """Close one lgpio chip handle when it exists."""
    if handle is not None:
        lgpio.gpiochip_close(handle)


class Motor:
    """One DC motor controlled by direction plus PWM duty cycle."""

    def __init__(self, handle, pwm_pin, dir_pin, freq=DEFAULT_PWM_FREQUENCY_HZ):
        self.h = handle
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.freq = freq
        self.period = 1.0 / self.freq

        self._lock = threading.Lock()
        self._use_native_pwm = callable(getattr(lgpio, "tx_pwm", None))
        self._running = False
        self._thread = None
        self.speed = 0.0
        self.direction = 0

        lgpio.gpio_claim_output(self.h, self.dir_pin)
        if not self._use_native_pwm:
            lgpio.gpio_claim_output(self.h, self.pwm_pin)
            self._running = True
            self._thread = threading.Thread(target=self._pwm_loop, daemon=True)
            self._thread.start()

        self._apply_output()

    def _snapshot(self):
        with self._lock:
            return self.speed, self.direction

    def _apply_output(self):
        speed, direction = self._snapshot()
        lgpio.gpio_write(self.h, self.dir_pin, direction)

        if self._use_native_pwm:
            if speed <= 0.0:
                lgpio.tx_pwm(self.h, self.pwm_pin, 0, 0)
            else:
                lgpio.tx_pwm(self.h, self.pwm_pin, self.freq, speed * 100.0)
            return

        if speed <= 0.0:
            lgpio.gpio_write(self.h, self.pwm_pin, 0)
        elif speed >= 1.0:
            lgpio.gpio_write(self.h, self.pwm_pin, 1)

    def _pwm_loop(self):
        while self._running:
            speed, direction = self._snapshot()
            lgpio.gpio_write(self.h, self.dir_pin, direction)

            if speed <= 0.0:
                lgpio.gpio_write(self.h, self.pwm_pin, 0)
                time.sleep(self.period)
                continue

            if speed >= 1.0:
                lgpio.gpio_write(self.h, self.pwm_pin, 1)
                time.sleep(self.period)
                continue

            on_time = self.period * speed
            off_time = self.period - on_time

            lgpio.gpio_write(self.h, self.pwm_pin, 1)
            time.sleep(on_time)
            lgpio.gpio_write(self.h, self.pwm_pin, 0)
            time.sleep(off_time)

    def set_speed(self, speed):
        with self._lock:
            self.speed = max(0.0, min(1.0, float(speed)))
        self._apply_output()

    def set_direction(self, direction):
        with self._lock:
            self.direction = 1 if direction else 0
        self._apply_output()

    def drive(self, command):
        """Drive with one signed normalized command in [-1.0, 1.0]."""
        command = max(-1.0, min(1.0, float(command)))
        self.set_direction(1 if command < 0.0 else 0)
        self.set_speed(abs(command))

    def stop(self):
        self.set_speed(0.0)

    def cleanup(self):
        self.stop()
        self._running = False
        if self._thread is not None:
            self._thread.join()
            self._thread = None

        if self._use_native_pwm:
            try:
                lgpio.tx_pwm(self.h, self.pwm_pin, 0, 0)
            except Exception:
                pass
        else:
            lgpio.gpio_write(self.h, self.pwm_pin, 0)

        lgpio.gpio_write(self.h, self.dir_pin, 0)


class Encoder:
    """Quadrature encoder helper that reports RPM and total tick count."""

    COUNTS_PER_REV = ENCODER_COUNTS_PER_REV

    def __init__(self, handle, gpio_a, gpio_b):
        self.handle = handle
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b

        self._lock = threading.Lock()
        self.count = 0
        self.last_count = 0
        self.last_time = time.time()

        lgpio.gpio_claim_input(self.handle, self.gpio_a, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self.handle, self.gpio_b, lgpio.SET_PULL_UP)
        lgpio.gpio_set_alert_func(self.handle, self.gpio_a, self._pulse)
        lgpio.gpio_set_alert_func(self.handle, self.gpio_b, self._pulse)

    def _pulse(self, chip, gpio, level, tick):
        del chip, gpio, level, tick
        with self._lock:
            self.count += 1

    def rpm(self):
        now = time.time()
        with self._lock:
            dt = now - self.last_time
            if dt <= 0.0:
                return 0.0

            delta = self.count - self.last_count
            self.last_count = self.count
            self.last_time = now

        return (delta / self.COUNTS_PER_REV) / dt * 60.0

    def speed(self):
        """Compatibility alias for older callers."""
        return self.rpm()

    @property
    def total_ticks(self):
        with self._lock:
            return self.count

    def cleanup(self):
        for pin in (self.gpio_a, self.gpio_b):
            try:
                lgpio.gpio_set_alert_func(self.handle, pin, None)
            except Exception:
                pass


def voice_command_listener(motor, get_default_speed):
    """Tail voice commands and apply emergency stop/resume actions."""
    while not os.path.exists(VOICE_COMMANDS_FILE):
        time.sleep(0.5)

    with open(VOICE_COMMANDS_FILE, "r", encoding="utf-8") as command_file:
        command_file.seek(0, os.SEEK_END)
        while True:
            line = command_file.readline()
            if not line:
                time.sleep(0.005)
                continue

            try:
                message = json.loads(line.strip())
            except json.JSONDecodeError:
                continue

            action = message.get("action")
            if action == "emergency_stop":
                motor.stop()
            elif action == "resume":
                motor.set_speed(get_default_speed())


if __name__ == "__main__":
    print("This module is intended to be imported by fusion or other robot scripts.")
