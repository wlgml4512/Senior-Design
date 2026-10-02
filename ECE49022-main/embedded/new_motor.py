"""Older motor-only PWM experiment kept for reference.

This file is not the main drivetrain backend anymore, but it still shows a
minimal direct `lgpio.tx_pwm(...)` setup that can help with isolated motor
bring-up and hardware debugging.
"""

import lgpio
import time


# Robot drivetrain wiring.
LEFT_MOTOR_PWM_PIN = 13
LEFT_MOTOR_DIR_PIN = 24
LEFT_ENCODER_A_PIN = 16
LEFT_ENCODER_B_PIN = 7

RIGHT_MOTOR_PWM_PIN = 12
RIGHT_MOTOR_DIR_PIN = 26
RIGHT_ENCODER_A_PIN = 21
RIGHT_ENCODER_B_PIN = 20


class Motor:
    def __init__(self, handle, pwm_pin, dir_pin, freq=1000):
        self.h = handle
        self.pwm_pin = pwm_pin
        self.dir_pin = dir_pin
        self.freq = freq

        lgpio.gpio_claim_output(self.h, dir_pin)
        # No need to claim pwm_pin — tx_pwm takes ownership
        self.set_speed(0)

    def set_speed(self, speed):
        speed = max(0.0, min(1.0, float(speed)))
        lgpio.tx_pwm(self.h, self.pwm_pin, self.freq, speed * 100)

    def set_direction(self, direction):
        lgpio.gpio_write(self.h, self.dir_pin, direction)

    def stop(self):
        self.set_speed(0)

    def cleanup(self):
        lgpio.tx_pwm(self.h, self.pwm_pin, self.freq, 0)  # stop PWM
        lgpio.gpio_write(self.h, self.dir_pin, 0)


if __name__ == "__main__":
    handle = lgpio.gpiochip_open(0)
    motor_left = None
    motor_right = None
    try:
        motor_left = Motor(handle, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_DIR_PIN)
        motor_right = Motor(handle, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_DIR_PIN)

        motor_left.set_direction(1)
        motor_right.set_direction(1)
        motor_left.set_speed(1)
        motor_right.set_speed(1)

        time.sleep(5)

    finally:
        if motor_left:
            motor_left.cleanup()
        if motor_right:
            motor_right.cleanup()
        lgpio.gpiochip_close(handle)
