"""HC-SR04 helpers for the embedded subsystem and fusion bridge."""

import threading
import time

import lgpio

# GPIO pin definitions for the two HC-SR04 ultrasonic sensors
TRIG_LEFT  = 23
ECHO_LEFT  = 25
TRIG_RIGHT = 17
ECHO_RIGHT = 27

class UltrasonicSensor:
    """
    HC-SR04 ultrasonic distance sensor driver.

    Args:
        handle: lgpio chip handle from lgpio.gpiochip_open()
        trig:   GPIO pin connected to TRIG
        echo:   GPIO pin connected to ECHO
        name:   Label used in print output (default: "Sensor")
    """
    def __init__(self, handle, trig, echo, name="Sensor"):
        self.h    = handle
        self.trig = trig
        self.echo = echo
        self.name = name

        lgpio.gpio_claim_output(self.h, self.trig)
        lgpio.gpio_claim_input(self.h, self.echo)

    def get_distance(self):
        """
        Trigger a measurement and return distance in cm, or -1 on timeout.
        """
        # Settle the trigger pin, then send a 10 µs pulse to start a measurement
        lgpio.gpio_write(self.h, self.trig, 0)
        time.sleep(0.002)
        lgpio.gpio_write(self.h, self.trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.h, self.trig, 0)

        timeout_start = time.time()
        pulse_start   = time.time()
        pulse_end     = time.time()

        # Wait for ECHO to go high (start of return pulse)
        while lgpio.gpio_read(self.h, self.echo) == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > 0.1:
                return -1

        # Wait for ECHO to go low (end of return pulse)
        while lgpio.gpio_read(self.h, self.echo) == 1:
            pulse_end = time.time()
            if pulse_end - pulse_start > 0.1:
                return -1

        # 17150 = 34300 cm/s (speed of sound) ÷ 2 (round-trip)
        return round((pulse_end - pulse_start) * 17150, 2)


_SENSOR_LOCK = threading.Lock()
_SENSOR_HANDLE = None
_SENSOR_CACHE = {}


def _get_chip_handle():
    global _SENSOR_HANDLE

    if _SENSOR_HANDLE is None:
        _SENSOR_HANDLE = lgpio.gpiochip_open(0)
    return _SENSOR_HANDLE


def _get_shared_sensor(trig, echo, name="Sensor"):
    key = (trig, echo)

    with _SENSOR_LOCK:
        sensor = _SENSOR_CACHE.get(key)
        if sensor is None:
            sensor = UltrasonicSensor(_get_chip_handle(), trig, echo, name)
            _SENSOR_CACHE[key] = sensor
        return sensor


def sensor_left_distance():
    """Return the left ultrasonic reading in centimeters."""
    return _get_shared_sensor(TRIG_LEFT, ECHO_LEFT, "Left").get_distance()


def sensor_right_distance():
    """Return the right ultrasonic reading in centimeters."""
    return _get_shared_sensor(TRIG_RIGHT, ECHO_RIGHT, "Right").get_distance()


def get_distance(trig, echo):
    """Return one ultrasonic reading for the provided trigger/echo pins."""
    return _get_shared_sensor(trig, echo, f"{trig}:{echo}").get_distance()


def cleanup():
    """Release the shared ultrasonic GPIO handle created by helper functions."""
    global _SENSOR_HANDLE

    with _SENSOR_LOCK:
        _SENSOR_CACHE.clear()
        if _SENSOR_HANDLE is not None:
            lgpio.gpiochip_close(_SENSOR_HANDLE)
            _SENSOR_HANDLE = None


if __name__ == '__main__':
    handle = lgpio.gpiochip_open(0)

    sensors = [
        UltrasonicSensor(handle, TRIG_LEFT,  ECHO_LEFT,  "Left"),
        UltrasonicSensor(handle, TRIG_RIGHT, ECHO_RIGHT, "Right"),
    ]

    try:
        while True:
            # Fire sensors sequentially to avoid crosstalk
            for s in sensors:
                dist = s.get_distance()
                if dist == -1:
                    print(f"{s.name}: timeout")
                else:
                    print(f"{s.name}: {dist:.2f} cm")
            print("---")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        lgpio.gpiochip_close(handle)
