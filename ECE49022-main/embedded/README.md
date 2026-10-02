# Embedded Drivers

This directory contains the Raspberry Pi hardware-facing pieces used by the robot:

- `motors_encoders.py`: drivetrain motor control, encoder RPM/tick tracking, and embedded voice stop/resume helper
- `load_cells.py`: HX711 load cell driver
- `ultrasonic_sensor.py`: HC-SR04 ultrasonic sensor driver plus module-level helpers used by fusion
- `server.py`: telemetry WebSocket server for encoder and load-cell data
- `robot_dashboard.html`: browser dashboard for telemetry
- `new_motor.py`: older motor-only experiment kept for reference

## Wiring

### `motors_encoders.py`

| GPIO Pin | Role |
|----------|------|
| 13 | Left motor PWM |
| 24 | Left motor direction |
| 16 | Left encoder A |
| 7  | Left encoder B |
| 12 | Right motor PWM |
| 26 | Right motor direction |
| 21 | Right encoder A |
| 20 | Right encoder B |

### `load_cells.py`

| GPIO Pin | Role |
|----------|------|
| 19 | HX711 DT |
| 6  | HX711 SCK |

### `ultrasonic_sensor.py`

| GPIO Pin | Role |
|----------|------|
| 23 | Left TRIG |
| 25 | Left ECHO |
| 17 | Right TRIG |
| 27 | Right ECHO |

## Motor And Encoder Usage

```python
import time

from motors_encoders import Encoder, Motor, close_gpiochip, open_gpiochip

handle = open_gpiochip(0)

left_motor = Motor(handle, pwm_pin=13, dir_pin=24)
right_motor = Motor(handle, pwm_pin=12, dir_pin=26)
left_encoder = Encoder(handle, gpio_a=16, gpio_b=7)
right_encoder = Encoder(handle, gpio_a=21, gpio_b=20)

left_motor.set_direction(0)
right_motor.set_direction(0)
left_motor.set_speed(0.5)
right_motor.set_speed(0.5)

time.sleep(1.0)

print(left_encoder.rpm())
print(right_encoder.total_ticks)

left_motor.stop()
right_motor.stop()
left_motor.cleanup()
right_motor.cleanup()
left_encoder.cleanup()
right_encoder.cleanup()
close_gpiochip(handle)
```

Notes:

- `Motor.set_speed()` expects a duty-cycle fraction from `0.0` to `1.0`
- `Motor.drive()` accepts a signed normalized command from `-1.0` to `1.0`
- the motor driver prefers native `lgpio.tx_pwm(...)` when available and falls back to a software PWM loop otherwise
- `voice_command_listener()` tails `voice-recognition/embedded-commands.jsonl` and handles `emergency_stop` / `resume`

## Load Cell Usage

```python
import lgpio

from load_cells import HX711

handle = lgpio.gpiochip_open(0)
scale = HX711(dt_pin=19, sck_pin=6, handle=handle)

scale.tare()
raw = sum(scale.read_raw() for _ in range(10)) / 10
scale.set_scale(raw / 500.0)  # example: 500 g calibration weight

print(scale.get_weight())

scale.close()
lgpio.gpiochip_close(handle)
```

## Ultrasonic Usage

### Class-Based API

```python
import lgpio

from ultrasonic_sensor import ECHO_LEFT, TRIG_LEFT, UltrasonicSensor

handle = lgpio.gpiochip_open(0)
left_sensor = UltrasonicSensor(handle, TRIG_LEFT, ECHO_LEFT, "Left")
print(left_sensor.get_distance())  # centimeters, or -1 on timeout
lgpio.gpiochip_close(handle)
```

### Fusion-Friendly Module Helpers

`ultrasonic_sensor.py` also exposes:

- `sensor_left_distance()`
- `sensor_right_distance()`
- `get_distance(trig, echo)`

Those helpers lazily open one shared GPIO handle and are the entry points used by `fusion/interfaces/ultrasonic_interface.py`.

## Telemetry Dashboard

The telemetry pair is:

- `server.py`
- `robot_dashboard.html`

`server.py` runs on the Raspberry Pi, reads encoder RPM/ticks and HX711 data, and broadcasts JSON over WebSocket.

`robot_dashboard.html` connects to that WebSocket and displays:

- left and right RPM
- left and right tick counts
- payload weight
- raw HX711 ADC values
- recent speed history

### Running The Server

Install dependencies on the Pi:

```bash
pip install websockets lgpio --break-system-packages
```

Then run:

```bash
python3 embedded/server.py
```

The server listens on:

- host: `0.0.0.0`
- port: `8765`

Open `robot_dashboard.html` in a browser on the same network and point it at:

```text
ws://<pi-ip>:8765
```

### Telemetry Fields

The dashboard expects these JSON fields from `server.py`:

- `rpm_left`
- `rpm_right`
- `ticks_left`
- `ticks_right`
- `weight_g`
- `raw_adc`
- `ts`
