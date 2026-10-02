"""
server.py — Run this on the Raspberry Pi.

Reads real encoder RPM and HX711 weight then broadcasts JSON
over a WebSocket so the dashboard can display live data.

Install deps on the Pi:
    pip install websockets lgpio --break-system-packages

Run:
    python server.py

Then open the dashboard and enter:
    ws://<pi-ip>:8765
"""

import asyncio
import json
import logging
import threading
import time

import lgpio
import websockets

# ── Pin assignments (must match motor_encoder.py / load_cells.py) ────────────
LEFT_ENCODER_A_PIN  = 16
LEFT_ENCODER_B_PIN  = 7

RIGHT_ENCODER_A_PIN = 21
RIGHT_ENCODER_B_PIN = 20

HX711_DT_PIN  = 19
HX711_SCK_PIN = 6

# ── HX711 calibration — set these after running calibration ──────────────────
# Run load_cells.py once to get these values for your specific load cell.
TARE_OFFSET = 0       # raw ADC value with no load (from hx711.tare())
SCALE_FACTOR = 1.0    # raw ADC units per gram (from hx711.set_scale())

# ── Server config ─────────────────────────────────────────────────────────────
HOST = "0.0.0.0"   # listen on all interfaces
PORT = 8765
BROADCAST_HZ = 5   # how often to send data (per second)

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
log = logging.getLogger(__name__)


class Encoder:
    """Quadrature encoder — counts rising/falling edges on both channels."""

    COUNTS_PER_REV = 9600  # match value in motor_encoder.py

    def __init__(self, handle, gpio_a, gpio_b):
        self.handle = handle
        self.gpio_a = gpio_a
        self.gpio_b = gpio_b
        self.count = 0
        self.last_count = 0
        self.last_time = time.time()

        lgpio.gpio_claim_input(self.handle, gpio_a, lgpio.SET_PULL_UP)
        lgpio.gpio_claim_input(self.handle, gpio_b, lgpio.SET_PULL_UP)
        lgpio.gpio_set_alert_func(self.handle, gpio_a, self._pulse)
        lgpio.gpio_set_alert_func(self.handle, gpio_b, self._pulse)

    def _pulse(self, chip, gpio, level, tick):
        self.count += 1

    def rpm(self):
        now = time.time()
        dt = now - self.last_time
        if dt < 0.001:
            return 0.0
        delta = self.count - self.last_count
        self.last_count = self.count
        self.last_time = now
        return (delta / self.COUNTS_PER_REV) / dt * 60.0

    @property
    def total_ticks(self):
        return self.count


class HX711:
    def __init__(self, dt_pin, sck_pin, handle):
        self.dt = dt_pin
        self.sck = sck_pin
        self.handle = handle
        lgpio.gpio_claim_input(self.handle, self.dt)
        lgpio.gpio_claim_output(self.handle, self.sck)
        lgpio.gpio_write(self.handle, self.sck, 0)

    def _is_ready(self, timeout=1.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if lgpio.gpio_read(self.handle, self.dt) == 0:
                return True
            time.sleep(0.001)
        return False  # return False instead of raising so the server keeps running

    def read_raw(self):
        if not self._is_ready():
            return None
        data = 0
        for _ in range(24):
            lgpio.gpio_write(self.handle, self.sck, 1)
            time.sleep(0.000001)
            bit = lgpio.gpio_read(self.handle, self.dt)
            lgpio.gpio_write(self.handle, self.sck, 0)
            data = (data << 1) | bit
        # 25th pulse — set gain=128 channel A
        lgpio.gpio_write(self.handle, self.sck, 1)
        time.sleep(0.000001)
        lgpio.gpio_write(self.handle, self.sck, 0)
        if data & 0x800000:
            data -= 1 << 24
        return data

    def get_weight(self, samples=5):
        """Return weight in grams, averaged over `samples` readings."""
        readings = []
        for _ in range(samples):
            r = self.read_raw()
            if r is not None:
                readings.append(r)
        if not readings:
            return None
        avg = sum(readings) / len(readings)
        return (avg - TARE_OFFSET) / SCALE_FACTOR if SCALE_FACTOR != 0 else avg


# ─────────────────────────────────────────────────────────────────────────────
# Shared state — written by sensor thread, read by websocket broadcasts
# ─────────────────────────────────────────────────────────────────────────────
state = {
    "rpm_left":    0.0,
    "rpm_right":   0.0,
    "ticks_left":  0,
    "ticks_right": 0,
    "weight_g":    0.0,
    "raw_adc":     0,
    "ts":          0.0,
}
state_lock = threading.Lock()


def sensor_loop(enc_left, enc_right, hx711, interval=0.2):
    """Runs in a background thread; updates `state` at `interval` seconds."""
    while True:
        rpm_l  = enc_left.rpm()
        rpm_r  = enc_right.rpm()
        weight = hx711.get_weight()
        raw    = hx711.read_raw()

        with state_lock:
            state["rpm_left"]    = round(max(0.0, rpm_l), 1)
            state["rpm_right"]   = round(max(0.0, rpm_r), 1)
            state["ticks_left"]  = enc_left.total_ticks
            state["ticks_right"] = enc_right.total_ticks
            state["weight_g"]    = round(weight, 1) if weight is not None else state["weight_g"]
            state["raw_adc"]     = raw if raw is not None else state["raw_adc"]
            state["ts"]          = time.time()

        time.sleep(interval)


# ─────────────────────────────────────────────────────────────────────────────
# WebSocket server
# ─────────────────────────────────────────────────────────────────────────────
connected_clients: set = set()


async def broadcast_loop():
    interval = 1.0 / BROADCAST_HZ
    while True:
        if connected_clients:
            with state_lock:
                payload = json.dumps(state)
            websockets.broadcast(connected_clients, payload)
        await asyncio.sleep(interval)


async def handler(websocket):
    connected_clients.add(websocket)
    log.info("Client connected: %s  (total: %d)", websocket.remote_address, len(connected_clients))
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        log.info("Client disconnected (total: %d)", len(connected_clients))


async def main():
    handle = lgpio.gpiochip_open(0)

    enc_left  = Encoder(handle, LEFT_ENCODER_A_PIN,  LEFT_ENCODER_B_PIN)
    enc_right = Encoder(handle, RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN)
    hx711     = HX711(HX711_DT_PIN, HX711_SCK_PIN, handle)

    # Sensor polling runs in a daemon thread so it doesn't block the event loop
    t = threading.Thread(target=sensor_loop, args=(enc_left, enc_right, hx711), daemon=True)
    t.start()

    log.info("WebSocket server listening on ws://%s:%d", HOST, PORT)
    log.info("Connect the dashboard to: ws://<this-pi-ip>:%d", PORT)

    async with websockets.serve(handler, HOST, PORT):
        await broadcast_loop()   # runs forever


if __name__ == "__main__":
    asyncio.run(main())
