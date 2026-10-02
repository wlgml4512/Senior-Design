"""HX711 load-cell driver for the embedded subsystem."""

import lgpio
import time

class HX711:
    def __init__(self, dt_pin, sck_pin, handle):
        """
        Initialize the HX711 load cell amplifier.
        
        :param dt_pin: GPIO pin number for data (DOUT)
        :param sck_pin: GPIO pin number for clock (PD_SCK)
        :param handle: lgpio chip handle from lgpio.gpiochip_open()
        """
        self.dt = dt_pin
        self.sck = sck_pin
        self.handle = handle
        self.offset = 0  # Tare offset (raw value with no load)
        self.scale = 1.0 # Conversion factor: raw units per gram
        self._closed = False

        # Configure DT as input and SCK as output, with clock starting low
        lgpio.gpio_claim_input(self.handle, self.dt)
        lgpio.gpio_claim_output(self.handle, self.sck)
        lgpio.gpio_write(self.handle, self.sck, 0)

    def _is_ready(self, timeout=2.0):
        """
        Wait until the HX711 signals it has data ready.
        The HX711 pulls DOUT low when a new reading is available.
        
        :param timeout: Max seconds to wait before raising an error
        :raises TimeoutError: If the sensor doesn't respond in time
        """
        self._ensure_open()
        deadline = time.time() + timeout
        while time.time() < deadline:
            if lgpio.gpio_read(self.handle, self.dt) == 0:
                return True
            time.sleep(0.001)
        raise TimeoutError("HX711 not ready — check wiring and power")

    def _ensure_open(self):
        if self._closed:
            raise RuntimeError("HX711 is closed")
    
    def read_raw(self):
        """
        Read a single raw 24-bit signed integer from the HX711.
        
        The HX711 protocol works by toggling SCK high/low 24 times and
        reading one bit from DOUT on each cycle. A 25th pulse sets the
        gain/channel for the next reading (gain=128, channel A by default).
        
        :return: Signed 24-bit integer representing the raw ADC value
        """
        self._is_ready()

        data = 0
        for _ in range(24):
            lgpio.gpio_write(self.handle, self.sck, 1)
            time.sleep(0.000001)
            bit = lgpio.gpio_read(self.handle, self.dt)  # read while clock is high
            lgpio.gpio_write(self.handle, self.sck, 0)
            data = (data << 1) | bit                     # shift after reading

        lgpio.gpio_write(self.handle, self.sck, 1)
        time.sleep(0.000001)
        lgpio.gpio_write(self.handle, self.sck, 0)

        if data & 0x800000:
            data = data - (1 << 24)

        return data

    def tare(self):
        """
        Calibrate the zero point by averaging 20 readings with no load.
        The result is stored as self.offset and subtracted in get_weight().
        Call this with nothing on the scale.
        """
        self._ensure_open()
        total = 0
        for _ in range(20):
            total = total + self.read_raw()

        self.offset = total / 20

    def set_scale(self, scale):
        """
        Set the scaling factor used to convert raw ADC units to grams.
        
        :param scale: Raw ADC units per gram (calculated during calibration)
        """
        self._ensure_open()
        self.scale = scale

    def get_weight(self):
        """
        Return the current weight in grams, averaged over 10 readings.
        Applies the tare offset and scale factor set during calibration.
        
        :return: Weight in grams as a float
        """
        self._ensure_open()
        total = 0
        for _ in range(10):
            total = total + self.read_raw()

        return ((total / 10) - self.offset) / self.scale

    def close(self):
        """Leave the HX711 pins in a safe state without closing a shared chip handle."""
        if self._closed:
            return

        try:
            lgpio.gpio_write(self.handle, self.sck, 0)
        except Exception:
            pass

        gpio_free = getattr(lgpio, "gpio_free", None)
        if callable(gpio_free):
            for pin in (self.dt, self.sck):
                try:
                    gpio_free(self.handle, pin)
                except Exception:
                    pass

        self._closed = True

    def cleanup(self):
        """Compatibility alias for callers that expect cleanup()."""
        self.close()



if __name__ == '__main__':
    chip_handle = lgpio.gpiochip_open(0)
    hx711 = HX711(dt_pin=19, sck_pin=6, handle=chip_handle)

    # Print 10 raw readings to verify the sensor is wired and responding
    for _ in range(10):
        print(hx711.read_raw())
        time.sleep(0.1)

    # Zero the scale with no load applied
    hx711.tare()
    print(hx711.get_weight()) 

    # --- Calibration step ---
    # Place a known 500 g weight, then calculate how many raw units = 1 gram
    input("Place 500g weight on scale, then press Enter...")
    raw = sum(hx711.read_raw() for _ in range(10)) / 10
    scale = raw / 500  # divide by known weight in grams
    hx711.set_scale(scale)
    print(hx711.get_weight())  # should read ~500

    hx711.close()
    lgpio.gpiochip_close(chip_handle)






