"""Standalone INA219 current and power monitor for ATLAS electronics.

This script is relevant as a hardware diagnostic tool: it is separate from the
main fusion loop, but it can help validate battery, bus-voltage, and motor-load
behavior during bring-up.
"""

import smbus2
import time

# INA219 Register addresses
INA219_ADDRESS = 0x40
INA219_REG_CONFIG = 0x00
INA219_REG_SHUNTVOLTAGE = 0x01
INA219_REG_BUSVOLTAGE = 0x02
INA219_REG_POWER = 0x03
INA219_REG_CURRENT = 0x04
INA219_REG_CALIBRATION = 0x05

class INA219:
    def __init__(self, i2c_bus=1, address=INA219_ADDRESS, shunt_ohms=0.002, max_current=20.0):
        """
        Initialize the INA219 current sensor.

        :param i2c_bus: I2C bus number (default: 1 for CM4 GP2/GP3)
        :param address: I2C address (default: 0x40)
        :param shunt_ohms: Shunt resistor value in ohms (default: 0.002 = 2mΩ)
        :param max_current: Maximum expected current in amps (default: 20A)
        """
        self.bus = smbus2.SMBus(i2c_bus)
        self.address = address
        self.shunt_ohms = shunt_ohms
        self.max_current = max_current
        self._calibrate()

    def _calibrate(self):
        """
        Calibrate the INA219 for the given shunt resistor and max current.
        Sets the calibration register and configuration register.
        """
        # Current LSB = max_current / 32768
        self.current_lsb = self.max_current / 32768.0

        # Calibration value = 0.04096 / (current_lsb * shunt_ohms)
        calibration = int(0.04096 / (self.current_lsb * self.shunt_ohms))

        # Write calibration register
        self.bus.write_word_data(
            self.address,
            INA219_REG_CALIBRATION,
            self._swap_bytes(calibration)
        )

        # Configuration: 32V range, ±320mV shunt, 12-bit, continuous
        config = 0x3FFF
        self.bus.write_word_data(
            self.address,
            INA219_REG_CONFIG,
            self._swap_bytes(config)
        )

    def _swap_bytes(self, value):
        """Swap bytes for INA219 (big-endian to little-endian)."""
        return ((value & 0xFF) << 8) | ((value >> 8) & 0xFF)

    def _read_register(self, register):
        """Read a 16-bit register from the INA219."""
        raw = self.bus.read_word_data(self.address, register)
        # Swap bytes (INA219 is big-endian)
        return ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)

    def get_shunt_voltage(self):
        """
        Read shunt voltage in millivolts.
        :return: Shunt voltage in mV
        """
        raw = self._read_register(INA219_REG_SHUNTVOLTAGE)
        # Handle signed value
        if raw > 32767:
            raw -= 65536
        return raw * 0.01  # LSB = 10uV = 0.01mV

    def get_bus_voltage(self):
        """
        Read bus voltage in volts.
        :return: Bus voltage in V
        """
        raw = self._read_register(INA219_REG_BUSVOLTAGE)
        # Shift right 3 bits, LSB = 4mV
        return (raw >> 3) * 0.004

    def get_current(self):
        """
        Read current in amps.
        :return: Current in A
        """
        raw = self._read_register(INA219_REG_CURRENT)
        if raw > 32767:
            raw -= 65536
        return raw * self.current_lsb

    def get_power(self):
        """
        Read power in watts.
        :return: Power in W
        """
        raw = self._read_register(INA219_REG_POWER)
        return raw * (self.current_lsb * 20)

    def close(self):
        """Close the I2C bus."""
        self.bus.close()


if __name__ == "__main__":
    # Initialize INA219 with 2mΩ shunt resistor and 20A max current
    sensor = INA219(
        i2c_bus=1,
        address=0x40,
        shunt_ohms=0.002,
        max_current=20.0
    )

    print("INA219 Current Sensor")
    print("---------------------")

    try:
        while True:
            bus_voltage = sensor.get_bus_voltage()
            shunt_voltage = sensor.get_shunt_voltage()
            current = sensor.get_current()
            power = sensor.get_power()

            print(f"Bus Voltage  : {bus_voltage:.3f} V")
            print(f"Shunt Voltage: {shunt_voltage:.3f} mV")
            print(f"Current      : {current:.3f} A")
            print(f"Power        : {power:.3f} W")
            print("---------------------")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        sensor.close()
