# SPDX-FileCopyrightText: Copyright (c) 2023 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

# Written by Liz Clark (Adafruit Industries) with OpenAI ChatGPT v4 September 25, 2023 build
# https://help.openai.com/en/articles/6825453-chatgpt-release-notes

# https://chat.openai.com/share/f4f94c37-66a1-42d9-879b-9624c13f3e26
"""
`adafruit_vcnl4020`
================================================================================

Driver for the VCNL4020 proximity and light sensor


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* Adafruit VCNL4020 Proximity and Light Sensor <https://www.adafruit.com/product/5810>

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_struct import Struct, ROUnaryStruct
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits

try:
    import typing  # pylint: disable=unused-import
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_VCNL4020.git"

_I2C_ADDRESS = const(0x13)
_REG_COMMAND = const(0x80)
_REG_PRODUCT_ID = const(0x81)
_REG_PROX_RATE = const(0x82)
_REG_IR_LED_CURRENT = const(0x83)
_REG_AMBIENT_PARAM = const(0x84)
_REG_AMBIENT_RESULT_HIGH = const(0x85)
_REG_AMBIENT_RESULT_LOW = const(0x86)
_REG_PROX_RESULT_HIGH = const(0x87)
_REG_PROX_RESULT_LOW = const(0x88)
_REG_INT_CTRL = const(0x89)
_REG_LOW_THRES_HIGH = const(0x8A)
_REG_LOW_THRES_LOW = const(0x8B)
_REG_HIGH_THRES_HIGH = const(0x8C)
_REG_HIGH_THRES_LOW = const(0x8D)
_REG_INT_STATUS = const(0x8E)
_REG_PROX_ADJUST = const(0x8F)
_INT_TH_HI = const(0x01)
_INT_TH_LOW = const(0x02)
_INT_ALS_READY = const(0x04)
_INT_PROX_READY = const(0x08)

PROX_RATE_1_95_PER_S = const(0x00)
PROX_RATE_3_9_PER_S = const(0x01)
PROX_RATE_7_8_PER_S = const(0x02)
PROX_RATE_16_6_PER_S = const(0x03)
PROX_RATE_31_2_PER_S = const(0x04)
PROX_RATE_62_5_PER_S = const(0x05)
PROX_RATE_125_PER_S = const(0x06)
PROX_RATE_250_PER_S = const(0x07)

AMBIENT_RATE_1_SPS = const(0x00)
AMBIENT_RATE_2_SPS = const(0x01)
AMBIENT_RATE_3_SPS = const(0x02)
AMBIENT_RATE_4_SPS = const(0x03)
AMBIENT_RATE_5_SPS = const(0x04)
AMBIENT_RATE_6_SPS = const(0x05)
AMBIENT_RATE_8_SPS = const(0x06)
AMBIENT_RATE_10_SPS = const(0x07)

AVG_1_SAMPLES = const(0x00)
AVG_2_SAMPLES = const(0x01)
AVG_4_SAMPLES = const(0x02)
AVG_8_SAMPLES = const(0x03)
AVG_16_SAMPLES = const(0x04)
AVG_32_SAMPLES = const(0x05)
AVG_64_SAMPLES = const(0x06)
AVG_128_SAMPLES = const(0x07)

INT_COUNT_1 = const(0x00)
INT_COUNT_2 = const(0x01)
INT_COUNT_4 = const(0x02)
INT_COUNT_8 = const(0x03)
INT_COUNT_16 = const(0x04)
INT_COUNT_32 = const(0x05)
INT_COUNT_64 = const(0x06)
INT_COUNT_128 = const(0x07)

PROX_FREQ_390_625_KHZ = const(0x00)
PROX_FREQ_781_25_KHZ = const(0x01)
PROX_FREQ_1_5625_MHZ = const(0x02)
PROX_FREQ_3_125_MHZ = const(0x03)


# pylint: disable=too-many-instance-attributes
class Adafruit_VCNL4020:
    """Adafruit VCNL4020 Proximity/Ambient Light sensor driver"""

    _auto_offset_comp_bit = RWBit(_REG_AMBIENT_PARAM, 3)
    _command_reg = RWBits(8, _REG_COMMAND, 0)
    _continuous_conversion_bit = RWBit(_REG_AMBIENT_PARAM, 7)
    _int_ctrl_reg = RWBits(8, _REG_INT_CTRL, 0)
    _int_status_reg = RWBits(3, _REG_INT_STATUS, 0)
    _led_current = RWBits(6, _REG_IR_LED_CURRENT, 0)
    _product_revision = ROUnaryStruct(_REG_PRODUCT_ID, "<B")
    lux = ROUnaryStruct(_REG_AMBIENT_RESULT_HIGH, ">H")
    """Reads the ambient light/lux sensor (ALS) measurement result"""
    lux_averaging = RWBits(3, _REG_AMBIENT_PARAM, 0)
    """Ambient averaging rate"""
    lux_enabled = RWBit(_REG_COMMAND, 2)
    """Enable/disable lux sensor"""
    lux_on_demand = RWBit(_REG_COMMAND, 4)
    """On-demand setting for lux measurements"""
    lux_rate = RWBits(3, _REG_AMBIENT_PARAM, 4)
    """Ambient light measurement rate"""
    proximity = ROUnaryStruct(_REG_PROX_RESULT_HIGH, ">H")
    """Reads the proximity measurement result"""
    proximity_enabled = RWBit(_REG_COMMAND, 1)
    """Enable/disable proximity sensor"""
    proximity_frequency = RWBits(2, _REG_PROX_ADJUST, 3)
    """Proximity frequency setting"""
    promixity_on_demand = RWBit(_REG_COMMAND, 3)
    """On-demand setting for proximity measurements"""
    proximity_rate = RWBits(3, _REG_PROX_RATE, 0)
    """Rate that proximity data is available"""
    low_threshold = Struct(_REG_LOW_THRES_HIGH, ">H")
    """Sets the low threshold for proximity measurement"""
    high_threshold = Struct(_REG_HIGH_THRES_HIGH, ">H")
    """Sets the high threshold for proximity measurement."""
    interrupt_count = RWBits(3, _REG_INT_CTRL, 5)
    """Interrupt count setting"""
    proximity_interrupt = RWBit(_REG_INT_CTRL, 3)
    """Enable/disable proximity interrupt"""
    lux_interrupt = RWBit(_REG_INT_CTRL, 2)
    """Enable/disable lux interrupt"""
    high_threshold_interrupt = RWBit(_REG_INT_CTRL, 1)
    """Enable/disable proximity high threshold interrupt"""
    low_threshold_interrupt = RWBit(_REG_INT_CTRL, 0)
    """Enable/disable proximity low threshold interrupt"""
    selftimed_enabled = RWBit(_REG_COMMAND, 0)
    """Enable/disable selftimed reading"""

    def __init__(self, i2c: I2C, addr: int = _I2C_ADDRESS) -> None:
        """
        Initializes the VCNL4020 sensor and checks for a valid Product ID Revision.
        :param i2c: The I2C interface to use
        :param addr: The I2C address of the VCNL4020, defaults to _I2C_ADDRESS
        """
        self.i2c_device = I2CDevice(i2c, addr)

        # Check the Product ID Revision
        if self._product_revision != 0x21:
            raise RuntimeError(f"Invalid Product ID Revision {self._product_revision}")
        try:
            # Disable to setup configuration
            self.lux_enabled = False
            self.proximity_enabled = False
            self.selftimed_enabled = False
            # Configuration settings
            self.proximity_rate = PROX_RATE_250_PER_S
            self.led_current = 200
            self.lux_rate = AMBIENT_RATE_10_SPS
            self.lux_averaging = AVG_1_SAMPLES
            # Reenable to activate configuration
            self.lux_enabled = True
            self.proximity_enabled = True
            self.selftimed_enabled = True
        except Exception as error:
            raise RuntimeError(f"Failed to initialize: {error}") from error

    @property
    def lux_ready(self) -> int:
        """
        Status of ambient light data.

        :return: True if ALS data is ready, otherwise false.
        """
        return (self._command_reg >> 6) & 0x01

    @property
    def proximity_ready(self) -> int:
        """
        Status of proximity data.

        :return: True if proximity data is ready, otherwise false.
        """
        return (self._command_reg >> 5) & 0x01

    @property
    def led_current(self) -> int:
        """
        The LED current for proximity mode in mA.

        :return: The LED current in mA.
        """
        return self._led_current * 10

    @led_current.setter
    def led_current(self, value: int) -> None:
        self._led_current = value // 10

    @property
    def clear_interrupts(self) -> None:
        """
        Clears the interrupt flags.

        :param value: True to clear all interrupt flags.
        """
        clear_bits = 0
        clear_bits |= _INT_PROX_READY
        clear_bits |= _INT_ALS_READY
        clear_bits |= _INT_TH_LOW
        clear_bits |= _INT_TH_HI
        self._int_status_reg |= clear_bits

    @property
    def auto_offset_comp(self) -> bool:
        """
        Auto offset compensation for ambient light measurement.

        :return: True if enabled, False if disabled.
        """
        return bool(self._auto_offset_comp_bit)

    @auto_offset_comp.setter
    def auto_offset_comp(self, enable: bool) -> None:
        self._auto_offset_comp_bit = enable

    @property
    def continuous_conversion(self) -> bool:
        """
        Continuous conversion mode for ambient light measurement.

        :return: True if enabled, False if disabled.
        """
        return bool(self._continuous_conversion_bit)

    @continuous_conversion.setter
    def continuous_conversion(self, enable: bool) -> None:
        self._continuous_conversion_bit = enable
