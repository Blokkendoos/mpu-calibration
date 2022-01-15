"""
This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.

Released under the MIT License
Copyright (c) 2015, 2016, 2017, 2021
Martijn (martijn@mrtijn.nl) and contributers
https://github.com/m-rtijn/mpu6050
"""

import logging
import smbus2 as smbus


class MPU6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    FILTER_BW_256 = 0x00
    FILTER_BW_188 = 0x01
    FILTER_BW_98 = 0x02
    FILTER_BW_42 = 0x03
    FILTER_BW_20 = 0x04
    FILTER_BW_10 = 0x05
    FILTER_BW_5 = 0x06

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def read_i2c_word(self, register):
        """
        Get two i2c registers and combine them

        param: register: The first register address to read from
        :return: The combined result (word)
        """
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def get_temp(self):
        """
        Get the temperature from the onboard temperature sensor

        :return: The temperature (degrees Celcius)
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)
        # actual temperature formula from: MPU-6050 Register Map
        # and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53
        return actual_temp

    def set_accel_range(self, accel_range):
        """
        Set the accelerometer range

        param: accel_range: The range to set the accelerometer to
                            Using a pre-defined range is advised
        """
        # first change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw=False):
        """
        Get the accelerometer range from the ACCEL_CONFIG register

        param: raw: True: raw value
                    False: integer value
        :return: The accelerometer range, raw or integer (2, 4, 8 or 16) value,
                 or -1 if something went wrong
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        if raw:
            return raw_data
        else:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g=False):
        """
        Get the accelerometer measurement (x,y,z) values

        :param g: True: unit is g
                  False: unit m/s^2
        :return: The measurement result (tuple) in g or m/s^2
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            logging.warning("Unkown range, accel_scale_modifier set to ACCEL_SCALE_MODIFIER_2G")  # noqa: E501
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier
        if g:
            return (x, y, z)
        else:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return (x, y, z)

    def set_gyro_range(self, gyro_range):
        """
        Set the range of the gyroscope

        :param gyro_range: The range, a pre-defined range is advised
        """
        # change to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def set_filter_range(self, filter_range=FILTER_BW_256):
        """Sets the low-pass bandpass filter frequency"""
        # Keep the current EXT_SYNC_SET configuration in bits 3, 4, 5 in the MPU_CONFIG register  # noqa: E501
        EXT_SYNC_SET = self.bus.read_byte_data(self.address,
                                               self.MPU_CONFIG) & 0b00111000
        return self.bus.write_byte_data(self.address,
                                        self.MPU_CONFIG,
                                        EXT_SYNC_SET | filter_range)

    def read_gyro_range(self, raw=False):
        """
        Get the gyroscope range from the GYRO_CONFIG register

        :param raw: True: Raw value
                    False: Integer value
        :return: The range, raw or integer value (250, 500, 1000, 2000),
                 or -1 if something went wrong
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        if raw:
            return raw_data
        else:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """
        Get the gyroscope measurement (x,y,z) values

        :return: The gyroscope readings (tuple)
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            logging.warning("Unkown range, gyro_scale_modifier set to GYRO_SCALE_MODIFIER_250DEG")  # noqa: E501
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x /= gyro_scale_modifier
        y /= gyro_scale_modifier
        z /= gyro_scale_modifier
        return (x, y, z)

    def get_all_data(self):
        """
        Get all the available data

        :return: Accelerometer, gyroscope, and temperature sensor values (list)
        """
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()
        return [accel, gyro, temp]


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    mpu = MPU6050(0x68)
    print(mpu.get_temp())

    accel_data = mpu.get_accel_data()
    print(accel_data[0])
    print(accel_data[1])
    print(accel_data[2])

    gyro_data = mpu.get_gyro_data()
    print(gyro_data[0])
    print(gyro_data[1])
    print(gyro_data[2])
