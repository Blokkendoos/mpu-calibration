import smbus2
import time
import logging


class HMC5883():
    """ The HMC5883 class facilitates read-out of the Magnetometer """

    # HMC5883 Registers (all address locations are 8 bits)
    CONFIG_A = 0x00  # configuration register A (R/W)
    CONFIG_B = 0x01  # configuration register B (R/W)
    MODE = 0x02  # mode register (R/W)
    OUTPUT_X_MSB = 0x03  # data output X MSB (R)
    OUTPUT_X_LSB = 0x04  # data output X LSB (R)
    OUTPUT_Z_MSB = 0x05  # data output Z MSB (R)
    OUTPUT_Z_LSB = 0x06  # data output Z LSB (R)
    OUTPUT_Y_MSB = 0x07  # data output Y MSB (R)
    OUTPUT_Y_LSB = 0x08  # data output Y LSB (R)
    STATUS = 0x09  # status register (R)
    IDENTIFICATION_A = 0x0A  # identification register A (R)
    IDENTIFICATION_B = 0x0B  # identification register B (R)
    IDENTIFICATION_C = 0x0C  # identification register C (R)

    def __init__(self):
        self.bus = smbus2.SMBus(1)  # get I2C bus
        self.bus_address = 0x1E  # HMC5883 address

        # 8 samples averaged
        # normal measurement configuration
        self.bus.write_byte_data(self.bus_address, self.CONFIG_A, 0x60)

        # device gain (default)
        # self.bus.write_byte_data(self.bus_address, self.CONFIG_B, 0x20)

        # continuous measurement mode
        self.bus.write_byte_data(self.bus_address, self.MODE, 0x00)
        time.sleep(0.5)

    def read(self):
        """
        Read the magnetometer values

        :return: (a list of) The magnetometer (xyz) readings
        """
        x_high, x_low, z_high, z_low, y_high, y_low = \
            self.bus.read_i2c_block_data(self.bus_address, 0x03, 6)

        x_mag = 0.0
        y_mag = 0.0
        z_mag = 0.0

        x_mag = (x_high << 8) | x_low
        if x_mag > 32767:
            x_mag -= 65536
        z_mag = (z_high << 8) | z_low
        if z_mag > 32767:
            z_mag -= 65536
        y_mag = (y_high << 8) | y_low
        if y_mag > 32767:
            y_mag -= 65536

        logging.debug(f"_magnetic field "
                      f"X: {x_mag:.4f} Y: {y_mag:.4f} Z: {z_mag:.4f}")

        return (x_mag, y_mag, z_mag)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    hmc = HMC5883()
    while (True):
        hmc.read()
        time.sleep(1)
