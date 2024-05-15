import serial
import struct
import time

GYRO_500 = 17.50e-3
ACC_2G = (0.061 / 1000.0)


class IMUDriver:

    def __init__(self, port='/dev/ttyACM0', baud=115200):
        self.__p = port
        self.__b = baud

    def open(self):
        self.__ser = serial.Serial(self.__p, self.__b, 8, "N", 1, 1000)

    def sample(self):
        self.__ser.write([0x0d])
        data = self.__ser.read(12)
        # print(data)
        (ax, ay, az, gx, gy, gz) = struct.unpack("<hhhhhh", data)
        return (ax * ACC_2G, ay * ACC_2G, az * ACC_2G,
                gx * GYRO_500, gy * GYRO_500, gz * GYRO_500)


if __name__ == "__main__":
    drv = IMUDriver()
    drv.open()

    while True:
        print(drv.sample()[0])
        time.sleep(0.1)
        
