#
# motor_driver.py
#

import serial

class MotorDriver:

    def __init__(self, port='/dev/ttyACM0', baud=115200):
        self.__p = port
        self.__b = baud

    def open(self):
        self.__ser = serial.Serial(self.__p, self.__b, 8, "N", 1, 1000)
        self.reset()

    def __transaction(self, uCmd, uExp):
        self.__ser.write((uCmd + "\r").encode('utf8'))
        l = len(uExp)
        while True:
            line = self.__ser.readline()
            line = line.decode('utf8')
            #print(line)
            if line[0:l] == uExp:
                break
        return line


    def reset(self):
        self.__transaction("CLR", "#CLROK")

    def pwm(self, val):
        self.__transaction("PWM%d" % (val), "#PWMOK")

    def encoder(self):
        encoder_cmd = self.__transaction("ENC", "#ENC")
        return int(encoder_cmd[4:])


