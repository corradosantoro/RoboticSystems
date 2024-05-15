import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.controllers.standard import PIDSat
from lib.data.plot import DataPlotter
from lib.godot.interface import *

import math
import keyboard

# propeller order
#
#  3     4
#
#  2     1
#


class MultirotorRobot:

    def __init__(self):
        self.MR = GodotDrone()
        self.vz_control = PIDSat(5.0, 10.0, 0.0,
                                 5, True)
        self.z_control = PIDSat(2.0, 0.0, 0.0, 2)  # 2 m/s

        self.w_roll_control = PIDSat(0.2, 0.5, 0.0, 2, True)
        self.roll_control = PIDSat(0.5, 0.0, 0.0, 2, True)

        self.w_pitch_control = PIDSat(0.2, 0.5, 0.0, 2, True)
        self.pitch_control = PIDSat(0.5, 0.0, 0.0, 2, True)

        self.z_target = 0.0
        self.roll_target = 0.0
        self.pitch_target = 0.0

        self.f = 0
        self.f_roll = 0
        self.f_pitch = 0

        self.t = 0

        self.plot = DataPlotter()

    def run(self):
        f1 = self.f + self.f_roll - self.f_pitch
        f2 = self.f - self.f_roll - self.f_pitch
        f3 = self.f - self.f_roll + self.f_pitch
        f4 = self.f + self.f_roll + self.f_roll
        (self.delta_t, x, y, z, roll, pitch, yaw, vx, vy, vz, w_roll, w_pitch, w_yaw) = self.MR.process(f1, f2, f3, f4)

        self.t += self.delta_t

        # altitude control
        vz_target = self.z_control.evaluate(self.delta_t, self.z_target, z)
        self.f = self.vz_control.evaluate(self.delta_t, vz_target, vz)

        # roll control
        self.w_roll_target = self.roll_control.evaluate(self.delta_t, self.roll_target, roll)
        self.f_roll = self.w_roll_control.evaluate(self.delta_t, self.w_roll_target, w_roll)

        # pitch control
        self.w_pitch_target = self.pitch_control.evaluate(self.delta_t, self.pitch_target, pitch)
        self.f_pitch = self.w_pitch_control.evaluate(self.delta_t, self.w_pitch_target, w_pitch)

        # self.plot.add('t', self.t)
        # self.plot.add('w_roll', w_roll)
        # self.plot.add('w_roll_target', self.w_roll_target)
        # self.plot.add('z', z)
        # self.plot.add('f', self.f)

        return True




if __name__ == '__main__':
    robot = MultirotorRobot()
    running = True
    degrees_increment = 2.0
    while running:
        robot.run()
        if keyboard.is_pressed('space'):
            # take-off and land
            if robot.z_target == 0.0:
                print('Take-off')
                robot.z_target = 1.0
            else:
                print('Land')
                robot.z_target = 0.0
        elif keyboard.is_pressed('left'):
            print('Left')
            robot.roll_target += math.radians(degrees_increment)
        elif keyboard.is_pressed('right'):
            print('Right')
            robot.roll_target -= math.radians(degrees_increment)
        elif keyboard.is_pressed('down'):
            print('Down')
            robot.pitch_target += math.radians(degrees_increment)
        elif keyboard.is_pressed('up'):
            print('Up')
            robot.pitch_target -= math.radians(degrees_increment)


