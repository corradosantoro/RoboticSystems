import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.controllers.standard import PIDSat
from lib.data.plot import DataPlotter
from lib.godot.interface import *

import math
import pygame


class MultirotorRobot:

    def __init__(self):
        self.MR = GodotDrone()
        self.vz_control = PIDSat(5.0, 10.0, 0.0,
                                 5, True)
        self.z_control = PIDSat(2.0, 0.0, 0.0, 2)  # 2 m/s

        self.w_roll_control = PIDSat(0.5, 1.0, 0.02, 1, True)
        self.roll_control = PIDSat(1.0, 0.0, 0.0, 2, True) # max 2 rad/s

        self.w_pitch_control = PIDSat(0.5, 1.0, 0.02, 1, True)
        self.pitch_control = PIDSat(1.0, 0.0, 0.0, 2, True) # max 2 rad/s

        self.y_control = PIDSat(1.5, 0.0, 0.0,
                                 2.0, True)
        self.vy_control = PIDSat(0.1, 0.1, 0.5,
                                 math.radians(40), True)

        self.x_control = PIDSat(1.5, 0.0, 0.0,
                                 2.0, True)
        self.vx_control = PIDSat(0.1, 0.1, 0.5,
                                 math.radians(40), True)

        self.z_target = 1.5
        self.roll_target = 0.0 #math.radians(2.5)
        self.pitch_target = 0.0 #math.radians(5)
        self.xy_target = 0.0
        self.x_target = 0.0
        self.vy_target = 0.0
        self.y_target = 0.0

        self.f = 0
        self.f_roll = 0
        self.f_pitch = 0

        self.t = 0

        self.plot = DataPlotter()

    def run(self):
        # propeller order
        #
        #  3     4
        #
        #  2     1
        #
        f1 = self.f + self.f_roll - self.f_pitch
        f2 = self.f - self.f_roll - self.f_pitch
        f3 = self.f - self.f_roll + self.f_pitch
        f4 = self.f + self.f_roll + self.f_roll
        (self.delta_t, x, y, z, roll, pitch, yaw, vx, vy, vz, w_roll, w_pitch, w_yaw) = self.MR.process(f1, f2, f3, f4)

        self.t += self.delta_t

        # altitude control
        vz_target = self.z_control.evaluate(self.delta_t, self.z_target, z)
        self.f = self.vz_control.evaluate(self.delta_t, vz_target, vz)

        # x + vx control
        self.vx_target = self.x_control.evaluate(self.delta_t, self.x_target, x)
        self.pitch_target = self.vx_control.evaluate(self.delta_t, self.vx_target, vx)

        # y + vy control
        self.vy_target = self.y_control.evaluate(self.delta_t, self.y_target, y)
        self.roll_target = - self.vy_control.evaluate(self.delta_t, self.vy_target, vy)

        # roll control
        self.w_roll_target = self.roll_control.evaluate(self.delta_t, self.roll_target, roll)
        self.f_roll = self.w_roll_control.evaluate(self.delta_t, self.w_roll_target, w_roll)

        # pitch control
        self.w_pitch_target = self.pitch_control.evaluate(self.delta_t, self.pitch_target, pitch)
        self.f_pitch = self.w_pitch_control.evaluate(self.delta_t, self.w_pitch_target, w_pitch)

        self.plot.add('t', self.t)
        self.plot.add('w_roll', w_roll)
        self.plot.add('w_roll_target', self.w_roll_target)
        self.plot.add('roll', roll)
        self.plot.add('roll_target', self.roll_target)
        self.plot.add('w_pitch', w_pitch)
        self.plot.add('w_pitch_target', self.w_pitch_target)
        self.plot.add('pitch', pitch)
        self.plot.add('pitch_target', self.pitch_target)
        self.plot.add('vy', vy)
        self.plot.add('vx', vx)

        return True


class Joy:

    def __init__(self):
        pygame.init()
        self.j = pygame.joystick.Joystick(0)
        self.j.init()


    def rescale(self, v):
        if v > 0.5:
            return 1
        elif v < -0.5:
            return -1
        else:
            return 0


    def remote(self, robot):
        pygame.event.pump()
        left_h = self.rescale(self.j.get_axis(0))
        left_v = self.rescale(self.j.get_axis(1))

        right_h = self.rescale(self.j.get_axis(3))
        right_v = self.rescale(self.j.get_axis(4))

        robot.z_target += (-left_v) * 0.01
        robot.x_target += right_v * 0.01
        robot.y_target += right_h * 0.01


if __name__ == '__main__':
    robot = MultirotorRobot()
    j = Joy()
    while robot.run():
        j.remote(robot)
        print(robot.x_target, ' ', robot.y_target, ' ', robot.z_target)

