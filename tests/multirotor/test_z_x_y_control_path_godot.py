import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.controllers.standard import PIDSat
from lib.data.plot import DataPlotter
from lib.godot.interface import *

import math


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
        f4 = self.f + self.f_roll + self.f_pitch
        (self.delta_t, x, y, z, roll, pitch, yaw, vx, vy, vz, w_roll, w_pitch, w_yaw) = self.MR.process(f1, f2, f3, f4)
        self.x = x
        self.y = y
        self.z = z

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


class Path:

    def __init__(self):
        pass

    def set_path(self, p):
        self.path = p
        
    def start(self, robot):
        self.path_index = 0
        self.robot = robot
        (xt,yt,zt) = self.path[self.path_index]
        self.robot.x_target = xt
        self.robot.y_target = yt
        self.robot.z_target = zt
        print(xt,yt,zt)
        self.end_of_path = False

    def execute(self):
        if self.end_of_path:
            return
        (xt,yt,zt) = self.path[self.path_index]
        x = self.robot.x
        y = self.robot.y
        z = self.robot.z
        dx = x - xt
        dy = y - yt
        dz = z - zt
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        print(x,y,z)
        if (dist < 0.1):
            self.path_index += 1
            if (self.path_index < len(self.path)):
                (xt,yt,zt) = self.path[self.path_index]
                self.robot.x_target = xt
                self.robot.y_target = yt
                self.robot.z_target = zt
                print(xt,yt,zt)
            else:
                self.end_of_path = True


if __name__ == '__main__':
    robot = MultirotorRobot()
    p = Path()
    p.set_path([ (0, 0 ,1.5),
                 (1.0,1.0, 1.5),
                 (-1.0,1.0, 1.5),
                 (-1.0,-1.0, 1.5),
                 (1.0,-1.0, 1.5),
                 (0.0, 0.0, 1.5),
                 (0,0,0)])
    p.start(robot)
    while robot.run():
        p.execute()





