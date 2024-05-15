import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.controllers.standard import PIDSat
from lib.data.plot import DataPlotter
from lib.godot.interface import *


class MultirotorRobot:

    def __init__(self):
        self.MR = GodotDrone()
        self.vz_control = PIDSat(5.0, 10.0, 0.0,
                                 5, True)
        self.z_control = PIDSat(2.0, 0.0, 0.0, 2)  # 2 m/s
        self.z_target = 1.0

        self.f = 0

        self.plot = DataPlotter()

    def run(self):
        (self.delta_t, x, y, z, roll, pitch, yaw, vx, vy, vz, w_roll, w_pitch, w_yaw) = self.MR.process(self.f, self.f, self.f, self.f)
        print(z, vz, roll, pitch)
        self.t += self.delta_t

        vz_target = self.z_control.evaluate(self.delta_t, self.z_target, z)

        self.f = self.vz_control.evaluate(self.delta_t, vz_target, vz)

        self.plot.add('t', self.t)
        self.plot.add('vz', vz)
        self.plot.add('vz_target', vz_target)
        self.plot.add('z', z)
        self.plot.add('f', self.f)

        if self.t >= 20:
            self.plot.plot(['t', 'time'],
                           [['vz', 'vz'], ['vz_target', 'vz target']])
            self.plot.plot(['t', 'time'],
                           [['f', 'F']])
            self.plot.plot(['t', 'time'],
                           [['z', 'Z']])
            self.plot.show()
            return False
        else:
            return True



if __name__ == '__main__':
    robot = MultirotorRobot()
    while robot.run():
        pass

