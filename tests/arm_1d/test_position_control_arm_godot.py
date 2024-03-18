import sys
import math

from pathlib import Path
from PyQt5.QtWidgets import QApplication

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.controllers.standard import *
from lib.data.plot import DataPlotter
from lib.godot.interface import *

class ArmRobot:

    def __init__(self):
        self.arm = GodotArm1D()
        self.plotter = DataPlotter()
        self.controller = PID(2, 0.5, 0)
        self.target = math.radians(20)
        self.t = 0
        self.torque = 0

    def run(self):
        (delta_t, theta, omega) = self.arm.process(self.torque)
        self.torque = self.controller.evaluate(delta_t, self.target, theta)

        self.t += delta_t

        self.plotter.add('t', self.t)
        self.plotter.add('T', self.torque)
        self.plotter.add('target', math.degrees(self.target))
        self.plotter.add('error', math.degrees(self.target - theta))
        self.plotter.add('omega', omega)
        self.plotter.add('theta', math.degrees(theta))
        if self.t >= 10:  # after 20 seconds plot data and stop simulation
            self.plotter.plot(['t', 'time'],
                              [['target', 'Target theta'],
                               ['theta', 'Current Theta']])
            self.plotter.show()
            return False
        else:
            return True


if __name__ == '__main__':
    arm_sys = ArmRobot()
    while arm_sys.run():
        print("Time = ", arm_sys.t, "\r", end='')

