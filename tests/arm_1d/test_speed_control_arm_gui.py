#
# test_speed_control_arm_gui.py
#

import sys
sys.path.insert(0, '../../lib')

import math

from models.arm import *
from models.robot import *
from controllers.standard import *
from gui.gui_1d import *
from data.plot import *

from PyQt5.QtWidgets import QApplication

class ArmRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3) # delta_t = 1e-3
        # Mass = 1kg
        # friction = 0.8
        # lenght = 0.6
        self.arm = Arm(1, 0.8, 0.6)
        self.plotter = DataPlotter()
        self.controller = PID(80, 400, 0)
        self.target_speed = 1.5 # 1.5 rad/s

    def run(self):
        T = self.controller.evaluate(self.delta_t, self.target_speed, self.get_speed())
        self.arm.evaluate(self.delta_t, T)
        self.plotter.add('t', self.t)
        self.plotter.add('T', T)
        self.plotter.add('target', self.target_speed)
        self.plotter.add('omega', self.get_speed())
        self.plotter.add('theta', math.degrees(self.get_pose()))
        if self.t >= 10: # after 20 seconds plot data and stop simulation
            self.plotter.plot(['t', 'time'],
                                  [ [ 'target', 'Target Speed' ],
                                    [ 'omega', 'Current Speed' ] ])
            self.plotter.show()
            return False
        else:
            return True

    def get_pose(self):
        return self.arm.theta

    def get_speed(self):
        return self.arm.omega


if __name__ == '__main__':
    cart_sys = ArmRobot()
    app = QApplication(sys.argv)
    ex = ArmWindow(cart_sys)
    sys.exit(app.exec_())

