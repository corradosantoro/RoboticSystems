#
# test_robot_2d_gui.py
#

import sys
sys.path.insert(0, '../../lib')

from models.cart2d import *
from models.robot import *
from gui.gui_2d import *

from PyQt5.QtWidgets import QApplication

class AckermannRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3) # delta_t = 1e-3
        # Mass = 10kg
        # side = 15cm
        # wheels radius = 2cm
        # friction = 0.8
        self.car = AckermannSteering(10, 0.8, 0.02, 0.15)

    def run(self):
        Torque = 0.02
        Steering = math.radians(10)

        self.car.evaluate(self.delta_t, Torque, Steering)
        return True

    def get_pose(self):
        return self.car.get_pose()

    def get_speed(self):
        return (self.car.v, self.car.w)


if __name__ == '__main__':
    cart_robot = AckermannRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot, 'ackermann_robot_2d.png')
    sys.exit(app.exec_())
