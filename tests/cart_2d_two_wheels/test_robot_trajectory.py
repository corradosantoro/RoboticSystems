#
# test_robot_trajectory.py
#

import sys
from pathlib import Path
import math

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import TwoWheelsCart2DEncodersOdometry
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.gui.gui_2d import CartWindow
from lib.data.plot import DataPlotter

from PyQt5.QtWidgets import QApplication


class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 20kg
        # radius = 15cm
        # friction = 0.8
        # Traction Wheels, radius = 2.5cm, wheelbase = 20cm
        # Sensing Wheels, radius = 2cm, wheelbase = 24cm
        # Encoder resolution = 4000 ticks/revolution
        self.cart = TwoWheelsCart2DEncodersOdometry(20, 0.15, 0.8, 0.8,
                                                    0.025, 0.025, 0.2,
                                                    0.02, 0.02, 0.24, 2 * math.pi / 4000.0)

        # 5 Nm of max torque, antiwindup
        self.left_controller = PIDSat(8.0, 3.0, 0.0, 5, True)
        self.right_controller = PIDSat(8.0, 3.0, 0.0, 5, True)

        self.polar_controller = Polar2DController(2.5, 2, 2.0, 2)
        self.trajectory = StraightLine2DMotion(0.2, 0.5, 0.5)
        (x, y, _) = self.get_pose()
        self.trajectory.start_motion((x, y), (0.5, 0.2))

        self.plotter = DataPlotter()

    def run(self):
        # virtual robot movement
        (x_target, y_target) = self.trajectory.evaluate(self.delta_t)

        # polar control
        (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())
        vref_l = v_target - w_target * self.cart.encoder_wheelbase / 2.0
        vref_r = v_target + w_target * self.cart.encoder_wheelbase / 2.0

        (vl, vr) = self.cart.get_wheel_speed()
        # speed control (left, right)
        Tleft = self.left_controller.evaluate(self.delta_t, vref_l, vl)
        Tright = self.right_controller.evaluate(self.delta_t, vref_r, vr)

        # robot model
        self.cart.evaluate(self.delta_t, Tleft, Tright)

        (x, y, _) = self.get_pose()
        self.plotter.add('t', self.t)
        self.plotter.add('x', x)
        self.plotter.add('y', y)
        self.plotter.add('x_target', x_target)
        self.plotter.add('y_target', y_target)

        if self.t > 10:
            self.plotter.plot(['t', 'time'],
                              [['x', 'X'], ['x_target', 'X Target']])
            self.plotter.plot(['t', 'time'],
                              [['y', 'Y'], ['y_target', 'Y Target']])
            self.plotter.show()
            return False

        return True

    def get_pose(self):
        return self.cart.get_pose()

    def get_speed(self):
        return (self.cart.v, self.cart.w)


if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot)
    sys.exit(app.exec_())
