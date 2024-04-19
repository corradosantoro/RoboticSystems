import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import Cart2D
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.gui.gui_2d import CartWindow
from lib.data.plot import DataPlotter

from PyQt5.QtWidgets import QApplication

from circular_trajectory import *

class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 1kg
        # radius = 15cm
        # friction = 0.8
        self.cart = Cart2D(1, 0.15, 0.8, 0.8)
        self.cart.x = 0.3
        self.cart.y = 0
        self.cart.theta = math.radians(90)
        
        self.linear_speed_controller = PIDSat(10, 3.5, 0, 5)  # 5 newton
        self.angular_speed_controller = PIDSat(6, 10, 0, 4)  # 4 newton * metro
        self.polar_controller = Polar2DController(2.5, 4, 4.0, 10)  # 4m/s, 6rad/s
        self.trajectory = CircularTrajectory(0.2, 0.5, 0.5,
        					-0.3, math.radians(90))  #  R = 0.3, angolo = 90 gradi
        self.trajectory.start(self.get_pose())

        self.plotter = DataPlotter()

    def run(self):
        (x_target, y_target) = self.trajectory.evaluate(self.delta_t)
        (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())
        Force = self.linear_speed_controller.evaluate(self.delta_t, v_target, self.cart.v)
        Torque = self.angular_speed_controller.evaluate(self.delta_t, w_target, self.cart.w)
        self.cart.evaluate(self.delta_t, Force, Torque)

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
            self.plotter.plot(['x', 'X'], [['y', 'Y']])
            self.plotter.show()
            return False

        return True

    def get_pose(self):
        return self.cart.get_pose()

    def get_speed(self):
        return self.cart.v, self.cart.w


if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot)
    sys.exit(app.exec_())
