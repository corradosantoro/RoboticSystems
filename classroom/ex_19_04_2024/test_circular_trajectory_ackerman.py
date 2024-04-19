import sys
import math

from pathlib import Path
from PyQt5.QtWidgets import QApplication

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import AckermannSteering
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.gui.gui_2d import CartWindow
from lib.data.plot import DataPlotter

from circular_trajectory import *

class AckermannRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 10kg
        # side = 15cm
        # wheels radius = 2cm
        # friction = 0.8
        self.car = AckermannSteering(10, 0.8, 0.02, 0.15)
        # 5 Nm max, antiwindup
        self.speed_controller = PIDSat(2.0, 2.0, 0, 5, True)
        self.polar_controller = Polar2DController(2.0, 1.5,  # kp = 2, vmax = 1.5 m/s
                                                  10.0, math.pi / 4)  # kp = 1, steering max = 45 deg

        self.trajectory = CircularTrajectory(0.2, 0.5, 0.5,
        					0.2, math.radians(90))  #  R = 0.3, angolo = 90 gradi
        self.trajectory.start(self.get_pose())
        self.plotter = DataPlotter()

    def run(self):
        (x_target, y_target) = self.trajectory.evaluate(self.delta_t)
        (vref, steering) = self.polar_controller.evaluate(self.delta_t,
                                                          x_target, y_target,
                                                          self.get_pose())
        (v, w) = self.get_speed()

        torque = self.speed_controller.evaluate(self.delta_t, vref, v)

        self.car.evaluate(self.delta_t, torque, steering)

        (x,y,_) = self.get_pose()
        self.plotter.add('t', self.t)
        self.plotter.add('x', x)
        self.plotter.add('y', y)
        self.plotter.add('x_target', x_target)
        self.plotter.add('y_target', y_target)

        if self.t > 10:
            self.plotter.plot ( [ 'x', 'X' ],
                                [ [ 'y', 'Y'] ])
            self.plotter.show()
            return False

        return True

    def get_pose(self):
        return self.car.get_pose()

    def get_speed(self):
        return self.car.v, self.car.w


if __name__ == '__main__':
    cart_robot = AckermannRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot, 'ackermann_robot_2d.png')
    sys.exit(app.exec_())
