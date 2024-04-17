import sys
import math
from PyQt5.QtWidgets import QApplication
from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import AckermannSteering
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController
from lib.gui.gui_2d import CartWindow


class AckermannRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 10kg
        # side = 15cm
        # wheels radius = 2cm
        # friction = 0.8
        self.car = AckermannSteering(10, 0.8, 0.02, 0.15)
        # 5 Nm max, antiwindup
        self.speed_controller = PIDSat(8.0, 2.0, 0, 5, True)
        self.polar_controller = Polar2DController(1.0, 0.1, #kp = 1, vmax = 0.3 m/s
                                                  2.0, math.pi/4)  # kp = 2, steering max = 45 deg

    def run(self):
        (vref, steering) = self.polar_controller.evaluate(self.delta_t,
                                                          0.3, 0.3,
                                                          self.get_pose())
        (v, w) = self.get_speed()

        torque = self.speed_controller.evaluate(self.delta_t, vref, v)

        self.car.evaluate(self.delta_t, torque, steering)

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
