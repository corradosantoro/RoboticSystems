import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import TwoWheelsCart2DEncodersOdometry
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.data.plot import DataPlotter
from lib.gui.gui_2d import CartWindow

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
        self.plotter = DataPlotter()
        # 5 Nm of max torque, antiwindup
        self.left_controller = PIDSat(3.0, 1.0, 0.0, 5, True)
        self.right_controller = PIDSat(3.0, 1.0, 0.0, 5, True)
        self.vref_l = 2
        self.vref_r = 2

    def run(self):
        (vl, vr) = self.cart.get_wheel_speed()

        # same vref
        Tleft = self.left_controller.evaluate(self.delta_t, self.vref_l, vl)
        Tright = self.right_controller.evaluate(self.delta_t, self.vref_r, vr)

        self.cart.evaluate(self.delta_t, Tleft, Tright)

        self.plotter.add('t', self.t)
        self.plotter.add('vl', vl)
        self.plotter.add('vr', vr)
        self.plotter.add('vref_l', self.vref_l)
        self.plotter.add('vref_r', self.vref_r)
        if self.t > 2:
            self.plotter.plot(['t', 'time'], [['vref_l', 'Vref'],
                                              ['vl', 'VL']])
            self.plotter.plot(['t', 'time'], [['vref_r', 'Vref'],
                                              ['vr', 'VR']])
            self.plotter.show()
            return False
        else:
            return True

    def get_pose(self):
        return self.cart.get_pose()

    def get_speed(self):
        return self.cart.get_speed()


if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot)
    sys.exit(app.exec_())
