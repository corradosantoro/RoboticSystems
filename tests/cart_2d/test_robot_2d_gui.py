import sys

from pathlib import Path
from PyQt5.QtWidgets import QApplication

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import Cart2D
from lib.models.robot import RoboticSystem
from lib.gui.gui_2d import CartWindow


class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 1kg
        # radius = 15cm
        # friction = 0.8
        self.cart = Cart2D(1, 0.15, 0.8, 0.8)

    def run(self):
        if self.t < 3:
            Force = 0.1  # 0.1 Newton
            Torque = 0  #
        elif self.t < 6:
            Force = 0.0  #
            Torque = 0.3  # 0.5 Nm
        else:
            Force = 0.1  # 0.2 Newton
            Torque = 0  #

        self.cart.evaluate(self.delta_t, Force, Torque)
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
