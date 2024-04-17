import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import TwoWheelsCart2D
from lib.models.robot import RoboticSystem
from lib.gui.gui_2d import CartWindow

from PyQt5.QtWidgets import QApplication


class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        # Mass = 1kg
        # radius = 15cm
        # friction = 0.8
        # distance between traction wheels = 25cm
        self.cart = TwoWheelsCart2D(1, 0.15, 0.8, 0.8, 0.25)

    def run(self):
        self.cart.evaluate(self.delta_t, 0.2, 0.2)
        return True

    def get_pose(self):
        return self.cart.x, self.cart.y, self.cart.theta

    def get_speed(self):
        return self.cart.v, self.cart.w


if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    app = QApplication(sys.argv)
    ex = CartWindow(cart_robot)
    sys.exit(app.exec_())
