import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.godot.interface import *
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController
from lib.data.plot import DataPlotter


class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        self.cart = GodotCartTwoWheels()

        self.polar_controller = Polar2DController(0.7, 2, 4.0, 10)  # v = 2 m/s, w = 10 rad/s

        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0

        self.wheel_base = 0.32
        self.wheel_radius = 0.051

        self.target = (1.5, 1.0)


    def run(self):
        (x_target, y_target) = self.target
        (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())
        vl = v_target - w_target * self.wheel_base / 2
        vr = v_target + w_target * self.wheel_base / 2

        # convert speeds from m/s to deg/sec
        vl = vl / self.wheel_radius
        vr = vr / self.wheel_radius

        (self.delta_t, self.x, self.y, self.theta, self.v, self.w) = self.cart.process(vl, vr)
        self.t += self.delta_t
        print(self.get_pose_deg())

        return True

    def get_pose_deg(self):
        return (self.x, self.y, math.degrees(self.theta))

    def get_pose(self):
        return (self.x, self.y, self.theta)

if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    while cart_robot.run():
        pass
