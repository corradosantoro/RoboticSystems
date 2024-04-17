import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.godot.interface import *
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.data.plot import DataPlotter


class Cart2DRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        self.cart = GodotCartTwoWheels()

        self.polar_controller = Polar2DController(2.5, 2, 2.0, 2)
        self.trajectory = StraightLine2DMotion(0.2, 0.5, 0.5)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0

        self.wheel_base = 0.32
        self.wheel_radius = 0.051

        (x, y, _) = self.get_pose()
        self.trajectory.start_motion((x, y), (1.5, 0.5))
        self.plotter = DataPlotter()

    def run(self):
        (x_target, y_target) = self.trajectory.evaluate(self.delta_t)
        (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, self.get_pose())
        vl = v_target - w_target * self.wheel_base / 2
        vr = v_target + w_target * self.wheel_base / 2

        # convert speeds from m/s to deg/sec
        vl = vl / self.wheel_radius
        vr = vr / self.wheel_radius

        (self.delta_t, self.x, self.y, self.theta, self.v, self.w) = self.cart.process(vl, vr)
        self.t += self.delta_t
        print(self.get_pose_deg())
        
        (x, y, _) = self.get_pose()
        self.plotter.add('t', self.t)
        self.plotter.add('x', x)
        self.plotter.add('y', y)
        self.plotter.add('x_target', x_target)
        self.plotter.add('y_target', y_target)
        
        #if self.t > 10:
        #    self.plotter.plot(['t', 'time'],
        #                      [['x', 'X'], ['x_target', 'X Target']])
        #    self.plotter.plot(['t', 'time'],
        #                      [['y', 'Y'], ['y_target', 'Y Target']])
        #    self.plotter.plot(['x', 'X'], [['y', 'Y']])
        #    self.plotter.show()
        #    return False

        return True

    def get_pose_deg(self):
        return (self.x, self.y, math.degrees(self.theta))

    def get_pose(self):
        return (self.x, self.y, self.theta)

if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    while cart_robot.run():
        pass
