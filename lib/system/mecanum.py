#
# mecanum.py
#

import math
import numpy as np

class MecanumFourWheels:

    def __init__(self, _mass: float, _wheelbase: float,
                     _lin_friction: float, _ang_friction: float, _wheelradius : float):
        self.M = _mass
        self.b = _lin_friction
        self.beta = _ang_friction
        self.wheelbase = _wheelbase
        self.wheelradius = _wheelradius
        self.Iz = (self.M * 2 * self.wheelbase * self.wheelbase) / 12
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vx_local = 0
        self.vy_local = 0
        self.angular_v = 0
        self.inv_wheel_radius = 1 / self.wheelradius
        self.ik_matrix = self.inv_wheel_radius * np.array( [ [ 1, -1, -2 * self.wheelbase],
                                                             [ 1,  1, 2 * self.wheelbase],
                                                             [ 1,  1, -2 * self.wheelbase],
                                                             [ 1, -1, 2 * self.wheelbase] ])
        self.w_vector = np.zeros( (4,1) )

    def evaluate(self, delta_t: float, _t1 : float, _t2 : float, _t3 : float, _t4 : float) -> None:
        f_x = (_t1 + _t2 + _t3 + _t4) / self.wheelradius
        f_y = (- _t1 + _t2 + _t3 - _t4) / self.wheelradius
        vx_loc = self.vx_local * (1 - self.b * delta_t / self.M) + delta_t * f_x / self.M
        vy_loc = self.vy_local * (1 - self.b * delta_t / self.M) + delta_t * f_y / self.M

        T = (math.sqrt(2) * self.wheelbase) * (- _t1 + _t2 - _t3 + _t4) / self.wheelradius

        self.angular_v = self.angular_v * (1 - self.beta * delta_t / self.Iz) + delta_t * T / self.Iz

        vx = vx_loc * math.cos(self.theta) - vy_loc * math.sin(self.theta)
        vy = vx_loc * math.sin(self.theta) + vy_loc * math.cos(self.theta)

        v = np.array( [vx_loc, vy_loc, self.angular_v] )
        self.w_vector = self.ik_matrix @ v

        self.vx_local = vx_loc
        self.vy_local = vy_loc

        self.x = self.x + vx * delta_t
        self.y = self.y + vy * delta_t
        self.theta = self.theta + delta_t * self.angular_v

    def get_pose(self) -> (float, float, float):
        return self.x, self.y, self.theta

    def set_pose(self, x, y, t):
        (self.x, self.y, self.theta) = (x, y, t)

    def get_wheel_speed(self):
        return self.w_vector.flatten().tolist()

