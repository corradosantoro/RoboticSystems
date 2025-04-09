#
# polar.py
#

import math
from lib.system.controllers import *
from lib.utils.geometry import *

class Polar2DController:

    def __init__(self, KP_linear, v_max, KP_heading, w_max):
        self.linear = PID_Controller(KP_linear, 0, 0, v_max)
        self.angular = PID_Controller(KP_heading, 0, 0, w_max)

    def evaluate(self, delta_t, xt, yt, current_pose):
        (x, y, theta) = current_pose

        dx = xt - x
        dy = yt - y

        target_heading = math.atan2(dy , dx)

        distance = math.sqrt(dx*dx + dy*dy)
        heading_error = normalize_angle(target_heading - theta)

        if (heading_error > math.pi/2)or(heading_error < -math.pi/2):
            distance = -distance
            heading_error = normalize_angle(heading_error + math.pi)

        v_target = self.linear.evaluate(delta_t, distance)
        w_target = self.angular.evaluate(delta_t, heading_error)

        return (v_target, w_target)

