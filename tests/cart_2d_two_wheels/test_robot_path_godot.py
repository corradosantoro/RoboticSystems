import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.godot.interface import *
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.data.plot import DataPlotter

import os
import threading
import time


class Cart2DRobot(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None):
        super(Cart2DRobot,self).__init__(group=group, target=target,
                                    name=name)

        self.daemon = True
        self.mutex = threading.Lock()

        self.t = 0
        self.cart = GodotCartTwoWheels()

        # KP_linear, v_max, KP_heading, w_max
        self.polar_controller = Polar2DController(2.5, 2, 2.0, 2)

        # vmax, acc, dec
        self.trajectory = StraightLine2DMotion(0.2, 0.5, 0.5)

        self.x = 0
        self.y = 0
        self.theta = 0
        self.v = 0
        self.w = 0

        self.vl = 0
        self.vr = 0

        self.wheel_base = 0.32
        self.wheel_radius = 0.051

        self.active = False


    def goto(self, x, y):
        try:
            self.mutex.acquire()
            self.trajectory.start_motion((self.x, self.y), (x, y))
            self.active = True
        finally:
            self.mutex.release()

    def stop(self):
        self.vl = 0
        self.vr = 0
        self.active = False

    def run(self):
        # important task, set maximum priority
        param = os.sched_param(os.sched_get_priority_max(os.SCHED_FIFO))
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
        while True:
            time.sleep(0.001)
            self.__execute()

    def __execute(self):
        try:
            self.mutex.acquire()

            (self.delta_t, self.x, self.y, self.theta, self.v, self.w) = self.cart.process(self.vl, self.vr)

            if self.active:

                (x_target, y_target) = self.trajectory.evaluate(self.delta_t)
                (v_target, w_target) = self.polar_controller.evaluate(self.delta_t, x_target, y_target, (self.x, self.y, self.theta))
                vl = v_target - w_target * self.wheel_base / 2
                vr = v_target + w_target * self.wheel_base / 2

                # convert speeds from m/s to deg/sec
                self.vl = vl / self.wheel_radius
                self.vr = vr / self.wheel_radius

            self.t += self.delta_t

        finally:
            self.mutex.release()


    def get_pose_deg(self):
        try:
            self.mutex.acquire()
            return (self.x, self.y, math.degrees(self.theta))
        finally:
            self.mutex.release()


    def get_pose(self):
        try:
            self.mutex.acquire()
            return (self.x, self.y, self.theta)
        finally:
            self.mutex.release()



class Path:

    def __init__(self):
        pass

    def set_path(self, p):
        self.path = p

    def start(self, robot):
        self.path_index = 0
        self.robot = robot
        (xt,yt) = self.path[self.path_index]
        self.robot.goto(xt, yt)
        self.end_of_path = False

    def execute(self):
        if self.end_of_path:
            self.robot.stop()
            return
        (xt,yt) = self.path[self.path_index]
        (x, y, _) = self.robot.get_pose()
        dx = x - xt
        dy = y - yt
        dist = math.sqrt(dx*dx + dy*dy)
        print("X={:.3f}".format(x), "Y={:.3f}".format(y), "Dist={:.3f}".format(dist))
        if (dist < 0.05):
            self.path_index += 1
            if (self.path_index < len(self.path)):
                (xt,yt) = self.path[self.path_index]
                self.robot.goto(xt, yt)
            else:
                self.end_of_path = True


if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    cart_robot.start()

    path = Path()
    path.set_path( [ (1.5, 1.5), (1.5, -1.5), (-1.5, -1.5), (-1.5, 1.5), (0,0) ] )
    path.start(cart_robot)

    while True:
        time.sleep(0.5)
        path.execute()


