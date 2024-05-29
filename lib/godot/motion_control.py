import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.godot.interface import *
from lib.controllers.standard import PIDSat
from lib.controllers.control2d import Polar2DController, StraightLine2DMotion
from lib.data.geometry import *

import os
import threading
import time

import abc

class MotionCommand(metaclass=abc.ABCMeta):

    @abc.abstractmethod
    def start(self, target, robot):
        pass

    @abc.abstractmethod
    def execute(self, delta_t, x, y, theta, v, w):
        pass

    @abc.abstractmethod
    def target(self):
        pass


class GotoPoint(MotionCommand):

    def __init__(self):
        # KP_linear, v_max, KP_heading, w_max
        self.polar_controller = Polar2DController(2.5, 2, 2.0, 2)
        # vmax, acc, dec
        self.trajectory = StraightLine2DMotion(0.2, 0.5, 0.5)

    def start(self, target, robot):
        self.__target_got = False
        (self.x, self.y) = target
        self.robot = robot
        (xr, yr, _) = self.robot.get_pose()
        self.trajectory.start_motion((xr, yr), (self.x, self.y))
        self.robot.set_controller(self)

    def execute(self, delta_t, x, y, theta, v, w):
        dx = x - self.x
        dy = y - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        if (dist < 0.05):
            self.__target_got = True
            return (0.0, 0.0)
        else:
            (x_target, y_target) = self.trajectory.evaluate(delta_t)
            (v_target, w_target) = self.polar_controller.evaluate(delta_t, x_target, y_target, (x, y, theta))
            return (v_target, w_target)

    def target(self):
        return self.__target_got


class RotateTo(MotionCommand):

    def __init__(self):
        # KP_linear, v_max, KP_heading, w_max
        self.rotation_controller = PIDSat(2.5, 0, 0.0, 2)  # 2 rad/sec max

    def start(self, target, robot):
        self.__target_got = False
        self.angle = math.radians(target)
        self.robot = robot
        self.robot.set_controller(self)

    def execute(self, delta_t, x, y, theta, v, w):
        err = normalize_angle(self.angle - theta)
        if (abs(err) < math.radians(2)):
            self.__target_got = True
            return (0.0, 0.0)
        else:
            w_target = self.rotation_controller.evaluate_error(delta_t, err)
            return (0, w_target)

    def target(self):
        return self.__target_got


class HeadingTo(RotateTo):

    def start(self, target, robot):
        self.__target_got = False
        (x, y) = target
        (xr, yr, _) = robot.get_pose()
        angle = math.atan2(y - yr, x - xr)
        super().start(math.degrees(angle), robot)


class Cart2DRobot(threading.Thread):

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None):
        super(Cart2DRobot,self).__init__(group=group, target=target,
                                    name=name)

        self.daemon = True
        self.mutex = threading.Lock()

        self.t = 0
        self.cart = GodotCartTwoWheels()

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
        self.controller = None


    def set_controller(self, c):
        self.controller = c
        self.active = True

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

                # execute control
                (v, w) = self.controller.execute(self.delta_t, self.x, self.y, self.theta, self.v, self.w)
                vl = v - w * self.wheel_base / 2
                vr = v + w * self.wheel_base / 2

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
        (command, target) = self.path[self.path_index]
        command.start(target, self.robot)
        self.end_of_path = False

    def execute(self):
        if self.end_of_path:
            self.robot.stop()
            return
        (command, _) = self.path[self.path_index]
        if command.target():
            self.path_index += 1
            if (self.path_index < len(self.path)):
                (command, target) = self.path[self.path_index]
                command.start(target, self.robot)
            else:
                self.end_of_path = True



if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    cart_robot.start()

    goto = GotoPoint()
    rotate = RotateTo()
    heading = HeadingTo()

    path = Path()
    path.set_path( [ (rotate, 45),
                     (goto, (1.5, 1.5)),

                     (heading, (1.5, -1.5)),
                     (goto, (1.5, -1.5)),

                     (heading, (-1.5, -1.5)),
                     (goto, (-1.5, -1.5)),

                     (heading, (-1.5, 1.5)),
                     (goto, (-1.5, 1.5)),

                     (heading, (0,0)),
                     (goto, (0,0)) ] )

    path.start(cart_robot)

    while True:
        time.sleep(0.5)
        path.execute()


