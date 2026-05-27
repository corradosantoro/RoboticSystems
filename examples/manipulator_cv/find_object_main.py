#
# find_object_main.py
#

import sys
sys.path.insert(0, "../..")

import cv2
import numpy as np

from lib.dds.dds import *
from lib.dds.image_reader import *
from lib.utils.time import *

from object_finder import *
from manipulator_control import *


robot = FourJointsManipulatorControl()

x_target_robot = 0.8
y_target_robot = 0.0
z_target_robot = 0.5
robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))

dds = DDS()
dds.start()

dds.subscribe(['tick'])

imr = ImageReader('localhost', 4445)
imr.connect()

blue_cube = ( [200, 0, 0], [255, 30, 30] ) # BGR
red_cube = ( [0, 0, 200], [30, 30, 255] ) # BGR

obj_finder = ObjectFinder(red_cube)

t = Time()
t.start()
while True:
    dds.publish('read_image', 1, DDS.DDS_TYPE_INT)
    dds.wait('tick')

    delta_t = t.elapsed()

    # robot control
    robot.evaluate(delta_t)
    (t0, t1, t2, t3) = robot.get_joint_angles()
    (x, y, z, a) = robot.get_pose()

    dds.publish('theta0', t0, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta1', t1, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta2', t2, DDS.DDS_TYPE_FLOAT)
    dds.publish('theta3', t3, DDS.DDS_TYPE_FLOAT)
    dds.publish('x', x, DDS.DDS_TYPE_FLOAT)
    dds.publish('y', y, DDS.DDS_TYPE_FLOAT)
    dds.publish('z', z, DDS.DDS_TYPE_FLOAT)
    dds.publish('a', a, DDS.DDS_TYPE_FLOAT)

    ## image processing
    img = imr.read_image(512, 512)
    cx, cy, binary_image = obj_finder.find(img)

    # image show
    cv2.imshow('image', img)
    cv2.imshow('binary_image', binary_image)

    k = cv2.waitKeyEx(1)
    # if k != -1:
    #     print(hex(k))
    if (k & 0xff) == ord('q'):
        break
    elif k == 0xff53: # right
        x_target_robot += 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))
    elif k == 0xff51: # left
        x_target_robot -= 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))
    elif k == 0xff54: # down
        y_target_robot -= 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))
    elif k == 0xff52: # up
        y_target_robot += 0.01
        robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))

dds.stop()
