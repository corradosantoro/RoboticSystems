
import sys
sys.path.insert(0, "../..")

import cv2
import numpy as np

from lib.dds.dds import *
from lib.dds.image_reader import *
from lib.utils.time import *
from lib.system.controllers import *
from manipulator_control import *


displ_x_controller = PID_Controller(0.0005, 0, 0, 0.1)
displ_y_controller = PID_Controller(0.0005, 0, 0, 0.1)

robot = FourJointsManipulatorControl()

x_target_robot = 0.15
y_target_robot = 0.8
z_target_robot = 0.4
robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))

dds = DDS()
dds.start()

dds.subscribe(['tick'])

t = Time()
t.start()

imr = ImageReader('localhost', 4445)
imr.connect()
while True:
    dds.publish('read_image', 1, DDS.DDS_TYPE_INT)
    dds.wait('tick')

    delta_t = t.elapsed()

    ## image processing
    img = imr.read_image(512, 512)

    #lower_limit = np.array([200, 0, 0]) # B G R
    #higher_limit = np.array([255, 30, 30]) # B G R

    lower_limit = np.array([0, 0, 200]) # B G R
    higher_limit = np.array([30, 30, 255]) # B G R

    binary_image = cv2.inRange(img, lower_limit, higher_limit)
    contours, hierarchy = cv2.findContours(binary_image,
                                            cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_NONE)

    if len(contours) > 0:
        cv2.drawContours(img, contours, -1,  (255, 0, 0))
        cnt = contours[0]
        M = cv2.moments(cnt)
        if M['m00'] != 0:
            # get the center of the contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            cv2.circle(img, (cx, cy), 3, (0,0,0), -1)
            errx = (256 - cx)
            erry = (256 - cy)

            displ_x = displ_x_controller.evaluate(delta_t, errx)
            displ_y = displ_y_controller.evaluate(delta_t, erry)

            print('Error vs. image center ', (errx, erry), ' - Displacement ', (displ_x, displ_y))

            if (abs(errx) < 5)and(abs(erry) < 5):
                z_target_robot -= 0.01
                if z_target_robot < 0.02:
                    print("Object got")
                    break

            (x, y, z, a) = robot.get_pose()
            x_target_robot = x + displ_x
            y_target_robot = y + displ_y

            print((x,y), " - " , (x_target_robot, y_target_robot))
            if not(robot.set_target(x_target_robot, y_target_robot, z_target_robot, math.radians(-90))):
                x_target_robot -= displ_x
                y_target_robot -= displ_y


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

    # image show
    cv2.imshow('image', img)
    #cv2.imshow('binary_image', binary_image)

    k = cv2.waitKey(1)
    if k == ord('q'):
        break

dds.stop()
