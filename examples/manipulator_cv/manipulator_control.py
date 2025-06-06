#
# manipulator_control.py
#


from lib.system.controllers import *
from lib.system.manipulator import *

class FourJointsManipulatorControl:

    def __init__(self):
        self.arm = FourJointsArm(0.6, 0.58, 0.056,
                                0.5, 0.5, 0.5,
                                0.8)

        # joint 0
        self.speed_control_0 = PID_Controller(20, 5, 0,
                                              20)  # 20Nm max torque, antiwindup

        # joint 1
        self.speed_control_1 = PID_Controller(20, 5, 0,
                                              20)  # 20Nm max torque, antiwindup

        # joint 2
        self.speed_control_2 = PID_Controller(20, 5, 0,
                                              20)  # 20Nm max torque, antiwindup

        # joint 3
        self.speed_control_3 = PID_Controller(1, 0.4, 0,
                                              20)  # 20Nm max torque, antiwindup

        self.pos_control_0 = PID_Controller(3, 0, 0, 2)  # 2 rad/s max speed
        self.pos_control_1 = PID_Controller(3, 0, 0, 2)  # 2 rad/s max speed
        self.pos_control_2 = PID_Controller(3, 0, 0, 2)  # 2 rad/s max speed
        self.pos_control_3 = PID_Controller(3, 0, 0, 2)  # 2 rad/s max speed

        (x, y, z, a) = self.get_pose()
        self.set_target(x, y, z, a)

    def set_target(self, x, y, z, a):
        (theta0, theta1, theta2, theta3) = self.arm.inverse_kinematics(x, y, z, a)
        if theta0 is None:
            return False
        (self.theta0, self.theta1, self.theta2, self.theta3) = (theta0, theta1, theta2, theta3)
        return True

    def evaluate(self, delta_t):

        wref_0 = self.pos_control_0.evaluate(delta_t, self.theta0 - self.arm.element_0.theta)
        wref_1 = self.pos_control_1.evaluate(delta_t, self.theta1 - self.arm.element_1.theta)
        wref_2 = self.pos_control_2.evaluate(delta_t, self.theta2 - self.arm.element_2.theta)
        wref_3 = self.pos_control_3.evaluate(delta_t, self.theta3 - self.arm.element_3.theta)

        torque0 = self.speed_control_0.evaluate(delta_t, wref_0 - self.arm.element_0.w)
        torque1 = self.speed_control_1.evaluate(delta_t, wref_1 - self.arm.element_1.w)
        torque2 = self.speed_control_2.evaluate(delta_t, wref_2 - self.arm.element_2.w)
        torque3 = self.speed_control_3.evaluate(delta_t, wref_3 - self.arm.element_3.w)

        self.arm.evaluate(delta_t, torque0, torque1, torque2, torque3)

    def get_joint_angles(self):
        return self.arm.get_joint_angles()

    def get_pose(self):
        return self.arm.get_pose()

