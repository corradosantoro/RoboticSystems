import math
from lib.utils.geometry import *

GRAVITY = 9.81


class ArmElementNoGravity:

    def __init__(self, _L, _M, _b):
        self.w = 0
        self.theta = 0
        self.L = _L
        self.M = _M
        self.b = _b

    def evaluate(self, delta_t, _input_torque):
        w = self.w - \
            (self.b * delta_t * self.w * self.L) / self.M + \
            delta_t * _input_torque / (self.M * self.L)
        self.theta = self.theta + delta_t * self.w
        self.w = w

    def get_pose(self):
        return self.L * math.cos(self.theta), self.L * math.sin(self.theta)


class ArmElement:

    def __init__(self, _L, _M, _b):
        self.w = 0
        self.theta = 0
        self.L = _L
        self.M = _M
        self.b = _b

    def evaluate(self, delta_t, _input_torque):
        w = self.w - GRAVITY * delta_t * math.cos(self.theta) - \
            (self.b * delta_t * self.w * self.L) / self.M + \
            delta_t * _input_torque / (self.M * self.L)
        self.theta = self.theta + delta_t * self.w
        self.w = w

    def get_pose(self):
        return self.L * math.cos(self.theta), self.L * math.sin(self.theta)


# --------------------------------------------------------------------------------

class ThreeJointsPlanarArm:

    def __init__(self, _L1, _L2, _L3, _M2, _M3, _Mend, _b):
        self.element_1 = ArmElement(_L1, _M2 + _M3 + _Mend, _b)
        self.element_2 = ArmElement(_L2, _M3 + _Mend, _b)
        self.element_3 = ArmElement(_L3, _Mend, _b)

    def evaluate(self, delta_t, _T1, _T2, _T3):
        self.element_1.evaluate(delta_t, _T1)
        self.element_2.evaluate(delta_t, _T2)
        self.element_3.evaluate(delta_t, _T3)

    def get_joint_angles(self):
        return (self.element_1.theta,
                self.element_2.theta,
                self.element_3.theta)

    def get_joint_positions(self):
        (x1, y1) = self.element_1.get_pose()

        (_x2, _y2) = self.element_2.get_pose()
        (x2, y2) = local_to_global(x1, y1, self.element_1.theta, _x2, _y2)

        alpha = self.element_1.theta + self.element_2.theta + self.element_3.theta
        (x3, y3) = local_to_global(x2, y2, alpha, self.element_3.L, 0)

        return [(x1, y1), (x2, y2), (x3, y3)]

    def get_pose(self):
        x_t = self.element_1.L * math.cos(self.element_1.theta) + \
          self.element_2.L * math.cos(self.element_1.theta + self.element_2.theta) + \
          self.element_3.L * math.cos(self.element_1.theta + self.element_2.theta + self.element_3.theta)
        y_t = self.element_1.L * math.sin(self.element_1.theta) + \
          self.element_2.L * math.sin(self.element_1.theta + self.element_2.theta) + \
          self.element_3.L * math.sin(self.element_1.theta + self.element_2.theta + self.element_3.theta)
        alpha = normalize_angle(self.element_1.theta + self.element_2.theta + self.element_3.theta)
        return x_t, y_t, alpha

    def inverse_kinematics(self, xt, yt, alpha):
        x2 = xt - self.element_3.L * math.cos(alpha)
        y2 = yt - self.element_3.L * math.sin(alpha)
        acos_arg = (x2 ** 2 + y2 ** 2 - self.element_1.L ** 2 - self.element_2.L ** 2) / (
                    2 * self.element_1.L * self.element_2.L)
        if (acos_arg < -1)or(acos_arg > 1):
            return None, None, None
        theta2 = - math.acos(acos_arg) # elbow up
        theta1 = math.atan2(y2, x2) - math.atan2(self.element_2.L * math.sin(theta2),
                                                 self.element_1.L + self.element_2.L * math.cos(theta2))
        theta3 = alpha - theta1 - theta2

        return theta1, theta2, theta3



# --------------------------------------------------------------------------------

class FourJointsArm(ThreeJointsPlanarArm):

    def __init__(self, _L1, _L2, _L3, _M2, _M3, _Mend, _b):
        super().__init__(_L1, _L2, _L3, _M2, _M3, _Mend, _b)
        self.element_0 = ArmElementNoGravity(_L1, _M2 + _M3 + _Mend, _b)

    def evaluate(self, delta_t, _T0, _T1, _T2, _T3):
        self.element_0.evaluate(delta_t, _T0)
        super().evaluate(delta_t, _T1, _T2, _T3)

    def inverse_kinematics(self, xt, yt, zt, alpha):
        x_prime = math.sqrt(xt**2 + yt**2)
        theta0 = math.atan2(yt, xt)
        (theta1, theta2, theta3) = super().inverse_kinematics(x_prime, zt, alpha)
        if theta1 is None:
            return None, None, None, None
        else:
            return theta0, theta1, theta2, theta3

    def get_joint_angles(self):
        (theta1, theta2, theta3) = super().get_joint_angles()
        theta0 = self.element_0.theta
        return theta0, theta1, theta2, theta3

    def get_pose(self):
        (x_prime, y_prime, alpha) = super().get_pose()
        x = x_prime * math.cos(self.element_0.theta)
        y = x_prime * math.sin(self.element_0.theta)
        z = y_prime
        return (x, y, z, alpha)

