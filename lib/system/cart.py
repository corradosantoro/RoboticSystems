#
# cart.py
#
import math

from lib.system.basic import *
from lib.utils.geometry import *

class Cart:
    def __init__(self, _mass: float, _friction: float):
        """
        Defines a cart robot of a 1D environment with the given mass and friction
        :param _mass: The mass of the cart, expressed in Kg
        :param _friction: The force of friction present in the system
        """
        self.M: float = _mass
        self.B: float = _friction
        self.speed: float = 0
        self.position: float = 0

    def evaluate(self, delta_t: float, _force: float) -> tuple:
        """
        Evaluates the linear speed and position at the given time with the applied force
        :param delta_t: The delta time
        :param _force: The applied force
        """
        new_speed: float = (1 - self.B * delta_t / self.M) * self.speed + delta_t * _force / self.M
        new_position: float = self.position + self.speed * delta_t
        self.speed = new_speed
        self.position = new_position
        return (self.position, self.speed)


# --------------------------------------------------------------------------

class Cart2D:

    def __init__(self, _mass: float, _radius: float, _lin_friction: float, _ang_friction: float):
        """
        Defines a cylinder robot with the given mass, radius, linear and angular frictions
        :param _mass: The mass of the cylinder robot, expressed in Kg
        :param _radius: The radius of the cylinder, expressed in meter
        :param _lin_friction: The force of linear friction present in the system
        :param _ang_friction: The force of angular friction present in the system
        """
        self.M: float = _mass
        self.b: float = _lin_friction
        self.beta: float = _ang_friction
        self.Iz: float = 0.5 * _mass * _radius * _radius
        # Iz = moment of inertia (the robot is a cylinder)
        self.v: float = 0
        self.w: float = 0
        self.x: float = 0
        self.y: float = 0
        self.theta: float = 0

    def evaluate(self, delta_t: float, _force, _torque: float) -> None:
        """
        Evaluates the linear speed and angular speed at the given time with the applied force and torque
        :param delta_t: The delta time
        :param _force: The applied force
        :param _torque: The applied torque
        """
        new_v: float = self.v * (1 - self.b * delta_t / self.M) + delta_t * _force / self.M
        new_w: float = self.w * (1 - self.beta * delta_t / self.Iz) + delta_t * _torque / self.Iz
        self.x = self.x + self.v * delta_t * math.cos(self.theta)
        self.y = self.y + self.v * delta_t * math.sin(self.theta)
        self.theta = self.theta + delta_t * self.w
        self.v = new_v
        self.w = new_w

    def get_pose(self) -> (float, float, float):
        """
        Returns the current robot's position
        :return: A tuple containing X, Y coordinate and Theta angle
        """
        return self.x, self.y, self.theta

    def get_speed(self) -> (float, float):
        """
        Returns the current robot's speed
        :return: A tuple containing linear and angular speed
        """
        return self.v, self.w


# --------------------------------------------------------------------------

class TwoWheelsCart2D(Cart2D):

    def __init__(self, _mass: float, _radius: float, _lin_friction: float, _ang_friction: float, _traction_wheelbase: float):
        """
        Defines a cylinder robot with the given mass, radius, linear & angular frictions and distance between traction wheels
        :param _mass: The mass of the cylinder robot, expressed in Kg
        :param _radius: The radius of the cylinder, expressed in meter
        :param _lin_friction: The force of linear friction present in the system
        :param _ang_friction: The force of angular friction present in the system
        :param _traction_wheelbase: The distance between the traction wheels, expressed in meter
        """
        super().__init__(_mass, _radius, _lin_friction, _ang_friction)
        self.traction_wheelbase: float = _traction_wheelbase

    def evaluate(self, delta_t: float, f_left: float, f_right: float) -> None:
        """
        Evaluates the linear speed and angular speed at the given time with both the forces applied on left and on right
        :param delta_t: The delta time
        :param f_left: The applied force on left
        :param f_right: The applied torque on right
        """
        f = f_left + f_right
        t = self.traction_wheelbase * (f_right - f_left)
        super().evaluate(delta_t, f, t)


# --------------------------------------------------------------------------

class TwoWheelsCart2DEncoders(TwoWheelsCart2D):

    def __init__(self, _mass, _radius, _lin_friction, _ang_friction,
                 _r_traction_left, _r_traction_right, _traction_wheelbase,
                 _r_encoder_left, _r_encoder_right, _encoder_wheelbase, _encoder_ticks):

        super().__init__(_mass, _radius, _lin_friction, _ang_friction, _traction_wheelbase)

        self.r_traction_left = _r_traction_left
        self.r_traction_right = _r_traction_right

        self.r_encoder_left = _r_encoder_left
        self.r_encoder_right = _r_encoder_right

        self.encoder_wheelbase = _encoder_wheelbase
        self.encoder_resolution = 2 * math.pi / _encoder_ticks


    def evaluate(self, delta_t, torque_left, torque_right) -> None:
        # torque to force
        f_left = torque_left / self.r_traction_left
        f_right = torque_right / self.r_traction_right

        # dynamic model
        super().evaluate(delta_t, f_left, f_right)

        # sensing wheels
        vl = self.v - self.w * self.encoder_wheelbase / 2
        vr = self.v + self.w * self.encoder_wheelbase / 2

        self.delta_rot_left = int(
            (vl / self.r_encoder_left) * (delta_t / self.encoder_resolution)) * self.encoder_resolution
        self.delta_rot_right = int(
            (vr / self.r_encoder_right) * (delta_t / self.encoder_resolution)) * self.encoder_resolution


# --------------------------------------------------------------------------


class TwoWheelsCart2DEncodersOdometry(TwoWheelsCart2DEncoders):

    def __init__(self, _mass: float, _radius: float, _lin_friction: float, _ang_friction: float,
                 _r_traction_left: float, _r_traction_right: float, _traction_wheelbase: float,
                 _r_encoder_left: float, _r_encoder_right: float, _encoder_wheelbase: float, _encoder_ticks: float):
        super().__init__(_mass, _radius, _lin_friction, _ang_friction,
                         _r_traction_left, _r_traction_right, _traction_wheelbase,
                         _r_encoder_left, _r_encoder_right, _encoder_wheelbase, _encoder_ticks)

        self.x_r: float = 0
        self.y_r: float = 0
        self.theta_r: float = 0

        self.vleft: float = 0
        self.vright: float = 0

    def evaluate(self, delta_t, torque_left, torque_right):
        # dynamic model
        super().evaluate(delta_t, torque_left, torque_right)

        # odometry model
        p_left: float = self.delta_rot_left * self.r_encoder_left
        p_right: float = self.delta_rot_right * self.r_encoder_right

        self.vleft = p_left / delta_t
        self.vright = p_right / delta_t

        self.v_r = (self.vleft + self.vright) / 2
        self.w_r = (self.vright - self.vleft) / self.encoder_wheelbase

        delta_p: float = (p_left + p_right) / 2

        delta_theta: float = (p_right - p_left) / self.encoder_wheelbase

        self.x_r = self.x_r + delta_p * math.cos(self.theta_r + delta_theta / 2)
        self.y_r = self.y_r + delta_p * math.sin(self.theta_r + delta_theta / 2)
        self.theta_r = normalize_angle(self.theta_r + delta_theta)

    def get_pose(self) -> (float, float, float):
        """
        Returns the current robot's position
        :return: A tuple containing X, Y coordinate and Theta angle
        """
        return self.x_r, self.y_r, self.theta_r

    def get_speed(self) -> (float, float):
        """
        Returns the current linear and angular speeds
        :return: A tuple containing V and W
        """
        return self.v_r, self.w_r

    def get_wheel_speed(self) -> (float, float):
        """
        Returns the current linear and angular speeds
        :return: A tuple containing V and W
        """
        return self.vleft, self.vright


# --------------------------------------------------------------------------

class AckermannSteering:

    def __init__(self, _mass: float, _lin_friction: float, _r_traction: float, _lateral_wheelbase: float):
        """
        Defines a vehicle ackermann steering robot with the given mass, linear friction, distance between traction and distance between wheels
        :param _mass: The mass of the cylinder robot, expressed in Kg
        :param _lin_friction: The force of linear friction present in the system
        :param _r_traction: The distance between the traction wheels, expressed in meter
        :param _lateral_wheelbase: The distance between the lateral wheels, expressed in meter
        """
        self.M: float = _mass
        self.b: float = _lin_friction
        self.r_wheels: float = _r_traction
        self.l_wb: float = _lateral_wheelbase

        self.v: float = 0
        self.w: float = 0
        self.x: float = 0
        self.y: float = 0
        self.theta: float = 0

    def evaluate(self, delta_t, torque, steering_angle):
        _force = torque / self.r_wheels
        new_v = self.v * (1 - self.b * delta_t / self.M) + delta_t * _force / self.M

        if steering_angle == 0:
            new_w = 0
        else:
            curvature_radius = self.l_wb / math.tan(steering_angle)
            new_w = new_v / curvature_radius

        self.x = self.x + self.v * delta_t * math.cos(self.theta)
        self.y = self.y + self.v * delta_t * math.sin(self.theta)
        self.theta = self.theta + delta_t * self.w
        self.v = new_v
        self.w = new_w

    def get_pose(self) -> (float, float, float):
        """
        Returns the current robot's position
        :return: A tuple containing X, Y coordinate and Theta angle
        """
        return self.x, self.y, self.theta

    def get_speed(self) -> (float, float):
        """
        Returns the current linear and angular speeds
        :return: A tuple containing V and W
        """
        return self.v, self.w


if __name__ == "__main__":
    c = Cart(1.0, 0.9)
    f = 1000
    delta_t = 1e-3
    while True:
        (p, v) = c.evaluate(delta_t, f)
        f = 0
        print(v)

