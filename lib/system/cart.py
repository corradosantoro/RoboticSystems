#
# cart.py
#
import math

from lib.system.basic import *

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


if __name__ == "__main__":
    c = Cart(1.0, 0.9)
    f = 1000
    delta_t = 1e-3
    while True:
        (p, v) = c.evaluate(delta_t, f)
        f = 0
        print(v)

