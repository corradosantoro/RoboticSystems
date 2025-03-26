#
# arm.py
#

import math

GRAVITY = 9.81

class Arm:
    def __init__(self, _mass: float, _len : float, _friction: float):
        """
        Defines an arm in a 2D environment with the given mass, the lengthm and the friction
        :param _mass: The mass of the arm, expressed in Kg
        :param _length: The length of the arm, expressed in meters
        :param _friction: The air friction coefficient
        """
        self.M: float = _mass
        self.B: float = _friction
        self.L: float = _len
        self.omega: float = 0
        self.theta: float = 0

    def evaluate(self, delta_t: float, _torque: float) -> tuple:
        """
        Evaluates the angular speed and position at the given time with the applied torque
        :param delta_t: The delta time
        :param _torque: The applied torque
        """
        new_omega: float = (1 - (3./2.) * (delta_t / (self.M * self.L) ) * self.B) * self.omega \
                                - (3./2.) * (delta_t / self.L ) * GRAVITY * math.cos(self.theta) \
                                + ((3 * delta_t) / (self.M * self.L * self.L)) * _torque
        new_theta: float = self.theta + self.omega * delta_t
        self.omega = new_omega
        self.theta = new_theta
        return (self.theta, self.omega)

if __name__ == "__main__":
    c = Arm(1.0, 1.0, 0.9)
    f = 3
    delta_t = 1e-3
    while True:
        (p, v) = c.evaluate(delta_t, f)
        print(p)

