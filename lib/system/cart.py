#
# cart.py
#

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

if __name__ == "__main__":
    c = Cart(1.0, 0.9)
    f = 1000
    delta_t = 1e-3
    while True:
        (p, v) = c.evaluate(delta_t, f)
        f = 0
        print(v)

