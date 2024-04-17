import sys

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart2d import Cart2D
from lib.data.plot import DataPlotter
from lib.controllers.standard import PIDSat

# Mass = 1kg
# radius = 15cm
# friction = 0.8
cart = Cart2D(1, 0.15, 0.8, 0.8)

plotter = DataPlotter()

linear_speed_controller = PIDSat(10, 3.5, 0, 5)  # 5 newton
angular_speed_controller = PIDSat(6, 10, 0, 4)  # 4 newton * metro

v_target = 6
w_target = 0

t = 0
delta_t = 1e-3

while t < 4:
    force = linear_speed_controller.evaluate(delta_t, v_target, cart.v)
    torque = angular_speed_controller.evaluate(delta_t, w_target, cart.w)

    plotter.add('t', t)

    plotter.add('v', cart.v)
    plotter.add('v_target', v_target)

    plotter.add('w', cart.w)
    plotter.add('w_target', w_target)

    cart.evaluate(delta_t, force, torque)
    t = t + delta_t

plotter.plot(['t', 'Time'],
             [['v', 'speed'],
              ['v_target', 'target speed']])
plotter.plot(['t', 'Time'],
             [['w', 'omega'],
              ['w_target', 'target omega']])
plotter.show()
