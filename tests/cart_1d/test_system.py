#
# test_cart_plot.py
#

import sys
from pathlib import Path
from matplotlib import pylab

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")


class System:

    def __init__(self):
        self.x1 = 0.0
        self.x2 = 0.0

    def evaluate(self, delta_t, _input):
        new_x1 = self.x1 + self.x2 * delta_t
        new_x2 = -4 * delta_t * self.x1 + (1 - 3 * delta_t) * self.x2 + 5 * delta_t * _input
        self.x1 = new_x1
        self.x2 = new_x2
        return self.x1


t = 0  # beginning of events
delta_t = 0.2  # sampling interval

_input = 1

my_sys = System()

time_array = []
output_array = []

while t < 10:  # let's simulate 10 seconds
    time_array.append(t)
    output_array.append(my_sys.x1)
    my_sys.evaluate(delta_t, _input)
    t = t + delta_t

pylab.figure(1)
pylab.plot(time_array, output_array, 'r-+', label='x1(t)')
pylab.xlabel('time')
pylab.legend()

pylab.show()
