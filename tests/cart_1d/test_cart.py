#
#
#

import sys
sys.path.insert(0, '../../lib')

from models.cart import *

cart = Cart(1, 0.8) # mass 1 Kg, friction = 0.8

t = 0           # beginning of events
delta_t = 1e-3  # sampling interval = 1ms

_input = 3   # constant input of 3 N

while t < 5:   # let's simulate 5 seconds
    print("T = {:.3f}, V = {:.3f}, P = {:.3f}".format(t, cart.speed, cart.position))
    cart.evaluate(delta_t, _input)
    t = t + delta_t


