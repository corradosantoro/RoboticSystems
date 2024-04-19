#
#
#

import math
import pylab
import sys

from pathlib import Path
CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.controllers.standard import *


class System:

	def __init__(self):
		self.x1 = 0
		self.x2 = 0
		
	def evaluate(self, delta_t, u):
		new_x1 = self.x1 + delta_t * self.x2
		new_x2 = self.x2 - 2 * self.x1 * delta_t - 10 * self.x2 * delta_t + 4 * u * delta_t
		self.x1 = new_x1
		self.x2 = new_x2
		return self.x1
	
	def get_y(self):
		return self.x1
		
		
class ControlledSystem:

	def __init__(self):
		self.s = System()
		self.pid = PIDSat(15, 6, 2, 15, True)
	
	def evaluate(self, delta_t, y_ref):
		current = self.s.get_y()
		u = self.pid.evaluate(delta_t, y_ref, current)
		return self.s.evaluate(delta_t,u)


def test_system():
	s = System()
	
	t = 0
	delta_t = 1e-3

	time_array = []
	data_array = []
	while t < 50:
		out = s.evaluate(delta_t, 10)
		time_array.append(t)
		data_array.append(out)
		t = t + delta_t

	pylab.figure(1)
	pylab.plot(time_array, data_array, 'r-+', label='y(t)')
	pylab.xlabel('time')
	pylab.legend()

	pylab.show()


def test_control():
	s = ControlledSystem()
	
	t = 0
	delta_t = 1e-3

	time_array = []
	data_array = []
	ref_array = []

	target = 5
	while t < 10:
		out = s.evaluate(delta_t, target)
		time_array.append(t)
		data_array.append(out)
		ref_array.append(target)
		t = t + delta_t

	pylab.figure(1)
	pylab.plot(time_array, ref_array, 'b-+', label='yref(t)')
	pylab.plot(time_array, data_array, 'r-+', label='y(t)')
	pylab.xlabel('time')
	pylab.legend()

	pylab.show()

test_control()
#test_system()

