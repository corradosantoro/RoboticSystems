#
# circular_trajectory.py
#

import math
import sys
import pylab

from pathlib import Path
CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.virtual_robot import *

class CircularTrajectory:

	def __init__(self, _vmax, _acc, _dec, _radius, _angle):
		self.radius = _radius
		self.angle = _angle
		l = self.angle * abs(self.radius)
		self.linear_trajectory = VirtualRobot(l, _vmax, _acc, _dec)
		
	def start(self, pose):
		(self.x_r, self.y_r, self.theta_r) = pose
		
	def evaluate(self, delta_t):
		self.linear_trajectory.evaluate(delta_t)
		alpha = self.linear_trajectory.p / abs(self.radius)
		x_prime = abs(self.radius) * math.sin(alpha)
		if self.radius > 0:
			y_prime = self.radius - self.radius * math.cos(alpha)
		else:
			y_prime = - (abs(self.radius) - abs(self.radius) * math.cos(alpha))
		
		x = x_prime * math.cos(self.theta_r) - y_prime * math.sin(self.theta_r) + self.x_r
		y = x_prime * math.sin(self.theta_r) + y_prime * math.cos(self.theta_r) + self.y_r
				
		return (x, y)


if __name__ == "__main__":
	c = CircularTrajectory(1.5, 3.0, 3.0, 
				-0.5, math.radians(45))
	c.start( (0,0,math.radians(0)) )
	x = []
	y = []
	delta_t = 1e-3
	t = 0
	while t < 3:
		(x_p, y_p) = c.evaluate(delta_t)
		x.append(x_p)
		y.append(y_p)
		t += delta_t
	pylab.figure(1)
	pylab.plot(x, y, 'r-+', label='traj')
	pylab.xlabel('x')
	pylab.legend()

	pylab.show()
	
