#
#
#

import math

phase_ACCEL = 0
phase_CRUISE = 1
phase_DECEL = 2
phase_TARGET = 3

class Trajectory:

	def __init__(self, acc, dec, vmax):
		self.acc = acc
		self.dec = dec
		self.vmax = vmax
		self.decel_distance = (self.vmax * self.vmax) / (2 * self.dec)
		self.v = 0
		self.p = 0
		self.phase = phase_ACCEL
		self.target_pos = 0
		
	def set_target_pos(self, p):
		self.target_pos = p
		
	def evaluate(self, delta_t):
		if self.phase == phase_ACCEL:
			self.p = self.p + self.v * delta_t + 0.5 * self.acc * delta_t * delta_t
			self.v = self.v + self.acc * delta_t
			if self.v >= self.vmax:
				self.phase = phase_CRUISE
				self.v = self.vmax
			distance = self.target_pos - self.p
			if distance <= self.decel_distance:
				v_exp = math.sqrt(2 * self.dec * distance)
				if v_exp < self.v:
					self.phase = phase_DECEL
		elif self.phase == phase_CRUISE:
			distance = self.target_pos - self.p
			self.p = self.p + self.vmax * delta_t
			self.v = self.vmax
			if distance <= self.decel_distance:
				self.phase = phase_DECEL
		elif self.phase == phase_DECEL:
			self.p = self.p + self.v * delta_t - 0.5 * self.dec * delta_t * delta_t
			self.v = self.v - self.dec * delta_t
			distance = self.target_pos - self.p
			if distance <= 0:
				self.phase = phase_TARGET
				self.v = 0
				self.p = self.target_pos
		elif self.phase == phase_TARGET:
			self.v = 0
			self.p = self.target_pos

		

import pylab

traj = Trajectory(4.0, 3.0, 1.5)

t = 0
time_array = []
speed_array = []
pos_array = []
delta_t = 1e-3

traj.set_target_pos(0.4)

while t < 4:
	traj.evaluate(delta_t)
	time_array.append(t)
	speed_array.append(traj.v)
	pos_array.append(traj.p)
	t = t + delta_t

pylab.figure(1)
pylab.plot(time_array, speed_array, 'r-+', label='v(t)')
pylab.xlabel('time')
pylab.legend()

pylab.figure(2)
pylab.plot(time_array, pos_array, 'r-+', label='p(t)')
pylab.xlabel('time')
pylab.legend()

pylab.show()

