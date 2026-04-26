#
#
#

from polar import *
from motion import *
import time
import math
import threading

class GotoPoint(Motion):

	def __init__(self, tb):
		super().__init__(tb)
		self.polar_controller = Polar2DController(1.0, 200.0, 3.0, 200.0 / self.wheelbase)

	def __repr__(self):
		return "Goto Point %f, %f" % (self.target_x, self.target_y) 
			

	def set_target(self, tx, ty):
		self.target_x = tx
		self.target_y = ty
		
	def execute_control(self, delta_t):
		current_pose = self.turtlebot.getPose()
		
		dx = current_pose.x - self.target_x
		dy = current_pose.y - self.target_y
		target_error = math.hypot(dx, dy)
		if target_error < 5:
			self.on = False
			self.sem.release()
		
		(v, w) = self.polar_controller.evaluate(delta_t,
				self.target_x, self.target_y,
				(current_pose.x, current_pose.y, current_pose.theta) )
		
		vl = v - (w * self.wheelbase) / 2
		vr = v + (w * self.wheelbase) / 2
		
		self.turtlebot.setSpeeds(vl, vr)
		# print(vl, vr, target_error)

	
	
	
