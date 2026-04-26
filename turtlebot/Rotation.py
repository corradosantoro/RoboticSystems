#
#
#

from controllers import *
from geometry import *
from motion import *
import time
import math
import threading

class AbsoluteRotation(Motion):

	def __init__(self, tb):
		super().__init__(tb)
		self.controller = P_Controller(3.0, 200.0 / self.wheelbase)
		
	def __repr__(self):
		return "AbsoluteRotation %f" % (math.degrees(self.theta_target)) 
		
	def set_target(self, theta_target):
		self.theta_target = theta_target
		
	def execute_control(self, delta_t):
		current_pose = self.turtlebot.getPose()
		
		target_error = normalize_angle(self.theta_target - current_pose.theta)
		if abs(target_error) < math.radians(2):
			self.on = False
			self.sem.release()
		
		w = self.controller.evaluate(delta_t, target_error)
		
		vl = - (w * self.wheelbase) / 2
		vr = + (w * self.wheelbase) / 2
		
		self.turtlebot.setSpeeds(vl, vr)
		#print(vl, vr, math.degrees(target_error))

	
	
class RelativeRotation(Motion):

	def __init__(self, tb):
		super().__init__(tb)
		self.controller = P_Controller(3.0, 200.0 / self.wheelbase)

	def __repr__(self):
		return "RelativeRotation %f" % (math.degrees(self.theta_target)) 
			
	def set_target(self, theta_target):
		self.theta_target = theta_target
		
	def go(self):
		self.turtlebot.clearDistances()
		super().go()
		
	def execute_control(self, delta_t):
		current_pose = self.turtlebot.getPose()
		
		target_error = self.theta_target - current_pose.angular
		if abs(target_error) < math.radians(2):
			self.on = False
			self.sem.release() # up
		
		w = self.controller.evaluate(delta_t, target_error)
		
		vl = - (w * self.wheelbase) / 2
		vr = + (w * self.wheelbase) / 2
		
		self.turtlebot.setSpeeds(vl, vr)
		#print(vl, vr, math.degrees(target_error))

class RotationTo(AbsoluteRotation):

	def __init__(self, tb):
		super().__init__(tb)
		
	def __repr__(self):
		return "RotationTo %f, %f" % (self.target_x, self.target_y) 
		
	def set_target(self, x, y):
		self.target_x = x
		self.target_y = y
		
	def go(self):
		current_pose = self.turtlebot.getPose()
		self.theta_target = math.atan2( self.target_y - current_pose.y,
						self.target_x - current_pose.x)
		print("Computed theta = %f" % (math.degrees(self.theta_target)))
		super().go()

