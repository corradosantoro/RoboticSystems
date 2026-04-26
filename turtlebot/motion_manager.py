#
#
#
import threading
from goto_point import *
from Rotation import *

class MotionManager(threading.Thread):

	def __init__(self, tb):
		super().__init__()
		self.turtlebot = tb
		self.motion_list = []
		self.running = False

	def rotate_to(self, x, y):
		m = RotationTo(self.turtlebot)
		m.set_target(x, y)
		self.motion_list.append(m)
		
	def rotate_absolute(self, theta):
		m = AbsoluteRotation(self.turtlebot)
		m.set_target(math.radians(theta))
		self.motion_list.append(m)
		
	def rotate_relative(self, theta):
		m = RelativeRotation(self.turtlebot)
		m.set_target(math.radians(theta))
		self.motion_list.append(m)
		
	def goto_point(self, x, y):
		m = GotoPoint(self.turtlebot)
		m.set_target(x, y)
		self.motion_list.append(m)
		
	def start_motion(self):
		if self.motion_list == []:
			return
		self.current_motion = self.motion_list.pop(0)
		self.current_motion.start()
		if not(self.running):
			self.running = True
			self.start()
		self.current_motion.go()
		print("Starting command ", self.current_motion)
		
	def run(self):
		while self.running:
			# attendi che il movimento corrente termini
			self.current_motion.wait_motion_completed()
			self.current_motion.stop()
			# esegui il prossimo movimento
			self.start_motion()



