#
#
#

import time
import math
import threading

class Motion(threading.Thread):

	def __init__(self, tb):
		super().__init__()
		self.turtlebot = tb
		self.wheelbase = 250.0
		self.on = False
		self.sem = threading.Semaphore(value=0)

	def wait_motion_completed(self):
		self.sem.acquire() # down

	def go(self):
		self.on = True
		
	def stop(self):
		self.running = False
		
	def execute_control(self, delta_t):
		pass
		
	def run(self):
		self.running = True
		last_time = time.time()
		while self.running:
			current_time = time.time()
			delta_t = current_time - last_time
			last_time = current_time

			if not(self.on):		
				self.turtlebot.setSpeeds(0, 0)
				continue

			self.execute_control(delta_t)
			
