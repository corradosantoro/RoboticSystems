#
#
#

from polar import *
from turtle3 import *
import time
import math
import sys

turtlebot = Turtlebot()
turtlebot.open()

wheelbase = 250.0
polar_controller = Polar2DController(1.0, 200.0, 3.0, 200.0 / wheelbase)

(target_x, target_y) = (float(sys.argv[1]), float(sys.argv[2]))

turtlebot.setPose(0,0,0)
last_time = time.time()
while True:
	current_time = time.time()
	delta_t = current_time - last_time
	last_time = current_time
	current_pose = turtlebot.getPose()
	
	dx = current_pose.x - target_x
	dy = current_pose.y - target_y
	target_error = math.hypot(dx, dy)
	if target_error < 5:
		turtlebot.setSpeeds(0, 0)
		break
	
	(v, w) = polar_controller.evaluate(delta_t, target_x, target_y,
			(current_pose.x, current_pose.y, current_pose.theta) )
	
	vl = v - (w * wheelbase) / 2
	vr = v + (w * wheelbase) / 2
	
	turtlebot.setSpeeds(vl, vr)
	print(vl, vr, target_error)

	
	
	
	
	
