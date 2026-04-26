#
#
#

from motion_manager import *
from turtle3 import *
import time
import math
import sys

turtlebot = Turtlebot()
turtlebot.open()
turtlebot.setPose(0,0,0)

motion = MotionManager(turtlebot)
motion.rotate_absolute(90)
motion.goto_point(0, 300)
motion.rotate_to(300, 300)
motion.goto_point(300, 300)
motion.start_motion()
#motion.wait_motion_completed()

