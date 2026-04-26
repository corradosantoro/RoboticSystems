#
#
#

from Rotation import *
from turtle3 import *
import time
import math
import sys

turtlebot = Turtlebot()
turtlebot.open()
turtlebot.setPose(0,0,0)

rot = RelativeRotation(turtlebot)
rot.start()

rot.go(math.radians(float(sys.argv[1])))
rot.wait_motion_completed()
print("Movimento completato")
rot.stop()

