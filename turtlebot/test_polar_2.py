#
#
#

from goto_point import *
from turtle3 import *
import time
import math
import sys

turtlebot = Turtlebot()
turtlebot.open()
turtlebot.setPose(0,0,0)

goto = GotoPoint(turtlebot)
goto.start()

goto.go(300,0)
while goto.on:
	time.sleep(0.1)
goto.go(500,100)
while goto.on:
	time.sleep(0.1)

