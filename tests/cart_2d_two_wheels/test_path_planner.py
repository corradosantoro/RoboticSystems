import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.godot.motion_control import *
from lib.planners.nf1 import *

if __name__ == '__main__':
    cart_robot = Cart2DRobot()
    cart_robot.start()

    goto = GotoPoint()
    rotate = RotateTo()
    heading = HeadingTo()

    path = Path()

    # here we use millimeters for units
    p = NF1Planner(-3000, -3000, 3000, 3000, 400)

    p.set_obstacle_center(1500, 0, 1000, 1000)
    p.set_obstacle_center(-500, -1500, 1000, 1000)
    p.set_obstacle_center(-1000, 1200, 1000, 1000)

    points = p.plan(0, 0, -750, -2500)
    p.print_map()

    commands = []
    print("Path ", end='')
    for (x,y) in points:
        print( (x,y), end='')
        # convert millimeters in meters
        commands.append( (heading, (x / 1000.0, y / 1000.0)) )
        commands.append( (goto, (x / 1000.0, y / 1000.0)) )
    print('')

    path.set_path( commands )

    path.start(cart_robot)

    while path.is_running():
        time.sleep(0.5)
        path.execute()

    print("End of path")


