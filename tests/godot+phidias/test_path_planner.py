import sys
import math
import time

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.godot.motion_control import *
from lib.planners.nf1 import *

from phidias.Types  import *
from phidias.Main import *
from phidias.Lib import *
from phidias.Agent import *

class start(Procedure): pass
class end_of_path(Reactor): pass


class PathExecutor(threading.Thread):

    path_manager = None

    def __init__(self, group=None, target=None, name=None,
                 args=(), kwargs=None):
        super(PathExecutor,self).__init__(group=group, target=target,
                                    name=name)

        self.daemon = True
        
    def run(self):
        print("PathExecutor Running")
        while PathExecutor.path_manager.is_running():
            time.sleep(0.5)
            PathExecutor.path_manager.execute()
        PHIDIAS.assert_belief(end_of_path(),'main')


class path_to(Action):
    planner = None # static attribute
    path_manager = None # static attribute
    def execute(self, x, y):
        target_x = x()
        target_y = y()
        print("Goto ", target_x, target_y)
        points = path_to.planner.plan(0, 0, target_x, target_y)
        path_to.planner.print_map()
        
        goto = GotoPoint()
        rotate = RotateTo()
        heading = HeadingTo()

        commands = []
        for (x,y) in points:
           # convert millimeters in meters
           commands.append( (heading, (x / 1000.0, y / 1000.0)) )
           commands.append( (goto, (x / 1000.0, y / 1000.0)) )
        print(commands)
        
        path_to.path_manager.set_path( commands )
        path_to.path_manager.start(cart_robot)  
         
        PathExecutor.path_manager = path_to.path_manager
        
        PathExecutor().start()     



class main(Agent):

    def main(self):
        start() >> [ path_to(-750, -2500) ]
        +end_of_path() >> [ show_line("End of path") ]
        

if __name__ == '__main__':

    cart_robot = Cart2DRobot()
    cart_robot.start()

    path = Path()

    # here we use millimeters for units
    p = NF1Planner(-3000, -3000, 3000, 3000, 400)
    path_to.planner = p
    path_to.path_manager = path

    p.set_obstacle_center(1500, 0, 1000, 1000)
    p.set_obstacle_center(-500, -1500, 1000, 1000)
    p.set_obstacle_center(-1000, 1200, 1000, 1000)

    main().start()
    # run the engine shell
    PHIDIAS.shell(globals())


