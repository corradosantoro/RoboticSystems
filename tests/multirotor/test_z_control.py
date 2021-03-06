#
# test_z_control.py
#

import sys
sys.path.insert(0, '../../lib')

from models.multirotor import *
from models.robot import *
from controllers.standard import *
from gui.multirotor_gui import *
from data.plot import *

from PyQt5.QtWidgets import QApplication

class MultirotorRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3) # delta_t = 1e-3
        self.MR = Multirotor2D(1.0, 0.25) # 1.0kg, L = 25cm
        self.vz_control = PIDSat(10.0, 50.0, 0.0,
                                 15, True)   # 15 N saturation + antiwindup
        self.z_control = PIDSat(2.0, 0.0, 0.0, 2)   # 2 m/s
        self.z_target = 0.4

        self.plot = DataPlotter()

    def run(self):
        (_, z, _) = self.get_pose()
        (_, vz, _) = self.get_speed()

        vz_target = self.z_control.evaluate(self.delta_t, self.z_target, z)

        f = self.vz_control.evaluate(self.delta_t, vz_target, vz)

        self.MR.evaluate(self.delta_t, f, f)

        self.plot.add( 't', self.t)
        self.plot.add( 'vz', vz)
        self.plot.add( 'vz_target', vz_target)
        self.plot.add( 'z', z)
        self.plot.add('f', f)

        if self.t >= 4:
            self.plot.plot( [ 't', 'time' ],
                            [ [ 'vz', 'vz' ] , [ 'vz_target', 'vz target' ] ])
            self.plot.plot( [ 't', 'time' ],
                            [ [ 'f', 'F' ]  ])
            self.plot.plot( [ 't', 'time' ],
                            [ [ 'z', 'Z' ]  ])
            self.plot.show()
            return False
        else:
            return True

    def get_pose(self):
        return (self.MR.x, self.MR.z, self.MR.theta)

    def get_speed(self):
        return (self.MR.vx, self.MR.vz, self.MR.omega)


if __name__ == '__main__':
    robot = MultirotorRobot()
    app = QApplication(sys.argv)
    ex = MultirotorWindow(robot)
    sys.exit(app.exec_())
