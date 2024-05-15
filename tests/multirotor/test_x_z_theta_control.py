import sys
import math

from pathlib import Path

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.multirotor import Multirotor2D
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.gui.multirotor_gui import MultirotorWindow
from lib.data.plot import DataPlotter

from PyQt5.QtWidgets import QApplication


class MultirotorRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        self.MR = Multirotor2D(1.0, 0.25)  # 1.0kg, L = 25cm

        self.vz_control = PIDSat(10.0, 50.0, 0.0,
                                 15, True)  # 15 N saturation + antiwindup
        self.z_control = PIDSat(2.0, 0.0, 0.0, 2)  # 2 m/s

        self.vx_control = PIDSat(1.0, 0.2, 0.2,
                                 math.radians(30), True)  # 15 N saturation + antiwindup
        self.x_control = PIDSat(2.0, 0.0, 0.0, 2)  # 2 m/s

        self.omega_control = PIDSat(1.0, 5.0, 0.0,
                                    15, True)  # 15 N saturation + antiwindup
        self.theta_control = PIDSat(2.0, 0.0, 0.0, 1.57)  # 1.57 m/s

        self.z_target = 0.4
        self.x_target = 0.5

        self.plot = DataPlotter()

    def run(self):
        (x, z, theta) = self.get_pose()
        (vx, vz, omega) = self.get_speed()

        # horizontal control
        vx_target = self.x_control.evaluate(self.delta_t, self.x_target, x)
        theta_target = - self.vx_control.evaluate(self.delta_t, vx_target, vx)
        omega_target = self.theta_control.evaluate(self.delta_t, theta_target, theta)
        diff = self.omega_control.evaluate(self.delta_t, omega_target, omega)

        # vertical control
        vz_target = self.z_control.evaluate(self.delta_t, self.z_target, z)
        f = self.vz_control.evaluate(self.delta_t, vz_target, vz)


        self.MR.evaluate(self.delta_t, f - diff, f + diff)

        self.plot.add('t', self.t)
        self.plot.add('theta', theta)
        self.plot.add('theta_target', theta_target)
        self.plot.add('vx', vx)
        self.plot.add('vx_target', vx_target)

        if self.t >= 4:
            self.plot.plot( [ 't', 'time' ],
                            [ [ 'theta', 'theta' ] , [ 'theta_target', 'theta_target' ] ])
            self.plot.plot( [ 't', 'time' ],
                            [ [ 'vx', 'vx' ], [ 'vx_target', 'vx_target']  ])
            self.plot.show()
            return False
        else:
            return True

    def get_pose(self):
        return self.MR.x, self.MR.z, self.MR.theta

    def get_speed(self):
        return self.MR.vx, self.MR.vz, self.MR.omega


if __name__ == '__main__':
    robot = MultirotorRobot()
    app = QApplication(sys.argv)
    ex = MultirotorWindow(robot)
    sys.exit(app.exec_())
