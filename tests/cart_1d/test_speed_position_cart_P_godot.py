import sys

from pathlib import Path
from PyQt5.QtWidgets import QApplication

CURRENT_POSITION = Path(__file__).parent
sys.path.append(f"{CURRENT_POSITION}/../../")

from lib.models.cart import Cart
from lib.models.robot import RoboticSystem
from lib.controllers.standard import PIDSat
from lib.data.plot import DataPlotter
from lib.gui.gui_1d import CartWindow
from lib.godot.interface import *

class CartRobot(RoboticSystem):

    def __init__(self):
        super().__init__(1e-3)  # delta_t = 1e-3
        self.cart = GodotCart1D()
        self.plotter = DataPlotter()
        self.speed_controller = PIDSat(20.0, 50.0, 0.0, 2500, True)  # Sat = 2500 Kg
        self.position_controller = PIDSat(4.0, 0.0, 0.0, 500)  # vmax = 500 pix/s
        self.target_position = 1000  # 1000 pixel
        self.position = 0
        self.speed = 0

    def run(self):
        v_target = self.position_controller.evaluate(self.delta_t, self.target_position, self.position)
        F = self.speed_controller.evaluate(self.delta_t, v_target, self.speed)
        (self.delta_t, self.position, self.speed) = self.cart.process(F)
        self.t += self.delta_t
	
        self.plotter.add('t', self.t)
        self.plotter.add('target_speed', v_target)
        self.plotter.add('speed', self.speed)
        self.plotter.add('Force', F)
        self.plotter.add('target_pos', self.target_position)
        self.plotter.add('pos', self.position)
        print(self.t)
        
        if self.t >= 5:
            self.plotter.plot(['t', 'time'], [['target_speed', 'Target Speed'],
                                              ['speed', 'Current Speed']])
            self.plotter.plot(['t', 'time'], [['Force', 'Force']])
            self.plotter.plot(['t', 'time'], [['target_pos', 'Target Position'],
                                              ['pos', 'Current Position']])
            self.plotter.show()
            return False
        else:
            return True


if __name__ == '__main__':
    cart_robot = CartRobot()
    while cart_robot.run():
         pass


