#
#
#
import sys
sys.path.append("../../")

import time
import math
import threading

from lib.data.dataplot import *
from lib.utils.time import *
from lib.system.basic import *
from lib.system.controllers import *
from lib.hardware.motor_driver import *

class PositionSpeedControl:
    
    def __init__(self, kp_pos, sat_pos, kp_speed, ki_speed, sat_speed):
        self.position_controller = P_Controller(kp_pos, sat_pos)
        self.speed_controller = PI_Controller(kp_speed, ki_speed, sat_speed)
        self.target_pos = 0
        self.target_speed = 0
        self.mutex = threading.Lock()
        
    def set_target_position(self, p):
        try:
            self.mutex.acquire()
            self.target_pos = p
        finally:
            self.mutex.release()
        
    def evaluate(self, delta_t, current_pos, current_speed):
        try:
            self.mutex.acquire()
            pos_error = self.target_pos - current_pos
            self.target_speed = self.position_controller.evaluate(delta_t, pos_error)
            
            speed_error = self.target_speed - current_speed
            out = self.speed_controller.evaluate(delta_t, speed_error)
            return out
        finally:
            self.mutex.release()

class MotorControlThread(threading.Thread):

    def set_target(self, new_target):
        self.ctrl.set_target_position(new_target)

    def run(self):
        m = MotorDriver()
        m.open()

        d = Derivator()

        self.ctrl = PositionSpeedControl(10.0,  # KP pos 
                                    90.0, # pos output saturation (max speed deg/s) 
                                    30.0,  # KP speed
                                    20.0,  # KI speed
                                    4200)  # speed output saturation (max motor power)
        self.ctrl.set_target_position(0)

        tm = Time()
        tm.start()
        while True:

            delta_t = tm.elapsed()
            
            position = m.encoder()
            speed = d.evaluate(delta_t, position)
            
            position = position * 360/84000
            speed = speed * 360/84000
            
            output = self.ctrl.evaluate(delta_t, position, speed)
            
            m.pwm(output) # range [-4200, 4200]
    
control_thread = MotorControlThread()
control_thread.start()

while True:

    new_target = input("Insert target position:")
    control_thread.set_target(float(new_target))


control_thread.join()
