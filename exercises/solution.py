import sys
sys.path.append("../")
import time

from lib.data.dataplot import *
from lib.utils.time import *

class G1:
    
    def __init__(self):
        self.x1 = 0
        self.x2 = 0
        
    def evaluate(self, delta_t, _input):
        new_x2 = self.x2 - 2 * delta_t * self.x2 - 3 * delta_t * self.x1 + 8 * delta_t * _input
        new_x1 = self.x1 + delta_t * self.x2
        
        self.x1 = new_x1
        self.x2 = new_x2
        
        return self.x1
        

class G2:

    def __init__(self):
        self.y = 0

    def evaluate(self, delta_t, _input):
        self.y = self.y - 6 * delta_t * self.y + 10 * _input * delta_t
        return self.y
    

class CompoundSystem:
    
    def __init__(self):
        self.g1 = G1()
        self.g2 = G2()
        
    def evaluate(self, delta_t, _input):
        out_g1 = self.g1.evaluate(delta_t, _input)
        out_g2 = self.g2.evaluate(delta_t, _input)
        return out_g1 + out_g2

dp = DataPlotter()
dp.set_x("time (seconds)")
dp.add_y("out", "Output")

s = CompoundSystem()
u = 1

t = Time()
t.start()
while t.get() < 10:

    time.sleep(0.001)
    delta_t = t.elapsed()
    
    y = s.evaluate(delta_t, u)
    
    dp.append_x(t.get())
    dp.append_y("out", y)

dp.plot()
print(y)

