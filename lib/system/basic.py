#
# basic.py
#

import numpy as np

class Proportional:

    def __init__(self, _kp):
        self.kp = _kp

    def evaluate(self, delta_t, _input):
        return _input * self.kp


class Derivator:

    def __init__(self):
        self.prev_input = 0

    def evaluate(self, delta_t, _input):
        out = (_input - self.prev_input) / delta_t
        self.prev_input = _input
        return out


class Integrator:

    def __init__(self):
        self.prev_output = 0

    def evaluate(self, delta_t, _input):
        out = self.prev_output + _input * delta_t
        self.prev_output = out
        return out


class LinearDynamicSystem:
    
    def __init__(self, A, B, C):
        self.A = np.array(A)
        self.B = np.array(B)
        self.C = np.array(C)
        self.order = len(A)
        self.x = np.array( [0] *  self.order)
    
    def evaluate(self, delta_t, _input):
        self.x = (self.A * delta_t + np.eye(self.order)) @ self.x + self.B * delta_t * _input
        output = self.C @ self.x
        return output.flatten().tolist()

    def eig(self):
        return np.linalg.eig(self.A)

        
        
        
        
        
        
        
        
        