#
# basic.py
#

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

