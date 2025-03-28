#
# controllers.py
#

from lib.system.basic import *
from typing import Optional

def saturate(inp: float, sat: float) -> tuple[float, bool]:
    if inp > sat:
        return (sat, True)
    if inp < - sat:
        return (-sat, True)
    return (inp, False)


class P_Controller:

    def __init__(self, _kp: float, _sat: Optional[float] = None):
        self.kp = _kp
        self.saturation = _sat

    def evaluate(self, delta_t: float, _error: float) -> float:
        out = self.kp * _error
        if self.saturation is not None:
            out, _ = saturate(out, self.saturation)
        return out


class PI_Controller:

    def __init__(self, _kp: float, _ki: float, _sat: Optional[float] = None):
        self.kp = _kp
        self.ki = _ki
        self.i = Integrator()
        self.saturation = _sat
        self.in_saturation = False

    def evaluate(self, delta_t: float, _error: float) -> float:
        out = self.kp * _error

        if self.in_saturation:
            out = out + self.ki * self.i.prev_output
        else:
            out = out + self.ki * self.i.evaluate(delta_t, _error)

        if self.saturation is not None:
            out, self.in_saturation = saturate(out, self.saturation)

        return out

class PID_Controller(PI_Controller):

    def __init__(self, _kp: float, _ki: float, _kd : float, _sat: Optional[float] = None):
        super().__init__(_kp, _ki, _sat)
        self.kd = _kd
        self.D = Derivator()

    def evaluate(self, delta_t: float, _error: float) -> float:
        out_PI = super().evaluate(delta_t, _error)
        out_D = self.D.evaluate(delta_t, _error)
        return out_PI + out_D * self.kd


