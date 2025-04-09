#
# trajectory.py
#

import math

# ------------------------------------------------------------

class VirtualRobot:
    """
    This class implements the algoritm of the linear trajectory
    generation for a virtual robot moving over a straight line of a certain distance
    with an acceleration, a cruise speed and a deceleration.
    """
    ACCEL = 0
    CRUISE = 1
    DECEL = 2
    TARGET = 3

    def __init__(self, _p_target : float, _vmax : float, _acc : float, _dec : float):
        self.p_target = _p_target
        self.vmax = _vmax
        self.accel = _acc
        self.decel = _dec
        self.v = 0  # current speed
        self.p = 0  # current position
        self.phase = VirtualRobot.ACCEL
        self.decel_distance = 0.5 * _vmax * _vmax / _dec

    def evaluate(self, delta_t : float) -> None:
        if self.phase == VirtualRobot.ACCEL:
            self.p = self.p + self.v * delta_t \
                     + self.accel * delta_t * delta_t / 2
            self.v = self.v + self.accel * delta_t
            distance = self.p_target - self.p
            if distance < 0:
                distance = 0
            if self.v >= self.vmax:
                self.v = self.vmax
                self.phase = VirtualRobot.CRUISE
            elif distance <= self.decel_distance:
                v_exp = math.sqrt(2 * self.decel * distance)
                if v_exp < self.v:
                    self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.CRUISE:
            self.p = self.p + self.vmax * delta_t
            distance = self.p_target - self.p
            if distance <= self.decel_distance:
                self.phase = VirtualRobot.DECEL

        elif self.phase == VirtualRobot.DECEL:
            self.p = self.p + self.v * delta_t \
                     - self.decel * delta_t * delta_t / 2
            v = self.v - self.decel * delta_t
            if v >= 0:
                self.v = v
            if self.p >= self.p_target:
                self.v = 0
                self.p = self.p_target
                self.phase = VirtualRobot.TARGET
        elif self.phase == VirtualRobot.TARGET:
            self.v = 0
            self.p = self.p_target

    def speed(self):
        return self.v

    def position(self):
        return self.p

# ------------------------------------------------------------

class StraightLine2DMotion:

    def __init__(self, _vmax, _acc, _dec):
        self.vmax = _vmax
        self.accel = _acc
        self.decel = _dec

    def start_motion(self, start, end):
        (self.xs,self.ys) = start
        (self.xe,self.ye) = end
        dx = self.xe - self.xs
        dy = self.ye - self.ys
        self.heading = math.atan2(dy , dx)
        self.distance = math.sqrt(dx*dx + dy*dy)
        self.virtual_robot = VirtualRobot(self.distance,
                                          self.vmax, self.accel, self.decel)

    def evaluate(self, delta_t):
        self.virtual_robot.evaluate(delta_t)
        xt = self.xs + self.virtual_robot.p * math.cos(self.heading)
        yt = self.ys + self.virtual_robot.p * math.sin(self.heading)
        return (xt, yt)

# ------------------------------------------------------------

class Path2D:
    def __init__(self, _vmax, _acc, _dec, _threshold):
        self.threshold = _threshold
        self.path = [ ]
        self.trajectory = StraightLine2DMotion(_vmax, _acc, _dec)

    def set_path(self, path):
        self.path = path

    def start(self, start_pos):
        self.current_target = self.path.pop(0)
        self.trajectory.start_motion(start_pos, self.current_target)

    def evaluate(self, delta_t, pose):
        (x, y) = self.trajectory.evaluate(delta_t)

        target_distance = math.hypot(pose[0] - self.current_target[0],
                                         pose[1] - self.current_target[1])

        if target_distance < self.threshold:
            if len(self.path) == 0:
                return None
            else:
                self.start( (x,y) )

        return (x,y)
