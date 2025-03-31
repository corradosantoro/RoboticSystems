#
# time.py
#

import time

class Time:
    def __init__(self, use_fake_time : bool =False):
        """
        Helper class to manage the simulation time
        :param use_fake_time: set to false to use the real-time, true to use a simulated time
        """
        self.use_fake_time = use_fake_time
        self.last_t = 0
        self.t = 0
        self.internal_time = 0

    def set_use_fake_time(self, use_fake_time : bool):
        self.use_fake_time = use_fake_time

    def start(self):
        """
        Starts the time
        """
        current = time.time()
        self.last_t = current
        self.internal_time = current
        self.t = 0

    def elapsed(self) -> float:
        """
        Returns the value of the time interval (Delta T) with respect to the last call
        """
        if self.use_fake_time:
            current = self.internal_time
        else:
            current = time.time()
        delta_t = current - self.last_t
        self.last_t = current
        self.t += delta_t
        return delta_t

    def get(self) -> float:
        """
        Returns the value of the current time
        """
        return self.t

    def sleep(self, seconds : float):
        """
        Waits the given amount of seconds
        """
        if self.use_fake_time:
            self.internal_time += seconds
        else:
            time.sleep(seconds)
