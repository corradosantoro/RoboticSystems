#
# time.py
#

import time

class Time:

    def __init__(self):
        self.last_t = 0

    def start(self):
        self.last_t = time.time()
        self.t = 0

    def elapsed(self):
        current = time.time()
        delta_t = current - self.last_t
        self.last_t = current
        self.t += delta_t
        return delta_t

    def get(self):
        return self.t

