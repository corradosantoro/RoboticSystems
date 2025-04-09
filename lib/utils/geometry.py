#
# geometry.py
#

import math

def normalize_angle(a):
    while a > math.pi:
        a = a - 2 * math.pi
    while a < - math.pi:
        a = a + 2 * math.pi
    return a


def global_to_local(xc, yc, t, x, y):
    cos_t = math.cos(t)
    sin_t = math.sin(t)
    dx = x - xc
    dy = y - yc
    local_x = dx * cos_t + dy * sin_t
    local_y = - dx * sin_t + dy * cos_t
    return local_x, local_y


def local_to_global(xc, yc, t, x, y):
    cos_t = math.cos(t)
    sin_t = math.sin(t)
    global_point_x = xc + x * cos_t - y * sin_t
    global_point_y = yc + x * sin_t + y * cos_t
    return global_point_x, global_point_y
