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


def rototranslate(xp, yp, xc, yc, t):
    """
    Rototranslate a point from a reference system (x',y') to a system (x,y)
    :param xp,yp: The point in the reference system (x', y')
    :param xc, yc: The origin of (x',y') in (x,y) coordinates
    :param t: The rotation of x' with respect to x (in radians)
    """
    cos_t = math.cos(t)
    sin_t = math.sin(t)
    global_point_x = xc + xp * cos_t - yp * sin_t
    global_point_y = yc + xp * sin_t + yp * cos_t
    return global_point_x, global_point_y


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
