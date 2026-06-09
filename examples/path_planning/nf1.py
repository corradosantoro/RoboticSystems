#
# nf1.py
#

from world import *
import numpy as np
import cv2

class NF1Planner:

    def __init__(self, world):
        self.world = world
        self.mark_map = {}
        for k in self.world.world_map:
            v = self.world.world_map[k]
            if v == 0:
                self.mark_map[k] = None # not a mark
            else:
                self.mark_map[k] = -1 # obsacle

    def world_to_image(self):
        image = np.zeros((self.world.x_size, self.world.y_size,3), dtype = np.uint8)
        for k in self.mark_map:
            v = self.mark_map[k]
            if v >= 0:
                x,y = self.world.to_world(k[0], k[1])
                cv2.putText(image, "%d" % (v),
                                (int(x - self.world.scale / 2), int(y - self.world.scale / 2)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 0, 0), 2)
        return image

    def plan(self, start, end):
        start = self.world.to_map(*start)
        end = self.world.to_map(*end)

        self.__mark_all(end, 0)


    def __mark_all(self, point, value):
        v = self.mark_map[point]
        if v == -1:
            return

        go = False
        if v is None:
            go = True
        elif value < v:
            go = True

        if go:
            self.mark_map[point] = value
            n = self.__add(point, (0, -1))
            s = self.__add(point, (0, 1))
            e = self.__add(point, (1, 0))
            w = self.__add(point, (-1, 0))
            if n is not None:
                self.__mark_all(n, value + 1)
            if s is not None:
                self.__mark_all(s, value + 1)
            if e is not None:
                self.__mark_all(e, value + 1)
            if w is not None:
                self.__mark_all(w, value + 1)

    def __add(self, pt, incr):
        x = pt[0] + incr[0]
        y = pt[1] + incr[1]
        if (x < 0)or(y < 0)or(x >= self.world.w)or(y >= self.world.h):
            return None
        else:
            return (x,y)


if __name__ == "__main__":
    w = World(1000, 1000, 50)
    w.add_rectangle_obstacle(500, 500, 700, 600)
    w.add_rectangle_obstacle(200, 200, 300, 800)
    nf1 = NF1Planner(w)
    nf1.plan((100,100),(900,900))
    img = nf1.world_to_image()
    cv2.imshow("world", img)
    cv2.waitKey(0)

