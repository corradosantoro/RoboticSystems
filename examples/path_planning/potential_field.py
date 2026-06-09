#
# nf1.py
#

from world import *
import numpy as np
import cv2

class PotentialFieldPlanner:

    def __init__(self, world):
        self.world = world
        self.__init_gradient()
        self.k_att = 0.02
        self.k_rep = 20
        self.repulsive_range = 10

    def __init_gradient(self):
        self.gradient_map = {}
        for k in self.world.world_map:
            v = self.world.world_map[k]
            if v == 0:
                self.gradient_map[k] = None # not a mark
            else:
                self.gradient_map[k] = -1 # obstacle

    def world_to_image(self):
        font = cv2.FONT_HERSHEY_SIMPLEX
        image = np.zeros((self.world.x_size, self.world.y_size,3), dtype = np.uint8)
        for k in self.gradient_map:
            v = self.gradient_map[k]
            if type(v) != int: # (is not an obstacle)
                x,y = self.world.to_world(k[0], k[1])
                fx, fy = v
                angle = np.arctan2(fy, fx)
                x0 = int(x - fx)
                y0 = int(y - fy)
                x1 = int(x + fx)
                y1 = int(y + fy)

                cv2.arrowedLine(image, (x0, y0), (x1, y1), (0, 255, 0), 1, tipLength=0.5)

        return image

    def plan(self, start, end):
        self.__init_gradient()
        self.__make_attractive_force(end)
        self.__make_repulsive_force()

    def __make_attractive_force(self, target):
        target = np.array(target)
        for k in self.gradient_map:
            v = self.gradient_map[k]
            if v is None:
                p = np.array(self.world.to_world(*k))
                distance = p - target
                force = - self.k_att * distance
                self.gradient_map[k] = force

    def __make_repulsive_force(self):
        a = self.world.obstacles
        obstacle_points = [ x for sublist in a for x in sublist ]
        for k in self.gradient_map:
            v = self.gradient_map[k]
            if type(v) != int: # (is not an obstacle)
                p = np.array(self.world.to_world(*k))

                # compute nearest obstacle
                min_point = None
                dmin = float("inf")
                for p in obstacle_points:
                    p = np.array(p)
                    d = np.linalg.norm(p - k)
                    if dmin >= d:
                        dmin = d
                        min_point = p

                if dmin <= self.repulsive_range:
                    f = self.k_rep * (1.0 / dmin - 1.0 / self.repulsive_range) * (1/ (dmin** 2)) * (k - min_point) / dmin
                else:
                    f = 0

                if v is None:
                    self.gradient_map[k] = f
                else:
                    self.gradient_map[k] += f




    def __add(self, pt, incr):
        x = pt[0] + incr[0]
        y = pt[1] + incr[1]
        if (x < 0)or(y < 0)or(x >= self.world.w)or(y >= self.world.h):
            return None
        else:
            return (x,y)


if __name__ == "__main__":
    w = World(1000, 1000, 25)
    w.add_rectangle_obstacle(500, 500, 700, 600)
    w.add_rectangle_obstacle(500, 200, 700, 300)
    w.add_rectangle_obstacle(200, 200, 300, 800)
    pot = PotentialFieldPlanner(w)
    pot.plan((500,200),(900,900))
    img = pot.world_to_image()
    cv2.imshow("world", img)
    cv2.waitKey(0)

