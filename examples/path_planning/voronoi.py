#
# voronoi.py
#

import numpy as np
import cv2

from world import *

class VoronoiPlanner:

    def __init__(self, world):
        self.world = world

    def world_to_image(self):
        image = np.zeros((self.world.x_size, self.world.y_size,3), dtype = np.uint8)
        for k in self.world.world_map:
            v = self.world.world_map[k]
            if v != 0:
                x,y = self.world.to_world(k[0], k[1])
                cv2.rectangle(image,
                                (int(x - self.world.scale / 2), int(y - self.world.scale / 2)),
                                (int(x + self.world.scale / 2), int(y + self.world.scale / 2)),
                                (0,0,255), -1)
        return image

    def make_voronoi(self, image):
        subdiv = cv2.Subdiv2D( (0, 0, self.world.x_size, self.world.y_size) )
        for k in self.world.world_map:
            v = self.world.world_map[k]
            if v != 0:
                x,y = self.world.to_world(k[0], k[1])
                subdiv.insert((int(x), int(y)))
        facets, centers = subdiv.getVoronoiFacetList([])
        for fs in facets:
            prev_point = None
            for f in fs:
                current_point = (int(f[0]), int(f[1]))
                if (current_point[0] >= 0)and(current_point[1] >= 0)and(current_point[0] < self.world.x_size)and(current_point[1] < self.world.y_size):
                    current_point_value = self.world.item_at(*current_point)
                    if current_point_value == 0:
                        if prev_point is None:
                            prev_point = current_point
                        else:
                            prev_point_value = self.world.item_at(*prev_point)
                            if (prev_point_value == 0):
                                cv2.line(image, prev_point, current_point, (0,255,0), 1)
                                prev_point = current_point
        return image

if __name__ == "__main__":
    w = World(1000, 1000, 10)
    w.add_rectangle_obstacle(500, 500, 700, 600)
    w.add_rectangle_obstacle(200, 200, 300, 800)
    voronoi = VoronoiPlanner(w)
    img = voronoi.world_to_image()
    img = voronoi.make_voronoi(img)
    cv2.imshow("world", img)
    cv2.waitKey(0)

