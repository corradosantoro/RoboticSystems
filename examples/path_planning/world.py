#
# world.py
#

class World:

    def __init__(self, x_size, y_size, scale):
        self.x_size = x_size
        self.y_size = y_size
        self.scale = scale
        self.w = int(self.x_size / scale)
        self.h = int(self.y_size / scale)

        self.world_map = {}
        for y in range(self.h):
            for x in range(self.w):
                self.world_map[ (x,y) ] = 0

        self.obstacles = []

        # borders
        for x in range(self.w):
            self.world_map[ (x,0) ] = 1
            self.world_map[ (x,self.h - 1) ] = 1

        for y in range(self.h):
            self.world_map[ (0, y) ] = 1
            self.world_map[ (self.w - 1, y) ] = 1

    def item_at(self, x, y):
        (x, y) = self.to_map(x, y)
        return self.world_map[(x,y)]

    def to_map(self, x, y):
        return int(x / self.scale), int(y / self.scale)

    def to_world(self, x, y):
        return int(x * self.scale + self.scale/2), int(y * self.scale + self.scale/2)

    def add_rectangle_obstacle(self, x0, y0, x1, y1):
        (x0, y0) = self.to_map(x0, y0)
        (x1, y1) = self.to_map(x1, y1)

        obstacle_points = []
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                obstacle_points.append( (x, y) )
                self.world_map[ (x,y) ] = 1
        self.obstacles.append(obstacle_points)


