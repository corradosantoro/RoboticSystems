#
#
#

import math

class NF1Planner:

    def __init__(self, x0, y0, x1, y1, cell_size):
        self.x0 = x0
        self.y0 = y0
        self.x1 = x1
        self.y1 = y1
        self.cell_size = cell_size
        self.width = self.x1 - self.x0
        self.height = self.y1 - self.y0
        (self.xc0, self.yc0) = self.__point_to_cell(x0, y0)

        self.i_w = int(self.width / self.cell_size)
        self.i_h = int(self.height / self.cell_size)

        self.map = {}

        for y in range(self.yc0, self.yc0 + self.i_h):
            for x in range(self.xc0, self.xc0 + self.i_w):
                self.map[ (x,y) ] = None

    def __point_to_cell(self, x, y):
        x = int(x / self.cell_size)
        y = int(y / self.cell_size)
        return (x,y)

    def __cell_to_point(self, x, y):
        x = x * self.cell_size
        y = y * self.cell_size
        return (x,y)

    def set_obstacle(self, x0, y0, x1, y1):
        (x0, y0) = self.__point_to_cell(x0, y0)
        (x1, y1) = self.__point_to_cell(x1, y1)
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                self.map[ (x,y) ] = -1

    def set_obstacle_center(self, x0, y0, width, height):
        self.set_obstacle(x0 - width/2, y0 - height/2, x0 + width/2, y0 + height/2)

    def __reset(self):
        for y in range(self.yc0, self.yc0 + self.i_h):
            for x in range(self.xc0, self.xc0 + self.i_w):
                if self.map[ (x,y) ] != -1:
                    self.map[ (x,y) ] = None

    def __map_full(self):
        for y in range(self.yc0, self.yc0 + self.i_h):
            for x in range(self.xc0, self.xc0 + self.i_w):
                if self.map[ (x,y) ] is None:
                    return False
        return True

    def __outside(self, x, y):
        return (x < self.xc0)or(y < self.yc0)or(x >= (self.xc0 + self.i_w))or(y >= (self.yc0 + self.i_h))

    def __distance(self, p1, p2):
        (x1, y1) = p1
        (x2, y2) = p2
        return math.hypot(x2 - x1, y2 - y1)

    def __make_map(self, x, y, mark):
        if self.__outside(x,y):
            return
        v = self.map[ (x,y) ]
        if (v is None)or(mark < v):
            self.map[ (x,y) ] = mark
            mark +=1
            self.__make_map(x - 1, y, mark)
            self.__make_map(x + 1, y, mark)
            self.__make_map(x, y - 1, mark)
            self.__make_map(x, y + 1, mark)


    def plan(self, xs, ys, xe, ye):
        (xe, ye) = self.__point_to_cell(xe, ye)
        (xs, ys) = self.__point_to_cell(xs, ys)
        self.__reset()
        self.__make_map(xe, ye, 0)
        self.print_map()

        # path finding
        path = [ (xe, ye) ]
        mark = self.map [ (xe, ye) ]
        self.map[ (xe, ye) ] = '**'
        neighbours = [ (-1, 0), (1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1) ]

        while (xe != xs)or(ye != ys):
            mark += 1
            selected_cell = None
            for (dx, dy) in neighbours:
                x = xe + dx
                y = ye + dy
                if self.__outside(x,y):
                    continue
                #print('Mark', mark, ', Current ', xe,',', ye, ' --> neighbour ', x, ',', y, ' = ', self.map[ (x,y) ], ' Path ', path)
                if self.map[(x,y)] == '**':
                    continue
                if self.map[ (x,y) ] != mark:
                    continue
                if selected_cell is None:
                    selected_cell = (x,y)
                elif (self.map[ (x,y) ] == mark )and(self.__distance((xs,ys), (x,y)) < self.__distance((xs,ys), selected_cell)):
                    selected_cell = (x,y)
            mark = self.map[ selected_cell ]
            self.map[ selected_cell ] = '**'
            path.append(selected_cell)
            #print(path)
            #self.print_map()
            (xe, ye) = selected_cell
        path.reverse()
        print(path)
        path = list(map(lambda p : self.__cell_to_point(p[0],p[1]), path))
        path.pop(0)
        return path

    def print_map(self):
        for y in range(self.yc0, self.yc0 + self.i_h):
            print("====", end='')
        print("")
        for y in range(self.yc0 + self.i_h - 1, self.yc0 - 1, -1):
            for x in range(self.xc0, self.xc0 + self.i_w):
                v = self.map[ (x,y) ]
                if v is None:
                    print(" NN ", end='')
                elif v == '**':
                    print(" ** ", end='')
                else:
                    print("{:3d} ".format(v), end='')
            print("")
        for y in range(self.i_h):
            print("====", end='')
        print("")


if __name__ == "__main__":

    p = NF1Planner(-3000, -3000, 3000, 3000, 300)

    p.set_obstacle_center(1500, 0, 1000, 1000)
    p.set_obstacle_center(-500, -1500, 1000, 1000)
    p.set_obstacle_center(-1000, 1200, 1000, 1000)

    p.print_map()

    print(p.plan(0,0, -750, -2500))
    p.print_map()

