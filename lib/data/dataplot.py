#
# dataplot.py
#

from matplotlib import pylab

class DataPlotter:

    FIGURE = 1

    def __init__(self):
        self.y_data = {}
        self.y_label = {}
        self.x_data = []
        self.x_label = "x"
        self.__options = ['r-', 'b-', 'g-']
        DataPlotter.FIGURE = 1

    def set_x(self, label):
        self.x_label = label

    def append_x(self, value):
        self.x_data.append(value)

    def add_y(self, varname, varlabel):
        self.y_data[varname] = []
        self.y_label[varname] = varlabel

    def append_y(self, varname, value):
        self.y_data[varname].append(value)

    def plot(self):
        pylab.figure(DataPlotter.FIGURE)

        i = 0
        for varname in self.y_label:
            pylab.plot(self.x_data, self.y_data[varname], self.__options[i], label=self.y_label[varname])
            i += 1
        pylab.xlabel(self.x_label)
        pylab.legend()
        pylab.show()
        DataPlotter.FIGURE += 1


if __name__ == "__main__":

    import math

    d = DataPlotter()
    d.set_x("time")
    d.add_y("sin", "Sin(t)")
    d.add_y("cos", "cos(t)")

    t = 0
    while t < 10:
        d.append_x(t)
        d.append_y("sin", math.sin(t))
        d.append_y("cos", math.cos(t))
        t += 0.01

    d.plot()

