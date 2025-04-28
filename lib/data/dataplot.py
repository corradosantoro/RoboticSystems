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


def plot_multiple(plotters, figsize=(20, 18)):
    """
    Plots multiple DataPlotter objects as subplots in a single figure
    :param plotters: list of DataPlotter objects
    :param figsize: tuple (width, height) of the figure
    """
    num_plots = len(plotters)
    pylab.figure(DataPlotter.FIGURE, figsize=figsize)

    for idx, dp in enumerate(plotters, 1):
        pylab.subplot(num_plots, 1, idx)
        i = 0
        for varname in dp.y_label:
            pylab.plot(dp.x_data, dp.y_data[varname],
                       dp._DataPlotter__options[i % len(dp._DataPlotter__options)],
                       label=dp.y_label[varname])
            i += 1
        pylab.xlabel(dp.x_label)
        pylab.legend()

    pylab.tight_layout()
    pylab.show()
    DataPlotter.FIGURE += 1



if __name__ == "__main__":
    import math

    d1 = DataPlotter()
    d1.set_x("time")
    d1.add_y("sin", "Sin(t)")
    t = 0
    while t < 10:
        d1.append_x(t)
        d1.append_y("sin", math.sin(t))
        t += 0.01

    d2 = DataPlotter()
    d2.set_x("time")
    d2.add_y("cos", "Cos(t)")
    t = 0
    while t < 10:
        d2.append_x(t)
        d2.append_y("cos", math.cos(t))
        t += 0.01

    plot_multiple([d1, d2], figsize=(10, 8))

