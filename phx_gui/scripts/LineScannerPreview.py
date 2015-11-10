from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np
import time


class SlidingLinePlotter3D:
    def __init__(self, log_length=100, data_shape=1024):
        self.widget = gl.GLViewWidget()
        self.widget.opts['distance'] = 40
        self.widget.show()

        # generate grid
        # self.gx = gl.GLGridItem()
        # self.gx.rotate(90, 0, 1, 0)
        # self.widget.addItem(self.gx)
        #
        # self.gy = gl.GLGridItem()
        # self.gy.rotate(90, 1, 0, 0)
        # self.widget.addItem(self.gy)

        self.gz = gl.GLGridItem()
        self.widget.addItem(self.gz)

        self.data_shape = data_shape
        self.log_length = log_length
        self.line_plots = []
        for line in range(0, log_length):
            x = np.ones((data_shape)) * 20. / log_length * line - 10.
            y = np.linspace(-10, 10, data_shape)
            val = np.cos(y * 2. * np.pi / 5 + np.pi * line / 25.)
            pts = np.vstack([x, y, val]).transpose()
            self.line_plots.append(gl.GLLinePlotItem(pos=pts, color=pg.glColor((1, 1*1.3))))
            self.widget.addItem(self.line_plots[-1])

    def add_new_data(self, data, color=(100, 100, 200)):
        if data.shape[0] == self.data_shape:
            # delete first
            self.widget.removeItem(self.line_plots[0])
            del self.line_plots[0]
            # translate all
            for plots in self.line_plots:
                plots.translate(-20. / self.log_length, 0, 0)
            # add new value
            x = np.zeros((self.data_shape)) + 10.
            y = np.linspace(-10, 10, self.data_shape)
            val = data
            pts = np.vstack([x, y, val]).transpose()
            self.line_plots.append(gl.GLLinePlotItem(pos=pts, color=pg.glColor(color)))
            self.widget.addItem(self.line_plots[-1])
        else:
            print 'LineScannerPreview.add_new_data got wrong shaped data', data.shape[0], self.data_shape


class PlaneRadialPlotter3D:
    def __init__(self, data_shape=1024, color=(100, 100, 200)):
        self.widget = gl.GLViewWidget()
        self.widget.opts['distance'] = 40
        self.widget.show()

        # generate grid
        # self.gx = gl.GLGridItem()
        # self.gx.rotate(90, 0, 1, 0)
        # self.widget.addItem(self.gx)
        #
        # self.gy = gl.GLGridItem()
        # self.gy.rotate(90, 1, 0, 0)
        # self.widget.addItem(self.gy)

        self.gz = gl.GLGridItem()
        self.widget.addItem(self.gz)

        radius = np.ones((data_shape)) * 5
        self.angle = np.linspace(-3*np.pi/4., 3*np.pi/4., data_shape)
        x = np.sin(self.angle) * radius
        y = np.cos(self.angle) * radius
        z = np.ones((data_shape))
        pts = np.vstack([x, y, z]).transpose()
        self.line_plot = gl.GLLinePlotItem(pos=pts, color=pg.glColor(color))
        self.widget.addItem(self.line_plot)

    def add_new_data(self, data, color=(100, 100, 200)):
        data_shape = data.shape[0]
        self.angle = np.linspace(-3*np.pi/4., 3*np.pi/4., data_shape)
        x = np.sin(self.angle) * data
        y = np.cos(self.angle) * data
        z = np.ones((data_shape))
        pts = np.vstack([x, y, z]).transpose()
        self.line_plot.setData(pos=pts, color=pg.glColor(color))


class LineScannerPreview:
    def __init__(self, log_length=100, data_shape=1024):
        """
        simple visualisation of LineScanner data. Specify log_length as the
        number of data curves cached and data_shape of the scanner data.
        :param log_length:  int, number of lines
        :param data_shape:  int, data shape provided by the scanner
        :return: None
        """
        self.app = QtGui.QApplication([])
        #self.line_plotter = SlidingLinePlotter3D(log_length, data_shape)
        self.line_plotter = PlaneRadialPlotter3D(log_length, data_shape)

    def add_new_data(self, new_data, color=(100, 100, 200)):
        """
        Add a new data line to the 3D plot. new_data.shape needs to match to the
        initialized data_shape. choose color as (r, g, b) tupel
        :param new_data: np.ndarray 1D
        :param color:    0-255, (r, g, b) tupel
        :return: None
        """
        self.line_plotter.add_new_data(new_data, color)

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        self.app.exec_()


if __name__ == '__main__':
    window = LineScannerPreview(100)

    def main_loop():
        y = np.linspace(-10, 10, 1024)
        new_data = np.sin(2. * np.pi / 25 * y + 2. * np.pi * time.time() / 2)
        window.add_new_data(new_data, color=(100, 100, 255 * (time.time() - int(time.time()))))

    timer = QtCore.QTimer()
    timer.timeout.connect(main_loop)
    timer.start(50)

    window.run()

    timer.stop()
