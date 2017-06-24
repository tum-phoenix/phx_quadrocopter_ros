from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np
import time


class PlaneRadialPlotter3D:
    def __init__(self, data_shape=1024, color=(1.0, 0.0, 0.0, 1.0), start_angle=-3*np.pi/4., stop_angle=3*np.pi/4., widget=None):
        if not widget:
            self.app = QtGui.QApplication([])
            self.widget = gl.GLViewWidget()
            self.widget.opts['distance'] = 40
            self.widget.show()
        else:
            self.widget = widget

        self.axis = gl.GLAxisItem()
        self.axis.translate(0, 0, 1)
        self.widget.addItem(self.axis)

        self.gz = gl.GLGridItem()
        self.widget.addItem(self.gz)

        radius = np.ones((data_shape)) * 5
        self.start_angle = start_angle
        self.stop_angle = stop_angle
        self.angle = np.linspace(self.start_angle, self.stop_angle, data_shape)
        x = np.sin(self.angle) * radius
        y = np.cos(self.angle) * radius
        z = np.ones((data_shape))
        pts = np.vstack([x, y, z]).transpose()
        # self.line_plot = gl.GLLinePlotItem(pos=pts, color=np.array(color * data_shape).reshape((data_shape, 3)))
        self.line_plot = gl.GLScatterPlotItem(pos=pts, color=np.array(color * data_shape).reshape((data_shape, 4)), size=3.0)
        self.widget.addItem(self.line_plot)

    def set_data(self, data, color=(100, 100, 200)):
        data_shape = data.shape[0]
        self.angle = np.linspace(self.start_angle, self.stop_angle, data_shape)
        x = np.sin(self.angle) * data
        y = np.cos(self.angle) * data
        z = np.ones((data_shape))
        pts = np.vstack([x, y, z]).transpose()
        self.line_plot.setData(pos=pts, color=np.array(color * data_shape).reshape((data_shape, 3)), size=3.0)

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        self.app.exec_()


class BoxPlotter3D:
    def __init__(self, pos=None, color=(100, 100, 200), data_shape=1000, widget=None):
        if not widget:
            self.app = QtGui.QApplication([])
            self.widget = gl.GLViewWidget()
            self.widget.opts['distance'] = 40
            self.widget.show()
        else:
            self.widget = widget

        self.axis = gl.GLAxisItem()
        self.widget.addItem(self.axis)

        self.box_item_list = []

        if pos is None:
            x = (np.random.rand((data_shape))-0.5) * 20
            y = (np.random.rand((data_shape))-0.5) * 20
            z = (np.random.rand((data_shape))-0.5) * 20
            pos = np.vstack([x, y, z]).transpose()
        self.add_new_box(pos)

        surface_plot = gl.GLSurfacePlotItem()
        colors = np.array((((0.5, 0.5, 0.5, 0.25), (0.5, 0.5, 1, 0.25)), ((1, 0.5, 0.5, 0.25), (0.5, 1, 0.5, 0.25))))
        surface_plot.setData(x=np.array((-10, 10)), y=np.array((-10, 10)), z=np.array(((0, 0), (0, 0))), colors=colors)
        self.widget.addItem(surface_plot)

    def add_new_box(self, pos=False, color=np.array((255, 255, 255)), size=0.25):
        if pos is False:
            return
        if pos.shape[0] == 3:
            new_box_item = gl.GLBoxItem()
            new_box_item.setSize(x=size, y=size, z=size)
            new_box_item.translate(pos[0], pos[1], pos[2])

            self.box_item_list.append(new_box_item)
            self.widget.addItem(new_box_item)
        else:
            for new in pos:
                self.add_new_box(pos=new, color=color, size=size)

    def delete_boxes(self):
        for box_id in range(0, len(self.box_item_list)):
            self.widget.removeItem(self.box_item_list[0])
            del(self.box_item_list[0])

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        self.app.exec_()


if __name__ == '__main__':
    ## fancy 3D line plot with log
    # window = SlidingLinePlotter3D(data_shape=681)
    #
    # def main_loop():
    #     y = np.linspace(-10, 10, 681)
    #     new_data = np.sin(2. * np.pi / 25 * y + 2. * np.pi * time.time() / 2)
    #     window.add_new_data(new_data, color=(100, 100, 255 * (time.time() - int(time.time())), 255))
    #
    # timer = QtCore.QTimer()
    # timer.timeout.connect(main_loop)
    # timer.start(50)
    #
    # window.run()
    #
    # timer.stop()

    # not so fancy but use full 3D plot of one scan using scatters
    #
    window = BoxPlotter3D(data_shape=100)

    window2 = PlaneRadialPlotter3D(data_shape=100, widget=window.widget)

    def main_loop():
        data_shape = 50
        angle = np.linspace(-np.pi, np.pi, data_shape)
        radius = (np.sin(2. * angle + 2. * np.pi * time.time() / 2) + 2) * 3

        window2.set_data(radius, color=(100, 100, 255 * (time.time() - int(time.time()))))

        x = np.sin(angle) * radius
        y = np.cos(angle) * radius
        z = np.ones((data_shape)) * np.sin(np.pi * time.time())
        new_data = np.vstack([x, y, z]).transpose()
        window.delete_boxes()
        window.add_new_box(new_data, color=(100, 100, 255 * abs(time.time() - int(time.time()))))


    timer = QtCore.QTimer()
    timer.timeout.connect(main_loop)
    timer.start(10)

    #main_loop()

    window.run()

    timer.stop()
