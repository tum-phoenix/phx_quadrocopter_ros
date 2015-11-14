__author__ = 'manuelviermetz'
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph.opengl as gl
import pyqtgraph as pg
import numpy as np
import time


class BoxWorld3D:
    """
    This class builds a world based on cubes or scattered points. the world is sampled in 0.1 samples.
    """
    def __init__(self, sampling=0.1, world_shape=(200, 200, 200), widget=None):
        """
        If there is no widget given a new window is created and its id is accessible via self.widget.
        During init also the sampling can be specified which is 0.1 by default.
        :param sampling: optional, float
        :param world_shape: optional, 3d tupel
        :param widget: optional, gl.GLViewWidget
        :return: None
        """
        if not widget:
            self.app = QtGui.QApplication([])
            self.widget = gl.GLViewWidget()
            self.widget.opts['distance'] = 40
            self.widget.show()
        else:
            self.app = False
            self.widget = widget

        self.sampling = sampling
        self.box_item = [None] * world_shape[0] * world_shape[1] * world_shape[2]
        self.world = np.zeros(world_shape, dtype=np.uint8)  # 0: no box, 1-255: box with color

    def add_new_box(self, pos, color=255):
        if pos is False:
            return
        if pos.shape[0] == 3:
            if self.world[pos[0] + self.world.shape[0]//2, pos[1] + self.world.shape[1]//2, pos[2] + self.world.shape[2]//2] == 0:
                new_box_item = gl.GLBoxItem()
                new_box_item.setSize(x=self.sampling, y=self.sampling, z=self.sampling)
                new_box_item.translate(pos[0] - (0.5 * self.sampling),
                                       pos[1] - (0.5 * self.sampling),
                                       pos[2] - (0.5 * self.sampling))
                new_box_item.setColor((255, 255, 255, color))
                self.box_item[pos[0] + pos[1] + pos[2]] = new_box_item
                self.widget.addItem(new_box_item)

    def delete_box(self, pos):
        self.widget.removeItem(self.box_item[pos[0] + pos[1] + pos[2]])
        temp = self.box_item[pos[0] + pos[1] + pos[2]]
        self.box_item[pos[0] + pos[1] + pos[2]] = None
        del(temp)

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        if self.app != False:
            self.app.exec_()
        else:
            print 'THIS IS NOT THE CORRECT OBJECT TO RUN THE APP!'


class Plotter3D:
    """
    This class just plots points of a given color to the specified coordinates.
    """
    def __init__(self, point_size=3.0, color=(255, 255, 0, 255), widget=None):
        """
        If there is no widget given a new window is created and its id is accessible via self.widget.
        :param point_size: optional, float
        :param color: optional, 4d tupel RGBA
        :param widget: optional, gl.GLViewWidget
        :return: None
        """
        if not widget:
            self.app = QtGui.QApplication([])
            self.widget = gl.GLViewWidget()
            self.widget.opts['distance'] = 40
            self.widget.show()
        else:
            self.app = False
            self.widget = widget

        self.coords_x = []
        self.coords_y = []
        self.coords_z = []
        self.color = color
        self.point_size = point_size
        pts = np.vstack([self.coords_x, self.coords_y, self.coords_z]).transpose()
        self.scatter_plot = gl.GLScatterPlotItem(pos=pts, color=self.color, size=self.point_size)
        self.widget.addItem(self.scatter_plot)

    def set_data(self, x, y, z, color=None):
        if color is not None:
            self.color = color
        self.coords_x = x
        self.coords_y = y
        self.coords_z = z
        self.update()

    def add_point(self, x, y, z):
        self.coords_x.append(x)
        self.coords_y.append(y)
        self.coords_z.append(z)

    def update(self):
        pts = np.vstack([self.coords_x, self.coords_y, self.coords_z]).transpose()
        self.scatter_plot.setData(pos=pts, color=self.color, size=self.point_size)

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        if self.app != False:
            self.app.exec_()
        else:
            print 'THIS IS NOT THE CORRECT OBJECT TO RUN THE APP!'


class Axes3D:
    """
    This class just adds the three axes to the 3d rendering.
    """
    def __init__(self, x=0, y=0, z=0, scale=0.1, widget=None):
        """
        If there is no widget given a new window is created and its id is accessible via self.widget.
        :param x: optional, float
        :param y: optional, float
        :param z: optional, float
        :param widget: optional, gl.GLViewWidget
        :return: None
        """
        if not widget:
            self.app = QtGui.QApplication([])
            self.widget = gl.GLViewWidget()
            self.widget.opts['distance'] = 40
            self.widget.show()
        else:
            self.app = False
            self.widget = widget

        self.scale = scale
        self.x = x
        self.y = y
        self.z = z
        self.rot0 = 0
        self.rot1 = 0
        self.rot2 = 0

        self.axis = gl.GLAxisItem()
        tr = pg.Transform3D()
        tr.scale(self.scale)
        tr.rotate(self.rot0, 1, 0, 0)
        tr.rotate(self.rot1, 0, 1, 0)
        tr.rotate(self.rot2, 0, 0, 1)
        self.axis.setTransform(tr)
        self.axis.translate(self.x, self.y, self.z)
        self.widget.addItem(self.axis)

    def update(self, x=None, y=None, z=None, rot0=None, rot1=None, rot2=None):
        if x:
            self.x = x
        if y:
            self.y = y
        if z:
            self.z = z
        if rot0:
            self.rot0 = rot0
        if rot1:
            self.rot1 = rot1
        if rot2:
            self.rot2 = rot2

        tr = pg.Transform3D()
        tr.scale(self.scale)
        tr.rotate(self.rot0, 1, 0, 0)
        tr.rotate(self.rot1, 0, 1, 0)
        tr.rotate(self.rot2, 0, 0, 1)
        self.axis.setTransform(tr)
        self.axis.translate(self.x, self.y, self.z)

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        if self.app != False:
            self.app.exec_()
        else:
            print 'THIS IS NOT THE CORRECT OBJECT TO RUN THE APP!'


class Grid3D:
    """
    This class just adds a horizontal grid to the 3d rendering.
    """
    def __init__(self, x=0, y=0, z=0, scale=0.1, widget=None):
        """
        If there is no widget given a new window is created and its id is accessible via self.widget.
        :param x: optional, float
        :param y: optional, float
        :param z: optional, float
        :param widget: optional, gl.GLViewWidget
        :return: None
        """
        if not widget:
            self.app = QtGui.QApplication([])
            self.widget = gl.GLViewWidget()
            self.widget.opts['distance'] = 40
            self.widget.show()
        else:
            self.app = False
            self.widget = widget

        self.scale = scale
        self.x = x
        self.y = y
        self.z = z
        self.rot0 = 0
        self.rot1 = 0
        self.rot2 = 0

        self.grid = gl.GLGridItem()
        self.grid.translate(self.x, self.y, self.z)
        self.widget.addItem(self.grid)

    def update(self, x=None, y=None, z=None, scale=None):
        if x:
            self.x = x
        if y:
            self.y = y
        if z:
            self.z = z
        if scale:
            self.scale = scale
        tr = pg.Transform3D()
        tr.scale(self.scale)
        self.grid.setTransform(tr)
        self.grid.translate(self.x, self.y, self.z)

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        if self.app != False:
            self.app.exec_()
        else:
            print 'THIS IS NOT THE CORRECT OBJECT TO RUN THE APP!'


class LinePlot2D:
    def __init__(self, color=(255, 0, 255), widget=None, app=None):
        if not widget and not app:
            self.app = QtGui.QApplication([])
            self.widget = pg.PlotWidget()
            self.widget.show()
        elif not app:
            self.app = False
            self.widget = widget
        else:
            self.app = app
            self.widget = pg.PlotWidget()
            self.widget.show()

        self.plot = self.widget.plotItem.plot()
        self.plot.setPen(pg.mkPen(color=color))

    def setData(self, y, x=None):
        if not isinstance(x, np.ndarray) and not isinstance(y, np.ndarray):
            return
        elif not isinstance(x, np.ndarray):
            x = np.linspace(0, y.shape[0], y.shape[0])
        self.plot.setData(x, y)

    def run(self):
        """
        Starts the visualisation
        :return: None
        """
        if self.app != False:
            self.app.exec_()
        else:
            print 'THIS IS NOT THE CORRECT OBJECT TO RUN THE APP!'


if __name__ == '__main__':
    axes = Axes3D()

    grid = Grid3D(widget=axes.widget)

    scatter = Plotter3D(widget=axes.widget)

    boxes = BoxWorld3D(widget=axes.widget)
    boxes.add_new_box(pos=np.array((-2,-2,-2)))
    boxes.add_new_box(pos=np.array((-2,2,-2)))
    boxes.add_new_box(pos=np.array((2,2,-2)))
    boxes.add_new_box(pos=np.array((2,-2,-2)))

    line = LinePlot2D(app=axes.app)
    line.setData(np.random.rand((20)))

    def main_loop():
        scatter.add_point(np.sin(1.5*time.time())*5, np.cos(1.2*time.time())*5, np.sin(15*time.time())+1)
        scatter.update()
        axes.update(z=np.sin(time.time()), rot2=180.*np.sin(time.time()))
        pass
    timer = QtCore.QTimer()
    timer.timeout.connect(main_loop)
    timer.start(10)

    axes.run()

    timer.stop()
