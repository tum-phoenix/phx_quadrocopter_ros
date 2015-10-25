import pyqtgraph
import numpy as np
from PyQt4 import QtGui
import matplotlib.pyplot as plt

class RCtab:
    def __init__(self, graphicsView_rc, sampling_length=600, sliders=(None, None, None, None, None, None, None, None)):
        self.sampling_length = sampling_length
        self.rc_data = np.zeros((self.sampling_length, 8), dtype=np.uint16) + 1000
        self.sliders = sliders

        # create plots in different colors
        self.plot_pitch = graphicsView_rc.plotItem.plot()
        self.plot_pitch.setPen(pyqtgraph.mkPen(color=(200, 0, 100)))
        self.plot_roll = graphicsView_rc.plotItem.plot()
        self.plot_roll.setPen(pyqtgraph.mkPen(color=(255, 0, 0)))
        self.plot_yaw = graphicsView_rc.plotItem.plot()
        self.plot_yaw.setPen(pyqtgraph.mkPen(color=(0, 0, 200)))
        self.plot_throttle = graphicsView_rc.plotItem.plot()
        self.plot_throttle.setPen(pyqtgraph.mkPen(color=(0, 200, 0)))

    def update_rc(self, update_plot=True):
        self.sliders[0].setValue(self.rc_data[-1, 0])
        self.sliders[1].setValue(self.rc_data[-1, 1])
        self.sliders[2].setValue(self.rc_data[-1, 2])
        self.sliders[3].setValue(self.rc_data[-1, 3])
        self.sliders[4].setValue(self.rc_data[-1, 4])
        self.sliders[5].setValue(self.rc_data[-1, 5])
        self.sliders[6].setValue(self.rc_data[-1, 6])
        self.sliders[7].setValue(self.rc_data[-1, 7])

        if update_plot:
            x_axis = np.arange(0, self.rc_data.shape[0])
            self.plot_pitch.setData(x_axis, self.rc_data[:, 0])
            self.plot_roll.setData(x_axis, self.rc_data[:, 1])
            self.plot_yaw.setData(x_axis, self.rc_data[:, 2])
            self.plot_throttle.setData(x_axis, self.rc_data[:, 3])