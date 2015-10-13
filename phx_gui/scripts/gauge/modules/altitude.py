import pyqtgraph
import numpy as np
from PyQt4 import QtGui
import matplotlib.pyplot as plt


class AltitudeTab:
    def __init__(self, graphicsView_altitude):
        self.graphicsView_altitude = graphicsView_altitude
        self.record_altitude = 1000
        self.altitude_dataset_index = {'fc_barometer': 0,
                                        'fc_gps': 1,
                                        'marvic_ir': 2,
                                        'marvic_lidar': 3,
                                        'marvic_baro': 4,
                                        'marvic_sonar': 5,
                                        'marvic_fused': 6}
        self.altitude_dataset = np.zeros((self.record_altitude, len(self.altitude_dataset_index), 2))
        
        # create plots in different colors
        # naze baro - lila
        self.plot_fc_barometer = self.graphicsView_altitude.plotItem.plot()
        self.plot_fc_barometer.setPen(pyqtgraph.mkPen(color=(200, 0, 200)))
        # naze gps - red
        self.plot_fc_gps = self.graphicsView_altitude.plotItem.plot()
        self.plot_fc_gps.setPen(pyqtgraph.mkPen(color=(200, 0, 0)))
        # lidar - blue
        self.plot_marvic_lidar = self.graphicsView_altitude.plotItem.plot()
        self.plot_marvic_lidar.setPen(pyqtgraph.mkPen(color=(0, 0, 200)))
        # ir - green
        self.plot_marvic_infra_red = self.graphicsView_altitude.plotItem.plot()
        self.plot_marvic_infra_red.setPen(pyqtgraph.mkPen(color=(0, 200, 0)))
        # fused - grey
        self.plot_marvic_fused = self.graphicsView_altitude.plotItem.plot()
        self.plot_marvic_fused.setPen(pyqtgraph.mkPen(color=(100, 100, 100)))
    
    def update_altitude_plot(self):
        cur_time = self.altitude_dataset[:, :, 1].max()
        self.plot_fc_barometer.setData(self.altitude_dataset[:, self.altitude_dataset_index['fc_barometer'], 1] - cur_time,
                                       self.altitude_dataset[:, self.altitude_dataset_index['fc_barometer'], 0])
        self.plot_fc_gps.setData(self.altitude_dataset[:, self.altitude_dataset_index['fc_gps'], 1] - cur_time,
                                 self.altitude_dataset[:, self.altitude_dataset_index['fc_gps'], 0])
        self.plot_marvic_infra_red.setData(self.altitude_dataset[:, self.altitude_dataset_index['marvic_ir'], 1] - cur_time,
                                           self.altitude_dataset[:, self.altitude_dataset_index['marvic_ir'], 0])
        self.plot_marvic_lidar.setData(self.altitude_dataset[:, self.altitude_dataset_index['marvic_lidar'], 1] - cur_time,
                                       self.altitude_dataset[:, self.altitude_dataset_index['marvic_lidar'], 0])
        self.plot_marvic_fused.setData(self.altitude_dataset[:, self.altitude_dataset_index['marvic_fused'], 1] - cur_time,
                                       self.altitude_dataset[:, self.altitude_dataset_index['marvic_fused'], 0])

