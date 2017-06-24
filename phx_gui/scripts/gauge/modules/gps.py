import pyqtgraph
import numpy as np
from PyQt4 import QtGui
import matplotlib.pyplot as plt
from goompy import GooMPy
import sys

def calc_geo_distance(lon0, lat0, lon1, lat1):
    earth_radius = 6371000                          # meter
    lat_0 = 2. * np.pi * (lat0 / 360.)              # rad
    lat_1 = 2. * np.pi * (lat1 / 360.)              # rad
    d_phi = 2. * np.pi * ((lat1 - lat0) / 360.)     # rad
    d_lamda = 2. * np.pi * ((lon1 - lon0) / 360.)   # rad
    a = np.sin(d_phi/2) * np.sin(d_phi / 2) + np.cos(lat_0) * np.cos(lat_1) * np.sin(d_lamda / 2) * np.sin(d_lamda / 2)
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    d = earth_radius * c                            # meter
    return d


def generate_geo_circle(est_lon, est_lat, diameter):
    # diameter in meter
    approximate_scale_lon_to_meter = calc_geo_distance(est_lon, est_lat, est_lon + 0.00001, est_lat) / 0.00001
    approximate_scale_lat_to_meter = calc_geo_distance(est_lon, est_lat, est_lon, est_lat + 0.00001) / 0.00001
    diameter_x = diameter / approximate_scale_lon_to_meter
    diameter_y = diameter / approximate_scale_lat_to_meter
    t = np.linspace(0, 2 * np.pi, 20)
    x = np.sin(t) * 0.5 * diameter_x
    y = np.cos(t) * 0.5 * diameter_y
    return [x, y]


class GPStab:
    def __init__(self,
                 graphicsView_gps,
                 text_boxes=(None, None, None),
                 mouse_click_callback=None,
                 mouse_move_callback=None,
                 way_point_list=None):
        self.graphicsView_gps = graphicsView_gps
        self.text_boxes = text_boxes

        # handle mouse click on map
        self.mouse_click_callback = mouse_click_callback
        self.graphicsView_gps.plotItem.scene().sigMouseClicked.connect(self.gps_plot_mouse_clicked)

        # handle mouse move on map
        self.mouse_move_callback = mouse_move_callback
        self.graphicsView_gps.plotItem.scene().sigMouseMoved.connect(self.gps_plot_mouse_moved)

        self.gps_data = [[], []]        # [[lon], [lat]]
        self.gps_geo_cycle_data = None
        self.gps_positions = {}
        self.gps_altitudes = {}

        self.way_point_path = []
        self.gps_path_scatter_plot = pyqtgraph.ScatterPlotItem()
        self.gps_path_scatter_points = {}
        self.gps_path_scatter_labels = {}
        self.graphicsView_gps.addItem(self.gps_path_scatter_plot)
        self.gps_path_line_plot = self.graphicsView_gps.plotItem.plot()
        self.gps_path_line_plot.setPen(pyqtgraph.mkPen(color=(100, 0, 0)))

        # setup graph
        self.graphicsView_gps.plotItem.showGrid(x=True, y=True, alpha=0.2)

        # create plot for gps path
        self.gps_qtgraph_plot = self.graphicsView_gps.plotItem.plot()
        self.gps_qtgraph_plot.setPen(pyqtgraph.mkPen(color=(0, 0, 100)))

        # create plot for geo circle
        self.gps_geo_cicle_qtgraph_plot = self.graphicsView_gps.plotItem.plot()
        self.gps_geo_cicle_qtgraph_plot.setPen(pyqtgraph.mkPen(color=(0, 0, 200)))

        # create scatter plot for poi
        self.gps_scatter_plot = pyqtgraph.ScatterPlotItem()
        self.gps_scatter_plot.setData(self.gps_positions.values())
        # self.gps_scatter_plot.setData([{'pos': (2, 2), 'symbol': 'o', 'pen': pyqtgraph.mkPen(color=(0, 0, 200))},..])
        self.graphicsView_gps.addItem(self.gps_scatter_plot)

        # create labels for poi
        self.gps_position_labels = {}
        # label = pyqtgraph.TextItem(text='test')
        # ui_win.graphicsView_gps.addItem(label)
        # label.setPos(11, 42)
        # ui_win.graphicsView_gps.removeItem(label)

        # create background map
        self.gps_background_image = pyqtgraph.ImageItem()
        self.graphicsView_gps.addItem(self.gps_background_image)
        self.gps_background_image.setZValue(-1)
        self.update_background_map = False

        if way_point_list:
            self.way_point_tab = WayPointTab(GPStab=self, qt_tabular_object=way_point_list)

    def init_geo_circle(self, lon, lat, diameter):
        self.gps_geo_cycle_data = generate_geo_circle(lon, lat, diameter)

    def update_gps_way_point_path(self):
        for i in range(0, len(self.way_point_path)):
            if i in self.gps_path_scatter_points.keys():
                self.gps_path_scatter_points[i]['pos'] = (self.way_point_path[i].position.longitude,
                                                          self.way_point_path[i].position.latitude)
            else:
                self.gps_path_scatter_points[i] = {'pos': (self.way_point_path[i].position.longitude,
                                                           self.way_point_path[i].position.latitude),
                                                   'symbol': 'o',
                                                   'pen': pyqtgraph.mkPen(color=(200, 10, 10))}
            if i in self.gps_path_scatter_labels.keys():
                self.gps_path_scatter_labels[i].setPos(self.way_point_path[i].position.longitude,
                                                       self.way_point_path[i].position.latitude)
            else:
                text_item = pyqtgraph.TextItem(text=str('WP%i' % self.way_point_path[i].wp_number), color=(200, 10, 10))
                self.graphicsView_gps.addItem(text_item)
                self.gps_path_scatter_labels[i] = text_item

        self.gps_path_scatter_plot.setData(self.gps_path_scatter_points.values())

        # delete unused scatter points
        if len(self.gps_path_scatter_points.keys()) > len(self.way_point_path):
            for i in range(len(self.way_point_path), len(self.gps_path_scatter_points.keys())):
                del self.gps_path_scatter_points[i]

        # delete unused scatter labels
        if len(self.gps_path_scatter_labels.keys()) > len(self.way_point_path):
            for i in range(len(self.way_point_path), len(self.gps_path_scatter_labels.keys())):
                text_item = self.gps_path_scatter_labels[i]
                self.graphicsView_gps.removeItem(text_item)
                del self.gps_path_scatter_labels[i]

        # generate lines
        line_data_x = []
        line_data_y = []
        for point in self.way_point_path:
            line_data_x.append(point.position.longitude)
            line_data_y.append(point.position.latitude)
        self.gps_path_line_plot.setData(line_data_x, line_data_y)

    def update_gps_plot(self, path=True, points=True):
        # update gps path
        if path:
            self.gps_qtgraph_plot.setData(self.gps_data[0], self.gps_data[1])

        # update gps geo circle
        if self.gps_geo_cycle_data:
            if 'phoenix' in self.gps_positions.keys():
                cur_pos_lon = self.gps_positions['phoenix']['pos'][0]
                cur_pos_lat = self.gps_positions['phoenix']['pos'][1]
                self.gps_geo_cicle_qtgraph_plot.setData(self.gps_geo_cycle_data[0] + cur_pos_lon, self.gps_geo_cycle_data[1] + cur_pos_lat)

        # update points of interest
        if points:
            self.update_gps_way_point_path()

            # update scatter plot
            self.gps_scatter_plot.setData(self.gps_positions.values())

            # update labels
            for label in self.gps_positions.keys():
                if label not in self.gps_position_labels.keys():
                    # create configuration for the new poi
                    if label == 'home':
                        color = (255, 255, 0)
                    elif label == 'phoenix':
                        color = (0, 0, 255)
                    elif label == 'way_point':
                        color = (0, 255, 0)
                    else:
                        color = (200, 255, 200)
                    text_item = pyqtgraph.TextItem(text=label, color=color)
                    self.graphicsView_gps.addItem(text_item)
                    self.gps_position_labels[label] = text_item
                self.gps_position_labels[label].setPos(self.gps_positions[label]['pos'][0],
                                                       self.gps_positions[label]['pos'][1])

            # remove unused labels
            for label in self.gps_position_labels.keys():
                if label not in self.gps_positions.keys():
                    text_item = self.gps_position_labels[label]
                    self.graphicsView_gps.removeItem(text_item)
                    del self.gps_position_labels[label]

            # update text boxes
            for label in self.gps_positions.keys():
                text_output = str('lon: %02.6f'% self.gps_positions[label]['pos'][0])
                text_output += str('\nlat: %02.6f'% self.gps_positions[label]['pos'][1])
                text_output += str('\nalt: %02.6f'% self.gps_altitudes[label])
                if label == 'home' and self.text_boxes[0]:
                    self.text_boxes[0].setText(text_output)
                elif label == 'way_point' and self.text_boxes[1]:
                    self.text_boxes[1].setText(text_output)
                elif label == 'phoenix' and self.text_boxes[2]:
                    self.text_boxes[2].setText(text_output)

    def gps_plot_mouse_clicked(self, event):
        if self.graphicsView_gps.plotItem.sceneBoundingRect().contains(event.scenePos()):
            mouse_position = self.graphicsView_gps.plotItem.mapToView(event.scenePos())
            button = event.button()         # 1: left   2:right
            x_val = mouse_position.x()
            y_val = mouse_position.y()
            print 'gps_plot_mouse_clicked', x_val, y_val, button

            if self.mouse_click_callback:
                if type(self.mouse_click_callback) == list:
                    if len(self.mouse_click_callback) > button:
                        if self.mouse_click_callback[button] != None:
                            self.mouse_click_callback[button](x_val, y_val)
                    else:
                        print 'for the clicked button', button, 'no function is defined'
                else:
                    self.mouse_click_callback(x_val, y_val)

    def gps_plot_mouse_moved(self, event):
        if self.graphicsView_gps.plotItem.sceneBoundingRect().contains(event):
            mouse_position = self.graphicsView_gps.plotItem.mapToView(event)
            x_val = mouse_position.x()
            y_val = mouse_position.y()
            #print 'gps_plot_mouse_moved', x_val, y_val

            if type(self.mouse_move_callback) == QtGui.QStatusBar:
                self.mouse_move_callback.showMessage('gps plot mouse lon: %02.6f   lat: %02.6f'%(x_val, y_val))
            elif self.mouse_move_callback:
                self.mouse_move_callback(x_val, y_val)

    def update_gps_map(self, use_map=None):
        if type(use_map) == str:
            map_file = np.load(use_map)
            gps_map_effective_size = map_file['scale']
            gps_map_array = map_file['map']
            gps_map_x_position = map_file['x']
            gps_map_y_position = map_file['y']
        elif type(use_map) == int:
            path = sys.argv[0][:-8]
            print path, path + 'maps/map' + str(use_map) + '.npz'
            map_file = np.load(path + 'maps/map' + str(use_map) + '.npz')
            gps_map_effective_size = map_file['scale']
            gps_map_array = map_file['map']
            gps_map_x_position = map_file['x']
            gps_map_y_position = map_file['y']
        elif 'phoenix' in self.gps_positions.keys():
            try:
                cur_pos_lon = self.gps_positions['phoenix']['pos'][0]
                cur_pos_lat = self.gps_positions['phoenix']['pos'][1]

                gps_map_coordinate = [cur_pos_lon, cur_pos_lat]
                gps_map_zoom = 19
                gps_map_type = 'satellite'
                gps_map_resolution = 1200
                gps_map = GooMPy(gps_map_resolution, gps_map_resolution, gps_map_coordinate[0], gps_map_coordinate[1], gps_map_zoom, gps_map_type)
                gps_map_tile = gps_map.bigimage
                gps_map_corner_upper_left = gps_map.northwest
                gps_map_corner_lower_right = gps_map.southeast
                gps_map_width = (gps_map_corner_lower_right[1] - gps_map_corner_upper_left[1]) * 4 / 3
                gps_map_height = (gps_map_corner_upper_left[0] - gps_map_corner_lower_right[0]) * 4 / 3
                gps_map_aspect_ratio = gps_map_width / gps_map_height
                gps_map_array = np.array(gps_map_tile.resize((int(gps_map_resolution * gps_map_aspect_ratio), gps_map_resolution)))
                gps_map_array = np.swapaxes(gps_map_array[::-1, :], 0, 1)
                gps_map_effective_size = gps_map_height / gps_map_resolution
                gps_map_x_position = gps_map_corner_upper_left[1] - gps_map_width / 8
                gps_map_y_position = gps_map_corner_upper_left[0] - gps_map_height + gps_map_height / 8
            except:
                pass
        else:
            return
        # update map from loaded data
        self.gps_background_image.setScale(gps_map_effective_size)
        self.gps_background_image.setImage(gps_map_array)
        self.gps_background_image.setX(gps_map_x_position)
        self.gps_background_image.setY(gps_map_y_position)
        self.gps_background_image.update()


class WayPointTab:
    def __init__(self, GPStab, qt_tabular_object):
        self.GPStab = GPStab
        self.current_way_points = self.GPStab.way_point_path
        self.qt_tabular_object = qt_tabular_object
        self.publish_way_point_remove = None

    def update_list(self):
        self.current_way_points = self.GPStab.way_point_path
        self.qt_tabular_object.setRowCount(len(self.current_way_points))
        number_of_rows = self.qt_tabular_object.rowCount()
        row = 0
        for way_point in self.current_way_points:
            if row < number_of_rows:
                if not self.qt_tabular_object.item(row, 0):
                    print self.qt_tabular_object.item(row, 0)
                    cell = QtGui.QTableWidgetItem(str(way_point.wp_number))
                    self.qt_tabular_object.setItem(row, 0, cell)
                elif self.qt_tabular_object.item(row, 0).text() != str(way_point.wp_number):
                    print self.qt_tabular_object.item(row, 0).text(), str(way_point.wp_number)
                    self.qt_tabular_object.item(row, 0).setText(str(way_point.wp_number))
                if not self.qt_tabular_object.item(row, 1):
                    cell = QtGui.QTableWidgetItem(str(way_point.stay_time))
                    self.qt_tabular_object.setItem(row, 1, cell)
                elif self.qt_tabular_object.item(row, 1).text() != str(way_point.stay_time):
                    self.qt_tabular_object.item(row, 1).setText(str(way_point.stay_time))
                if not self.qt_tabular_object.item(row, 2):
                    cell = QtGui.QTableWidgetItem(str(way_point.position.longitude))
                    self.qt_tabular_object.setItem(row, 2, cell)
                elif self.qt_tabular_object.item(row, 2).text() != str(way_point.position.longitude):
                    self.qt_tabular_object.item(row, 2).setText(str(way_point.position.longitude))
                if not self.qt_tabular_object.item(row, 3):
                    cell = QtGui.QTableWidgetItem(str(way_point.position.latitude))
                    self.qt_tabular_object.setItem(row, 3, cell)
                elif self.qt_tabular_object.item(row, 3).text() != str(way_point.position.latitude):
                    self.qt_tabular_object.item(row, 3).setText(str(way_point.position.latitude))
            row += 1

    def way_point_remove(self):
        number_of_rows = self.qt_tabular_object.rowCount()
        row = 0
        for way_point in self.current_way_points:
            if row < number_of_rows:
                for i in range(0, 4):
                    cell = self.qt_tabular_object.item(row, i)
                    if not cell:
                        continue
                    if self.qt_tabular_object.isItemSelected(cell):
                        if self.publish_way_point_remove:
                            self.publish_way_point_remove(wp_number=int(self.qt_tabular_object.item(row, 0).text()),
                                                          stay_time=int(self.qt_tabular_object.item(row, 1).text()),
                                                          lon=float(self.qt_tabular_object.item(row, 2).text()),
                                                          lat=float(self.qt_tabular_object.item(row, 3).text()))
                        else:
                            print 'no publish_way_point_remove set'
            row += 1

class MapGenerator:
    def __init__(self, tile_archive_path):
        self.tile_archive_path = tile_archive_path      # '.../maps/tiles/'
        self.archive = []     # elements: [identifier, zoom, upper_left_0, upper_left_1, lower_right_0, lower_right_1]

    def load_tile_archive(self):
        out = np.loadtxt(self.tile_archive_path + 'archive.txt')
        return out.tolist()

    def save_tile_archive(self):
        f = open(self.tile_archive_path + 'archive.txt', 'w')
        for line in self.archive:
            f.write(str(line[0])); f.write('\t')                # identifier
            f.write(str(line[1])); f.write('\t')                # zoom
            f.write(str('%2.16f' % line[2])); f.write('\t')     # upper_left_0
            f.write(str('%2.16f' % line[3])); f.write('\t')     # upper_left_1
            f.write(str('%2.16f' % line[4])); f.write('\t')     # lower_right_0
            f.write(str('%2.16f' % line[5])); f.write('\n')     # lower_right_1
        f.close()

    def check_tile_archive(self, upper_left_0, upper_left_1):
        for line in self.archive:
            if np.round(line[2], 10) == np.round(upper_left_0, 10) and np.round(line[3], 10) == np.round(upper_left_1, 10):
                return True
        return False

    def save_tile_to_archive(self, upper_left, lower_right, zoom, map):
        if len(self.archive) > 0:
            identifier = int(self.archive[-1][0]) + 1
        else:
            identifier = 0
        self.archive.append([identifier, zoom, upper_left[0], upper_left[1], lower_right[0], lower_right[1]])
        np.savez(self.tile_archive_path + 'tile_' + str('%04i' % identifier), gps_map=map, upper_left=upper_left, lower_right=lower_right)
        plt.imsave(self.tile_archive_path + 'tile_' + str('%04i.png' % identifier), map)

