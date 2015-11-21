#!/usr/bin/env python
import rospy

from phx_arduino_uart_bridge.msg import WayPoint
from phx_arduino_uart_bridge.msg import WayPoints
from phx_arduino_uart_bridge.msg import Management
from sensor_msgs.msg import NavSatFix

import numpy as np
import time


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


class GPWWayPointNode:
    def __init__(self):
        # init ros
        rospy.init_node('way_point_controller')

        self.ros_sub_management = rospy.Subscriber('/phx/management', Management, self.callback_management)

        self.ros_sub_gps_position = rospy.Subscriber('/phx/gps', NavSatFix, self.callback_gps_position)
        self.ros_pub_gps_way_point = rospy.Publisher('/phx/gps_way_point', NavSatFix, queue_size=1)

        self.ros_sub_way_point_add = rospy.Subscriber('/phx/way_points/add', WayPoint, self.callback_add_way_point)
        self.ros_sub_way_point_remove = rospy.Subscriber('/phx/way_points/remove', WayPoint, self.callback_remove_way_point)
        self.ros_pub_way_points = rospy.Publisher('/phx/way_points/path', WayPoints, queue_size=1)
        self.current_way_points = []

        self.target_reached_radius = 5
        self.target_reached_time = None
        self.active_directions = False
        self.active_planing = False
        self.frequency = 10                 # Hz

        self.rate_timer = rospy.Rate(self.frequency)
        self.loop_counter = 0

    def callback_management(self, management_input=Management()):
        """
        This is the ros callback for an incoming management message:
            if input is 0 -> no change
            if input is 1 -> standby
            if input is 2 -> initialize
            if input is 3 -> run
            if input is 4 -> stop
            if input is 9 -> abort
            if input is 10 -> kill
        :param management_input:
        :return:
        """
        if management_input.gps_way_point_controller == 1:      # standby
            self.active_directions = False
            self.active_planing = False
        elif management_input.gps_way_point_controller == 2:    # initialize
            self.active_directions = False
            self.active_planing = True
        elif management_input.gps_way_point_controller == 3:    # run
            self.active_directions = True
            self.active_planing = True
        elif management_input.gps_way_point_controller == 4:    # stop
            self.active_directions = False
            self.active_planing = False
            self.current_way_points = []
        elif management_input.gps_way_point_controller == 9:    # abort
            self.active_directions = False
            self.active_planing = False
        elif management_input.gps_way_point_controller == 10:   # kill
            self.active_directions = False
            self.active_planing = False
            self.kill()

    def callback_gps_position(self, gps_input=NavSatFix()):
        lon = gps_input.longitude
        lat = gps_input.latitude
        alt = gps_input.altitude
        if len(self.current_way_points) > 0:
            if calc_geo_distance(lon0=lon,
                                 lat0=lat,
                                 lon1=self.current_way_points[0][0],
                                 lat1=self.current_way_points[0][1]) <= self.target_reached_radius:
                # Target position reached
                print 'reached target zone'
                if not self.target_reached_time:
                    self.target_reached_time = time.time()
                if time.time() - self.target_reached_time >= self.current_way_points[0][3]:
                    self.target_reached_time = None
                    self.current_way_points.pop(0)
                    self.publish_current_way_points()

    def callback_add_way_point(self, wp_input=WayPoint()):
        lon = wp_input.position.longitude
        lat = wp_input.position.latitude
        alt = wp_input.position.altitude
        number = wp_input.wp_number
        stay_time = wp_input.stay_time
        way_point = [lon, lat, alt, stay_time]
        if self.active_planing:
            if number == 0 or len(self.current_way_points) < number:
                self.current_way_points.append(way_point)
                return
            else:
                new_way_point_list = []
                for i in range(0, number):
                    new_way_point_list.append(self.current_way_points[i])
                new_way_point_list.append(way_point)
                for i in range(number, len(self.current_way_points)):
                    new_way_point_list.append(self.current_way_points[i])
                self.current_way_points = new_way_point_list

    def callback_remove_way_point(self, wp_input=WayPoint()):
        print 'removing way point', wp_input.wp_number, wp_input.stay_time
        lon = wp_input.position.longitude
        lat = wp_input.position.latitude
        alt = wp_input.position.altitude
        number = wp_input.wp_number
        stay_time = wp_input.stay_time
        way_point = [lon, lat, alt, stay_time]
        if number < len(self.current_way_points):
            print 'removing via number', number
            self.current_way_points.pop(number)

    def publish_current_way_points(self):
        way_points_msg = WayPoints()
        index = 0
        for point in self.current_way_points:
            way_point = WayPoint()
            way_point.position.longitude = point[0]
            way_point.position.latitude = point[1]
            way_point.position.altitude = point[2]
            way_point.stay_time = point[3]
            way_point.wp_number = index
            index += 1
            way_points_msg.way_points.append(way_point)
        self.ros_pub_way_points.publish(way_points_msg)

    def publish_active_way_point(self):
        if self.active_directions:
            if len(self.current_way_points) > 0:
                way_point_msg = NavSatFix()
                way_point_msg.longitude = self.current_way_points[0][0]
                way_point_msg.latitude = self.current_way_points[0][1]
                way_point_msg.altitude = self.current_way_points[0][2]
                self.ros_pub_gps_way_point.publish(way_point_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.rate_timer.sleep()
            if self.loop_counter % 5 == 0:
                self.publish_current_way_points()
                self.publish_active_way_point()
            if self.loop_counter % 20 == 0:
                print 'way points:', self.current_way_points
            self.loop_counter += 1

    def kill(self):
        self.ros_pub_gps_way_point.unregister()
        self.ros_pub_way_points.unregister()
        self.ros_sub_gps_position.unregister()
        self.ros_sub_way_point_add.unregister()
        self.ros_sub_way_point_remove.unregister()
        exit()

if __name__ == '__main__':
    controller = GPWWayPointNode()

    controller.run()
