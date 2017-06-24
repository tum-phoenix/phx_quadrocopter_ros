import rospy
import numpy as np
import gmplot

from sensor_msgs.msg import NavSatFix

class GPSPlot:
    def __init__(self):
        rospy.init_node('gps_plotter')
        self.ros_sub_gps_position = rospy.Subscriber('/phx/gps', NavSatFix, self.callback_gps_position)

        self.frequency = 10
        self.rate_timer = rospy.Rate(self.frequency)

    def callback_gps_position(self, gps_input=NavSatFix()):
        lon = [48.1374, 48.1555]
        lat = [11.575, 11.579]
        lon.append(gps_input.longitude)
        lat.append(gps_input.latitude)
        alt = gps_input.altitude

        print "Longitude: ", gps_input.longitude, "\tLatitude: ", gps_input.latitude, "\tAltitude: ", gps_input.altitude

        gmap = gmplot.GoogleMapPlotter(48.1374, 11.575, 13)
        gmap.plot(lon, lat, 'cornflowerblue', edge_width=10)
        gmap.scatter(lon, lat, 'm', marker=True)
        gmap.draw("mymap.html")

    def run(self):
        while not rospy.is_shutdown():
            self.rate_timer.sleep()

if __name__ == '__main__':
    controller = GPSPlot()
    controller.run()
