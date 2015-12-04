from LineScannerPreview import PlaneRadialPlotter3D
from pyvis import Plotter3D, Axes3D, Grid3D, PlaneRadialPlotter3D
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from phx_uart_msp_bridge.msg import Attitude


class hokuyo3D:
    def __init__(self):

        self.current_pose = [0,0,0]
        # create first 3D window with a PlaneRadialPlotter3D
        self.radial_plot = PlaneRadialPlotter3D(start_angle=-2.08621382713, stop_angle=2.08621382713)
        self.radial_plot_ax = Axes3D(widget=self.radial_plot.widget)            # add a axis to the window of the PlaneRadialPlotter3D
        self.radial_plot_grid = Grid3D(widget=self.radial_plot.widget)          # add a grid to the window of the PlaneRadialPlotter3D
        # create second 3D window with a Plotter3D
        self.world_scatter = Plotter3D(app=self.radial_plot.app)                # here we have to specify the 3D app we are using. Only one app per runtime!!!
        self.axes = Axes3D(widget=self.world_scatter.widget)                    # again add a axis
        self.grid = Grid3D(widget=self.world_scatter.widget)                    # and a grid

        rospy.init_node('hokuyo_test')
        self.ros_subscribe_LaserScan = rospy.Subscriber('/scan', LaserScan, self.callback_LaserScan)
        self.ros_subscribe_imu = rospy.Subscriber('/phx/fc/attitude', Attitude, self.callback_Attitude)

        self.radial_plot.run()                                                  # to view all 3D windows we need to run the first 3D object we initialized
        #self.world_scatter.run()

    def callback_LaserScan(self, new_LaserScan=LaserScan()):
        print len(new_LaserScan.ranges), new_LaserScan.angle_min, new_LaserScan.angle_max
        print self.current_pose
        data = np.array(new_LaserScan.ranges)[::-1]
        data[data > 6] = 6

        self.radial_plot.set_data(data=data)                                    # display input data in the radial_plot 3D window

        angle = np.linspace(new_LaserScan.angle_min, new_LaserScan.angle_max, len(new_LaserScan.ranges))
        distance_x = np.sin(angle) * new_LaserScan.ranges
        distance_y = np.cos(angle) * new_LaserScan.ranges
        distance_z = np.zeros(len(new_LaserScan.ranges))                        #2D-radial plot

        pitch, roll, yaw = np.array(self.current_pose) / 180 * np.pi
        roll = -roll

        rotation_y = np.array([[np.cos(roll), 0, -np.sin(roll)],
                             [0, 1, 0],
                             [np.sin(roll), 0, np.cos(roll)]])

        rotation_x = np.array([[1, 0, 0],
                            [0, np.cos(pitch), np.sin(pitch)],
                             [0, -np.sin(pitch), np.cos(pitch)]])

        rotation_z = np.array([[np.cos(yaw), np.sin(yaw),0],
                             [-np.sin(yaw), np.cos(yaw),0],
                               [0, 0, 1]])


        points = np.vstack([distance_x, distance_y, distance_z])
        points= rotation_y.dot(points)
        points= rotation_x.dot(points)
        points= rotation_z.dot(points)
        self.world_scatter.add_point(points[0].tolist(), points[1].tolist(), points[2].tolist())                 #add coordinates to world_scatter window
        self.world_scatter.update()

    def callback_Attitude(self, new_attitude=Attitude()):

        #print new_imu.linear_acceleration.x, new_imu.linear_acceleration.y, new_imu.linear_acceleration.z
        self.current_pose = [new_attitude.pitch, new_attitude.roll, new_attitude.yaw]

H = hokuyo3D()
