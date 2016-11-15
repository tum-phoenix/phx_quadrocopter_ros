import numpy as np
import rospy
import tf
import tf2_py
import tf2_ros
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg


class BugTwo():
    def __init__(self):
        rospy.init.node('BugTwo')
        self.rate = 10
        self.line_of_sight = True
        self.copter_pos = np.zeroes[3]  # starposition x,y&ź
        self.copter_tar = np.zeroes[3]  # zielposition x,y&z
        self.copter_rot = np.zeroes[3]  # startrotation
        self.current_state = np.zeroes[6]  # zusammenfassung aktuelle pos, aktuelle rot

    def get_target(self):
        # todo
        return

    def get_current_pos(self):

        try:
            trans = self.tfBuffer.lookup_transform('map', 'footprint', rospy.Time())
            self.copter_pos[0] = trans.transform.translation.x
            self.copter_pos[1] = trans.transform.translation.y
            self.copter_pos[2] = trans.transform.translation.z
            quaternion = (trans.transform.rotation.x,
                          trans.transform.rotation.y,
                          trans.transform.rotation.z,
                          trans.transform.rotation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.copter_rot[0] = euler[0]  # pitch
            self.copter_rot[1] = euler[1]  # roll
            self.copter_rot[2] = euler[2]  # yaw
            self.current_pos = [self.copter_pos, self.copter_rot]

        except tf2_ros.ConnectivityExeption:
            pass
        return self.current_pos

    def line_to_target(copter_pos, copter_tar):
        # Generates Line l from Starpoint to Target
        if not copter_pos == copter_tar:

            if copter_tar[0] != 0 & copter_pos[0] != 0 & copter_tar[0] != copter_pos[0]:
                m = (copter_tar[1] - copter_pos[1]) / (copter_tar[0] - copter_pos[0])

            t = m * copter_tar[0] - copter_tar[1]
            l = [m, t]
            # m(x) = a*x+b
            return l

            # else return 0,0? oder position hold aufrufen?

    def callback_find_obstacle(self, new_LaserScan=LaserScan()):
        # takes Laser Scans and Orientation and calculates potential obstacle

        data = np.array(new_LaserScan.ranges)
        obstacle = np.zeroes_like(data)

        obstacle[data > 20] = 0
        obstacle[data <= 20 & data >= 10] = 0.5
        obstacle[data < 10] = 1.

        # self.current_state[6] entspricht jaw in euler

        relevant_angle = 60 / 2
        angle = abs(LaserScan.angle_min) + abs(LaserScan.angle_max)

        # entfernt alle Messwerte aus dem Array die außerhalb des relevanten Winkels liegen
        data = data[(abs(LaserScan.angle_min) - relevant_angle) / angle * len(new_LaserScan): (abs(
            LaserScan.angel_max) - relevant_angle) / angle * len(new_LaserScan)]

        # wenn Hindernis zu nah ändere den loop auf 'Follow the Wall'
        if max(obstacle) == 1:
            self.line_of_sight = False

    def run(self):
        # todo
        start = True
        self.ros_subscribe_target_position = rospy.Subscriber('/clicked_point', geometry_msgs.msg.PointStamped,
                                                              self.get_target)

        self.ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, self.callback_find_obstacle)

        self.get_target()
        self.get_current_pos()
        self.line_to_target()

        while not rospy.is_shutdown():

            # loop 'Move in straight line'
            if self.line_of_sight:
                v = 1


            # loop 'Follow the Wall'
            else:
                pass
