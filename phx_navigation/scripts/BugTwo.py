import numpy as np
import rospy
import tf
import tf2_py
import tf2_ros
from sensor_msgs.msg import LaserScan
import geometry_msgs.msg

"""
The BugTwo algorithm, which generates a path around an object without knowing the whole map.
It's not finished, so don't use it with other classes!
"""
class BugTwo():
    def __init__(self):
        rospy.init.node('BugTwo')
        self.rate = 10
        self.line_of_sight = True
        self.copter_pos = np.zeroes[3]  # starposition x,y&ź
        self.copter_tar = np.zeroes[3]  # zielposition x,y&z
        self.copter_rot = np.zeroes[3]  # startrotation
        self.current_state = np.zeroes[6]  # zusammenfassung aktuelle pos, aktuelle rot

        # beim vorbeifliegen wird das object erst ab 80° ignoriert - sonst evtl problematisch
        self.relevant_angle = 160 / 2
        # ueber dieser Distanz wird das Hindernis ganz ignoriert
        self.relevant_distance = 20
        # unter dieser Distanz wird das Hindernis fuer Colision Avoidance relevant
        self.dangerous_distance = 3

        self.vektor = [0, 0]


    def get_target(self, clicked_point):
        #TODO: change target funktion
        #TODO: listen to target topic
        self.copter_tar[0] = clicked_point.x
        self.copter_tar[1] = clicked_point.y
        self.copter_tar[2] = clicked_point.z

        self.line_to_target(self.copter_pos, self.copter_tar)

        #TODO: Return missing


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
        return self.current_pos     #TODO: Will throw error if ConnectivityException occures
    """
    Generates line from startpoint to target
    """
    def line_to_target(self):
        if not self.copter_pos == self.copter_tar:
            #TODO: Rename the variables to show what they are doing (I honestly don't know what m and t are standing for)
            if self.copter_tar[0] != 0 & self.copter_pos[0] != 0 & self.copter_tar[0] != self.copter_pos[0]:
                m = (self.copter_tar[1] - self.copter_pos[1]) / (self.copter_tar[0] - self.copter_pos[0])

            t = m * self.copter_tar[0] - self.copter_tar[1]
            line = [m, t]
            # m(x) = a*x+b
            return line
            #TODO: position hold aufrufen?

    """
    Takes Laser Scans and Orientation and calculates potential obstacle
    """
    def callback_find_obstacle(self, new_LaserScan=LaserScan()):
        data = np.array(new_LaserScan.ranges)
        obstacle = np.zeroes_like(data)

        # in meter
        # relevant_distance = z. B. 20
        #dangerous_distance = z.B. 3

        obstacle[data > self.relevant_distance] = 0
        obstacle[data <= self.relevant_distance & data >= self.dangerous_distance] = 0.5
        obstacle[data < self.dangerous_distance] = 1.


        angle = abs(LaserScan.angle_min) + abs(LaserScan.angle_max)

        # removes all measurements outside of a relevant angle
        data = data[(abs(LaserScan.angle_min) - self.relevant_angle) / angle * len(new_LaserScan): (abs(
            LaserScan.angel_max) - self.relevant_angle) / angle * len(new_LaserScan)]

        # if a obsctacel is to close change loop to 'Follow the Wall'
        if max(obstacle) == 1:
            self.line_of_sight = False
    """
    Calculates a tangential to the obstacle
    """
    def tangente(self, angle_min, angle_max, Array):
        #TODO: LaserScan durch Array mit Hinderniskoordinaten austauschen

        data = np.array(Array)
        length = len(data)
        vektor = [0, 0]
        i = 0
        if length < 4:
            # big obstacle
            #TODO: Tangenten berechnen
            angle_avg = (angle_min + angle_max) / 2
            vektor[0] = np.cos(angle_avg - 45 + 90)
            vektor[1] = np.sine(angle_avg - 45 + 90)

        else:
            #TODO: Alle Messwerte aufsummieren & anders umfliegen
            # small obstacle

            while i < length:
                sum = sum + data[i]
                i += 1
            avg = sum / length  # avg distance to obstacle

            #TODO: vektor durch twist erstetzen
            #TODO: vektorteil des twists in abhaengigkeit von winkel
            # +-40° reichen fuer Sicherheitsabstand von 1m bei Groeße der Drohne von 1.5m
            # zu einem Hindernis von Durchmesser ~40cm

            if (angle_min < 180 - 45 & angle_max < 180 - 45):
                vektor = []  # right
            else:
                if (angle_min > 180 - 45 & angle_max > 180 - 45):
                    vektor = []  # left
                else:
                    # object is straight ahead

                    if abs(angle_min - 135) < abs(angle_max - 135):
                        # obstacle is more to the right
                        vektor = []  # right
                    else:
                        # obstacle is more to the left
                        vektor = [] #left


    def run(self):
        #TODO

        #subscibing to relevant topics
        self.ros_subscribe_target_position = rospy.Subscriber('/clicked_point', geometry_msgs.msg.PointStamped,
                                                              self.get_target)
        self.ros_subscribe_LaserScan = rospy.Subscriber('/scan_filtered', LaserScan, self.callback_find_obstacle)

        #Legt alle wichtigen Starteinstellungen fest
        self.get_target()
        self.get_current_pos()
        self.get_parameters()

        while not rospy.is_shutdown():

            # loop 'Move in straight line'
            if self.line_of_sight:
                # todo vektor durch twist ersetzen
                vektor = [0,1]


            # loop 'Follow the Wall'
            else:
                #TODO
                pass

            rospy.sleep(10)
