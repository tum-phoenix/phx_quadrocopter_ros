import rospy
from phx_arduino_uart_bridge.msg import Servo
from phx_arduino_uart_bridge.msg import LED
from phx_arduino_uart_bridge.msg import LEDstrip
from phx_arduino_uart_bridge.msg import Altitude
from phx_arduino_uart_bridge.msg import PID
from phx_arduino_uart_bridge.msg import PID_cleanflight
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image

import pyqtgraph
import numpy as np
import time


def generate_led_strip_msg(color_r, color_g, color_b):
    LEDstrip_msg = LEDstrip()
    color_r = int(color_r)
    color_g = int(color_g)
    color_b = int(color_b)
    LEDstrip_msg.led_0_r = color_r; LEDstrip_msg.led_0_g = color_g; LEDstrip_msg.led_0_b = color_b
    LEDstrip_msg.led_1_r = color_r; LEDstrip_msg.led_1_g = color_g; LEDstrip_msg.led_1_b = color_b
    LEDstrip_msg.led_2_r = color_r; LEDstrip_msg.led_2_g = color_g; LEDstrip_msg.led_2_b = color_b
    LEDstrip_msg.led_3_r = color_r; LEDstrip_msg.led_3_g = color_g; LEDstrip_msg.led_3_b = color_b
    LEDstrip_msg.led_4_r = color_r; LEDstrip_msg.led_4_g = color_g; LEDstrip_msg.led_4_b = color_b
    LEDstrip_msg.led_5_r = color_r; LEDstrip_msg.led_5_g = color_g; LEDstrip_msg.led_5_b = color_b
    LEDstrip_msg.led_6_r = color_r; LEDstrip_msg.led_6_g = color_g; LEDstrip_msg.led_6_b = color_b
    LEDstrip_msg.led_7_r = color_r; LEDstrip_msg.led_7_g = color_g; LEDstrip_msg.led_7_b = color_b
    LEDstrip_msg.led_8_r = color_r; LEDstrip_msg.led_8_g = color_g; LEDstrip_msg.led_8_b = color_b
    LEDstrip_msg.led_9_r = color_r; LEDstrip_msg.led_9_g = color_g; LEDstrip_msg.led_9_b = color_b
    return LEDstrip_msg


class ROSgauge:
    def __init__(self, gps_tab=None, altitude_tab=None, pid_tab=None, video_tab=None, led_tab=None, rc_fc_tab=None, rc_marvic_tab=None, parameter_tab=None):
        self.gps_tab = gps_tab
        self.altitude_tab = altitude_tab
        self.pid_tab = pid_tab
        self.video_tab = video_tab
        self.led_tab = led_tab
        self.rc_fc_tab = rc_fc_tab
        self.rc_marvic_tab = rc_marvic_tab
        self.parameter_tab = parameter_tab
        rospy.init_node('gauge_gui')

        # setup subscriptions and their callbacks
        if self.parameter_tab:
            self.ros_sub_cur_servo_cmd = rospy.Subscriber('/phx/marvicServo/cur_servo_cmd', Servo, self.callback_cur_servo)
        if self.gps_tab:
            self.ros_sub_gps_position = rospy.Subscriber('/phx/gps', NavSatFix, self.callback_gps_position)
            self.ros_sub_gps_way_point = rospy.Subscriber('/phx/fc/gps_way_point', NavSatFix, self.callback_gps_way_point)
            self.ros_sub_gps_home = rospy.Subscriber('/phx/fc/gps_home', NavSatFix, self.callback_gps_home)
        if self.rc_fc_tab:
            self.ros_sub_fc_rc = rospy.Subscriber('/phx/fc/rc', Joy, self.callback_fc_rc)
        if self.altitude_tab:
            self.ros_sub_fc_altitude = rospy.Subscriber('/phx/fc/altitude', Altitude, self.callback_fc_altitude)
            self.ros_sub_marvic_altitude = rospy.Subscriber('/phx/marvicAltitude/altitude', Altitude, self.callback_marvic_altitude_fused)
            self.ros_sub_marvic_lidar = rospy.Subscriber('/phx/marvicAltitude/lidar', Altitude, self.callback_marvic_altitude_lidar)
            self.ros_sub_marvic_infra_red = rospy.Subscriber('/phx/marvicAltitude/infra_red', Altitude, self.callback_marvic_altitude_infra_red)
        if self.pid_tab:
            self.ros_sub_fc_pid_in_use = rospy.Subscriber('/phx/fc/pid_in_use', PID_cleanflight, self.pid_tab.callback_fc_pid_cleanflight)
        if self.rc_marvic_tab:
            self.ros_sub_marvic_rc = rospy.Subscriber('/phx/marvicRC/rc_input', Joy, self.callback_marvic_rc)
        if self.video_tab:
            self.ros_sub_image_mono = rospy.Subscriber('/image_mono', Image, self.callback_image_mono)

        # setup publishers and their callbacks
        if self.parameter_tab:
            self.ros_pub_servo_cmd = rospy.Publisher('/phx/marvicServo/servo_cmd', Servo, queue_size=1)
        if self.gps_tab:
            self.ros_pub_gps_way_point = rospy.Publisher('/phx/gps_way_point', NavSatFix, queue_size=1)
        if self.pid_tab:
            self.ros_pub_fc_set_pid = rospy.Publisher('/phx/fc/pid_set', PID_cleanflight, queue_size=1)
        if self.led_tab:
            self.ros_pub_led_strip_0_cmd = rospy.Publisher('phx/led/led_strip_0', LEDstrip, queue_size=1)
            self.ros_pub_led_strip_1_cmd = rospy.Publisher('phx/led/led_strip_1', LEDstrip, queue_size=1)
            self.ros_pub_led_strip_2_cmd = rospy.Publisher('phx/led/led_strip_2', LEDstrip, queue_size=1)
            self.ros_pub_led_strip_3_cmd = rospy.Publisher('phx/led/led_strip_3', LEDstrip, queue_size=1)

    def shutdown_ros(self):
        if self.gps_tab:
            self.ros_sub_gps_position.unregister()
            self.ros_sub_gps_way_point.unregister()
            self.ros_sub_gps_home.unregister()
            self.ros_pub_gps_way_point.unregister()
        if self.altitude_tab:
            self.ros_sub_fc_altitude.unregister()
            self.ros_sub_marvic_altitude.unregister()
            self.ros_sub_marvic_lidar.unregister()
            self.ros_sub_marvic_infra_red.unregister()
        if self.pid_tab:
            self.ros_sub_fc_pid_in_use.unregister()
            self.ros_pub_fc_set_pid.unregister()
        if self.led_tab:
            self.ros_pub_led_strip_0_cmd.unregister()
            self.ros_pub_led_strip_1_cmd.unregister()
            self.ros_pub_led_strip_2_cmd.unregister()
            self.ros_pub_led_strip_3_cmd.unregister()
        if self.video_tab and self.ros_sub_image_mono.callback != None:
            self.ros_sub_image_mono.unregister()
        if self.rc_fc_tab:
            self.ros_sub_fc_rc.unregister()
        if self.rc_marvic_tab:
            self.ros_sub_marvic_rc.unregister()
        print 'all ros topics unregistered'

    def optimize_ros_video_subscription(self, active):
        if active and self.ros_sub_image_mono.callback != None:
            self.ros_sub_image_mono = rospy.Subscriber('/image_mono', Image, self.callback_image_mono)
        elif not active and self.ros_sub_image_mono.callback == None:
            self.ros_sub_image_mono.unregister()

    def callback_gps_home(self, cur_gps_input):
        gps_pos = (cur_gps_input.longitude, cur_gps_input.latitude)
        if 'home' in self.gps_tab.gps_positions.keys():
            if gps_pos != self.gps_tab.gps_positions['home']['pos']:
                self.gps_tab.gps_positions['home']['pos'] = gps_pos
                self.gps_tab.gps_altitudes['home'] = cur_gps_input.altitude
        else:
            # first home way point, create bib entry
            self.gps_tab.gps_positions['home'] = {'pos': gps_pos,
                                                  'symbol': 'o',
                                                  'brush': pyqtgraph.mkBrush(color=(255, 255, 0))}
            self.gps_tab.gps_altitudes['home'] = cur_gps_input.altitude

    def callback_gps_way_point(self, cur_gps_input):
        gps_pos = (cur_gps_input.longitude, cur_gps_input.latitude)
        if 'way_point' in self.gps_tab.gps_positions.keys():
            if gps_pos != self.gps_tab.gps_positions['way_point']['pos']:
                self.gps_tab.gps_positions['way_point']['pos'] = gps_pos
                self.gps_tab.gps_altitudes['way_point'] = cur_gps_input.altitude
        else:
            # first way point, create bib entry
            self.gps_tab.gps_positions['way_point'] = {'pos': gps_pos,
                                                       'symbol': 'o',
                                                       'brush': pyqtgraph.mkBrush(color=(0, 255, 0))}
            self.gps_tab.gps_altitudes['way_point'] = cur_gps_input.altitude

    def callback_gps_position(self, cur_gps_input):
        time_stamp = cur_gps_input.header.stamp.to_nsec() / 1e9
        if len(self.gps_tab.gps_data[0]) == 0:
            # this is the first gps position we receive -> generate geo circle for this region
            self.gps_tab.init_geo_circle(cur_gps_input.longitude, cur_gps_input.latitude, 5)
            self.gps_tab.gps_data[0].append(cur_gps_input.longitude)
            self.gps_tab.gps_data[1].append(cur_gps_input.latitude)
        elif ((cur_gps_input.longitude == self.gps_tab.gps_data[0][-1]) and (cur_gps_input.latitude != self.gps_tab.gps_data[1][-1])):
            # new position is identical to previous one
            pass
        else:
            self.gps_tab.gps_data[0].append(cur_gps_input.longitude)
            self.gps_tab.gps_data[1].append(cur_gps_input.latitude)

        if self.altitude_tab:
            if 'fc_gps' in self.altitude_tab.altitude_dataset_index.keys():
                index = self.altitude_tab.altitude_dataset_index['fc_gps']
                if np.sum(self.altitude_tab.altitude_dataset[:, index, 1]) == 0:
                    self.altitude_tab.altitude_dataset[:, index, 1] = time_stamp
                self.altitude_tab.altitude_dataset[:-1, index, :] = self.altitude_tab.altitude_dataset[1:, index, :]
                self.altitude_tab.altitude_dataset[-1, index, 0] = cur_gps_input.altitude
                self.altitude_tab.altitude_dataset[-1, index, 1] = time_stamp

        gps_pos = (cur_gps_input.longitude, cur_gps_input.latitude)
        if 'phoenix' in self.gps_tab.gps_positions.keys():
            if gps_pos != self.gps_tab.gps_positions['phoenix']['pos']:
                self.gps_tab.gps_positions['phoenix']['pos'] = gps_pos
                self.gps_tab.gps_altitudes['phoenix'] = cur_gps_input.altitude
        else:
            # first position, create bib entry
            self.gps_tab.gps_positions['phoenix'] = {'pos': gps_pos,
                                                     'symbol': 'o',
                                                     'brush': pyqtgraph.mkBrush(color=(0, 0, 255))}
            self.gps_tab.gps_altitudes['phoenix'] = cur_gps_input.altitude

    def callback_fc_altitude(self, cur_altitude):
        time_stamp = cur_altitude.header.stamp.to_nsec() / 1e9
        index = self.altitude_tab.altitude_dataset_index['fc_barometer']
        if np.sum(self.altitude_tab.altitude_dataset[:, index, 1]) == 0:
            self.altitude_tab.altitude_dataset[:, index, 1] = time_stamp
        self.altitude_tab.altitude_dataset[:-1, index, :] = self.altitude_tab.altitude_dataset[1:, index, :]
        self.altitude_tab.altitude_dataset[-1, index, 0] = cur_altitude.estimated_altitude
        self.altitude_tab.altitude_dataset[-1, index, 1] = time_stamp

    def callback_marvic_altitude_fused(self, cur_altitude):
        time_stamp = cur_altitude.header.stamp.to_nsec() / 1e9
        index = self.altitude_tab.altitude_dataset_index['marvic_fused']
        if np.sum(self.altitude_tab.altitude_dataset[:, index, 1]) == 0:
            self.altitude_tab.altitude_dataset[:, index, 1] = time_stamp
        self.altitude_tab.altitude_dataset[:-1, index, :] = self.altitude_tab.altitude_dataset[1:, index, :]
        self.altitude_tab.altitude_dataset[-1, index, 0] = cur_altitude.estimated_altitude
        self.altitude_tab.altitude_dataset[-1, index, 1] = time_stamp
        print 'callback_marvic_altitude_fused', time_stamp

    def callback_marvic_altitude_lidar(self, cur_altitude):
        time_stamp = cur_altitude.header.stamp.to_nsec() / 1e9
        index = self.altitude_tab.altitude_dataset_index['marvic_lidar']
        if np.sum(self.altitude_tab.altitude_dataset[:, index, 1]) == 0:
            self.altitude_tab.altitude_dataset[:, index, 1] = time_stamp
        self.altitude_tab.altitude_dataset[:-1, index, :] = self.altitude_tab.altitude_dataset[1:, index, :]
        self.altitude_tab.altitude_dataset[-1, index, 0] = cur_altitude.estimated_altitude
        self.altitude_tab.altitude_dataset[-1, index, 1] = time_stamp
        print 'callback_marvic_altitude_lidar', time_stamp

    def callback_marvic_altitude_infra_red(self, cur_altitude):
        time_stamp = cur_altitude.header.stamp.to_nsec() / 1e9
        index = self.altitude_tab.altitude_dataset_index['marvic_ir']
        if np.sum(self.altitude_tab.altitude_dataset[:, index, 1]) == 0:
            self.altitude_tab.altitude_dataset[:, index, 1] = time_stamp
        self.altitude_tab.altitude_dataset[:-1, index, :] = self.altitude_tab.altitude_dataset[1:, index, :]
        self.altitude_tab.altitude_dataset[-1, index, 0] = cur_altitude.estimated_altitude
        self.altitude_tab.altitude_dataset[-1, index, 1] = time_stamp

    def callback_fc_rc(self, cur_joy_cmd):
        if self.rc_fc_tab:
            self.rc_fc_tab.rc_data[:-1, :] = self.rc_fc_tab.rc_data[1:, :]
            self.rc_fc_tab.rc_data[-1, 0] = cur_joy_cmd.axes[1]          # pitch
            self.rc_fc_tab.rc_data[-1, 1] = cur_joy_cmd.axes[0]          # roll
            self.rc_fc_tab.rc_data[-1, 2] = cur_joy_cmd.axes[2]          # yaw
            self.rc_fc_tab.rc_data[-1, 3] = cur_joy_cmd.axes[3]          # Throttle
            self.rc_fc_tab.rc_data[-1, 4] = cur_joy_cmd.buttons[0]       # gps
            self.rc_fc_tab.rc_data[-1, 5] = cur_joy_cmd.buttons[1]       #
            self.rc_fc_tab.rc_data[-1, 6] = cur_joy_cmd.buttons[2]       #
            self.rc_fc_tab.rc_data[-1, 7] = cur_joy_cmd.buttons[3]       # barometer

    def callback_marvic_rc(self, cur_joy_cmd):
        if self.rc_marvic_tab:
            self.rc_marvic_tab.rc_data[:-1, :] = self.rc_marvic_tab.rc_data[1:, :]
            self.rc_marvic_tab.rc_data[-1, 0] = cur_joy_cmd.axes[1]          # pitch
            self.rc_marvic_tab.rc_data[-1, 1] = cur_joy_cmd.axes[0]          # roll
            self.rc_marvic_tab.rc_data[-1, 2] = cur_joy_cmd.axes[2]          # yaw
            self.rc_marvic_tab.rc_data[-1, 3] = cur_joy_cmd.axes[3]          # Throttle
            self.rc_marvic_tab.rc_data[-1, 4] = cur_joy_cmd.buttons[0]       # gps
            self.rc_marvic_tab.rc_data[-1, 5] = cur_joy_cmd.buttons[1]       #
            self.rc_marvic_tab.rc_data[-1, 6] = cur_joy_cmd.buttons[2]       #
            self.rc_marvic_tab.rc_data[-1, 7] = cur_joy_cmd.buttons[3]       # barometer

    def callback_image_mono(self, cur_image_mono):
        if self.video_tab:
            if time.time() - self.video_tab.time_of_last_image > 0.05 or not self.video_tab.time_of_last_image:
                print 'updating image via ros'
                self.video_tab.live_image = np.reshape(np.fromstring(cur_image_mono.data, np.uint8), (cur_image_mono.height, cur_image_mono.step))
                self.video_tab.time_of_last_image = time.time()

    def callback_cur_servo(self, cur_servo_cmd):
        if self.parameter_tab:
            self.parameter_tab.set_parameters_slider(0, cur_servo_cmd.servo0)
            self.parameter_tab.set_parameters_slider(1, cur_servo_cmd.servo1)
            self.parameter_tab.set_parameters_slider(2, cur_servo_cmd.servo2)
            self.parameter_tab.set_parameters_slider(3, cur_servo_cmd.servo3)
            self.parameter_tab.set_parameters_slider(4, cur_servo_cmd.servo4)
            self.parameter_tab.set_parameters_slider(5, cur_servo_cmd.servo5)
            self.parameter_tab.set_parameters_slider(6, cur_servo_cmd.servo6)
            self.parameter_tab.set_parameters_slider(7, cur_servo_cmd.servo7)
            self.parameter_tab.set_parameters_slider(8, cur_servo_cmd.servo8)
            self.parameter_tab.set_parameters_slider(9, cur_servo_cmd.servo9)
            self.parameter_tab.set_parameters_slider(10, cur_servo_cmd.servo10)
            self.parameter_tab.set_parameters_slider(11, cur_servo_cmd.servo11)
            self.parameter_tab.set_parameters_slider(12, cur_servo_cmd.servo12)
            self.parameter_tab.set_parameters_slider(13, cur_servo_cmd.servo13)
            self.parameter_tab.set_parameters_slider(14, cur_servo_cmd.servo14)
            self.parameter_tab.set_parameters_slider(15, cur_servo_cmd.servo15)
            self.parameter_tab.set_parameters_slider(16, cur_servo_cmd.servo16)
            self.parameter_tab.set_parameters_slider(17, cur_servo_cmd.servo17)

    def publish_led_strip(self, strip_index, color_r, color_g, color_b):
        if strip_index == 0:
            self.ros_pub_led_strip_0_cmd.publish(generate_led_strip_msg(color_r, color_g, color_b))
        if strip_index == 1:
            self.ros_pub_led_strip_1_cmd.publish(generate_led_strip_msg(color_r, color_g, color_b))
        if strip_index == 2:
            self.ros_pub_led_strip_2_cmd.publish(generate_led_strip_msg(color_r, color_g, color_b))
        if strip_index == 3:
            self.ros_pub_led_strip_3_cmd.publish(generate_led_strip_msg(color_r, color_g, color_b))

    def publish_gps_way_point(self, lon, lat):
        way_point_msg = NavSatFix()
        way_point_msg.longitude = lon
        way_point_msg.latitude = lat
        way_point_msg.altitude = 0              # need to fix this!
        self.ros_pub_gps_way_point.publish(way_point_msg)
    
    def publish_servos(self):
        if self.parameter_tab:
            send_servos_msg = Servo()
            send_servos_msg.servo0 = self.parameter_tab.get_parameters_slider(0)
            send_servos_msg.servo1 = self.parameter_tab.get_parameters_slider(1)
            send_servos_msg.servo2 = self.parameter_tab.get_parameters_slider(2)
            send_servos_msg.servo3 = self.parameter_tab.get_parameters_slider(3)
            send_servos_msg.servo4 = self.parameter_tab.get_parameters_slider(4)
            send_servos_msg.servo5 = self.parameter_tab.get_parameters_slider(5)
            send_servos_msg.servo6 = self.parameter_tab.get_parameters_slider(6)
            send_servos_msg.servo7 = self.parameter_tab.get_parameters_slider(7)
            send_servos_msg.servo8 = self.parameter_tab.get_parameters_slider(8)
            send_servos_msg.servo9 = self.parameter_tab.get_parameters_slider(9)
            send_servos_msg.servo10 = self.parameter_tab.get_parameters_slider(10)
            send_servos_msg.servo11 = self.parameter_tab.get_parameters_slider(11)
            send_servos_msg.servo12 = self.parameter_tab.get_parameters_slider(12)
            send_servos_msg.servo13 = self.parameter_tab.get_parameters_slider(13)
            send_servos_msg.servo14 = self.parameter_tab.get_parameters_slider(14)
            send_servos_msg.servo15 = self.parameter_tab.get_parameters_slider(15)
            send_servos_msg.servo16 = self.parameter_tab.get_parameters_slider(16)
            send_servos_msg.servo17 = self.parameter_tab.get_parameters_slider(17)
            self.ros_pub_servo_cmd.publish(send_servos_msg)

    def publish_pid(self, pid_msg):
        self.ros_pub_fc_set_pid.publish(pid_msg)