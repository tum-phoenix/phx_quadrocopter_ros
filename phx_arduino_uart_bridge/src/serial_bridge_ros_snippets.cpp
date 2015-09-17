#include <unistd.h>
#include <ctime>
#include <sstream>
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/NavSatFix.h"
#include "phx_arduino_uart_bridge/Motor.h"
#include "phx_arduino_uart_bridge/Status.h"
#include "phx_arduino_uart_bridge/Altitude.h"
#include "phx_arduino_uart_bridge/Battery.h"
#include "phx_arduino_uart_bridge/LED.h"

std_msgs::Header headerMsg;
sensor_msgs::Imu imuMsg;
phx_arduino_uart_bridge::Motor motorMsg;
phx_arduino_uart_bridge::Status statusMsg;
phx_arduino_uart_bridge::Altitude altitudeMsg;
sensor_msgs::NavSatFix gpsMsg;
// handle this with care!
sensor_msgs::Joy joyMsg;
joyMsg.axes = std::vector<float> (4, 0);
joyMsg.buttons = std::vector<int> (4, 0);



// serial incoming MULTIWII_RC info piped to ros:joy_pub with joyMsg format
if (input_msg.msg_code == MULTIWII_RC) {
    headerMsg.seq = received_rc;
    headerMsg.stamp = ros::Time::now();
    headerMsg.frame_id = "multiwii";
    joyMsg.header = headerMsg;
    joyMsg.axes[0] = (float) input_msg.msg_data.multiwii_rc.roll;
    joyMsg.axes[1] = (float) input_msg.msg_data.multiwii_rc.pitch;
    joyMsg.axes[2] = (float) input_msg.msg_data.multiwii_rc.yaw;
    joyMsg.axes[3] = (float) input_msg.msg_data.multiwii_rc.throttle;
    joyMsg.buttons[0] = (int) input_msg.msg_data.multiwii_rc.aux1;
    joyMsg.buttons[1] = (int) input_msg.msg_data.multiwii_rc.aux2;
    joyMsg.buttons[2] = (int) input_msg.msg_data.multiwii_rc.aux3;
    joyMsg.buttons[3] = (int) input_msg.msg_data.multiwii_rc.aux4;
    joy_pub.publish(joyMsg);
    /*
    // this demonstrates that it is possible to fly the copter via the serial bridge by sending every second rc update back.
    if (received_rc % 2 == 0) {
        serial_interface.prepare_msg_rc((uint16_t) input_msg.msg_data.multiwii_rc.throttle,
                                       (uint16_t) input_msg.msg_data.multiwii_rc.pitch,
                                       (uint16_t) input_msg.msg_data.multiwii_rc.roll,
                                       (uint16_t) input_msg.msg_data.multiwii_rc.yaw,
                                       (uint16_t) input_msg.msg_data.multiwii_rc.aux1,
                                       (uint16_t) input_msg.msg_data.multiwii_rc.aux2,
                                       (uint16_t) input_msg.msg_data.multiwii_rc.aux3,
                                       (uint16_t) input_msg.msg_data.multiwii_rc.aux4);
        serial_interface.send_from_buffer();
    }
    */
    received_rc++;
}
// callback an incoming ros:joy_pub with joyMsg format to MULTIWII_RC
void rc_direct_callback(const sensor_msgs::Joy::ConstPtr& joyMsg) {
    std::cout << "\033[1;31m>>> rc_direct_callback\033[0m"<< std::endl;
    multiwii_serial.prepare_msg_rc((uint16_t) (*joyMsg).axes[3],
                                   (uint16_t) joyMsg->axes[1],
                                   (uint16_t) joyMsg->axes[0],
                                   (uint16_t) joyMsg->axes[2],
                                   (uint16_t) joyMsg->buttons[0],
                                   (uint16_t) joyMsg->buttons[1],
                                   (uint16_t) joyMsg->buttons[2],
                                   (uint16_t) joyMsg->buttons[3]);
    multiwii_serial.send_from_buffer();
}


if (input_msg.msg_code == MULTIWII_STATUS) {
    headerMsg.seq = received_status;
    headerMsg.stamp = ros::Time::now();
    headerMsg.frame_id = "multiwii";
    statusMsg.header = headerMsg;
    statusMsg.cycleTime = input_msg.msg_data.multiwii_status.cycleTime;
    statusMsg.i2c_errors_count = input_msg.msg_data.multiwii_status.i2c_errors_count;
    status_pub.publish(statusMsg);
    received_status++;
}

if (input_msg.msg_code == MULTIWII_IMU) {
    // if raw_imu data is received this is updated in the imu ros message but not directly published.
    // the message is only published if fresh attitude data is present.
    imuMsg.linear_acceleration.x = input_msg.msg_data.multiwii_raw_imu.accx;
    imuMsg.linear_acceleration.y = input_msg.msg_data.multiwii_raw_imu.accy;
    imuMsg.linear_acceleration.z = input_msg.msg_data.multiwii_raw_imu.accz;
    imuMsg.angular_velocity.x = input_msg.msg_data.multiwii_raw_imu.gyrx;
    imuMsg.angular_velocity.y = input_msg.msg_data.multiwii_raw_imu.gyry;
    imuMsg.angular_velocity.z = input_msg.msg_data.multiwii_raw_imu.gyrz;
    received_imu++;
}

if (input_msg.msg_code == MULTIWII_ATTITUDE) {
    headerMsg.seq = received_imu;
    headerMsg.stamp = ros::Time::now();
    headerMsg.frame_id = "multiwii";
    imuMsg.header = headerMsg;
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(input_msg.msg_data.multiwii_attitude.roll, input_msg.msg_data.multiwii_attitude.pitch, input_msg.msg_data.multiwii_attitude.yaw);
    imuMsg.orientation = quaternion;
    imu_pub.publish(imuMsg);
    received_attitude++;
}

if (input_msg.msg_code == MULTIWII_MOTOR) {
    headerMsg.seq = received_motor;
    headerMsg.stamp = ros::Time::now();
    headerMsg.frame_id = "multiwii";
    motorMsg.header = headerMsg;
    motorMsg.motor0 = input_msg.msg_data.multiwii_motor.motor0;
    motorMsg.motor1 = input_msg.msg_data.multiwii_motor.motor1;
    motorMsg.motor2 = input_msg.msg_data.multiwii_motor.motor2;
    motorMsg.motor3 = input_msg.msg_data.multiwii_motor.motor3;
    motor_pub.publish(motorMsg);
    received_motor++;
}
void motor_computer_callback(const phx_arduino_uart_bridge::Motor::ConstPtr& motorMsg) {
    std::cout << "\033[1;31m>>> motor_computer_callback" << (*motorMsg).motor0 << " " << (*motorMsg).motor1 << " "  << (*motorMsg).motor2 << " " << (*motorMsg).motor3 << "\033[0m"<< std::endl;
    multiwii_serial.prepare_msg_motor((uint16_t) (*motorMsg).motor0,
                                      (uint16_t) (*motorMsg).motor1,
                                      (uint16_t) (*motorMsg).motor2,
                                      (uint16_t) (*motorMsg).motor3,
                                      (uint16_t) 1000,
                                      (uint16_t) 1000,
                                      (uint16_t) 1000,
                                      (uint16_t) 1000);
    multiwii_serial.send_from_buffer();
}

if (input_msg.msg_code == MULTIWII_GPS) {
    headerMsg.seq = received_gps;
    headerMsg.stamp = ros::Time::now();
    headerMsg.frame_id = "multiwii";
    gpsMsg.header = headerMsg;
    gpsMsg.latitude = input_msg.msg_data.multiwii_gps.coordLAT;
    gpsMsg.longitude = input_msg.msg_data.multiwii_gps.coordLON;
    gpsMsg.altitude = input_msg.msg_data.multiwii_gps.altitude;
    gps_pub.publish(gpsMsg);
    received_gps++;
}

if (input_msg.msg_code == MULTIWII_ALTITUDE) {
    altitudeMsg.estimated_altitude = input_msg.msg_data.multiwii_altitude.estAlt;
    altitudeMsg.variation = input_msg.msg_data.multiwii_altitude.variation;
    altitude_pub.publish(altitudeMsg);
    received_altitude++;
}