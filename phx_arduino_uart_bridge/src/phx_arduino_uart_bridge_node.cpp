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

#include <chrono>
#include <iostream>
#include "phx_arduino_uart_bridge/serial_com.h"

/*
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix, NavSatStatus
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16
from sensor_msgs.msg import FluidPressure #Barometer
from sensor_msgs.msg import Temperature #For compensation gyrodrift
from sensor_msgs.msg import Range #Distance to ground
from geometry_msgs.msg import Twist, Quaternion
from phx_arduino_uart_bridge.msg import Motor
from phx_arduino_uart_bridge.msg import Battery
from phx_arduino_uart_bridge.msg import Cycletime
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue #For Battery status
 */

int main(int argc, char **argv)
{
    // init ------------------------------------------------------------------------------------------------
    // ros init
    ros::init(argc, argv, "UARTBridge");
    ros::NodeHandle n;
    std_msgs::Header headerMsg;
    sensor_msgs::Imu imuMsg;
    sensor_msgs::Joy joyMsg;
    joyMsg.axes = std::vector<float> (4, 0);
    joyMsg.buttons = std::vector<int> (4, 0);
    phx_arduino_uart_bridge::Motor motorMsg;
    sensor_msgs::NavSatFix gpsMsg;
    
    // ros init publishers
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("phx/imu_multiwii", 1);
    ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("phx/rc_multiwii", 1);
    ros::Publisher motor_pub = n.advertise<phx_arduino_uart_bridge::Motor>("phx/motor_multiwii", 1);
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("phx/gps_multiwii", 1);
    
    // ros loopspeed (this might interfere with the serial reading and the size of the serial buffer!)
    ros::Rate loop_rate(500);
    
    // serialcom init
    SerialCom multiwii_serial;                                              // create SerialCom instance
    multiwii_serial.set_device("/dev/ttyUSB0");                             // select the device
    multiwii_serial.set_baudrate(115200);                                   // set the communication baudrate
    multiwii_serial.set_max_io(200);                                        // set maximum bytes per reading
    multiwii_serial.init();                                                 // start serial connection
    sleep(2);                                                               // wait for arduino bootloader
    multiwii_serial.clear_input_buffer();                                   // clear serial buffer
    Message input_msg;                                                      // the latest received message
    uint32_t loop_counter = 0;                                              // a counter which is used for sending requests
    
    // init statistics
    uint32_t request_total = 0;
    uint32_t request_status = 0;
    uint32_t request_rc = 0;
    uint32_t request_imu = 0;
    uint32_t request_motor = 0;
    uint32_t request_gps = 0;
    uint32_t request_attitude = 0;
    uint32_t request_altitude = 0;
    uint32_t request_battery = 0;
    uint32_t request_distance = 0;
    uint32_t request_led = 0;
    uint32_t received_total = 0;
    uint32_t received_status = 0;
    uint32_t received_rc = 0;
    uint32_t received_imu = 0;
    uint32_t received_motor = 0;
    uint32_t received_gps = 0;
    uint32_t received_attitude = 0;
    uint32_t received_altitude = 0;
    uint32_t received_battery = 0;
    uint32_t received_distance = 0;
    uint32_t received_led = 0;
    
    // set start timestamps in real and in cpu time
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;
    double system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
    
    
    // main loop -------------------------------------------------------------------------------------------
    while (ros::ok())
    {
        loop_counter++;
        // print statistics from while to while
        if (loop_counter % 1000 == 0) {
            system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
            std::cout << "       request\tin\tloss" << std::endl;
            std::cout << "total:   " << request_total       << "\t" << received_total    << "\t" << request_total - received_total         << std::endl;
            std::cout << "status   " << request_status      << "\t" << received_status   << "\t" << request_status - received_status       << std::endl;
            std::cout << "rc       " << request_rc          << "\t" << received_rc       << "\t" << request_rc - received_rc               << std::endl;
            std::cout << "imu      " << request_imu         << "\t" << received_imu      << "\t" << request_imu - received_imu             << std::endl;
            std::cout << "attitude " << request_attitude    << "\t" << received_attitude << "\t" << request_attitude - received_attitude   << std::endl;
            std::cout << "motor    " << request_motor       << "\t" << received_motor    << "\t" << request_motor - received_motor         << std::endl;
            std::cout << "gps      " << request_gps         << "\t" << received_gps      << "\t" << request_gps - received_gps             << std::endl;
            
            t1 = std::chrono::high_resolution_clock::now();
            real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
            std::cout << "freq status " << received_status / real_duration << " msg/s" << std::endl;
            std::cout << "freq imu    " << received_imu    / real_duration << " msg/s" << std::endl;
            std::cout << "freq attitude" << received_attitude / real_duration << " msg/s" << std::endl;
            std::cout << "freq motor  " << received_motor  / real_duration << " msg/s" << std::endl;
            std::cout << "freq gps    " << received_gps    / real_duration << " msg/s" << std::endl;
            std::cout << "freq total  " << received_total  / real_duration << " msg/s" << std::endl;
            
            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
        }
        
        // serialcom send requests
        if (loop_counter % 3 == 0) {
            multiwii_serial.send_request(MULTIWII_RC); request_rc++; request_total++;
            multiwii_serial.send_request(MULTIWII_GPS); request_gps++; request_total++;
            multiwii_serial.send_request(MULTIWII_STATUS); request_status++; request_total++;
            multiwii_serial.send_request(MULTIWII_IMU); request_imu++; request_total++;
            multiwii_serial.send_request(MULTIWII_ATTITUDE); request_attitude++; request_total++;
            multiwii_serial.send_request(MULTIWII_MOTOR); request_motor++; request_total++;
            multiwii_serial.send_from_buffer();
            usleep(10);
        }
        
        
        // serialcom receive serial stuff
        multiwii_serial.receive_to_buffer();
        
        
        // interprete stuff
        while (multiwii_serial.read_msg_from_buffer(&input_msg) == true) {
            received_total++;
            if (input_msg.msg_code == MULTIWII_STATUS) {
                // publish to ros here
                received_status++;
            } else if (input_msg.msg_code == MULTIWII_RC) {
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
                received_rc++;
            } else if (input_msg.msg_code == MULTIWII_IMU) {
                imuMsg.linear_acceleration.x = input_msg.msg_data.multiwii_raw_imu.accx;
                imuMsg.linear_acceleration.y = input_msg.msg_data.multiwii_raw_imu.accy;
                imuMsg.linear_acceleration.z = input_msg.msg_data.multiwii_raw_imu.accz;
                imuMsg.angular_velocity.x = input_msg.msg_data.multiwii_raw_imu.gyrx;
                imuMsg.angular_velocity.y = input_msg.msg_data.multiwii_raw_imu.gyry;
                imuMsg.angular_velocity.z = input_msg.msg_data.multiwii_raw_imu.gyrz;
                received_imu++;
            } else if (input_msg.msg_code == MULTIWII_ATTITUDE) {
                headerMsg.seq = received_imu;
                headerMsg.stamp = ros::Time::now();
                headerMsg.frame_id = "multiwii";
                imuMsg.header = headerMsg;
                geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(input_msg.msg_data.multiwii_attitude.roll, input_msg.msg_data.multiwii_attitude.pitch, input_msg.msg_data.multiwii_attitude.yaw);
                imuMsg.orientation = quaternion;
                imu_pub.publish(imuMsg);
                received_attitude++;
            } else if (input_msg.msg_code == MULTIWII_MOTOR) {
                motorMsg.motor0 = input_msg.msg_data.multiwii_motor.motor0;
                motorMsg.motor1 = input_msg.msg_data.multiwii_motor.motor1;
                motorMsg.motor2 = input_msg.msg_data.multiwii_motor.motor2;
                motorMsg.motor3 = input_msg.msg_data.multiwii_motor.motor3;
                motor_pub.publish(motorMsg);
                received_motor++;
            } else if (input_msg.msg_code == MULTIWII_GPS) {
                gpsMsg.latitude = input_msg.msg_data.multiwii_gps.coordLAT;
                gpsMsg.longitude = input_msg.msg_data.multiwii_gps.coordLON;
                gpsMsg.altitude = input_msg.msg_data.multiwii_gps.altitude;
                gps_pub.publish(gpsMsg);
                received_gps++;
            } else if (input_msg.msg_code == MULTIWII_ATTITUDE) {
                received_attitude++;
            } else if (input_msg.msg_code == MULTIWII_ALTITUDE) {
                received_altitude++;
            } else if (input_msg.msg_code == MARVIC_BATTERY) {
                received_battery++;
            } else if (input_msg.msg_code == MARVIC_LED) {
                received_led++;
            } else if (input_msg.msg_code == MARVIC_DISTANCE) {
                received_distance++;
            }

            multiwii_serial.receive_to_buffer();
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    // shutdown -------------------------------------------------------------------------------------------
    ROS_INFO("uart bridge is shutting down");
    
    
    multiwii_serial.deinitialize();
    
    
    system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::cout << "       request\tin\tloss" << std::endl;
    std::cout << "total: " << request_total   << "\t" << received_total  << "\t" << request_total - received_total   << std::endl;
    std::cout << "status " << request_status  << "\t" << received_status << "\t" << request_status - received_status << std::endl;
    std::cout << "rc     " << request_rc      << "\t" << received_rc     << "\t" << request_rc - received_rc         << std::endl;
    std::cout << "imu    " << request_imu     << "\t" << received_imu    << "\t" << request_imu - received_imu       << std::endl;
    std::cout << "motor  " << request_motor   << "\t" << received_motor  << "\t" << request_motor - received_motor   << std::endl;
    std::cout << "gps    " << request_gps     << "\t" << received_gps    << "\t" << request_gps - received_gps       << std::endl;
    
    t1 = std::chrono::high_resolution_clock::now();
    real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
    std::cout << "freq status " << received_status / real_duration << " msg/s" << std::endl;
    std::cout << "freq rc     " << received_rc     / real_duration << " msg/s" << std::endl;
    std::cout << "freq imu    " << received_imu    / real_duration << " msg/s" << std::endl;
    std::cout << "freq motor  " << received_motor  / real_duration << " msg/s" << std::endl;
    std::cout << "freq gps    " << received_gps    / real_duration << " msg/s" << std::endl;
    std::cout << "freq total  " << received_total  / real_duration << " msg/s" << std::endl;
    std::cout << "End " << std::endl;
    
    std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
    std::cout << " communication took " << real_duration << " real seconds" << std::endl;
    std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
    
    return 0;
}
