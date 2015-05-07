#include <unistd.h>
#include <ctime>
#include <chrono>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/NavSatFix.h"
#include "phx_arduino_uart_bridge/Motor.h"
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
    ros::init(argc, argv, "UARTBridge");
    ros::NodeHandle n;
    std_msgs::Header headerMsg;
    sensor_msgs::Imu imuMsg;
    sensor_msgs::Joy joyMsg;
    phx_arduino_uart_bridge::Motor motorMsg;
    sensor_msgs::NavSatFix gpsMsg;

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("phx/imu_multiwii", 1);
    ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("phx/rc_multiwii", 1);
    ros::Publisher motor_pub = n.advertise<phx_arduino_uart_bridge::Motor>("phx/motor_multiwii", 1);
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("phx/gps_multiwii", 1);

    ros::Rate loop_rate(10);
    int count = 0;
    
    // establish connection:
    SerialCom multiwii_serial;                                              // create SerialCom instance
    multiwii_serial.set_device("/dev/ttyUSB0");
    multiwii_serial.set_baudrate(115200);
    multiwii_serial.set_max_io(200);
    multiwii_serial.init();                                                 // initialize serial connection
    sleep(2);                                                               // wait for arduino bootloader
    multiwii_serial.receive_to_buffer();
    multiwii_serial.receive_to_buffer();
    multiwii_serial.clear_input_buffer();
    uint16_t request_total = 0;
    uint16_t received_total = 0;
    uint16_t received_status = 0;
    uint16_t received_imu = 0;
    uint16_t received_motor = 0;
    uint16_t received_gps = 0;
    Message input_msg;
    
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;
    
    while (ros::ok())
    {
        // send requests
        if (count % 3 == 0) {
            multiwii_serial.send_request(MULTIWII_STATUS);
        } else if (count % 3 == 1) {
            multiwii_serial.send_request(MULTIWII_IMU);
        } else {
            multiwii_serial.send_request(MULTIWII_MOTOR);
        }
        multiwii_serial.send_from_buffer();
        
        
        // receive serial stuff
        multiwii_serial.receive_to_buffer();
        // > option one for handling input (no buffer overflow, but more slowly)
        while (multiwii_serial.read_msg_from_buffer(&input_msg) == true) {
            //print_multiwii_message(&input_msg);
            received_total++;
            usleep(1000);    // publishing stuff to ros here
            if (input_msg.msg_code == MULTIWII_STATUS) {
                headerMsg.seq = count;
                headerMsg.stamp = ros::Time::now();
                headerMsg.frame_id = "multiwii";
                joyMsg.header = headerMsg;
                joyMsg.axes[0] = input_msg.msg_data.multiwii_rc.roll;
                joyMsg.axes[1] = input_msg.msg_data.multiwii_rc.pitch;
                joyMsg.axes[2] = input_msg.msg_data.multiwii_rc.yaw;
                joyMsg.axes[3] = input_msg.msg_data.multiwii_rc.throttle;
                joy_pub.publish(joyMsg);
                received_status++;
            } else if (input_msg.msg_code == MULTIWII_IMU) {
                headerMsg.seq = count;
                headerMsg.stamp = ros::Time::now();
                headerMsg.frame_id = "multiwii";
                imuMsg.header = headerMsg;
                imuMsg.linear_acceleration.x = input_msg.msg_data.multiwii_raw_imu.accx;
                imuMsg.linear_acceleration.y = input_msg.msg_data.multiwii_raw_imu.accy;
                imuMsg.linear_acceleration.z = input_msg.msg_data.multiwii_raw_imu.accz;
                imuMsg.angular_velocity.x = input_msg.msg_data.multiwii_raw_imu.gyrx;
                imuMsg.angular_velocity.y = input_msg.msg_data.multiwii_raw_imu.gyry;
                imuMsg.angular_velocity.z = input_msg.msg_data.multiwii_raw_imu.gyrz;
                imu_pub.publish(imuMsg);
                received_imu++;
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
            }
            multiwii_serial.receive_to_buffer();
        }
        // > option two for handling input (fast but can segfault in some cases)
        /*
         if (multiwii_serial.read_msg_from_buffer(&input_msg) == true) {
         print_multiwii_message(&input_msg);
         usleep(1000);    // publishing stuff to ros here
         received++;
         }
         */
    
        double stop_communication = double(clock()) / CLOCKS_PER_SEC;
        double elapsed_secs = stop_communication - begin_communication;
        //std::cout << count << "  loop mean duration in cpu time " << elapsed_secs / received_total << " seconds" << std::endl;
        
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
        //std::cout << count << "  loop mean duration in real time " << duration / received_total << " us" << std::endl;

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    
    
    double stop_communication = double(clock()) / CLOCKS_PER_SEC;
    double time_of_active_communication = stop_communication - begin_communication;
    std::cout << " communication took " << time_of_active_communication << " cpu seconds -> " << received_total / time_of_active_communication << " messages/s cpu_time" << std::endl;
    
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
    std::cout << " communication took " << duration / 1000000. << " real seconds -> " << received_total / (duration / 1000000.) << " messages/s" << std::endl;
    
    multiwii_serial.deinitialize();
    
    std::cout << "requested total: " << request_total << std::endl << std::endl;
    std::cout << "received status " << received_status << std::endl;
    std::cout << "received imu    " << received_imu    << std::endl;
    std::cout << "received motor  " << received_motor  << std::endl;
    std::cout << "received gps  " << received_gps  << std::endl;
    std::cout << "received total  " << received_total  << std::endl;
    
    std::cout << "freq status " << received_status / (duration / 1000000.) << " messages/s" << std::endl;
    std::cout << "freq imu    " << received_imu    / (duration / 1000000.) << " messages/s" << std::endl;
    std::cout << "freq motor  " << received_motor  / (duration / 1000000.) << " messages/s" << std::endl;
    std::cout << "freq gps  " << received_gps  / (duration / 1000000.) << " messages/s" << std::endl;
    std::cout << "freq total  " << received_total  / (duration / 1000000.) << " messages/s" << std::endl;
    
    std::cout << "End " << std::endl;
    
    return 0;
}