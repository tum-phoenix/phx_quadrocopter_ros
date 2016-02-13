#include <unistd.h>
#include <ctime>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "phx_arduino_uart_bridge/Servo.h"

#include <chrono>
#include <iostream>
#include "phx_arduino_uart_bridge/serial_com.h"

void servo_direct_callback(const phx_arduino_uart_bridge::Servo::ConstPtr&);

SerialCom serial_interface;                                              // create SerialCom instance
uint32_t msg_sent = 0;
std_msgs::Header headerMsg;
phx_arduino_uart_bridge::Servo servoMsg;
ros::Publisher servo_pub;

int main(int argc, char **argv)
{
    // init ------------------------------------------------------------------------------------------------
    // ros init
    ros::init(argc, argv, "UART_bridge_crab");
    ros::NodeHandle n;

    // ros init publishers
    servo_pub = n.advertise<phx_arduino_uart_bridge::Servo>("crab/uart_bridge/cur_servo_cmd", 1);

    // ros init subscribers
    ros::Subscriber servo_sub = n.subscribe<phx_arduino_uart_bridge::Servo>("crab/uart_bridge/servo_cmd", 1, servo_direct_callback);
    
    // ros loop speed (this might interfere with the serial reading and the size of the serial buffer!)
    ros::Rate loop_rate(500);
    
    // serialcom init
    serial_interface.set_device("/dev/ttyUSB0");                             // select the device
    serial_interface.set_baudrate(115200);                                   // set the communication baudrate
    serial_interface.set_max_io(250);                                        // set maximum bytes per reading
    serial_interface.init();                                                 // start serial connection
    sleep(1);                                                                // wait for boot loader and calibration
    serial_interface.clear_input_buffer();                                   // clear serial buffer
    Message input_msg;                                                       // the latest received message
    uint32_t loop_counter = 0;                                               // a counter which is used for sending requests


    // set start timestamps in real and in cpu time
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;
    double system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
    

    ROS_INFO("uart bridge CRAB starts main loop");
    // main loop -------------------------------------------------------------------------------------------
    while (ros::ok())
    {
        if (serial_interface.error_count > 10) {
            ROS_INFO("uart bridge is shutting down due to serial port problem, restarting in 5sec");
            serial_interface.deinitialize();
            sleep(5);
            serial_interface.init();                                                 // start serial connection
            sleep(1);                                                                // wait for boot loader and calibration
            serial_interface.clear_input_buffer();                                   // clear serial buffer
            ROS_INFO("uart bridge CRAB goes back to main loop");
        }
        loop_counter++;
        // print statistics from while to while
        if (loop_counter % 1000 == 0) {
            system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
            t1 = std::chrono::high_resolution_clock::now();
            real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;

            std::cout << " out messages:\t" << msg_sent << "   \t" << msg_sent / real_duration << " msg/s" << std::endl;
            std::cout << "   communication active since " << system_duration << " cpu seconds or " << real_duration << " real seconds" << std::endl;
            std::cout << "   CPU workload " << system_duration / real_duration << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    
    // shutdown -------------------------------------------------------------------------------------------
    ROS_INFO("uart bridge is shutting down");
    
    
    serial_interface.deinitialize();
    return 0;
}

// callbacks
void servo_direct_callback(const phx_arduino_uart_bridge::Servo::ConstPtr& servo_values) {
    std::cout << "\033[1;31m>>> servo_direct_callback\033[0m"<< std::endl;
    serial_interface.prepare_msg_servo((uint16_t) servo_values->servo0,
                                       (uint16_t) servo_values->servo1,
                                       (uint16_t) servo_values->servo2,
                                       (uint16_t) servo_values->servo3,
                                       (uint16_t) servo_values->servo4,
                                       (uint16_t) servo_values->servo5,
                                       (uint16_t) servo_values->servo6,
                                       (uint16_t) servo_values->servo7,
                                       (uint16_t) servo_values->servo8,
                                       (uint16_t) servo_values->servo9,
                                       (uint16_t) servo_values->servo10,
                                       (uint16_t) servo_values->servo11,
                                       (uint16_t) servo_values->servo12,
                                       (uint16_t) servo_values->servo13,
                                       (uint16_t) servo_values->servo14,
                                       (uint16_t) servo_values->servo15,
                                       (uint16_t) servo_values->servo16,
                                       (uint16_t) servo_values->servo17);
    serial_interface.send_from_buffer();

    headerMsg.seq = msg_sent;
    headerMsg.stamp = ros::Time::now();
    headerMsg.frame_id = "crab";
    servoMsg.header = headerMsg;
    servoMsg.servo0 = servo_values->servo0;
    servoMsg.servo1 = servo_values->servo1;
    servoMsg.servo2 = servo_values->servo2;
    servoMsg.servo3 = servo_values->servo3;
    servoMsg.servo4 = servo_values->servo4;
    servoMsg.servo5 = servo_values->servo5;
    servoMsg.servo6 = servo_values->servo6;
    servoMsg.servo7 = servo_values->servo7;
    servoMsg.servo8 = servo_values->servo8;
    servoMsg.servo9 = servo_values->servo9;
    servoMsg.servo10 = servo_values->servo10;
    servoMsg.servo11 = servo_values->servo11;
    servoMsg.servo12 = servo_values->servo12;
    servoMsg.servo13 = servo_values->servo13;
    servoMsg.servo14 = servo_values->servo14;
    servoMsg.servo15 = servo_values->servo15;
    servoMsg.servo16 = servo_values->servo16;
    servoMsg.servo17 = servo_values->servo17;
    servo_pub.publish(servoMsg);

    msg_sent++;
}