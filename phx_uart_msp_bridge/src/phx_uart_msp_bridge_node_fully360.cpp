#include <unistd.h>
#include <ctime>
#include <sstream>
#include "ros/ros.h"

#include <math.h>               // for M_PI
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <chrono>
#include <iostream>
#include "phx_uart_msp_bridge/serial_com.h"


SerialCom serial_interface;                                              // create SerialCom instance

int main(int argc, char **argv)
{
    // init ------------------------------------------------------------------------------------------------
    // ros init
    ros::init(argc, argv, "UART_bridge_fully360");
    ros::NodeHandle n;
    std_msgs::Header headerMsg;
    sensor_msgs::LaserScan laserMsg;


    // ros init publishers
    ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scanner/fully360", 1);

    // ros init subscribers

    // ros loop speed (this might interfere with the serial reading and the size of the serial buffer!)
    ros::Rate loop_rate(500);

    // scanner init
    const uint16_t laser_scanner_readings_max = 512;
    uint16_t laser_scanner_buffer_1[laser_scanner_readings_max];
    uint16_t laser_scanner_buffer_2[laser_scanner_readings_max];
    uint16_t laser_scanner_buffer_write_position = 0;

    // serialcom init
    serial_interface.set_device("/dev/ttyACM0");                             // select the device
    //serial_interface.set_device("/dev/ttyAMA0");                             // select the device
    //serial_interface.set_device("/dev/fully360");                             // select the device
    serial_interface.set_baudrate(115200);                                   // set the communication baudrate
    serial_interface.set_max_io(20);                                         // set maximum bytes per reading
    serial_interface.init();                                                 // start serial connection
    sleep(5);                                                                // wait for boot loader and calibration
    serial_interface.clear_input_buffer();                                   // clear serial buffer
    Message input_msg;                                                       // the latest received message
    uint32_t loop_counter = 0;                                               // a counter which is used for sending requests

    // MICRO CONTROLLER IDENTIFIER
    uint8_t controller_version = 0;
    uint8_t controller_type = 0;

    // init statistics
    uint32_t request_total = 0;         uint32_t received_total = 0;
    uint32_t request_reading = 0;       uint32_t received_reading = 0;
    uint32_t request_restart = 0;       uint32_t received_restart = 0;

    // set start timestamps in real and in cpu time
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;
    double system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;


    // main loop -------------------------------------------------------------------------------------------
    while (ros::ok())
    {
        if (serial_interface.error_count > 10) {
            ROS_INFO("uart bridge FULLY360 is shutting down due to serial port problem, restarting in 5sec");
            serial_interface.deinitialize();
            sleep(5);
            serial_interface.init();                                                 // start serial connection
            sleep(1);                                                                // wait for boot loader and calibration
            serial_interface.clear_input_buffer();                                   // clear serial buffer
            ROS_INFO("uart bridge FULLY360 goes back to main loop");
        }
        loop_counter++;

        // print statistics from while to while
        if (loop_counter % 1000 == 0) {
            system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
            std::cout << "       request\tin\tloss" << std::endl;
            std::cout << "reading  " << request_reading     << "\t" << received_reading    << "\t" << request_reading - received_reading       << std::endl;
            std::cout << "restart  " << request_restart     << "\t" << received_restart    << "\t" << request_restart - received_restart       << std::endl;
            std::cout << "total:   " << request_total       << "\t" << received_total      << "\t" << request_total - received_total           << std::endl;

            t1 = std::chrono::high_resolution_clock::now();
            real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
            std::cout << "freq reading  " << received_reading  / real_duration << " msg/s" << std::endl;
            std::cout << "freq restart  " << received_restart  / real_duration << " msg/s" << std::endl;
            std::cout << "freq total    " << received_total    / real_duration << " msg/s" << std::endl;

            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
        }

        // serial com send requests
        /*
        if (loop_counter % 50 == 0) {
        }
        if (loop_counter % 5 == 0) {
        } else {
            if (loop_counter % 1 == 0) {
            }
            if (loop_counter % 2 == 0) {
            }
        }
        serial_interface.send_from_buffer();
        */

        // receive serial stuff
        while (serial_interface.receive_to_buffer() == true){
            // interpret new input
            //std::cout << "interpreting_loop >>" << std::endl;
            while (serial_interface.read_msg_from_buffer(&input_msg) == true) {
                received_total++;
                //std::cout << "interpreting_loop >> received request" << std::endl;
                if (input_msg.msg_length == 0) {
                    //std::cout << "interpreting_loop >> received request" << std::endl;
                } else {
                    if (input_msg.msg_code == SCANNER_READING) {
                        // laser scanner reading incoming
                        laser_scanner_buffer_1[laser_scanner_buffer_write_position] = (uint16_t) input_msg.msg_data.scanner_distance.distance_scanner_1;
                        laser_scanner_buffer_2[laser_scanner_buffer_write_position] = (uint16_t) input_msg.msg_data.scanner_distance.distance_scanner_2;
                        laser_scanner_buffer_write_position++;
                        if (laser_scanner_buffer_write_position >= laser_scanner_readings_max) {
                            std::cout << "ERROR TOO MANY SCAN READINGS" << laser_scanner_buffer_write_position << std::endl;
                            laser_scanner_buffer_write_position = 0;
                        }
                        received_reading++;
                    } else if (input_msg.msg_code == SCANNER_RESTART) {
                        if (laser_scanner_buffer_write_position > 1) {
                            std::cout << " SCANNER_RESTART received!" << std::endl;
                            uint16_t number_of_angles = laser_scanner_buffer_write_position;

                            headerMsg.seq = received_restart;
                            headerMsg.stamp = ros::Time::now();
                            headerMsg.frame_id = "fully360";
                            laserMsg.header = headerMsg;
                            float offset = M_PI / 180. * (40. / 180.) * 0.;
                            if ((received_restart % 2) == 0) {
                                laserMsg.angle_min = - M_PI;
                                laserMsg.angle_max = M_PI;
                            } else {
                                laserMsg.angle_min = - M_PI - offset;
                                laserMsg.angle_max = M_PI - offset;
                            }
                            laserMsg.angle_increment = 2. * M_PI / number_of_angles / 2;
                            laserMsg.time_increment = 0;
                            laserMsg.range_min = 0.05;
                            laserMsg.range_max = 40.0;
                            laserMsg.ranges.resize(number_of_angles * 2);
                            for (uint16_t index=0; index < laser_scanner_buffer_write_position; index++) {
                                laserMsg.ranges[index] = 0.01 * (uint16_t) laser_scanner_buffer_1[index];
                            }
                            for (uint16_t index=0; index < laser_scanner_buffer_write_position; index++) {
                                laserMsg.ranges[index+number_of_angles] = 0.01 * (uint16_t) laser_scanner_buffer_2[index];
                            }


                            scan_pub.publish(laserMsg);
                        } else {
                            std::cout << " SCANNER_RESTART received but not enough measurements there" << std::endl;
                        }
                        laser_scanner_buffer_write_position = 0;
                        received_restart++;
                    }
                }
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    // shutdown -------------------------------------------------------------------------------------------
    ROS_INFO("uart bridge is shutting down");


    serial_interface.deinitialize();


    system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
            std::cout << "       request\tin\tloss" << std::endl;
            std::cout << "reading  " << request_reading     << "\t" << received_reading    << "\t" << request_reading - received_reading       << std::endl;
            std::cout << "restart  " << request_restart     << "\t" << received_restart    << "\t" << request_restart - received_restart       << std::endl;
            std::cout << "total:   " << request_total       << "\t" << received_total      << "\t" << request_total - received_total           << std::endl;

            t1 = std::chrono::high_resolution_clock::now();
            real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
            std::cout << "freq reading  " << received_reading  / real_duration << " msg/s" << std::endl;
            std::cout << "freq restart  " << received_restart  / real_duration << " msg/s" << std::endl;
            std::cout << "freq total    " << received_total    / real_duration << " msg/s" << std::endl;

            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
    return 0;
}