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
#include "phx_arduino_uart_bridge/LEDstrip.h"
#include "phx_arduino_uart_bridge/LED.h"

#include <chrono>
#include <iostream>
#include "phx_arduino_uart_bridge/serial_com.h"


SerialCom multiwii_serial;                                              // create SerialCom instance

int main(int argc, char **argv)
{
    // init ------------------------------------------------------------------------------------------------
    // ros init
    ros::init(argc, argv, "UART_bridge_marvicAltitude");
    ros::NodeHandle n;
    std_msgs::Header headerMsg;
    phx_arduino_uart_bridge::Status statusMsg;
    phx_arduino_uart_bridge::Altitude altitudeMsg;
    
    // ros init publishers
    ros::Publisher status_pub = n.advertise<phx_arduino_uart_bridge::Status>("phx/marvicAltitude/status", 1);
    ros::Publisher altitude_pub = n.advertise<phx_arduino_uart_bridge::Altitude>("phx/marvicAltitude/altitude", 1);
    ros::Publisher sonar_pub = n.advertise<phx_arduino_uart_bridge::Altitude>("phx/marvicAltitude/sonar", 1);
    ros::Publisher lidar_pub = n.advertise<phx_arduino_uart_bridge::Altitude>("phx/marvicAltitude/lidar", 1);
    ros::Publisher infra_red_pub = n.advertise<phx_arduino_uart_bridge::Altitude>("phx/marvicAltitude/infra_red", 1);
    ros::Publisher barometer_pub = n.advertise<phx_arduino_uart_bridge::Altitude>("phx/marvicAltitude/barometer", 1);

    // ros init subscriber
    
    // ros loop speed (this might interfere with the serial reading and the size of the serial buffer!)
    ros::Rate loop_rate(500);
    
    // serialcom init
    multiwii_serial.set_device("/dev/ttyUSB0");                             // select the device
    multiwii_serial.set_baudrate(115200);                                   // set the communication baudrate
    multiwii_serial.set_max_io(250);                                        // set maximum bytes per reading
    multiwii_serial.init();                                                 // start serial connection
    sleep(12);                                                              // wait for arduino boot loader
    multiwii_serial.clear_input_buffer();                                   // clear serial buffer
    Message input_msg;                                                      // the latest received message
    uint32_t loop_counter = 0;                                              // a counter which is used for sending requests
    
    // init statistics
    uint32_t request_total = 0;         uint32_t received_total = 0;
    uint32_t request_status = 0;        uint32_t received_status = 0;
    uint32_t request_rc = 0;            uint32_t received_rc = 0;
    uint32_t request_imu = 0;           uint32_t received_imu = 0;
    uint32_t request_attitude = 0;      uint32_t received_attitude = 0;
    uint32_t request_motor = 0;         uint32_t received_motor = 0;
    uint32_t request_gps = 0;           uint32_t received_gps = 0;
    uint32_t request_altitude = 0;      uint32_t received_altitude = 0;
    uint32_t request_battery = 0;       uint32_t received_battery = 0;
    uint32_t request_sonar = 0;         uint32_t received_sonar = 0;
    uint32_t request_lidar = 0;     uint32_t received_lidar = 0;
    uint32_t request_infra_red = 0;     uint32_t received_infra_red = 0;
    uint32_t request_barometer = 0;     uint32_t received_barometer = 0;
    uint32_t request_autonomous = 0;    uint32_t received_autonomous = 0;



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
            std::cout << "status   " << request_status      << "\t" << received_status     << "\t" << request_status - received_status         << std::endl;
            std::cout << "rc       " << request_rc          << "\t" << received_rc         << "\t" << request_rc - received_rc                 << std::endl;
            std::cout << "imu      " << request_imu         << "\t" << received_imu        << "\t" << request_imu - received_imu               << std::endl;
            std::cout << "attitude " << request_attitude    << "\t" << received_attitude   << "\t" << request_attitude - received_attitude     << std::endl;
            std::cout << "motor    " << request_motor       << "\t" << received_motor      << "\t" << request_motor - received_motor           << std::endl;
            std::cout << "gps      " << request_gps         << "\t" << received_gps        << "\t" << request_gps - received_gps               << std::endl;
            std::cout << "altitude " << request_altitude    << "\t" << received_altitude   << "\t" << request_altitude - received_altitude     << std::endl;
            std::cout << "battery  " << request_battery     << "\t" << received_battery    << "\t" << request_battery - received_battery       << std::endl;
            std::cout << "sonar    " << request_sonar       << "\t" << received_sonar      << "\t" << request_sonar - received_sonar           << std::endl;
            std::cout << "lidar    " << request_lidar       << "\t" << received_lidar      << "\t" << request_lidar - received_lidar           << std::endl;
            std::cout << "infra red" << request_infra_red   << "\t" << received_infra_red  << "\t" << request_infra_red - received_infra_red   << std::endl;
            std::cout << "autonom  " << request_autonomous  << "\t" << received_autonomous << "\t" << request_autonomous - received_autonomous << std::endl;
            std::cout << "total:   " << request_total       << "\t" << received_total      << "\t" << request_total - received_total           << std::endl;
            
            t1 = std::chrono::high_resolution_clock::now();
            real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
            std::cout << "freq status   " << received_status     / real_duration << " msg/s" << std::endl;
            std::cout << "freq rc       " << received_rc         / real_duration << " msg/s" << std::endl;
            std::cout << "freq imu      " << received_imu        / real_duration << " msg/s" << std::endl;
            std::cout << "freq attitude " << received_attitude   / real_duration << " msg/s" << std::endl;
            std::cout << "freq motor    " << received_motor      / real_duration << " msg/s" << std::endl;
            std::cout << "freq gps      " << received_gps        / real_duration << " msg/s" << std::endl;
            std::cout << "freq altitude " << received_altitude   / real_duration << " msg/s" << std::endl;
            std::cout << "freq battery  " << received_battery    / real_duration << " msg/s" << std::endl;
            std::cout << "freq sonar    " << received_sonar      / real_duration << " msg/s" << std::endl;
            std::cout << "freq lidar    " << received_lidar      / real_duration << " msg/s" << std::endl;
            std::cout << "freq infra red" << received_infra_red  / real_duration << " msg/s" << std::endl;
            std::cout << "freq autonom  " << received_autonomous / real_duration << " msg/s" << std::endl;
            std::cout << "freq total    " << received_total      / real_duration << " msg/s" << std::endl;
            
            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
        }
        
        // serialcom send requests
        if (loop_counter % 5 == 0) {
            multiwii_serial.prepare_request(MULTIWII_STATUS); request_status++; request_total++;
            multiwii_serial.prepare_request(MULTIWII_ALTITUDE); request_altitude++; request_total++;
            multiwii_serial.prepare_request(MARVIC_LIDAR, PHOENIX_PROTOCOL); request_lidar++; request_total++;
            multiwii_serial.prepare_request(MARVIC_INFRA_RED, PHOENIX_PROTOCOL); request_infra_red++; request_total++;

        } else {
            if (loop_counter % 1 == 0) {
            }

            if (loop_counter % 2 == 0) {
            }
        }
        multiwii_serial.send_from_buffer();

        // receive serial stuff
        while (multiwii_serial.receive_to_buffer() == true){
            // interpret new input
            while (multiwii_serial.read_msg_from_buffer(&input_msg) == true) {
                received_total++;
                if (input_msg.msg_length == 0) {
                    //std::cout << "interpreting_loop >> received request" << std::endl;
                } else {
                    if (input_msg.msg_code == MULTIWII_STATUS) {
                        statusMsg.cycleTime = input_msg.msg_data.multiwii_status.cycleTime;
                        statusMsg.i2c_errors_count = input_msg.msg_data.multiwii_status.i2c_errors_count;
                        status_pub.publish(statusMsg);
                        received_status++;
                    } else if (input_msg.msg_code == MULTIWII_ALTITUDE) {
                        altitudeMsg.estimated_altitude = input_msg.msg_data.multiwii_altitude.estAlt;
                        altitudeMsg.variation = input_msg.msg_data.multiwii_altitude.variation;
                        altitude_pub.publish(altitudeMsg);
                        received_altitude++;
                    } else if ((input_msg.msg_protocol == PHOENIX_PROTOCOL) && (input_msg.msg_code == MARVIC_LIDAR)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance;
                        altitudeMsg.variation = 0;
                        lidar_pub.publish(altitudeMsg);
                        received_lidar++;
                    } else if ((input_msg.msg_protocol == PHOENIX_PROTOCOL) && (input_msg.msg_code == MARVIC_INFRA_RED)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance;
                        altitudeMsg.variation = 0;
                        infra_red_pub.publish(altitudeMsg);
                        received_infra_red++;
                    } else if ((input_msg.msg_protocol == PHOENIX_PROTOCOL) && (input_msg.msg_code == MARVIC_SONAR)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance;
                        altitudeMsg.variation = 0;
                        sonar_pub.publish(altitudeMsg);
                        received_sonar++;
                    } else if ((input_msg.msg_protocol == PHOENIX_PROTOCOL) && (input_msg.msg_code == MARVIC_BAROMETER)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance;
                        altitudeMsg.variation = 0;
                        barometer_pub.publish(altitudeMsg);
                        received_barometer++;
                    }
                }
            }
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


// callbacks
void rc_computer_callback(const sensor_msgs::Joy::ConstPtr& joyMsg) {
    std::cout << "\033[1;31m>>> rc_computer_callback\033[0m"<< std::endl;
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

