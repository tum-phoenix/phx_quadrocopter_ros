#include <unistd.h>
#include <ctime>
#include <sstream>
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/NavSatFix.h"
#include "phx_uart_msp_bridge/Motor.h"
#include "phx_uart_msp_bridge/Status.h"
#include "phx_uart_msp_bridge/Altitude.h"
#include "phx_uart_msp_bridge/LEDstrip.h"
#include "phx_uart_msp_bridge/LED.h"
#include "phx_uart_msp_bridge/Servo.h"
#include "phx_uart_msp_bridge/RemoteControl.h"

#include <chrono>
#include <iostream>
#include "phx_uart_msp_bridge/serial_com.h"

void rc_direct_callback(const phx_uart_msp_bridge::RemoteControl::ConstPtr&);
void led_strip_0_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr&);
void led_strip_1_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr&);
void led_strip_2_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr&);
void led_strip_3_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr&);
void led_single_callback(const phx_uart_msp_bridge::LED::ConstPtr&);
void servo_callback(const phx_uart_msp_bridge::Servo::ConstPtr&);

SerialCom serial_interface;                                              // create SerialCom instance

int main(int argc, char **argv)
{
    // init ------------------------------------------------------------------------------------------------
    // ros init
    ros::init(argc, argv, "UART_bridge_marvicRCaltitude");
    ros::NodeHandle n;
    std_msgs::Header headerMsg;
    sensor_msgs::Imu imuMsg;
    phx_uart_msp_bridge::RemoteControl RemoteControlMsg;
    phx_uart_msp_bridge::Motor motorMsg;
    phx_uart_msp_bridge::Status statusMsg;
    phx_uart_msp_bridge::Altitude altitudeMsg;
    sensor_msgs::NavSatFix gpsMsg;
    
    // ros init publishers
    ros::Publisher active_rc_pub = n.advertise<phx_uart_msp_bridge::RemoteControl>("phx/marvicRC/rc_input", 1);
    ros::Publisher status_pub = n.advertise<phx_uart_msp_bridge::Status>("phx/marvicRC/status", 1);

    ros::Publisher altitude_pub = n.advertise<phx_uart_msp_bridge::Altitude>("phx/marvicAltitude/altitude", 1);
    ros::Publisher sonar_pub = n.advertise<phx_uart_msp_bridge::Altitude>("phx/marvicAltitude/sonar", 1);
    ros::Publisher lidar_pub = n.advertise<phx_uart_msp_bridge::Altitude>("phx/marvicAltitude/lidar", 1);
    ros::Publisher infra_red_pub = n.advertise<phx_uart_msp_bridge::Altitude>("phx/marvicAltitude/infra_red", 1);
    ros::Publisher barometer_pub = n.advertise<phx_uart_msp_bridge::Altitude>("phx/marvicAltitude/barometer", 1);

    // ros init subscriber
    //ros::Subscriber rc_sub = n.subscribe<phx_uart_msp_bridge::RemoteControl>("phx/rc_computer", 1, rc_computer_callback);
    ros::Subscriber led_strip_0_sub = n.subscribe<phx_uart_msp_bridge::LEDstrip>("phx/led/led_strip_0", 1, led_strip_0_callback);
    ros::Subscriber led_strip_1_sub = n.subscribe<phx_uart_msp_bridge::LEDstrip>("phx/led/led_strip_1", 1, led_strip_1_callback);
    ros::Subscriber led_strip_2_sub = n.subscribe<phx_uart_msp_bridge::LEDstrip>("phx/led/led_strip_2", 1, led_strip_2_callback);
    ros::Subscriber led_strip_3_sub = n.subscribe<phx_uart_msp_bridge::LEDstrip>("phx/led/led_strip_3", 1, led_strip_3_callback);
    ros::Subscriber led_single_sub = n.subscribe<phx_uart_msp_bridge::LED>("phx/led/led_single", 1, led_single_callback);
    ros::Subscriber servo_sub = n.subscribe<phx_uart_msp_bridge::Servo>("phx/marvicServo/servo_cmd", 1, servo_callback);

    // ros loop speed (this might interfere with the serial reading and the size of the serial buffer!)
    ros::Rate loop_rate(500);
    
    // serialcom init
    serial_interface.set_device("/dev/marvic");                             // select the device
    //serial_interface.set_device("/dev/ttyUSB1");
    serial_interface.set_baudrate(115200);                                   // set the communication baudrate
    serial_interface.set_max_io(250);                                        // set maximum bytes per reading
    serial_interface.init();                                                 // start serial connection
    sleep(1);                                                                // wait for arduino boot loader
    serial_interface.clear_input_buffer();                                   // clear serial buffer
    Message input_msg;                                                       // the latest received message
    uint32_t loop_counter = 0;                                               // a counter which is used for sending requests
    
    // init statistics
    uint32_t request_total = 0;         uint32_t received_total = 0;
    uint32_t request_status = 0;        uint32_t received_status = 0;
    uint32_t request_rc = 0;            uint32_t received_rc = 0;
    uint32_t request_altitude = 0;      uint32_t received_altitude = 0;
    uint32_t request_sonar = 0;         uint32_t received_sonar = 0;
    uint32_t request_lidar = 0;         uint32_t received_lidar = 0;
    uint32_t request_infra_red = 0;     uint32_t received_infra_red = 0;
    uint32_t request_barometer = 0;     uint32_t received_barometer = 0;

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
            ROS_INFO("uart bridge marvicRCaltitude is shutting down due to serial port problem, restarting in 5sec");
            serial_interface.deinitialize();
            sleep(5);
            serial_interface.init();                                                 // start serial connection
            sleep(1);                                                                // wait for boot loader and calibration
            serial_interface.clear_input_buffer();                                   // clear serial buffer
            ROS_INFO("uart bridge marvicRCaltitude goes back to main loop");
        }
        loop_counter++;
        // print statistics from while to while
        if (loop_counter % 1000 == 0) {
            system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
            std::cout << "       request\tin\tloss" << std::endl;
            std::cout << "status    " << request_status      << "\t" << received_status     << "\t" << request_status - received_status         << std::endl;
            std::cout << "rc        " << request_rc          << "\t" << received_rc         << "\t" << request_rc - received_rc                 << std::endl;
            std::cout << "altitude  " << request_altitude    << "\t" << received_altitude   << "\t" << request_altitude - received_altitude     << std::endl;
            std::cout << "sonar     " << request_sonar       << "\t" << received_sonar      << "\t" << request_sonar - received_sonar           << std::endl;
            std::cout << "barometer " << request_barometer   << "\t" << received_barometer  << "\t" << request_barometer - received_barometer   << std::endl;
            std::cout << "LIDAR     " << request_lidar       << "\t" << received_lidar      << "\t" << request_lidar - received_lidar           << std::endl;
            std::cout << "infra red " << request_infra_red   << "\t" << received_infra_red  << "\t" << request_infra_red - received_infra_red   << std::endl;
            std::cout << "total:    " << request_total       << "\t" << received_total      << "\t" << request_total - received_total           << std::endl;
            
            t1 = std::chrono::high_resolution_clock::now();
            real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
            std::cout << "freq status    " << received_status     / real_duration << " msg/s" << std::endl;
            std::cout << "freq rc        " << received_rc         / real_duration << " msg/s" << std::endl;
            std::cout << "freq altitude  " << received_altitude   / real_duration << " msg/s" << std::endl;
            std::cout << "freq sonar     " << received_sonar      / real_duration << " msg/s" << std::endl;
            std::cout << "freq barometer " << received_barometer  / real_duration << " msg/s" << std::endl;
            std::cout << "freq LIDAR     " << received_lidar      / real_duration << " msg/s" << std::endl;
            std::cout << "freq infra red " << received_infra_red  / real_duration << " msg/s" << std::endl;
            std::cout << "freq total     " << received_total      / real_duration << " msg/s" << std::endl;
            
            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
        }
        
        // serialcom send requests
        if (loop_counter % 10 == 0) {
            serial_interface.prepare_request(MULTIWII_STATUS); request_status++; request_total++;
            serial_interface.prepare_request(MARVIC_BAROMETER, PHOENIX_RC_PROTOCOL); request_barometer++; request_total++;
        } else {
            if (loop_counter % 2 == 0) {
                serial_interface.prepare_request(MULTIWII_RC); request_rc++; request_total++;
                serial_interface.prepare_request(MULTIWII_ALTITUDE); request_altitude++; request_total++;
            }
            if (loop_counter % 3 == 0) {
            serial_interface.prepare_request(MARVIC_LIDAR, PHOENIX_RC_PROTOCOL); request_lidar++; request_total++;
            serial_interface.prepare_request(MARVIC_INFRA_RED, PHOENIX_RC_PROTOCOL); request_infra_red++; request_total++;
            serial_interface.prepare_request(MARVIC_SONAR, PHOENIX_RC_PROTOCOL); request_sonar++; request_total++;
            }
        }
        serial_interface.send_from_buffer();

        // receive serial stuff
        while (serial_interface.receive_to_buffer() == true){
            // interpret new input
            while (serial_interface.read_msg_from_buffer(&input_msg) == true) {
                received_total++;
                if (input_msg.msg_length == 0) {
                    //std::cout << "interpreting_loop >> received request" << std::endl;
                } else {
                    if (input_msg.msg_code == MULTIWII_STATUS) {
                        statusMsg.cycleTime = input_msg.msg_data.multiwii_status.cycleTime;
                        statusMsg.i2c_errors_count = input_msg.msg_data.multiwii_status.i2c_errors_count;
                        status_pub.publish(statusMsg);
                        received_status++;
                    } else if (input_msg.msg_code == MULTIWII_RC) {
                        headerMsg.seq = received_rc;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicRC";
                        RemoteControlMsg.header = headerMsg;
                        RemoteControlMsg.roll = (uint16_t) input_msg.msg_data.multiwii_rc.roll;
                        RemoteControlMsg.pitch = (uint16_t) input_msg.msg_data.multiwii_rc.pitch;
                        RemoteControlMsg.yaw = (uint16_t) input_msg.msg_data.multiwii_rc.yaw;
                        RemoteControlMsg.throttle = (uint16_t) input_msg.msg_data.multiwii_rc.throttle;
                        RemoteControlMsg.aux1 = (uint16_t) input_msg.msg_data.multiwii_rc.aux1;
                        RemoteControlMsg.aux2 = (uint16_t) input_msg.msg_data.multiwii_rc.aux2;
                        RemoteControlMsg.aux3 = (uint16_t) input_msg.msg_data.multiwii_rc.aux3;
                        RemoteControlMsg.aux4 = (uint16_t) input_msg.msg_data.multiwii_rc.aux4;
                        active_rc_pub.publish(RemoteControlMsg);
                        received_rc++;
                    } else if (input_msg.msg_code == MULTIWII_ALTITUDE) {
                        altitudeMsg.estimated_altitude = input_msg.msg_data.multiwii_altitude.estAlt / 100.;
                        altitudeMsg.variation = input_msg.msg_data.multiwii_altitude.variation / 100.;
                        altitude_pub.publish(altitudeMsg);
                        received_altitude++;
                    } else if ((input_msg.msg_protocol == PHOENIX_RC_PROTOCOL) && (input_msg.msg_code == MARVIC_LIDAR)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance / 100.;
                        altitudeMsg.variation = 0;
                        lidar_pub.publish(altitudeMsg);
                        received_lidar++;
                    } else if ((input_msg.msg_protocol == PHOENIX_RC_PROTOCOL) && (input_msg.msg_code == MARVIC_INFRA_RED)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance / 100.;
                        altitudeMsg.variation = 0;
                        infra_red_pub.publish(altitudeMsg);
                        received_infra_red++;
                    } else if ((input_msg.msg_protocol == PHOENIX_RC_PROTOCOL) && (input_msg.msg_code == MARVIC_SONAR)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance / 100.;
                        altitudeMsg.variation = 0;
                        sonar_pub.publish(altitudeMsg);
                        received_sonar++;
                    } else if ((input_msg.msg_protocol == PHOENIX_RC_PROTOCOL) && (input_msg.msg_code == MARVIC_BAROMETER)) {
                        headerMsg.seq = input_msg.msg_data.marvic_altitude.millisecond_time_stamp;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "marvicAltitude";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.marvic_altitude.distance / 100.;
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
    ROS_INFO("uart bridge marvicRCaltitude is shutting down");
    serial_interface.deinitialize();
    
    
    system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::cout << "   request\tin\tloss" << std::endl;
    std::cout << "status    " << request_status      << "\t" << received_status     << "\t" << request_status - received_status         << std::endl;
    std::cout << "rc        " << request_rc          << "\t" << received_rc         << "\t" << request_rc - received_rc                 << std::endl;
    std::cout << "altitude  " << request_altitude    << "\t" << received_altitude   << "\t" << request_altitude - received_altitude     << std::endl;
    std::cout << "sonar     " << request_sonar       << "\t" << received_sonar      << "\t" << request_sonar - received_sonar           << std::endl;
    std::cout << "barometer " << request_barometer   << "\t" << received_barometer  << "\t" << request_barometer - received_barometer   << std::endl;
    std::cout << "LIDAR     " << request_lidar       << "\t" << received_lidar      << "\t" << request_lidar - received_lidar           << std::endl;
    std::cout << "infra red " << request_infra_red   << "\t" << received_infra_red  << "\t" << request_infra_red - received_infra_red   << std::endl;
    std::cout << "total:    " << request_total       << "\t" << received_total      << "\t" << request_total - received_total           << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
    std::cout << "freq status    " << received_status     / real_duration << " msg/s" << std::endl;
    std::cout << "freq rc        " << received_rc         / real_duration << " msg/s" << std::endl;
    std::cout << "freq altitude  " << received_altitude   / real_duration << " msg/s" << std::endl;
    std::cout << "freq sonar     " << received_sonar      / real_duration << " msg/s" << std::endl;
    std::cout << "freq barometer " << received_barometer  / real_duration << " msg/s" << std::endl;
    std::cout << "freq LIDAR     " << received_lidar      / real_duration << " msg/s" << std::endl;
    std::cout << "freq infra red " << received_infra_red  / real_duration << " msg/s" << std::endl;
    std::cout << "freq total     " << received_total      / real_duration << " msg/s" << std::endl;
    std::cout << "End " << std::endl;
    
    std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
    std::cout << " communication took " << real_duration << " real seconds" << std::endl;
    std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
    
    return 0;
}


// callbacks
void rc_computer_callback(const phx_uart_msp_bridge::RemoteControl::ConstPtr& RemoteControlMsg_input) {
    std::cout << "\033[1;31m>>> rc_computer_callback\033[0m"<< std::endl;
    serial_interface.prepare_msg_rc((uint16_t) RemoteControlMsg_input->roll,
                                    (uint16_t) RemoteControlMsg_input->yaw,
                                    (uint16_t) RemoteControlMsg_input->throttle,
                                    (uint16_t) RemoteControlMsg_input->pitch,
                                    (uint16_t) RemoteControlMsg_input->aux1,
                                    (uint16_t) RemoteControlMsg_input->aux2,
                                    (uint16_t) RemoteControlMsg_input->aux3,
                                    (uint16_t) RemoteControlMsg_input->aux4);
    serial_interface.send_from_buffer();
}

void led_strip_0_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr& led_command) {
    std::cout << "\033[1;31m>>> led_arm_0_callback" << "\033[0m"<< std::endl;
    serial_interface.prepare_msg_led_strip((uint8_t) led_command->led_0_r,
                                           (uint8_t) led_command->led_0_g,
                                           (uint8_t) led_command->led_0_b,
                                           (uint8_t) led_command->led_1_r,
                                           (uint8_t) led_command->led_1_g,
                                           (uint8_t) led_command->led_1_b,
                                           (uint8_t) led_command->led_2_r,
                                           (uint8_t) led_command->led_2_g,
                                           (uint8_t) led_command->led_2_b,
                                           (uint8_t) led_command->led_3_r,
                                           (uint8_t) led_command->led_3_g,
                                           (uint8_t) led_command->led_3_b,
                                           (uint8_t) led_command->led_4_r,
                                           (uint8_t) led_command->led_4_g,
                                           (uint8_t) led_command->led_4_b,
                                           (uint8_t) led_command->led_5_r,
                                           (uint8_t) led_command->led_5_g,
                                           (uint8_t) led_command->led_5_b,
                                           (uint8_t) led_command->led_6_r,
                                           (uint8_t) led_command->led_6_g,
                                           (uint8_t) led_command->led_6_b,
                                           (uint8_t) led_command->led_7_r,
                                           (uint8_t) led_command->led_7_g,
                                           (uint8_t) led_command->led_7_b,
                                           (uint8_t) led_command->led_8_r,
                                           (uint8_t) led_command->led_8_g,
                                           (uint8_t) led_command->led_8_b,
                                           (uint8_t) led_command->led_9_r,
                                           (uint8_t) led_command->led_9_g,
                                           (uint8_t) led_command->led_9_b,
                                           0);
    serial_interface.send_from_buffer();
}

void led_strip_1_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr& led_command) {
    std::cout << "\033[1;31m>>> led_arm_1_callback" << "\033[0m"<< std::endl;
    serial_interface.prepare_msg_led_strip((uint8_t) led_command->led_0_r,
                                           (uint8_t) led_command->led_0_g,
                                           (uint8_t) led_command->led_0_b,
                                           (uint8_t) led_command->led_1_r,
                                           (uint8_t) led_command->led_1_g,
                                           (uint8_t) led_command->led_1_b,
                                           (uint8_t) led_command->led_2_r,
                                           (uint8_t) led_command->led_2_g,
                                           (uint8_t) led_command->led_2_b,
                                           (uint8_t) led_command->led_3_r,
                                           (uint8_t) led_command->led_3_g,
                                           (uint8_t) led_command->led_3_b,
                                           (uint8_t) led_command->led_4_r,
                                           (uint8_t) led_command->led_4_g,
                                           (uint8_t) led_command->led_4_b,
                                           (uint8_t) led_command->led_5_r,
                                           (uint8_t) led_command->led_5_g,
                                           (uint8_t) led_command->led_5_b,
                                           (uint8_t) led_command->led_6_r,
                                           (uint8_t) led_command->led_6_g,
                                           (uint8_t) led_command->led_6_b,
                                           (uint8_t) led_command->led_7_r,
                                           (uint8_t) led_command->led_7_g,
                                           (uint8_t) led_command->led_7_b,
                                           (uint8_t) led_command->led_8_r,
                                           (uint8_t) led_command->led_8_g,
                                           (uint8_t) led_command->led_8_b,
                                           (uint8_t) led_command->led_9_r,
                                           (uint8_t) led_command->led_9_g,
                                           (uint8_t) led_command->led_9_b,
                                           1);
    serial_interface.send_from_buffer();
}

void led_strip_2_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr& led_command) {
    std::cout << "\033[1;31m>>> led_arm_2_callback" << "\033[0m"<< std::endl;
    serial_interface.prepare_msg_led_strip((uint8_t) led_command->led_0_r,
                                           (uint8_t) led_command->led_0_g,
                                           (uint8_t) led_command->led_0_b,
                                           (uint8_t) led_command->led_1_r,
                                           (uint8_t) led_command->led_1_g,
                                           (uint8_t) led_command->led_1_b,
                                           (uint8_t) led_command->led_2_r,
                                           (uint8_t) led_command->led_2_g,
                                           (uint8_t) led_command->led_2_b,
                                           (uint8_t) led_command->led_3_r,
                                           (uint8_t) led_command->led_3_g,
                                           (uint8_t) led_command->led_3_b,
                                           (uint8_t) led_command->led_4_r,
                                           (uint8_t) led_command->led_4_g,
                                           (uint8_t) led_command->led_4_b,
                                           (uint8_t) led_command->led_5_r,
                                           (uint8_t) led_command->led_5_g,
                                           (uint8_t) led_command->led_5_b,
                                           (uint8_t) led_command->led_6_r,
                                           (uint8_t) led_command->led_6_g,
                                           (uint8_t) led_command->led_6_b,
                                           (uint8_t) led_command->led_7_r,
                                           (uint8_t) led_command->led_7_g,
                                           (uint8_t) led_command->led_7_b,
                                           (uint8_t) led_command->led_8_r,
                                           (uint8_t) led_command->led_8_g,
                                           (uint8_t) led_command->led_8_b,
                                           (uint8_t) led_command->led_9_r,
                                           (uint8_t) led_command->led_9_g,
                                           (uint8_t) led_command->led_9_b,
                                           2);
    serial_interface.send_from_buffer();
}

void led_strip_3_callback(const phx_uart_msp_bridge::LEDstrip::ConstPtr& led_command) {
    std::cout << "\033[1;31m>>> led_arm_3_callback" << "\033[0m"<< std::endl;
    serial_interface.prepare_msg_led_strip((uint8_t) led_command->led_0_r,
                                           (uint8_t) led_command->led_0_g,
                                           (uint8_t) led_command->led_0_b,
                                           (uint8_t) led_command->led_1_r,
                                           (uint8_t) led_command->led_1_g,
                                           (uint8_t) led_command->led_1_b,
                                           (uint8_t) led_command->led_2_r,
                                           (uint8_t) led_command->led_2_g,
                                           (uint8_t) led_command->led_2_b,
                                           (uint8_t) led_command->led_3_r,
                                           (uint8_t) led_command->led_3_g,
                                           (uint8_t) led_command->led_3_b,
                                           (uint8_t) led_command->led_4_r,
                                           (uint8_t) led_command->led_4_g,
                                           (uint8_t) led_command->led_4_b,
                                           (uint8_t) led_command->led_5_r,
                                           (uint8_t) led_command->led_5_g,
                                           (uint8_t) led_command->led_5_b,
                                           (uint8_t) led_command->led_6_r,
                                           (uint8_t) led_command->led_6_g,
                                           (uint8_t) led_command->led_6_b,
                                           (uint8_t) led_command->led_7_r,
                                           (uint8_t) led_command->led_7_g,
                                           (uint8_t) led_command->led_7_b,
                                           (uint8_t) led_command->led_8_r,
                                           (uint8_t) led_command->led_8_g,
                                           (uint8_t) led_command->led_8_b,
                                           (uint8_t) led_command->led_9_r,
                                           (uint8_t) led_command->led_9_g,
                                           (uint8_t) led_command->led_9_b,
                                          3);
    serial_interface.send_from_buffer();
}

void led_single_callback(const phx_uart_msp_bridge::LED::ConstPtr& led_command) {
    std::cout << "\033[1;31m>>> led_single_callback sending" << "\033[0m"<< std::endl;
    serial_interface.prepare_msg_single_led((uint8_t) led_command->led_index,
                                            (uint8_t) led_command->strip_id,
                                            (uint8_t) led_command->led_r,
                                            (uint8_t) led_command->led_g,
                                            (uint8_t) led_command->led_b);
    serial_interface.send_from_buffer();
}

void servo_callback(const phx_uart_msp_bridge::Servo::ConstPtr& servo_values) {
    std::cout << "\033[1;31m>>> servo_direct_callback\033[0m"<< std::endl;
    serial_interface.prepare_msg_servo_small((uint16_t) servo_values->servo0,
                                             (uint16_t) servo_values->servo1,
                                             (uint16_t) servo_values->servo2,
                                             (uint16_t) servo_values->servo3,
                                             PHOENIX_LED_PROTOCOL);
    serial_interface.send_from_buffer();
}
