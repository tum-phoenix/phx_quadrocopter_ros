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
#include "phx_arduino_uart_bridge/PID_cleanflight.h"
#include "phx_arduino_uart_bridge/PID.h"

#include <chrono>
#include <iostream>
#include "phx_arduino_uart_bridge/serial_com.h"

void rc_direct_callback(const sensor_msgs::Joy::ConstPtr&);
void gps_way_point_callback(const sensor_msgs::NavSatFix::ConstPtr&);
void set_pid_callback(const phx_arduino_uart_bridge::PID_cleanflight::ConstPtr&);

SerialCom serial_interface;                                              // create SerialCom instance

int main(int argc, char **argv)
{
    // init ------------------------------------------------------------------------------------------------
    // ros init
    ros::init(argc, argv, "UART_bridge_naze");
    ros::NodeHandle n;
    std_msgs::Header headerMsg;

    sensor_msgs::Imu imuMsg;
    phx_arduino_uart_bridge::Motor motorMsg;
    phx_arduino_uart_bridge::Status statusMsg;
    phx_arduino_uart_bridge::Altitude altitudeMsg;
    sensor_msgs::NavSatFix gpsMsg;

    sensor_msgs::Joy joyMsg;
    joyMsg.axes = std::vector<float> (4, 0);
    joyMsg.buttons = std::vector<int> (4, 0);
    
    phx_arduino_uart_bridge::PID_cleanflight pidMsg;
    
    // ros init publishers
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("phx/imu", 1);
    ros::Publisher joy_pub = n.advertise<sensor_msgs::Joy>("phx/fc/rc", 1);
    ros::Publisher motor_pub = n.advertise<phx_arduino_uart_bridge::Motor>("phx/fc/motor", 1);
    ros::Publisher status_pub = n.advertise<phx_arduino_uart_bridge::Status>("phx/fc/status", 1);
    ros::Publisher altitude_pub = n.advertise<phx_arduino_uart_bridge::Altitude>("phx/fc/altitude", 1);
    ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("phx/gps", 1);
    ros::Publisher gps_wp_pub = n.advertise<sensor_msgs::NavSatFix>("phx/fc/gps_way_point", 1);
    ros::Publisher gps_home_pub = n.advertise<sensor_msgs::NavSatFix>("phx/fc/gps_home", 1);
    ros::Publisher pid_in_use = n.advertise<phx_arduino_uart_bridge::PID_cleanflight>("phx/fc/pid_in_use", 1);

    // ros init subscribers
    //ros::Subscriber rc_sub = n.subscribe<sensor_msgs::Joy>("phx/fc/rc_computer_direct", 1, rc_direct_callback);
    ros::Subscriber gps_wp = n.subscribe<sensor_msgs::NavSatFix>("phx/gps_way_point", 1, gps_way_point_callback);
    ros::Subscriber set_pid = n.subscribe<phx_arduino_uart_bridge::PID_cleanflight>("phx/fc/pid_set", 1, set_pid_callback);
    
    // ros loop speed (this might interfere with the serial reading and the size of the serial buffer!)
    ros::Rate loop_rate(500);
    
    // serialcom init
    //SerialCom serial_interface;                                              // create SerialCom instance
    serial_interface.set_device("/dev/ttyUSB0");                             // select the device
    //serial_interface.set_device("/dev/ttyAMA0");                             // select the device
    //serial_interface.set_device("/dev/naze");                             // select the device
    serial_interface.set_baudrate(115200);                                   // set the communication baudrate
    serial_interface.set_max_io(250);                                        // set maximum bytes per reading
    serial_interface.init();                                                 // start serial connection
    sleep(5);                                                                // wait for boot loader and calibration
    serial_interface.clear_input_buffer();                                   // clear serial buffer
    Message input_msg;                                                       // the latest received message
    uint32_t loop_counter = 0;                                               // a counter which is used for sending requests

    // MICRO CONTROLLER IDENTIFIER
    uint8_t controller_version = 1;
    uint8_t controller_type = 3;

    // init statistics
    uint32_t request_total = 0;         uint32_t received_total = 0;
    uint32_t request_status = 0;        uint32_t received_status = 0;
    uint32_t request_rc = 0;            uint32_t received_rc = 0;
    uint32_t request_imu = 0;           uint32_t received_imu = 0;
    uint32_t request_attitude = 0;      uint32_t received_attitude = 0;
    uint32_t request_motor = 0;         uint32_t received_motor = 0;
    uint32_t request_gps = 0;           uint32_t received_gps = 0;
    uint32_t request_gps_way_point = 0; uint32_t received_gps_way_point = 0;
    uint32_t request_altitude = 0;      uint32_t received_altitude = 0;
    uint32_t request_battery = 0;       uint32_t received_battery = 0;
    uint32_t request_sonar = 0;         uint32_t received_sonar = 0;
    uint32_t request_autonomous = 0;    uint32_t received_autonomous = 0;

    float prevLAT = -1, prevLON= -1, prevALT= -1;

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
            ROS_INFO("uart bridge NAZE is shutting down due to serial port problem, restarting in 5sec");
            serial_interface.deinitialize();
            sleep(5);
            serial_interface.init();                                                 // start serial connection
            sleep(1);                                                                // wait for boot loader and calibration
            serial_interface.clear_input_buffer();                                   // clear serial buffer
            ROS_INFO("uart bridge NAZE goes back to main loop");
        }
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
            std::cout << "gps_wp   " << request_gps_way_point << "\t" << received_gps_way_point << "\t" << request_gps_way_point - received_gps_way_point << std::endl;
            std::cout << "altitude " << request_altitude    << "\t" << received_altitude   << "\t" << request_altitude - received_altitude     << std::endl;
            std::cout << "battery  " << request_battery     << "\t" << received_battery    << "\t" << request_battery - received_battery       << std::endl;
            std::cout << "sonar    " << request_sonar       << "\t" << received_sonar      << "\t" << request_sonar - received_sonar           << std::endl;
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
            std::cout << "freq gps_wp   " << received_gps_way_point / real_duration << " msg/s" << std::endl;
            std::cout << "freq altitude " << received_altitude   / real_duration << " msg/s" << std::endl;
            std::cout << "freq battery  " << received_battery    / real_duration << " msg/s" << std::endl;
            std::cout << "freq sonar    " << received_sonar      / real_duration << " msg/s" << std::endl;
            std::cout << "freq autonom  " << received_autonomous / real_duration << " msg/s" << std::endl;
            std::cout << "freq total    " << received_total      / real_duration << " msg/s" << std::endl;
            
            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
        }
        
        // serial com send requests
        if (loop_counter % 50 == 0) {
            serial_interface.prepare_request(MULTIWII_STATUS); request_status++; request_total++;
            serial_interface.prepare_request(MULTIWII_PID);
        }
        if (loop_counter % 5 == 0) {
            serial_interface.prepare_request(MULTIWII_MOTOR); request_motor++; request_total++;
            serial_interface.prepare_request(MULTIWII_GPS); request_gps++; request_total++;
            serial_interface.prepare_msg_gps_get_way_point(/* way_point_number = */ 16); request_gps_way_point++; request_total++;
            serial_interface.prepare_msg_gps_get_way_point(/* way_point_number = */ 0); request_gps_way_point++; request_total++;
        } else {
            if (loop_counter % 1 == 0) {
                serial_interface.prepare_request(MULTIWII_ATTITUDE); request_attitude++; request_total++;
            }
            if (loop_counter % 2 == 0) {
                serial_interface.prepare_request(MULTIWII_RC); request_rc++; request_total++;
                serial_interface.prepare_request(MULTIWII_ALTITUDE); request_altitude++; request_total++;
                serial_interface.prepare_request(MULTIWII_IMU); request_imu++; request_total++;
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
                        headerMsg.seq = received_status;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
                        statusMsg.header = headerMsg;
                        statusMsg.cycleTime = input_msg.msg_data.multiwii_status.cycleTime;
                        statusMsg.i2c_errors_count = input_msg.msg_data.multiwii_status.i2c_errors_count;
                        status_pub.publish(statusMsg);
                        received_status++;
                    } else if (input_msg.msg_code == MARVIC_IDENTIFIER) {
                        if ((controller_version != input_msg.msg_data.identifier.version) || (controller_type != input_msg.msg_data.identifier.type)) {
                            ROS_INFO("uart bridge is probably connected to wrong micro controller");
                            ROS_INFO("uart bridge is probably connected to wrong micro controller: type %i version %i", input_msg.msg_data.identifier.version, input_msg.msg_data.identifier.type);
                        }
                        std::cout << "-> received controller info: version " << input_msg.msg_data.identifier.version << " type " << input_msg.msg_data.identifier.type << std::endl;
                        printf("uart bridge is probably connected to wrong micro controller: type %i version %i", input_msg.msg_data.identifier.version, input_msg.msg_data.identifier.type);
                        received_status++;
                    } else if (input_msg.msg_code == MULTIWII_PID) {
                        //std::cout << "-> received controller pid" << std::endl;
                        headerMsg.seq = received_rc;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
                        pidMsg.header = headerMsg;
                        pidMsg.roll.p = input_msg.msg_data.multiwii_pid.p1;
                        pidMsg.roll.i = input_msg.msg_data.multiwii_pid.i1;
                        pidMsg.roll.d = input_msg.msg_data.multiwii_pid.d1;
                        pidMsg.pitch.p = input_msg.msg_data.multiwii_pid.p2;
                        pidMsg.pitch.i = input_msg.msg_data.multiwii_pid.i2;
                        pidMsg.pitch.d = input_msg.msg_data.multiwii_pid.d2;
                        pidMsg.yaw.p = input_msg.msg_data.multiwii_pid.p3;
                        pidMsg.yaw.i = input_msg.msg_data.multiwii_pid.i3;
                        pidMsg.yaw.d = input_msg.msg_data.multiwii_pid.d3;
                        pidMsg.alt.p = input_msg.msg_data.multiwii_pid.p4;
                        pidMsg.alt.i = input_msg.msg_data.multiwii_pid.i4;
                        pidMsg.alt.d = input_msg.msg_data.multiwii_pid.d4;
                        pidMsg.vel.p = input_msg.msg_data.multiwii_pid.p5;
                        pidMsg.vel.i = input_msg.msg_data.multiwii_pid.i5;
                        pidMsg.vel.d = input_msg.msg_data.multiwii_pid.d5;
                        pidMsg.pos.p = input_msg.msg_data.multiwii_pid.p6;
                        pidMsg.pos.i = input_msg.msg_data.multiwii_pid.i6;
                        pidMsg.pos.d = input_msg.msg_data.multiwii_pid.d6;
                        pidMsg.posrate.p = input_msg.msg_data.multiwii_pid.p7;
                        pidMsg.posrate.i = input_msg.msg_data.multiwii_pid.i7;
                        pidMsg.posrate.d = input_msg.msg_data.multiwii_pid.d7;
                        pidMsg.navrate.p = input_msg.msg_data.multiwii_pid.p8;
                        pidMsg.navrate.i = input_msg.msg_data.multiwii_pid.i8;
                        pidMsg.navrate.d = input_msg.msg_data.multiwii_pid.d8;
                        pidMsg.level.p = input_msg.msg_data.multiwii_pid.p9;
                        pidMsg.level.i = input_msg.msg_data.multiwii_pid.i9;
                        pidMsg.level.d = input_msg.msg_data.multiwii_pid.d9;
                        pidMsg.mag.p = input_msg.msg_data.multiwii_pid.p10;
                        pidMsg.mag.i = input_msg.msg_data.multiwii_pid.i10;
                        pidMsg.mag.d = input_msg.msg_data.multiwii_pid.d10;
                        pid_in_use.publish(pidMsg);
                    } else if (input_msg.msg_code == MULTIWII_RC) {
                        headerMsg.seq = received_rc;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
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
                        // if raw_imu data is received this is updated in the imu ros message but not directly published.
                        // the message is only published if fresh attitude data is present.
                        imuMsg.linear_acceleration.x = ((float) input_msg.msg_data.multiwii_raw_imu.accx / 50.);
                        imuMsg.linear_acceleration.y = ((float) input_msg.msg_data.multiwii_raw_imu.accy / 50.);
                        imuMsg.linear_acceleration.z = ((float) input_msg.msg_data.multiwii_raw_imu.accz / 50.);
                        imuMsg.angular_velocity.x = ((float) input_msg.msg_data.multiwii_raw_imu.gyrx / 8192 * 2000);
                        imuMsg.angular_velocity.y = ((float) input_msg.msg_data.multiwii_raw_imu.gyry / 8192 * 2000);
                        imuMsg.angular_velocity.z = ((float) input_msg.msg_data.multiwii_raw_imu.gyrz / 8192 * 2000);
                        received_imu++;
                    } else if (input_msg.msg_code == MULTIWII_ATTITUDE) {
                        headerMsg.seq = received_imu;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
                        imuMsg.header = headerMsg;
                        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(input_msg.msg_data.multiwii_attitude.roll, input_msg.msg_data.multiwii_attitude.pitch, input_msg.msg_data.multiwii_attitude.yaw);
                        imuMsg.orientation = quaternion;
                        imu_pub.publish(imuMsg);
                        received_attitude++;
                    } else if (input_msg.msg_code == MULTIWII_MOTOR) {
                        headerMsg.seq = received_motor;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
                        motorMsg.header = headerMsg;
                        motorMsg.motor0 = input_msg.msg_data.multiwii_motor.motor0;
                        motorMsg.motor1 = input_msg.msg_data.multiwii_motor.motor1;
                        motorMsg.motor2 = input_msg.msg_data.multiwii_motor.motor2;
                        motorMsg.motor3 = input_msg.msg_data.multiwii_motor.motor3;
                        motor_pub.publish(motorMsg);
                        received_motor++;
                    } else if (input_msg.msg_code == MULTIWII_GPS) {
                        headerMsg.seq = received_gps;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
                        gpsMsg.header = headerMsg;
                        gpsMsg.latitude = ((float) fix_int32(&input_msg.msg_data.multiwii_gps.coordLAT)) / 10000000.0;
                        gpsMsg.longitude = ((float) fix_int32(&input_msg.msg_data.multiwii_gps.coordLON)) / 10000000.0;
                        gpsMsg.altitude = input_msg.msg_data.multiwii_gps.altitude;
                        gps_pub.publish(gpsMsg);
                        received_gps++;
                    } else if (input_msg.msg_code == MULTIWII_GPS_WP) {
                        headerMsg.seq = received_gps_way_point;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
                        if (input_msg.msg_data.multiwii_gps_way_point.wp_number == 16) {
                            //std::cout << " publishing way point" << std::endl;
                            gpsMsg.header = headerMsg;
                            gpsMsg.latitude = ((float) fix_int32(&input_msg.msg_data.multiwii_gps_way_point.coordLAT)) / 10000000.0;
                            gpsMsg.longitude = ((float) fix_int32(&input_msg.msg_data.multiwii_gps_way_point.coordLON)) / 10000000.0;
                            gpsMsg.altitude = input_msg.msg_data.multiwii_gps_way_point.altitude;
                            
                            if ((gpsMsg.latitude != prevLAT ) || ( gpsMsg.longitude !=  prevLON) || ( gpsMsg.altitude != prevALT)) {
                                gps_wp_pub.publish(gpsMsg);
                            }
                            
                            prevLAT = gpsMsg.latitude;
                            prevLON = gpsMsg.longitude;
                            prevALT = gpsMsg.altitude;
                            
                        } else if (input_msg.msg_data.multiwii_gps_way_point.wp_number == 0) {
                            //std::cout << " publishing home point" << std::endl;
                            gpsMsg.header = headerMsg;
                            gpsMsg.latitude = ((float) fix_int32(&input_msg.msg_data.multiwii_gps_way_point.coordLAT)) / 10000000.0;
                            gpsMsg.longitude = ((float) fix_int32(&input_msg.msg_data.multiwii_gps_way_point.coordLON)) / 10000000.0;
                            gpsMsg.altitude = input_msg.msg_data.multiwii_gps_way_point.altitude;
                            gps_home_pub.publish(gpsMsg);
                        }
                        received_gps_way_point++;
                    } else if (input_msg.msg_code == MULTIWII_ALTITUDE) {
                        headerMsg.seq = received_altitude;
                        headerMsg.stamp = ros::Time::now();
                        headerMsg.frame_id = "naze_fc";
                        altitudeMsg.header = headerMsg;
                        altitudeMsg.estimated_altitude = input_msg.msg_data.multiwii_altitude.estAlt;
                        altitudeMsg.variation = input_msg.msg_data.multiwii_altitude.variation;
                        altitude_pub.publish(altitudeMsg);
                        received_altitude++;
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
    std::cout << "status   " << request_status      << "\t" << received_status     << "\t" << request_status - received_status         << std::endl;
    std::cout << "rc       " << request_rc          << "\t" << received_rc         << "\t" << request_rc - received_rc                 << std::endl;
    std::cout << "imu      " << request_imu         << "\t" << received_imu        << "\t" << request_imu - received_imu               << std::endl;
    std::cout << "attitude " << request_attitude    << "\t" << received_attitude   << "\t" << request_attitude - received_attitude     << std::endl;
    std::cout << "motor    " << request_motor       << "\t" << received_motor      << "\t" << request_motor - received_motor           << std::endl;
    std::cout << "gps      " << request_gps         << "\t" << received_gps        << "\t" << request_gps - received_gps               << std::endl;
    std::cout << "gps_wp   " << request_gps_way_point << "\t" << received_gps_way_point << "\t" << request_gps_way_point - received_gps_way_point << std::endl;
    std::cout << "altitude " << request_altitude    << "\t" << received_altitude   << "\t" << request_altitude - received_altitude     << std::endl;
    std::cout << "battery  " << request_battery     << "\t" << received_battery    << "\t" << request_battery - received_battery       << std::endl;
    std::cout << "sonar    " << request_sonar       << "\t" << received_sonar      << "\t" << request_sonar - received_sonar           << std::endl;
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
    std::cout << "freq gps_wp   " << received_gps_way_point / real_duration << " msg/s" << std::endl;
    std::cout << "freq altitude " << received_altitude   / real_duration << " msg/s" << std::endl;
    std::cout << "freq battery  " << received_battery    / real_duration << " msg/s" << std::endl;
    std::cout << "freq sonar    " << received_sonar      / real_duration << " msg/s" << std::endl;
    std::cout << "freq autonom  " << received_autonomous / real_duration << " msg/s" << std::endl;
    std::cout << "freq total    " << received_total      / real_duration << " msg/s" << std::endl;

    std::cout << "End " << std::endl;

    std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
    std::cout << " communication took " << real_duration << " real seconds" << std::endl;
    std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
    
    return 0;
}

// callbacks
void rc_direct_callback(const sensor_msgs::Joy::ConstPtr& joyMsg) {
    std::cout << "\033[1;31m>>> rc_direct_callback\033[0m"<< std::endl;
    serial_interface.prepare_msg_rc((uint16_t) (*joyMsg).axes[3],
                                    (uint16_t) joyMsg->axes[1],
                                    (uint16_t) joyMsg->axes[0],
                                    (uint16_t) joyMsg->axes[2],
                                    (uint16_t) joyMsg->buttons[0],
                                    (uint16_t) joyMsg->buttons[1],
                                    (uint16_t) joyMsg->buttons[2],
                                    (uint16_t) joyMsg->buttons[3]);
    serial_interface.send_from_buffer();
}

void gps_way_point_callback(const sensor_msgs::NavSatFix::ConstPtr& set_gps_way_point) {
    std::cout << "\033[1;31m>>> gps_way_point_callback\033[0m"<< std::endl;
    serial_interface.prepare_msg_gps_set_way_point((uint8_t) 16,
                                                   (uint32_t) (set_gps_way_point->latitude * 10000000.0),
                                                   (uint32_t) (set_gps_way_point->longitude * 10000000.0),
                                                   (uint32_t) 0, //set_gps_way_point->altitude,
                                                   (uint16_t) 0,
                                                   (uint16_t) 0,
                                                   (uint8_t) 0);
    serial_interface.send_from_buffer();
}

void set_pid_callback(const phx_arduino_uart_bridge::PID_cleanflight::ConstPtr& set_pid) {
    std::cout << "\033[1;31m>>> set_pid_callback  not implemented jet\033[0m"<< std::endl;
    serial_interface.prepare_msg_pid((uint8_t) (set_pid->roll.p),
                                     (uint8_t) (set_pid->roll.i),
                                     (uint8_t) (set_pid->roll.d),
                                     (uint8_t) (set_pid->pitch.p),
                                     (uint8_t) (set_pid->pitch.i),
                                     (uint8_t) (set_pid->pitch.d),
                                     (uint8_t) (set_pid->yaw.p),
                                     (uint8_t) (set_pid->yaw.i),
                                     (uint8_t) (set_pid->yaw.d),
                                     (uint8_t) (set_pid->alt.p),
                                     (uint8_t) (set_pid->alt.i),
                                     (uint8_t) (set_pid->alt.d),
                                     (uint8_t) (set_pid->vel.p),
                                     (uint8_t) (set_pid->vel.i),
                                     (uint8_t) (set_pid->vel.d),
                                     (uint8_t) (set_pid->pos.p),
                                     (uint8_t) (set_pid->pos.i),
                                     (uint8_t) (set_pid->pos.d),
                                     (uint8_t) (set_pid->posrate.p),
                                     (uint8_t) (set_pid->posrate.i),
                                     (uint8_t) (set_pid->posrate.d),
                                     (uint8_t) (set_pid->navrate.p),
                                     (uint8_t) (set_pid->navrate.i),
                                     (uint8_t) (set_pid->navrate.d),
                                     (uint8_t) (set_pid->level.p),
                                     (uint8_t) (set_pid->level.i),
                                     (uint8_t) (set_pid->level.d),
                                     (uint8_t) (set_pid->mag.p),
                                     (uint8_t) (set_pid->mag.i),
                                     (uint8_t) (set_pid->mag.d));
    serial_interface.send_from_buffer();
}