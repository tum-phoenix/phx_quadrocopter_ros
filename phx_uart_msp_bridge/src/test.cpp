#include <chrono> 
#include <iostream>
#include <sstream>
#include "phx_arduino_uart_bridge/serial_com.h"

int main(int argc, char **argv)
{
    int32_t loop_counter = 10000;
    bool debug_printout = false;
    
    // establish connection:
    SerialCom multiwii_serial;                                              // create SerialCom instance
    if (argc >= 2) {
        multiwii_serial.set_device(argv[1]);
    } else {
        multiwii_serial.set_device("/dev/ttyACM0");  //ttyUSB0");
    }
    if (argc >= 3) {
        std::istringstream iss(argv[2]);
        uint32_t x;
        if (!(iss >> x)) {
            multiwii_serial.set_baudrate(x);
        } else {
            multiwii_serial.set_baudrate(115200);
        }
    } else {
        multiwii_serial.set_baudrate(115200);
    }

    multiwii_serial.set_max_io(200);
    multiwii_serial.init();                                                 // initialize serial connection
    sleep(2);                                                               // wait for arduino bootloader
    Message input_msg;
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
    uint32_t request_autonomous = 0;    uint32_t received_autonomous = 0;

    std::cout << " start up done" << std::endl;

    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;
    double system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;


    // start continuous sending
    multiwii_serial.prepare_msg_continuous_sending(1);
    multiwii_serial.prepare_msg_continuous_sending(1);
    multiwii_serial.prepare_msg_continuous_sending(1);
    multiwii_serial.send_from_buffer();
    
    while (loop_counter-- > 0) {
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
            std::cout << "freq autonom  " << received_autonomous / real_duration << " msg/s" << std::endl;
            std::cout << "freq total    " << received_total      / real_duration << " msg/s" << std::endl;
            
            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
        }

        // send requests
        if (loop_counter % 5 == 0) {
            //multiwii_serial.prepare_request(MULTIWII_RC); request_rc++; request_total++;
            //multiwii_serial.prepare_request(MULTIWII_IMU); request_imu++; request_total++;
            //multiwii_serial.prepare_request(MULTIWII_ATTITUDE); request_attitude++; request_total++;
//            multiwii_serial.prepare_request(MULTIWII_MOTOR); request_motor++; request_total++;
            multiwii_serial.prepare_request(MULTIWII_GPS); request_gps++; request_total++;
            multiwii_serial.prepare_request(MULTIWII_STATUS); request_status++; request_total++;
            multiwii_serial.prepare_request(MULTIWII_ALTITUDE, 'P'); request_altitude++; request_total++;
            multiwii_serial.prepare_request(MARVIC_BATTERY); request_battery++; request_total++;
            multiwii_serial.send_from_buffer();
        } else {
            if (loop_counter % 1 == 0) {
                multiwii_serial.prepare_request(MULTIWII_RC); request_rc++; request_total++;
//                multiwii_serial.prepare_request(MULTIWII_ATTITUDE); request_attitude++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_IMU); request_imu++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_MOTOR); request_motor++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_GPS); request_gps++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_STATUS); request_status++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_ALTITUDE); request_altitude++; request_total++;
            }

            if (loop_counter % 2 == 0) {
                //multiwii_serial.prepare_request(MULTIWII_RC); request_rc++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_ATTITUDE); request_attitude++; request_total++;
                multiwii_serial.prepare_request(MULTIWII_IMU); request_imu++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_MOTOR); request_motor++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_GPS); request_gps++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_STATUS); request_status++; request_total++;
                //multiwii_serial.prepare_request(MULTIWII_ALTITUDE); request_altitude++; request_total++;
            }
        }
        //multiwii_serial.send_from_buffer();

        // receive serial stuff
        while (multiwii_serial.receive_to_buffer() == true){
            // interpret new input
            while (multiwii_serial.read_msg_from_buffer(&input_msg) == true) {
                received_total++;
                if (input_msg.msg_length == 0) {
                    //std::cout << "interpreting_loop >> received request" << std::endl;
                } else {
                    if (input_msg.msg_code == MULTIWII_STATUS) {
                        received_status++;
                    } else if (input_msg.msg_code == MULTIWII_RC) {
                        received_rc++;
                    } else if (input_msg.msg_code == MULTIWII_IMU) {
                        received_imu++;
                    } else if ((input_msg.msg_protocol == 'M') && (input_msg.msg_code == MULTIWII_ATTITUDE)) {
                        received_attitude++;
                    } else if (input_msg.msg_code == MULTIWII_MOTOR) {
                        received_motor++;
                    } else if (input_msg.msg_code == MULTIWII_GPS) {
                        received_gps++;
                    } else if (input_msg.msg_code == MULTIWII_ALTITUDE) {
                        received_altitude++;
                    } else if (input_msg.msg_code == MARVIC_BATTERY) {
                        received_battery++;
                    } else if (input_msg.msg_code == MARVIC_SONAR) {
                        received_sonar++;
                    } else if (input_msg.msg_code == MARVIC_AUTONOMOUS_FLIGHT) {
                        received_autonomous++;
                    }
                }
            }
        }
    }

    // stop continuous sending
    multiwii_serial.prepare_msg_continuous_sending(1);
    multiwii_serial.send_from_buffer();
    sleep(1);

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
