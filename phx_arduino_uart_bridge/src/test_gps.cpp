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
        multiwii_serial.set_device("/dev/tty.SLAB_USBtoUART");  //ttyUSB0");
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
    uint32_t request_gps = 0;           uint32_t received_gps = 0;

    std::cout << " start up done" << std::endl;

    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;
    double system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;

    while (loop_counter-- > 0) {
        // print statistics from while to while
        if (loop_counter % 1000 == 0) {
            system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
            std::cout << "       request\tin\tloss" << std::endl;
            std::cout << "status   " << request_status      << "\t" << received_status     << "\t" << request_status - received_status         << std::endl;
            std::cout << "gps      " << request_gps         << "\t" << received_gps        << "\t" << request_gps - received_gps               << std::endl;
            std::cout << "total:   " << request_total       << "\t" << received_total      << "\t" << request_total - received_total           << std::endl;

            t1 = std::chrono::high_resolution_clock::now();
            real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
            std::cout << "freq status   " << received_status     / real_duration << " msg/s" << std::endl;
            std::cout << "freq gps      " << received_gps        / real_duration << " msg/s" << std::endl;
            std::cout << "freq total    " << received_total      / real_duration << " msg/s" << std::endl;
            
            std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
            std::cout << " communication took " << real_duration << " real seconds" << std::endl;
            std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
        }

        // send requests
        if (loop_counter % 5 == 0) {
            multiwii_serial.prepare_request(MULTIWII_GPS); request_gps++; request_total++;
            multiwii_serial.prepare_request(MULTIWII_STATUS); request_status++; request_total++;
            multiwii_serial.send_from_buffer();
        } else {
            if (loop_counter % 1 == 0) {
            }

            if (loop_counter % 2 == 0) {
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
                    } else if (input_msg.msg_code == MULTIWII_GPS) {
                        received_gps++;
                        std::cout << " gps msg received: " << std::endl;
                        std::cout << "           fix: " << input_msg.msg_data.multiwii_gps.fix << std::endl;
                        std::cout << "           sat: " << input_msg.msg_data.multiwii_gps.numSat << std::endl;
                        std::cout << "           lon: " << input_msg.msg_data.multiwii_gps.coordLON << std::endl;
                        std::cout << "           lat: " << input_msg.msg_data.multiwii_gps.coordLAT << std::endl;
                        std::cout << "           alt: " << input_msg.msg_data.multiwii_gps.altitude << std::endl;
                        std::cout << "          raw msg: " << std::endl;
                        std::cout << "           fix: " << printf("%c ", input_msg.msg_data.multiwii_gps_save.fix) << std::endl;
                        std::cout << "           sat: " << printf("%c ", input_msg.msg_data.multiwii_gps_save.numSat) << std::endl;
                        std::cout << "           lon: " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLON0) << " " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLON1) << " " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLON2) << " " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLON3) << std::endl;
                        std::cout << "           lat: " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLAT0) << " " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLAT1) << " " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLAT2) << " " << printf("%c ", input_msg.msg_data.multiwii_gps_save.coordLAT3) << std::endl;
                        std::cout << "           alt: " << printf("%i ", input_msg.msg_data.multiwii_gps_save.altitude) << std::endl;
                    }
                }
            }
        }
    }

    multiwii_serial.deinitialize();
    
    system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::cout << "       request\tin\tloss" << std::endl;
    std::cout << "total: " << request_total   << "\t" << received_total  << "\t" << request_total - received_total   << std::endl;
    std::cout << "status " << request_status  << "\t" << received_status << "\t" << request_status - received_status << std::endl;
    std::cout << "gps    " << request_gps     << "\t" << received_gps    << "\t" << request_gps - received_gps       << std::endl;

    t1 = std::chrono::high_resolution_clock::now();
    real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
    std::cout << "freq status " << received_status / real_duration << " msg/s" << std::endl;
    std::cout << "freq gps    " << received_gps    / real_duration << " msg/s" << std::endl;
    std::cout << "freq total  " << received_total  / real_duration << " msg/s" << std::endl;
    std::cout << "End " << std::endl;

    std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
    std::cout << " communication took " << real_duration << " real seconds" << std::endl;
    std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
    
    return 0;
}
