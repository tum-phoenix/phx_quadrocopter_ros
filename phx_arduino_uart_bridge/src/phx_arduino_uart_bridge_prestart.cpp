#include <chrono>
#include <iostream>
#include "phx_arduino_uart_bridge/serial_com.h"

int main(int argc, char **argv)
{
    int32_t count = 200;
    
    // establish connection:
    SerialCom multiwii_serial;                                              // create SerialCom instance
    multiwii_serial.set_device("/dev/ttyUSB0");
    multiwii_serial.set_baudrate(115200);
    multiwii_serial.set_max_io(200);
    multiwii_serial.init();                                                 // initialize serial connection
    sleep(12);                                                               // wait for arduino bootloader
    uint32_t request_total = 0;
    uint32_t request_status = 0;
    uint32_t request_imu = 0;
    uint32_t request_motor = 0;
    uint32_t request_gps = 0;
    uint32_t received_total = 0;
    uint32_t received_status = 0;
    uint32_t received_imu = 0;
    uint32_t received_motor = 0;
    uint32_t received_gps = 0;
    Message input_msg;
    
    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;
    
    while (count-- > 0) {
        // request serial stuff
        if (count % 3 == 0) {
            //multiwii_serial.send_request(MULTIWII_GPS); request_gps++; request_total++;
            //multiwii_serial.send_request(MULTIWII_STATUS); request_status++; request_total++;
            multiwii_serial.send_request(MULTIWII_IMU); request_imu++; request_total++;
            //multiwii_serial.send_request(MULTIWII_MOTOR); request_motor++; request_total++;
            multiwii_serial.send_from_buffer();
            usleep(10);
        }
        
        // receive serial stuff
        multiwii_serial.receive_to_buffer();
        
        // interprete stuff
        while (multiwii_serial.read_msg_from_buffer(&input_msg) == true) {
            received_total++;
            if (input_msg.msg_code == MULTIWII_STATUS) {
                received_status++;
            } else if (input_msg.msg_code == MULTIWII_IMU) {
                received_imu++;
            } else if (input_msg.msg_code == MULTIWII_MOTOR) {
                received_motor++;
            } else if (input_msg.msg_code == MULTIWII_GPS) {
                received_gps++;
            }
            usleep(600);
            multiwii_serial.receive_to_buffer();
        }
    }
    usleep(5000);
    multiwii_serial.clear_input_buffer();
    multiwii_serial.deinitialize();
    
    double system_duration = (double(clock()) / CLOCKS_PER_SEC) - begin_communication;
    std::cout << "       request\tin\tloss" << std::endl;
    std::cout << "total: " << request_total   << "\t" << received_total  << "\t" << request_total - received_total   << std::endl;
    std::cout << "status " << request_status  << "\t" << received_status << "\t" << request_status - received_status << std::endl;
    std::cout << "imu    " << request_imu     << "\t" << received_imu    << "\t" << request_imu - received_imu       << std::endl;
    std::cout << "motor  " << request_motor   << "\t" << received_motor  << "\t" << request_motor - received_motor   << std::endl;
    std::cout << "gps    " << request_gps     << "\t" << received_gps    << "\t" << request_gps - received_gps       << std::endl << std::endl;
    
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    auto real_duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() / 1000000.;
    std::cout << "freq status " << received_status / real_duration << " msg/s" << std::endl;
    std::cout << "freq imu    " << received_imu    / real_duration << " msg/s" << std::endl;
    std::cout << "freq motor  " << received_motor  / real_duration << " msg/s" << std::endl;
    std::cout << "freq gps    " << received_gps    / real_duration << " msg/s" << std::endl;
    std::cout << "freq total  " << received_total  / real_duration << " msg/s" << std::endl << std::endl;
    
    std::cout << " communication took " << system_duration << " cpu seconds" << std::endl;
    std::cout << " communication took " << real_duration << " real seconds" << std::endl;
    std::cout << "       CPU workload " << system_duration / real_duration << std::endl;
    
    return 0;
}
