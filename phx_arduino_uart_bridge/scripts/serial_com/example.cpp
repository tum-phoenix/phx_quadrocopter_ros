#include <unistd.h>
#include <ctime>
#include <chrono>


#include "serial_com.cpp"

int main() {
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
    uint16_t counter = 10000;
    uint16_t request_total = 0;
    uint16_t received_total = 0;
    uint16_t received_status = 0;
    uint16_t received_imu = 0;
    uint16_t received_motor = 0;
    Message input_msg;


    std::chrono::high_resolution_clock::time_point t0 = std::chrono::high_resolution_clock::now();
    double begin_communication = double(clock()) / CLOCKS_PER_SEC;


    while (counter-- > 0){
        // send requests
        if (counter % 3 == 0) {
            multiwii_serial.send_request(MULTIWII_STATUS);
        } else if (counter % 3 == 1) {
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
                received_status++;
            } else if (input_msg.msg_code == MULTIWII_IMU) {
                received_imu++;
            } else if (input_msg.msg_code == MULTIWII_MOTOR) {
                received_motor++;
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
        //std::cout << counter << "  loop mean duration in cpu time " << elapsed_secs / received_total << " seconds" << std::endl;

        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count();
        //std::cout << counter << "  loop mean duration in real time " << duration / received_total << " us" << std::endl;
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
    std::cout << "received total  " << received_total  << std::endl;

    std::cout << "freq status " << received_status / (duration / 1000000.) << " messages/s" << std::endl;
    std::cout << "freq imu    " << received_imu    / (duration / 1000000.) << " messages/s" << std::endl;
    std::cout << "freq motor  " << received_motor  / (duration / 1000000.) << " messages/s" << std::endl;
    std::cout << "freq total  " << received_total  / (duration / 1000000.) << " messages/s" << std::endl;

    std::cout << "End " << std::endl;
	return 0;
}