#include "phx_uart_msp_bridge/serial_com.h"

uint32_t fix_int32(uint8_t *input_pointer) {
    uint8_t raw_input[4];
    memcpy(raw_input, input_pointer, 4);
    // printf("\n byte0 %i\n", raw_input[0]);
    // printf(" byte1 %i\n", raw_input[1]);
    // printf(" byte2 %i\n", raw_input[2]);
    // printf(" byte3 %i\n", raw_input[3]);
    uint32_t raw_output = 0;
    raw_output += raw_input[0];
    raw_output += ((uint32_t) raw_input[1]) * 256;
    raw_output += ((uint32_t) raw_input[2]) * 65536;
    raw_output += ((uint32_t) raw_input[3]) * 16777216;

    // uint8_t test[4];
    // memcpy(test, &raw_output, 4);
    // printf("\n out byte0 %i\n", test[0]);
    // printf(" out byte1 %i\n", test[1]);
    // printf(" out byte2 %i\n", test[2]);
    // printf(" out byte3 %i\n", test[3]);
    return raw_output;
}

bool SerialCom::set_device(std::string device_path) {
    serial_device_path = device_path;
    std::cout << "SerialCom::set_device  serial_device_path was set to " << serial_device_path << std::endl;
    return true;
}

bool SerialCom::set_baudrate(uint32_t baudrate) {
    serial_device_baud_rate = baudrate;
    std::cout << "SerialCom::set_baudrate  serial_device_baud_rate was set to " << serial_device_baud_rate << std::endl;
    return true;
}

bool SerialCom::set_max_io(uint16_t max_io_bytes) {
    buffer_io_max = max_io_bytes;
    std::cout << "SerialCom::set_max_io  buffer_io_max was set to " << buffer_io_max << std::endl;
    return true;
}

bool SerialCom::init() {
    std::cout << "SerialCom::init  initializing SerialCom on device " << serial_device_path << std::endl;
    serial_interface = open(serial_device_path.c_str(), O_RDWR | O_NOCTTY | O_SYNC);    // original
    //serial_interface = open(serial_device_path, O_RDWR | O_NOCTTY | O_NDELAY);

    std::cout << "SerialCom::init  configuring serial device " << serial_interface << std::endl;
    tcgetattr(serial_interface, &usb_tio);
    usb_tio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    //usb_tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INPCK | ISTRIP | IXON); // original
    usb_tio.c_oflag = 0;
    usb_tio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    usb_tio.c_cflag &= ~(CSIZE | PARENB | HUPCL);
    usb_tio.c_cflag |= CS8;
    usb_tio.c_cc[VMIN]  = 10;
    usb_tio.c_cc[VTIME] = 0;
    int input_baud_set, output_baud_set;
    if  (serial_device_baud_rate == 230400) {
        input_baud_set = cfsetispeed(&usb_tio, B230400);
        output_baud_set = cfsetospeed(&usb_tio, B230400);
        std::cout << "SerialCom::init  using 230400 baudrate " << std::endl;
    } else if  (serial_device_baud_rate == 115200) {
        input_baud_set = cfsetispeed(&usb_tio, B115200);
        output_baud_set = cfsetospeed(&usb_tio, B115200);
        std::cout << "SerialCom::init  using 115200 baudrate " << std::endl;
    } else if (serial_device_baud_rate == 57600) {
        input_baud_set = cfsetispeed(&usb_tio, B57600);
        output_baud_set = cfsetospeed(&usb_tio, B57600);
        std::cout << "SerialCom::init  using 57600 baudrate " << std::endl;
    } else if (serial_device_baud_rate == 9600) {
        input_baud_set = cfsetispeed(&usb_tio, B9600);
        output_baud_set = cfsetospeed(&usb_tio, B9600);
        std::cout << "SerialCom::init  using 9600 baudrate " << std::endl;
    } else {
        serial_device_baud_rate = 9600;
        std::cout << "SerialCom::init  using default baudrate " << serial_device_baud_rate << std::endl;
        input_baud_set = cfsetispeed(&usb_tio, B9600);
        output_baud_set = cfsetospeed(&usb_tio, B9600);
        std::cout << "SerialCom::init  using 9600 baudrate " << std::endl;
    }
    if (input_baud_set < 0 || output_baud_set < 0) {
        std::cout << "SerialCom::init  error in setting baudrate to " << serial_device_baud_rate << std::endl;
        return false;
    }
    if (tcsetattr(serial_interface, TCSANOW, &usb_tio) < 0) {
        std::cout << "SerialCom::init  error in setting attributes" << std::endl;
        return false;
    }
    
    input_buffer_write_position = 0;
    input_buffer_read_position = 0;
    output_buffer_write_position = 0;
    output_buffer_read_position = 0;
    std::cout << "SerialCom::init  done" << std::endl;

    error_count = 0;
    sleep(2);
    tcflush(serial_interface, TCOFLUSH);
    tcflush(serial_interface, TCIFLUSH);
    return true;
}

bool SerialCom::clear_input_buffer() {
    while (receive_to_buffer() == true){
        std::cout << "SerialCom::clear_input_buffer  cleans up" << std::endl;
    }
    memset(input_buffer, '_', sizeof(input_buffer) - 1);
    input_buffer_write_position = 0;
    return true;
}

bool SerialCom::clear_output_buffer() {
    memset(output_buffer, '_', sizeof(output_buffer) - 1);
    output_buffer_write_position = 0;
    return true;
}

bool SerialCom::receive_to_buffer() {
    if (do_debug_printout == true) std::cout << "SerialCom::read_to_buffer  starts receiving from serial port " << serial_device_path << std::endl;
    int result = 0;
    uint16_t new_stuff = 0;
    int loopCount = buffer_io_max;
    
    struct pollfd fds[1];
    fds[0].fd = serial_interface;
    fds[0].events = POLLIN;
    int timeout_msecs = 2;  // 2 is works fine without double checking mode!
    
    while (poll(fds, 1, timeout_msecs) > 0) {
        char msg_buffer[loopCount];
        result = read(serial_interface, &msg_buffer, loopCount);
        if (do_debug_printout == true) std::cout << "SerialCom::read_to_buffer  > positive poll >> reading " << result << " bytes from the serial port: ";
        if (result > 0) {
            // if there was no error while reading
            for (uint8_t index = 0; index<result; index++) {
                input_buffer[input_buffer_write_position] = msg_buffer[index];
                new_stuff++;
                if (do_debug_printout == true) {
                    printf("%i ", input_buffer[input_buffer_write_position]);
                }
                input_buffer_write_position++;
                if (input_buffer_write_position == input_buffer_length) {
                    input_buffer_write_position = 0;
                }
            }
            if (do_debug_printout == true) std::cout << std::endl;
        } else if (result == 0) {
            // no input available
            if (do_debug_printout == true) std::cout << "SerialCom::read_to_buffer  >> nothing available" << std::endl;
        } else {
            // we will skip errors here
            if (do_debug_printout == true) std::cout << "SerialCom::read_to_buffer  >> error" << std::endl;
            error_count++;
        }
    }
    // nothing polled
    if (do_debug_printout == true) std::cout << "SerialCom::read_to_buffer  > negative poll -> exiting after having read " << new_stuff << " new bytes" << std::endl;
    if (new_stuff == 0) {
        return false;    // no input was waiting
    } else {
        return true;     // new input was added to the input buffer
    }
}

bool SerialCom::send_from_buffer() {
    if (do_debug_printout == true) std::cout << "SerialCom::send_from_buffer  starts sending on serial port " << serial_device_path << std::endl;
    int result, loopCount = buffer_io_max;
    uint8_t temp_buffer;
    if (do_debug_printout == true) std::cout << "SerialCom::send_from_buffer  sends " << std::endl;
    while (loopCount-- > 0) {
        if (output_buffer_read_position < output_buffer_write_position) {
            temp_buffer = output_buffer[output_buffer_read_position];
            result = write(serial_interface, &temp_buffer, 1);
            if (do_debug_printout == true) {
                printf("%i ", temp_buffer);
            }
            if (result != -1) {
                // if there was no error while reading then add reading to input buffer.
                output_buffer_read_position++;
            } else {
                // we will skip errors here
                std::cout << "SerialCom::send_from_buffer  error -> count " << error_count++ << std::endl;
            }
        } else {
            if (do_debug_printout == true) std::cout << std::endl << "SerialCom::send_from_buffer  all serial data was sent from the output_buffer to the serial device" << std::endl;
            output_buffer_read_position = 0;
            output_buffer_write_position = 0;
            return true;
        }
    }
    if (do_debug_printout == true) std::cout << "SerialCom::send_from_buffer  sending repetition reached buffer_io_max " << buffer_io_max << std::endl;
    return false;
}

bool SerialCom::print_input_buffer() {
    std::cout << "SerialCom::print_input_buffer  ";
    char cc;
    for (uint16_t index=0; index < input_buffer_write_position; index++) {
        printf("%i ", input_buffer[index]);
    }
    std::cout << std::endl;
    return true;
}

bool SerialCom::print_output_buffer() {
    std::cout << "SerialCom::print_output_buffer  ";
    char cc;
    for (uint16_t index=0; index < output_buffer_write_position; index++) {
        std::cout << "\033[34;1m";
        printf("%i ", output_buffer[index]);
        std::cout << "\033[0m";
    }
    std::cout << std::endl;
    return true;
}

bool SerialCom::deinitialize() {
    close(serial_interface);
    std::cout << "SerialCom::deinitialize serial interface closed" << std::endl;
    return true;
}

bool SerialCom::prepare_request(MessageCode msg_code, MessageProtocol protocol){
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_request sending a request on msg_code " << msg_code  << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = protocol;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = REQUEST;
    msg.msg_code = msg_code;
    msg.checksum = msg_code;
    
    write_msg_to_buffer(msg);
    return true;
}

bool SerialCom::prepare_msg_rc(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4, uint16_t ch5, uint16_t ch6, uint16_t ch7, MessageProtocol msg_protocol) {
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_rc sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = msg_protocol;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MULTIWII_RC_LENGTH;
    msg.msg_code = MULTIWII_RC_SET;
    msg.msg_data.multiwii_rc_set.roll = ch0;
    msg.msg_data.multiwii_rc_set.pitch = ch1;
    msg.msg_data.multiwii_rc_set.yaw = ch2;
    msg.msg_data.multiwii_rc_set.throttle = ch3;
    msg.msg_data.multiwii_rc_set.aux1 = ch4;
    msg.msg_data.multiwii_rc_set.aux2 = ch5;
    msg.msg_data.multiwii_rc_set.aux3 = ch6;
    msg.msg_data.multiwii_rc_set.aux4 = ch7;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::prepare_msg_gps_get_way_point(uint8_t wp_no) {
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_gps_get_way_point sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = MULTIWII_PROTOCOL;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = REQUEST_GPS_WP;
    msg.msg_code = MULTIWII_GPS_WP;
    msg.msg_data.multiwii_gps_way_point.wp_number = wp_no;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;
    write_msg_to_buffer(msg);
    return true;
}

bool SerialCom::prepare_msg_gps_set_way_point(uint8_t wp_no, uint32_t lat, uint32_t lon, uint32_t alt, uint16_t heading, uint16_t stay_time, uint8_t nav_flag) {
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_gps_set_way_point sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = MULTIWII_PROTOCOL;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MULTIWII_GPS_WP_SET_LENGTH;
    msg.msg_code = MULTIWII_GPS_WP_SET;
    msg.msg_data.multiwii_gps_set_way_point.wp_number = wp_no;
    uint8_t temp_lat[4];
    memcpy(temp_lat, &lat, 4);
    msg.msg_data.multiwii_gps_set_way_point.coordLAT = temp_lat[0];
    msg.msg_data.multiwii_gps_set_way_point.coordLAT1 = temp_lat[1];
    msg.msg_data.multiwii_gps_set_way_point.coordLAT2 = temp_lat[2];
    msg.msg_data.multiwii_gps_set_way_point.coordLAT3 = temp_lat[3];
    uint8_t test_lon[4];
    memcpy(test_lon, &lon, 4);
    msg.msg_data.multiwii_gps_set_way_point.coordLON = test_lon[0];
    msg.msg_data.multiwii_gps_set_way_point.coordLON1 = test_lon[1];
    msg.msg_data.multiwii_gps_set_way_point.coordLON2 = test_lon[2];
    msg.msg_data.multiwii_gps_set_way_point.coordLON3 = test_lon[3];
    uint8_t temp_alt[4];
    memcpy(temp_alt, &alt, 4);
    msg.msg_data.multiwii_gps_set_way_point.altitude = temp_alt[0];
    msg.msg_data.multiwii_gps_set_way_point.altitude1 = temp_alt[1];
    msg.msg_data.multiwii_gps_set_way_point.altitude2 = temp_alt[2];
    msg.msg_data.multiwii_gps_set_way_point.altitude3 = temp_alt[3];
    msg.msg_data.multiwii_gps_set_way_point.heading = heading;
    msg.msg_data.multiwii_gps_set_way_point.stay_time = stay_time;
    msg.msg_data.multiwii_gps_set_way_point.nav_flag = nav_flag;
    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::prepare_msg_motor(uint16_t motor0, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4, uint16_t motor5, uint16_t motor6, uint16_t motor7) {
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_motor sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = MULTIWII_PROTOCOL;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MULTIWII_MOTOR_SET_LENGTH;
    msg.msg_code = MULTIWII_MOTOR_SET;
    msg.msg_data.multiwii_motor_set.motor0 = motor0;
    msg.msg_data.multiwii_motor_set.motor1 = motor1;
    msg.msg_data.multiwii_motor_set.motor2 = motor2;
    msg.msg_data.multiwii_motor_set.motor3 = motor3;
    msg.msg_data.multiwii_motor_set.motor4 = motor4;
    msg.msg_data.multiwii_motor_set.motor5 = motor5;
    msg.msg_data.multiwii_motor_set.motor6 = motor6;
    msg.msg_data.multiwii_motor_set.motor7 = motor7;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::prepare_msg_servo_big(uint16_t servo0, uint16_t servo1, uint16_t servo2, uint16_t servo3, uint16_t servo4, uint16_t servo5, uint16_t servo6, uint16_t servo7, uint16_t servo8, uint16_t servo9, uint16_t servo10, uint16_t servo11, uint16_t servo12, uint16_t servo13, uint16_t servo14, uint16_t servo15, uint16_t servo16, uint16_t servo17, MessageProtocol msg_protocol) {
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_servo sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = msg_protocol;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MARVIC_SERVO_BIG_LENGTH;
    msg.msg_code = MARVIC_SERVO_BIG;
    msg.msg_data.marvic_servo.servo0 = servo0;
    msg.msg_data.marvic_servo.servo1 = servo1;
    msg.msg_data.marvic_servo.servo2 = servo2;
    msg.msg_data.marvic_servo.servo3 = servo3;
    msg.msg_data.marvic_servo.servo4 = servo4;
    msg.msg_data.marvic_servo.servo5 = servo5;
    msg.msg_data.marvic_servo.servo6 = servo6;
    msg.msg_data.marvic_servo.servo7 = servo7;
    msg.msg_data.marvic_servo.servo8 = servo8;
    msg.msg_data.marvic_servo.servo9 = servo9;
    msg.msg_data.marvic_servo.servo10 = servo10;
    msg.msg_data.marvic_servo.servo11 = servo11;
    msg.msg_data.marvic_servo.servo12 = servo12;
    msg.msg_data.marvic_servo.servo13 = servo13;
    msg.msg_data.marvic_servo.servo14 = servo14;
    msg.msg_data.marvic_servo.servo15 = servo15;
    msg.msg_data.marvic_servo.servo16 = servo16;
    msg.msg_data.marvic_servo.servo17 = servo17;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::prepare_msg_servo_small(uint16_t servo0, uint16_t servo1, uint16_t servo2, uint16_t servo3, MessageProtocol msg_protocol) {
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_servo_small sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = msg_protocol;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MARVIC_SERVO_SMALL_LENGTH;
    msg.msg_code = MARVIC_SERVO_SMALL;
    msg.msg_data.marvic_servo_small.servo0 = servo0;
    msg.msg_data.marvic_servo_small.servo1 = servo1;
    msg.msg_data.marvic_servo_small.servo2 = servo2;
    msg.msg_data.marvic_servo_small.servo3 = servo3;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::prepare_msg_led_strip(uint8_t led_0_r, uint8_t led_0_g, uint8_t led_0_b, uint8_t led_1_r, uint8_t led_1_g, uint8_t led_1_b,
                                      uint8_t led_2_r, uint8_t led_2_g, uint8_t led_2_b, uint8_t led_3_r, uint8_t led_3_g, uint8_t led_3_b,
                                      uint8_t led_4_r, uint8_t led_4_g, uint8_t led_4_b, uint8_t led_5_r, uint8_t led_5_g, uint8_t led_5_b,
                                      uint8_t led_6_r, uint8_t led_6_g, uint8_t led_6_b, uint8_t led_7_r, uint8_t led_7_g, uint8_t led_7_b,
                                      uint8_t led_8_r, uint8_t led_8_g, uint8_t led_8_b, uint8_t led_9_r, uint8_t led_9_g, uint8_t led_9_b, uint8_t strip_index){
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_led sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = PHOENIX_LED_PROTOCOL;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MARVIC_STRIP_LED_LENGTH;
    if (strip_index  == 0) {
        msg.msg_code = MARVIC_LED_0;
    } else if (strip_index  == 1) {
        msg.msg_code = MARVIC_LED_1;
    } else if (strip_index  == 2) {
        msg.msg_code = MARVIC_LED_2;
    } else if (strip_index  == 3) {
        msg.msg_code = MARVIC_LED_3;
    } else {
        msg.msg_code = MARVIC_LED_0;
    }
    msg.msg_data.marvic_led_strip.led_0_r = led_0_r;
    msg.msg_data.marvic_led_strip.led_0_g = led_0_g;
    msg.msg_data.marvic_led_strip.led_0_b = led_0_b;
    msg.msg_data.marvic_led_strip.led_1_r = led_1_r;
    msg.msg_data.marvic_led_strip.led_1_g = led_1_g;
    msg.msg_data.marvic_led_strip.led_1_b = led_1_b;
    msg.msg_data.marvic_led_strip.led_2_r = led_2_r;
    msg.msg_data.marvic_led_strip.led_2_g = led_2_g;
    msg.msg_data.marvic_led_strip.led_2_b = led_2_b;
    msg.msg_data.marvic_led_strip.led_3_r = led_3_r;
    msg.msg_data.marvic_led_strip.led_3_g = led_3_g;
    msg.msg_data.marvic_led_strip.led_3_b = led_3_b;
    msg.msg_data.marvic_led_strip.led_4_r = led_4_r;
    msg.msg_data.marvic_led_strip.led_4_g = led_4_g;
    msg.msg_data.marvic_led_strip.led_4_b = led_4_b;
    msg.msg_data.marvic_led_strip.led_5_r = led_5_r;
    msg.msg_data.marvic_led_strip.led_5_g = led_5_g;
    msg.msg_data.marvic_led_strip.led_5_b = led_5_b;
    msg.msg_data.marvic_led_strip.led_6_r = led_6_r;
    msg.msg_data.marvic_led_strip.led_6_g = led_6_g;
    msg.msg_data.marvic_led_strip.led_6_b = led_6_b;
    msg.msg_data.marvic_led_strip.led_7_r = led_7_r;
    msg.msg_data.marvic_led_strip.led_7_g = led_7_g;
    msg.msg_data.marvic_led_strip.led_7_b = led_7_b;
    msg.msg_data.marvic_led_strip.led_8_r = led_8_r;
    msg.msg_data.marvic_led_strip.led_8_g = led_8_g;
    msg.msg_data.marvic_led_strip.led_8_b = led_8_b;
    msg.msg_data.marvic_led_strip.led_9_r = led_9_r;
    msg.msg_data.marvic_led_strip.led_9_g = led_9_g;
    msg.msg_data.marvic_led_strip.led_9_b = led_9_b;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::prepare_msg_single_led(uint8_t led_id, uint8_t strip_index, uint8_t led_r, uint8_t led_g, uint8_t led_b){
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_single_led sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = PHOENIX_LED_PROTOCOL;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MARVIC_SINGLE_LED_LENGTH;
    msg.msg_code = MARVIC_SINGLE_LED;
    msg.msg_data.marvic_led_single.led_id = led_id;
    msg.msg_data.marvic_led_single.strip_index = strip_index;
    msg.msg_data.marvic_led_single.led_r = led_r;
    msg.msg_data.marvic_led_single.led_g = led_g;
    msg.msg_data.marvic_led_single.led_b = led_b;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::prepare_msg_pid(uint8_t roll_p, uint8_t roll_i, uint8_t roll_d,
                                uint8_t pitch_p, uint8_t pitch_i, uint8_t pitch_d,
                                uint8_t yaw_p, uint8_t yaw_i, uint8_t yaw_d,
                                uint8_t alt_p, uint8_t alt_i, uint8_t alt_d,
                                uint8_t vel_p, uint8_t vel_i, uint8_t vel_d,
                                uint8_t pos_p, uint8_t pos_i, uint8_t pos_d,
                                uint8_t posrate_p, uint8_t posrate_i, uint8_t posrate_d,
                                uint8_t navrate_p, uint8_t navrate_i, uint8_t navrate_d,
                                uint8_t level_p, uint8_t level_i, uint8_t level_d,
                                uint8_t mag_p, uint8_t mag_i, uint8_t mag_d) {
    if (do_debug_printout == true) std::cout << "SerialCom::prepare_msg_pid sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = MULTIWII_PROTOCOL;
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MULTIWII_PID_SET_LENGTH;
    msg.msg_code = MULTIWII_PID_SET;
    msg.msg_data.multiwii_pid.p1 = roll_p;
    msg.msg_data.multiwii_pid.p2 = pitch_p;
    msg.msg_data.multiwii_pid.p3 = yaw_p;
    msg.msg_data.multiwii_pid.p4 = alt_p;
    msg.msg_data.multiwii_pid.p5 = vel_p;
    msg.msg_data.multiwii_pid.p6 = pos_p;
    msg.msg_data.multiwii_pid.p7 = posrate_p;
    msg.msg_data.multiwii_pid.p8 = navrate_p;
    msg.msg_data.multiwii_pid.p9 = level_p;
    msg.msg_data.multiwii_pid.p10 = mag_p;
    msg.msg_data.multiwii_pid.i1 = roll_i;
    msg.msg_data.multiwii_pid.i2 = pitch_i;
    msg.msg_data.multiwii_pid.i3 = yaw_i;
    msg.msg_data.multiwii_pid.i4 = alt_i;
    msg.msg_data.multiwii_pid.i5 = vel_i;
    msg.msg_data.multiwii_pid.i6 = pos_i;
    msg.msg_data.multiwii_pid.i7 = posrate_i;
    msg.msg_data.multiwii_pid.i8 = navrate_i;
    msg.msg_data.multiwii_pid.i9 = level_i;
    msg.msg_data.multiwii_pid.i10 = mag_i;
    msg.msg_data.multiwii_pid.d1 = roll_d;
    msg.msg_data.multiwii_pid.d2 = pitch_d;
    msg.msg_data.multiwii_pid.d3 = yaw_d;
    msg.msg_data.multiwii_pid.d4 = alt_d;
    msg.msg_data.multiwii_pid.d5 = vel_d;
    msg.msg_data.multiwii_pid.d6 = pos_d;
    msg.msg_data.multiwii_pid.d7 = posrate_d;
    msg.msg_data.multiwii_pid.d8 = navrate_d;
    msg.msg_data.multiwii_pid.d9 = level_d;
    msg.msg_data.multiwii_pid.d10 = mag_d;

    uint8_t msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;

    write_msg_to_buffer(msg);
}

bool SerialCom::write_to_output_buffer(uint8_t byte){
    if (do_debug_printout == true) {std::cout << "SerialCom::write_to_output_buffer sending byte";
                                    printf(" %i", byte);
                                    std::cout << std::endl;
                                    }
    output_buffer[output_buffer_write_position] = byte;
    output_buffer_write_position++;
    if (output_buffer_write_position >= output_buffer_length) {
        output_buffer_write_position -= output_buffer_length;
    }
    //if (do_debug_printout == true) std::cout << "SerialCom::write_to_output_buffer output_buffer_write_position " << output_buffer_write_position << std::endl;
    //if (do_debug_printout == true) std::cout << "SerialCom::write_to_output_buffer output_buffer_read_position " << output_buffer_read_position << std::endl;


    return true;
}

bool SerialCom::write_msg_to_buffer(Message msg){
    write_to_output_buffer(msg.msg_preamble);
    write_to_output_buffer(msg.msg_protocol);
    write_to_output_buffer(msg.msg_direction);
    write_to_output_buffer(msg.msg_length);
    write_to_output_buffer(msg.msg_code);
    if (msg.msg_length != 0){
        char msg_data_bytes[sizeof(msg.msg_data)];
        memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
        for (uint8_t index=0; index < msg.msg_length; index++) {
            write_to_output_buffer(msg_data_bytes[index]);
        }
    }
    write_to_output_buffer(msg.checksum);
    return true;
}

uint8_t SerialCom::read_from_input_buffer() {
    uint8_t byte = input_buffer[input_buffer_read_position];
    input_buffer_read_position++;
    if (input_buffer_read_position >= input_buffer_length) {
        input_buffer_read_position = 0;
    }
    return byte;
}

uint16_t SerialCom::available() {
    uint16_t available_bytes = 0;
    if (input_buffer_write_position > input_buffer_read_position) {
        available_bytes = input_buffer_write_position - input_buffer_read_position;
    } else if (input_buffer_write_position < input_buffer_read_position) {
        available_bytes = input_buffer_length - input_buffer_read_position + input_buffer_write_position;
    }
    return available_bytes;
}

bool SerialCom::read_msg_from_buffer(Message* msg) {
    if (do_debug_printout == true) {
        std::cout << "SerialCom::read_msg_from_buffer  analyses the input buffer for incoming messages: ";
        char cc;
        for (uint16_t index=input_buffer_read_position; index < input_buffer_write_position; index++) {
            if ((input_buffer[index] == '$') || (input_buffer[index] == 'M') || (input_buffer[index] == '<') || (input_buffer[index] == '>') || (input_buffer[index] == '!')) {
                std::cout << "\033[34;1m";
                printf("%c ", input_buffer[index]);
                std::cout << "\033[0m";
            } else {
                std::cout << "\033[34;0m";
                printf("%i ", input_buffer[index]);
                std::cout << "\033[0m";
            }
        }
        std::cout << "\033[0m" << std::endl;
    }
    uint16_t analysis_start = input_buffer_read_position;

    uint8_t msg_preamble = 0;
    uint8_t msg_protocol = 0;
    uint8_t msg_direction = 0;
    uint8_t msg_length = 0;
    uint8_t msg_code = 0;
    char msg_data_bytes[sizeof(msg->msg_data)];
    memcpy(msg_data_bytes, &msg->msg_data, sizeof(msg->msg_data));
    uint8_t msg_checksum = 0;
    uint8_t checksum = 0;
    uint8_t data = 0;

    while (available() >= 6) {
        analysis_start = input_buffer_read_position;
        if (do_debug_printout == true) std::cout << "SerialCom::read_msg_from_buffer   > analyse starting from input_buffer_read_position " << input_buffer_read_position << " to " << input_buffer_write_position << std::endl;

        for (uint16_t index=0; index < sizeof(msg->msg_data); index++) {
            data = 0;
            msg_data_bytes[index] = data;
        }

        if (read_from_input_buffer() == '$') {
            // start byte found
            if (do_debug_printout == true) std::cout << "SerialCom::read_msg_from_buffer   >> start byte found";
            msg_preamble = '$';
            uint8_t temp_protocol_type = read_from_input_buffer();
            if (temp_protocol_type == 'M' || temp_protocol_type == 'R' || temp_protocol_type == 'L' || temp_protocol_type == 'S') {
                // MultiWii protocol byte found
                if ((do_debug_printout == true) && (temp_protocol_type == 'M')) std::cout << "  >> MultiWii protocol byte found";
                if ((do_debug_printout == true) && (temp_protocol_type == 'P')) std::cout << "  >> PHOENIX protocol byte found";
                if ((do_debug_printout == true) && (temp_protocol_type == 'S')) std::cout << "  >> SCANNER protocol byte found";
                msg_protocol = temp_protocol_type;
                msg_direction = read_from_input_buffer();
                msg_length = read_from_input_buffer();
                msg_code = read_from_input_buffer();

                if (do_debug_printout == true) printf("  >> msg_length byte: %i", msg_length);
                if (do_debug_printout == true) printf("  >> msg_code byte: %i", msg_code);
                if (do_debug_printout == true) std::cout << std::endl;

                if (msg_length > sizeof(msg->msg_data)) {
                    // this message is bigger that any possible message....so it is very likely it is bull shit
                    if (do_debug_printout == true) std::cout << "\033[1;31mSerialCom::read_msg_from_buffer   >>> received message with payload_length " << msg_length << " > " << sizeof(msg->msg_data) << "\033[0m" << std::endl;
                    if (do_debug_printout == true) {
                        std::cout << "SerialCom::read_msg_from_buffer   >>> message was: \033[34;1m";
                        char cc;
                        for (uint16_t index=analysis_start; index < input_buffer_read_position; index++) {
                            printf("%i ", input_buffer[index]);
                        }
                        std::cout << "\033[34;2m";
                        for (uint16_t index=input_buffer_read_position; index < input_buffer_write_position; index++) {
                            printf("%i ", input_buffer[index]);
                        }
                        std::cout << "\033[0m" << std::endl;
                    }
                    input_buffer_read_position = analysis_start + 1;
                    if (input_buffer_read_position >= input_buffer_length) {
                        input_buffer_read_position -= input_buffer_length;
                    }
                } else if (available() >= msg_length + 1) {
                    // the full message was received and can now be read
                    if (do_debug_printout == true) std::cout << "SerialCom::read_msg_from_buffer   >> available bytes: " << available() << std::endl;
                    if (do_debug_printout == true) std::cout << "SerialCom::read_msg_from_buffer      reading payload: ";
                    checksum = msg_length ^ msg_code;
                    for (uint16_t index=0; index < msg_length; index++) {
                        data = read_from_input_buffer();
                        msg_data_bytes[index] = data;
                        if (do_debug_printout == true) printf("%i ", (uint8_t) msg_data_bytes[index]);
                        checksum = checksum ^ data;
                    }
                    msg_checksum = read_from_input_buffer();
                    if (do_debug_printout == true) std::cout << std::endl;

                    if (msg_length == 0) {
                        if (do_debug_printout == true) std::cout << "\033[1;31m" << "SerialCom::read_msg_from_buffer   >>> this message was probably a request" << "\033[0m" << std::endl;
                        if (do_debug_printout == true) {
                            std::cout << "SerialCom::read_msg_from_buffer   >>> message was: " << "\033[34;1m";
                            char cc;
                            for (uint16_t index=analysis_start; index < input_buffer_read_position; index++) {
                                printf("%i ", input_buffer[index]);
                            }
                            std::cout << "\033[34;2m";
                            for (uint16_t index=input_buffer_read_position; index < input_buffer_write_position; index++) {
                                printf("%i ", input_buffer[index]);
                            }
                            std::cout << "\033[0m" << std::endl;
                        }

                        //input_buffer_read_position = analysis_start + 1;
                        msg->msg_preamble = msg_preamble;
                        msg->msg_protocol = (MessageProtocol) msg_protocol;
                        msg->msg_direction = (MessageDirection) msg_direction;
                        msg->msg_length = (MessageLength) msg_length;
                        msg->msg_code = (MessageCode) msg_code;
                        Payload tmp;
                        memcpy(&tmp, msg_data_bytes, sizeof(tmp));
                        msg->msg_data = tmp;
                        msg->checksum = checksum;
                        if (do_debug_printout == true) {
                            std::cout << "sleeping " << std::endl;
                            //print_multiwii_message(msg);
                            std::cout << "SerialCom::read_msg_from_buffer  returns true 1" << std::endl;
                        }
                        return true;
                    } else if (msg_checksum == checksum) {
                        // the message is valid and can now be saved
                        if (do_debug_printout == true) {
                            std::cout << "SerialCom::read_msg_from_buffer   >>> this message was probably a valid message" << std::endl;
                            std::cout << "SerialCom::read_msg_from_buffer   >>> message was: \033[34;1m";
                            char cc;
                            for (uint16_t index=analysis_start; index < input_buffer_read_position; index++) {
                                printf("%i ", input_buffer[index]);
                            }
                            std::cout << "\033[34;2m";
                            for (uint16_t index=input_buffer_read_position; index < input_buffer_write_position; index++) {
                                printf("%i ", input_buffer[index]);
                            }
                            std::cout << "\033[0m" << std::endl;
                        }

                        msg->msg_preamble = msg_preamble;
                        msg->msg_protocol = (MessageProtocol) msg_protocol;
                        msg->msg_direction = (MessageDirection) msg_direction;
                        msg->msg_length = (MessageLength) msg_length;
                        msg->msg_code = (MessageCode) msg_code;
                        Payload tmp;
                        memcpy(&tmp, msg_data_bytes, sizeof(tmp));
                        msg->msg_data = tmp;
                        msg->checksum = checksum;
                        if (do_debug_printout == true) {
                            printf("SerialCom::read_msg_from_buffer  received message! code %i length %i checksum: %i <-> %i \n", msg_code, msg_length, checksum, msg_checksum);
                            //print_multiwii_message(msg);
                            std::cout << "SerialCom::read_msg_from_buffer  returns true 2" << std::endl;
                        }
                        return true;
                    } else {
                        printf("SerialCom::read_msg_from_buffer  broken message received! code %i length %i checksum: %i <-> %i \n", msg_code, msg_length, checksum, msg_checksum);
                        input_buffer_read_position = analysis_start + 1;
                        if (input_buffer_read_position >= input_buffer_length) {
                            input_buffer_read_position -= input_buffer_length;
                        }
                    }
                } else {
                    // the data of this msg was not received completely jet, wait a bit.
                    input_buffer_read_position = analysis_start;
                    if (do_debug_printout == true) std::cout << "                         >> data not fully received! returning false" << std::endl;
                    return false;   // exiting because the available data is not entirely read into the input_buffer.
                }
            } else {
                std::cout << "  >> no supported protocol byte" << std::endl;
                std::cout << "SerialCom::read_msg_from_buffer   >>> message was: \033[34;1m";
                char cc;
                for (uint16_t index=analysis_start; index < input_buffer_read_position; index++) {
                    printf("%i ", input_buffer[index]);
                }
                std::cout << "\033[34;2m";
                for (uint16_t index=input_buffer_read_position; index < input_buffer_write_position; index++) {
                    printf("%i ", input_buffer[index]);
                }
                std::cout << "\033[0m" << std::endl;
                input_buffer_read_position = analysis_start + 1;
            }
        } else {
            if (do_debug_printout == true) std::cout << "SerialCom::read_msg_from_buffer    >> no start byte" << std::endl;
        }
    }
    if (do_debug_printout == true) std::cout << "SerialCom::read_msg_from_buffer    >> analysis reached real time" << std::endl;
    return false;   // exiting because there is now new data in the input_buffer
}

void print_multiwii_message(Message* msg) {
    printf(" print_multiwii_message:\n");
    printf("   msg_header: %c %c %c\n", msg->msg_preamble, msg->msg_protocol, msg->msg_direction);
    printf("   msg_length: %i\n", msg->msg_length);
    printf("   msg_code:   %i\n", msg->msg_code);
    switch (msg->msg_code){
        case MULTIWII_STATUS:
            printf("   msg_data: MULTIWII_STATUS\n");
            printf("     cycleTime:        %i\n", msg->msg_data.multiwii_status.cycleTime);
            printf("     i2c_errors_count: %i\n", msg->msg_data.multiwii_status.i2c_errors_count);
            break;
        case MULTIWII_IMU:
            printf("   msg_data: MULTIWII_IMU\n");
            printf(" acc:  %i,\t %i,\t %i\n", msg->msg_data.multiwii_raw_imu.accx, msg->msg_data.multiwii_raw_imu.accy, msg->msg_data.multiwii_raw_imu.accz);
            printf(" gyro: %i,\t %i,\t %i\n", msg->msg_data.multiwii_raw_imu.gyrx, msg->msg_data.multiwii_raw_imu.gyry, msg->msg_data.multiwii_raw_imu.gyrz);
            printf(" mag:  %i,\t %i,\t %i\n", msg->msg_data.multiwii_raw_imu.magx, msg->msg_data.multiwii_raw_imu.magy, msg->msg_data.multiwii_raw_imu.magz);
            break;

        case MULTIWII_RC:
            printf("   msg_data: MULTIWII_RC\n");
            printf("     roll:     %i\n", msg->msg_data.multiwii_rc.roll);
            printf("     pitch:    %i\n", msg->msg_data.multiwii_rc.pitch);
            printf("     yaw:      %i\n", msg->msg_data.multiwii_rc.yaw);
            printf("     throttle: %i\n", msg->msg_data.multiwii_rc.throttle);
            printf("     aux1:     %i\n", msg->msg_data.multiwii_rc.aux1);
            printf("     aux2:     %i\n", msg->msg_data.multiwii_rc.aux2);
            printf("     aux3:     %i\n", msg->msg_data.multiwii_rc.aux3);
            printf("     aux4:     %i\n", msg->msg_data.multiwii_rc.aux4);
            break;

        case MULTIWII_RC_SET:
            printf("   msg_data: MULTIWII_RC_SET\n");
            printf("     roll:     %i\n", msg->msg_data.multiwii_rc_set.roll);
            printf("     pitch:    %i\n", msg->msg_data.multiwii_rc_set.pitch);
            printf("     yaw:      %i\n", msg->msg_data.multiwii_rc_set.yaw);
            printf("     throttle: %i\n", msg->msg_data.multiwii_rc_set.throttle);
            printf("     aux1:     %i\n", msg->msg_data.multiwii_rc_set.aux1);
            printf("     aux2:     %i\n", msg->msg_data.multiwii_rc_set.aux2);
            printf("     aux3:     %i\n", msg->msg_data.multiwii_rc_set.aux3);
            printf("     aux4:     %i\n", msg->msg_data.multiwii_rc_set.aux4);
            break;

        case MULTIWII_MOTOR:
            printf("   msg_data: MULTIWII_MOTOR\n");
            printf("     motor0: %i\n", msg->msg_data.multiwii_motor.motor0);
            printf("     motor1: %i\n", msg->msg_data.multiwii_motor.motor1);
            printf("     motor2: %i\n", msg->msg_data.multiwii_motor.motor2);
            printf("     motor3: %i\n", msg->msg_data.multiwii_motor.motor3);
            printf("     motor4: %i\n", msg->msg_data.multiwii_motor.motor4);
            printf("     motor5: %i\n", msg->msg_data.multiwii_motor.motor5);
            printf("     motor6: %i\n", msg->msg_data.multiwii_motor.motor6);
            printf("     motor7: %i\n", msg->msg_data.multiwii_motor.motor7);
            break;

        case MULTIWII_ATTITUDE:
            printf("   msg_data: MULTIWII_ATTITUDE\n");
            printf("     roll:  %i\n", msg->msg_data.multiwii_attitude.roll);
            printf("     pitch: %i\n", msg->msg_data.multiwii_attitude.pitch);
            printf("     yaw:   %i\n", msg->msg_data.multiwii_attitude.yaw);
            break;

        case MULTIWII_GPS:
            printf("   msg_data: MULTIWII_GPS\n");
            printf("     fix:      %i\n", msg->msg_data.multiwii_gps.fix);
            printf("     numSat:   %i\n", msg->msg_data.multiwii_gps.numSat);
            printf("     coordLAT: %i\n", fix_int32(&msg->msg_data.multiwii_gps.coordLAT));
            printf("     coordLON: %i\n", fix_int32(&msg->msg_data.multiwii_gps.coordLON));
            printf("     altitude: %i\n", msg->msg_data.multiwii_gps.altitude);
            printf("     speed:    %i\n", msg->msg_data.multiwii_gps.speed);
            break;
        default:
            break;
    }
}
