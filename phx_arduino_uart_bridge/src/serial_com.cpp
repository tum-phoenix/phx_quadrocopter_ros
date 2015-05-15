#include "phx_arduino_uart_bridge/serial_com.h"

bool SerialCom::set_device(std::string device_path) {
    serial_device_path = device_path.c_str();
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
    serial_interface = open(serial_device_path, O_RDWR | O_NOCTTY | O_SYNC);    // original
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
    if  (serial_device_baud_rate == 115200) {
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
    sleep(2);
    tcflush(serial_interface, TCOFLUSH);
    tcflush(serial_interface, TCIFLUSH);
    return true;
}

bool SerialCom::clear_input_buffer() {
    while (receive_to_buffer() != true){
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
    std::cout << "SerialCom::read_to_buffer  starts reading on serial port " << serial_device_path << std::endl;
    int result = 0;
    int loopCount = buffer_io_max;
    
    struct pollfd fds[1];
    fds[0].fd = serial_interface;
    fds[0].events = POLLIN;
    int timeout_msecs = 2;  // 2 is works fine without double checking mode!
    
    while (poll(fds, 1, timeout_msecs) > 0) {
        std::cout << "SerialCom::read_to_buffer  positive poll " << std::endl;
        char msg_buffer[loopCount];
        result = read(serial_interface, &msg_buffer, loopCount);
        std::cout << "SerialCom::read_to_buffer  result " << result << std::endl;
        if (result > 0) {
            // if there was no error while reading
            //std::cout << "SerialCom::read_to_buffer  input ";
            //char cc;
            for (uint8_t index = 0; index<result; index++) {
                input_buffer[input_buffer_write_position] = msg_buffer[index];

                //cc = printf("%i ", input_buffer[input_buffer_write_position]);
                //std::cout << cc;

                input_buffer_write_position++;
                if (input_buffer_write_position == input_buffer_length) {
                    input_buffer_write_position = 0;
                }
            }
            //std::cout << std::endl;
            //std::cout << "SerialCom::read_to_buffer  read " << result << std::endl;
        } else if (result == 0) {
            // no input available
            std::cout << "SerialCom::read_to_buffer  nothing available" << std::endl;
        } else {
            // we will skip errors here
            std::cout << "SerialCom::read_to_buffer  error" << std::endl;
        }
    }
    // nothing polled
    std::cout << "SerialCom::read_to_buffer  negative poll" << std::endl;
    if (result == 0) {
        return true;    // no input waiting
    } else {
        return false;
    }
}


bool SerialCom::send_from_buffer() {
    //std::cout << "SerialCom::send_from_buffer  starts sending on serial port " << serial_device_path << std::endl;
    int result, loopCount = buffer_io_max;
    char temp_buffer;

    while (loopCount-- > 0) {
        //std::cout << "SerialCom::send_from_buffer  " << loopCount << " sends" << std::endl;
        if (output_buffer_read_position < output_buffer_write_position) {
            temp_buffer = output_buffer[output_buffer_read_position];
            result = write(serial_interface, &temp_buffer, 1);
            //std::cout << "SerialCom::send_from_buffer  " << loopCount << " sent " << temp_buffer << " with result " << result << std::endl;
            if (result != -1) {
                // if there was no error while reading then add reading to input buffer.
                output_buffer_read_position++;
            } else {
                // we will skip errors here
                std::cout << "SerialCom::send_from_buffer  error" << std::endl;
            }
        } else {
            //std::cout << "SerialCom::send_from_buffer  all serial data was sent from the output_buffer to the serial device" << std::endl;
            output_buffer_read_position = 0;
            output_buffer_write_position = 0;
            return true;
        }
    }
    //std::cout << "SerialCom::send_from_buffer  sending repetition reached buffer_io_max " << buffer_io_max << std::endl;
    return false;
}

bool SerialCom::print_input_buffer() {
    std::cout << "SerialCom::print_input_buffer  ";
    char cc;
    for (uint16_t index=0; index < input_buffer_write_position; index++) {
        cc = printf("%c ", input_buffer[index]);
        //cc = printf("%i ", input_buffer[index]);
        std::cout << cc;
    }
    std::cout << std::endl;
    return true;
}

bool SerialCom::print_output_buffer() {
    std::cout << "SerialCom::print_output_buffer  ";
    char cc;
    for (uint16_t index=0; index < 30; index++) {
        cc = printf("%c ", output_buffer[index]);
        //cc = printf("%i ", output_buffer[index]);
        std::cout << cc;
    }
    std::cout << std::endl;
    return true;
}

bool SerialCom::deinitialize() {
    close(serial_interface);
    std::cout << "SerialCom::deinitialize serial interface closed" << std::endl;
    return true;
}

bool SerialCom::send_request(MessageCode msg_code){
    std::cout << "SerialCom::send_request sending a request on msg_code " << msg_code  << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = 'M';
    msg.msg_direction = MULTIWII_TO_COM;
    msg.msg_length = REQUEST;
    msg.msg_code = msg_code;
    msg.checksum = msg_code;
    
    write_msg_to_buffer(msg);
    return true;
}

bool SerialCom::send_msg_rc(uint16_t throttle, uint16_t pitch, uint16_t roll, uint16_t yaw, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4) {
    std::cout << "SerialCom::send_msg_rc sending" << std::endl;
    Message msg;
    msg.msg_preamble = '$';
    msg.msg_protocol = 'M';
    msg.msg_direction = COM_TO_MULTIWII;
    msg.msg_length = MULTIWII_RC_LENGTH;
    msg.msg_code = MULTIWII_RC;
    msg.msg_data.multiwii_rc.roll = roll;
    msg.msg_data.multiwii_rc.pitch = pitch;
    msg.msg_data.multiwii_rc.yaw = yaw;
    msg.msg_data.multiwii_rc.throttle = throttle;
    msg.msg_data.multiwii_rc.aux1 = aux1;
    msg.msg_data.multiwii_rc.aux2 = aux2;
    msg.msg_data.multiwii_rc.aux3 = aux3;
    msg.msg_data.multiwii_rc.aux4 = aux4;

    char msg_data_bytes[sizeof(msg.msg_data)];
    memcpy(msg_data_bytes, &msg.msg_data, sizeof(msg.msg_data));
    uint8_t checksum = msg.msg_length ^ msg.msg_code;
    for (uint16_t index=0; index < msg.msg_length; index++) {
        checksum = checksum ^ msg_data_bytes[index];
    }
    msg.checksum = checksum;
    write_msg_to_buffer(msg);
}

bool SerialCom::write_to_output_buffer(uint8_t byte){
    //std::cout << "SerialCom::write_to_output_buffer byte" << byte << std::endl;
    //std::cout << "SerialCom::write_to_output_buffer output_buffer_write_position" << output_buffer_write_position << std::endl;
    //std::cout << "SerialCom::write_to_output_buffer output_buffer" << output_buffer[output_buffer_write_position] << std::endl;
    output_buffer[output_buffer_write_position] = byte;
    output_buffer_write_position++;
    if (output_buffer_write_position >= output_buffer_length) {
        output_buffer_write_position -= output_buffer_length;
    }
    return true;
}

bool SerialCom::write_msg_to_buffer(Message msg){
    write_to_output_buffer(msg.msg_preamble);
    write_to_output_buffer(msg.msg_protocol);
    write_to_output_buffer(msg.msg_direction);
    write_to_output_buffer(msg.msg_length);
    write_to_output_buffer(msg.msg_code);
    if (msg.msg_length != 0){
        char* msg_bytes = reinterpret_cast<char*>(&msg.msg_data);
        for (uint8_t index=0; index < msg.msg_length; index++) {
            write_to_output_buffer(msg_bytes[index]);
        }
    }
    write_to_output_buffer(msg.checksum);
    return true;
}

uint8_t SerialCom::read_from_input_buffer() {
    std::cout << "SerialCom::read_from_input_buffer at " << input_buffer_read_position << " with " << available() << std::endl;
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
    //std::cout << "SerialCom::available   write " << input_buffer_write_position << "  read " << input_buffer_read_position << "  available " << available_bytes << std::endl;
    return available_bytes;
}

bool SerialCom::read_msg_from_buffer(Message* msg) {
    std::cout << "SerialCom::read_msg_from_buffer  starts reading the input buffer " << sizeof(msg->msg_data) << std::endl;
    uint16_t analysis_start;

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

        for (uint16_t index=0; index < sizeof(msg->msg_data); index++) {
            data = 0;
            msg_data_bytes[index] = data;
        }
        std::cout << "SerialCom::read_msg_from_buffer  analyse starting from " << input_buffer_read_position << " " << input_buffer_write_position << std::endl;
        if (read_from_input_buffer() == '$') {
            // start byte found
            msg_preamble = '$';
            if (read_from_input_buffer() == 'M') {
                msg_protocol = 'M';
                msg_direction = read_from_input_buffer();
                msg_length = read_from_input_buffer();
                msg_code = read_from_input_buffer();

                if (available() >= msg_length + 1) {
                    // the full message was received and can now be read
                    //printf("SerialCom::read_msg_from_buffer  receiving new msg on msg_code %i \n", msg_code);

                    checksum = msg_length ^ msg_code;
                    for (uint16_t index=0; index < msg_length; index++) {
                        std::cout << "SerialCom::read_msg_from_buffer  adding data" << std::endl;
                        data = read_from_input_buffer();
                        msg_data_bytes[index] = data;
                        checksum = checksum ^ data;
                    }
                    msg_checksum = read_from_input_buffer();

                    if (msg_length == 0) {
                        std::cout << "SerialCom::read_msg_from_buffer  received request: " << msg_protocol << msg_direction << msg_length << msg_code << std::endl;
                        char cc;
                        for (uint16_t index=analysis_start; index < input_buffer_read_position; index++) {
                            //cc = printf("%c ", input_buffer[index]);
                            cc = printf("%i ", input_buffer[index]);
                            std::cout << cc;
                        }
                        std::cout << std::endl;
                        //input_buffer_read_position = analysis_start + 1;
                        msg->msg_preamble = msg_preamble;
                        msg->msg_protocol = msg_protocol;
                        msg->msg_direction = (MessageDirection) msg_direction;
                        msg->msg_length = (MessageLength) msg_length;
                        msg->msg_code = (MessageCode) msg_code;
                        Payload tmp;
                        memcpy(&tmp, msg_data_bytes, sizeof(tmp));
                        msg->msg_data = tmp;
                        msg->checksum = checksum;
                        std::cout << "sleeping " << std::endl;
                        sleep(1);
                        std::cout << "returning " << std::endl;
                        print_multiwii_message(msg);
                        return true;
                    } else if (msg_checksum == checksum) {
                        // the message is valid and can now be saved
                        msg->msg_preamble = msg_preamble;
                        msg->msg_protocol = msg_protocol;
                        msg->msg_direction = (MessageDirection) msg_direction;
                        msg->msg_length = (MessageLength) msg_length;
                        msg->msg_code = (MessageCode) msg_code;
                        Payload tmp;
                        memcpy(&tmp, msg_data_bytes, sizeof(tmp));
                        msg->msg_data = tmp;
                        msg->checksum = checksum;
                        printf("SerialCom::read_msg_from_buffer  received message! code %i length %i checksum: %i <-> %i \n", msg_code, msg_length, checksum, msg_checksum);
                        std::cout << "SerialCom::read_msg_from_buffer  returns true" << std::endl;
                        std::cout << "SerialCom::read_msg_from_buffer  " << msg_protocol << msg_direction << msg_length << msg_code << std::endl;
                        print_multiwii_message(msg);
                        std::cout << "SerialCom::read_msg_from_buffer  " << available() << std::endl;
                        return true;
                    } else {
                        //printf("SerialCom::read_msg_from_buffer  broken message received! code %i length %i checksum: %i <-> %i \n", msg_code, msg_length, checksum, msg_checksum);
                        input_buffer_read_position = analysis_start + 1;
                        if (input_buffer_read_position >= input_buffer_length) {
                            input_buffer_read_position -= input_buffer_length;
                        }
                    }
                } else {
                    // the data of this msg was not received completely jet, wait a bit.
                    input_buffer_read_position = analysis_start;
                    std::cout << "                         >> data not fully received! returning false" << std::endl;
                    return false;   // exiting because the available data is not entirely read into the input_buffer.
                }
            } else {
                std::cout << "                         >> no supported protocol byte" << std::endl;
                input_buffer_read_position = analysis_start + 1;
            }
        } else {
            std::cout << "                         >> no start byte" << std::endl;
        }
    }
    std::cout << "                         >> analysis reached real time" << std::endl;
    return false;   // exiting because there is now new data in the input_buffer
}


void print_multiwii_message(Message* msg) {
    printf(" print_multiwii_message:\n");
    printf(" msg_header: %c %c %c\n", msg->msg_preamble, msg->msg_protocol, msg->msg_direction);
    printf(" msg_length: %i\n", msg->msg_length);
    printf(" msg_code: %i\n", msg->msg_code);
    switch (msg->msg_code){
        case MULTIWII_STATUS:
            printf(" msg_data: MULTIWII_STATUS\n");
            printf(" cycleTime: %i\n", msg->msg_data.multiwii_status.cycleTime);
            printf(" i2c_errors_count: %i\n", msg->msg_data.multiwii_status.i2c_errors_count);
            break;
        case MULTIWII_IMU:
            printf(" msg_data: MULTIWII_IMU\n");
            printf(" acc: %i, %i, %i\n", msg->msg_data.multiwii_raw_imu.accx, msg->msg_data.multiwii_raw_imu.accy, msg->msg_data.multiwii_raw_imu.accz);
            printf(" gyro: %i, %i, %i\n", msg->msg_data.multiwii_raw_imu.gyrx, msg->msg_data.multiwii_raw_imu.gyry, msg->msg_data.multiwii_raw_imu.gyrz);
            printf(" mag: %i, %i, %i\n", msg->msg_data.multiwii_raw_imu.magx, msg->msg_data.multiwii_raw_imu.magy, msg->msg_data.multiwii_raw_imu.magz);
            break;
        case MULTIWII_RC:
            printf(" msg_data: MULTIWII_RC\n");
            printf(" roll: %i\n", msg->msg_data.multiwii_rc.roll);
            printf(" pitch: %i\n", msg->msg_data.multiwii_rc.pitch);
            printf(" yaw: %i\n", msg->msg_data.multiwii_rc.yaw);
            printf(" throttle: %i\n", msg->msg_data.multiwii_rc.throttle);
            printf(" aux1: %i\n", msg->msg_data.multiwii_rc.aux1);
            printf(" aux2: %i\n", msg->msg_data.multiwii_rc.aux2);
            printf(" aux3: %i\n", msg->msg_data.multiwii_rc.aux3);
            printf(" aux4: %i\n", msg->msg_data.multiwii_rc.aux4);
            break;
        case MULTIWII_MOTOR:
            printf(" msg_data: MULTIWII_MOTOR\n");
            printf(" motor0: %i\n", msg->msg_data.multiwii_motor.motor0);
            printf(" motor1: %i\n", msg->msg_data.multiwii_motor.motor1);
            printf(" motor2: %i\n", msg->msg_data.multiwii_motor.motor2);
            printf(" motor3: %i\n", msg->msg_data.multiwii_motor.motor3);
            printf(" motor4: %i\n", msg->msg_data.multiwii_motor.motor4);
            printf(" motor5: %i\n", msg->msg_data.multiwii_motor.motor5);
            printf(" motor6: %i\n", msg->msg_data.multiwii_motor.motor6);
            printf(" motor7: %i\n", msg->msg_data.multiwii_motor.motor7);
            break;

        case MULTIWII_ATTITUDE:
            printf(" msg_data: MULTIWII_ATTITUDE\n");
            printf(" roll: %i\n", msg->msg_data.multiwii_attitude.roll);
            printf(" pitch: %i\n", msg->msg_data.multiwii_attitude.pitch);
            printf(" yaw: %i\n", msg->msg_data.multiwii_attitude.yaw);
            break;
        case MULTIWII_GPS:
            printf(" msg_data: MULTIWII_GPS\n");
            printf(" fix: %i\n", msg->msg_data.multiwii_gps.fix);
            printf(" numSat: %i\n", msg->msg_data.multiwii_gps.numSat);
            printf(" coordLAT: %i\n", msg->msg_data.multiwii_gps.coordLAT);
            printf(" coordLON: %i\n", msg->msg_data.multiwii_gps.coordLON);
            printf(" altitude: %i\n", msg->msg_data.multiwii_gps.altitude);
            printf(" speed: %i\n", msg->msg_data.multiwii_gps.speed);
            break;
        default:
            break;
    }
}
