#include <iostream>
#include <string.h>  // string function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <poll.h>
#include "multiwii_msg_protocol.h"

void print_multiwii_message(Message*);

class SerialCom {
public:
    // for configuration
    bool set_device(std::string);                   // for pre start configuration: define the serial device path here
    bool set_baudrate(uint32_t);                    // for pre start configuration: define in and out baudrate here
    bool set_max_io(uint16_t);                      // for pre start configuration: define maximum bytes read/send during one receive or send


    bool init();                                    // opens and configures serial com port. will return true on success, else false

    bool clear_input_buffer();                      // reads all available data on the serial com port but dismisses it! all buffered data is lost!
    bool clear_output_buffer();                     // resets the output_buffer array, so there are no bytes waiting for sending in next send_from_buffer()

    bool receive_to_buffer();                       // this reads the available bytes on the serial line into the input_buffer array, the upper limit of bytes is set by set_max_io()
    bool send_from_buffer();                        // this sends all data which is in the output_buffer.

    // for user information and debug
    bool print_input_buffer();
    bool print_output_buffer();

    bool send_msg_rc(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t)
    bool send_request(MessageCode);                 // sends a request via multiwii protocol, basically it creates a request message and uses write_msg_to_buffer(Message)
    bool write_msg_to_buffer(Message);
    bool write_to_output_buffer(uint8_t);
    uint8_t read_from_input_buffer();
    bool read_msg_from_buffer(Message*);

    bool deinitialize();
private:
    // usually /dev/tty.* or /dev/cu.*
    const char* serial_device_path;
    // the used baudrate like 9600, 57600 or 115200
    uint32_t serial_device_baud_rate;
    // maximum numbers of bytes which are read or written during one cycle
    uint16_t buffer_io_max;

    // the handles for the serial device and its config after it is opened
    int serial_interface;
    struct termios usb_tio;

    // buffer which is used for reading and writing
    // funny error occurs if you use:
    /*
    static const uint16_t input_buffer_length = 128;
    static const uint16_t output_buffer_length = 128;
    */
    static const uint16_t input_buffer_length = 256;
    static const uint16_t output_buffer_length = 256;

    uint8_t input_buffer[input_buffer_length];
    uint16_t input_buffer_write_position;
    uint16_t input_buffer_read_position;

    uint8_t output_buffer[output_buffer_length];
    uint16_t output_buffer_write_position;          // the function adding data to be sent writes to this position
    uint16_t output_buffer_read_position;           // the sending function reads from this buffer
};
