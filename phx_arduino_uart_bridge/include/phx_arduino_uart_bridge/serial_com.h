#include <iostream>
#include <string.h>  // string function definitions
#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <poll.h>
#include "multiwii_msg_protocol.h"

void print_multiwii_message(Message*);

class SerialCom {

/*
MultiWii Protocol:

$ M {<, >, !} {msg_length} {msg_code} [payload] {check byte}

$ -> start byte marks the start of every valid message

M -> protocol byte marks that in this case MultiWii protocol is used. This allows implementation of up to 255 different protocols at the same time!

{<, >, !} -> direction byte: < means this message is meant for the micro controller,
                             > means this message is meant for the computer,
                             ! indicates an error message. no direction information is available.

{msg_length} -> gives the length of the [payload] in number of bytes. For a request the payload is empty, therefore the msg_length = 0

{msg_code} -> a byte which indicates which type of message is delivered.

[payload] -> sequence of bytes containing the actual data of the message.
             The number of bytes is given by the {msg_length}. It is possible that there is payload byte at all

{check byte} -> this is a bitwise xor ^ of all bytes starting with the {msg_length} byte.
*/


/*
This implementation:

serial input:
    all serial input is saved into an array ->
        this array gets analysed for new messages.

serial output:
        we write the new message into an array ->
    this array is sent to the serial port from time to time.
*/

public:
    // for configuration
    bool set_device(std::string);                   // for pre start configuration: define the serial device path here
    bool set_baudrate(uint32_t);                    // for pre start configuration: define in and out baudrate here
    bool set_max_io(uint16_t);                      // for pre start configuration: define maximum bytes read/send during one receive or send

    // starting up
    bool init();                                    // opens and configures serial com port. will return true on success, else false

    bool clear_input_buffer();                      // reads all available data on the serial com port but dismisses it! all buffered data is lost!
    bool clear_output_buffer();                     // resets the output_buffer array, so there are no bytes waiting for sending in next send_from_buffer()

    // communication with the serial port
    bool receive_to_buffer();                       // this reads the available bytes on the serial line into the input_buffer array, the upper limit of bytes is set by set_max_io()
    bool send_from_buffer();                        // this sends all data which is in the output_buffer.

    // for user information and debug
    bool print_input_buffer();
    bool print_output_buffer();

    // preparing different messages -> they all call write_msg_to_buffer() in the end
    bool prepare_msg_continuous_sending(uint8_t);
    bool prepare_msg_rc(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);
    bool prepare_msg_motor(uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t);
    bool prepare_msg_gps_get_way_point(uint8_t);
    bool prepare_msg_gps_set_way_point(uint8_t, uint32_t, uint32_t, uint32_t, uint16_t, uint16_t, uint8_t);
    bool prepare_request(MessageCode);                 // sends a request via multiwii protocol, basically it creates a request message and uses write_msg_to_buffer(Message)

    // adding message to the output buffer -> afterwards send_from_buffer() can be used to send the message to the serial port
    bool write_msg_to_buffer(Message);
    bool write_to_output_buffer(uint8_t);

    // interpreting messages which are waiting in the input buffer.
    uint16_t available();                           // returns the available bytes in the input_buffer
    uint8_t read_from_input_buffer();               // reads the next byte from the input_buffer
    bool read_msg_from_buffer(Message*);            // this reads the next full message from the buffer and writes it into the given Message*

    bool deinitialize();                            // closes the serial port properly
private:
    bool do_debug_printout = false;

    const char* serial_device_path;                 // usually /dev/tty.* or /dev/cu.*
    uint32_t serial_device_baud_rate;               // the used baudrate like 9600, 57600 or 115200
    uint16_t buffer_io_max;                         // maximum numbers of bytes which are read or written during one cycle
    int serial_interface;                           // the handle for the serial device and its config after it is opened
    struct termios usb_tio;                         // for configuration during init

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
    uint16_t output_buffer_write_position;
    uint16_t output_buffer_read_position;
};
