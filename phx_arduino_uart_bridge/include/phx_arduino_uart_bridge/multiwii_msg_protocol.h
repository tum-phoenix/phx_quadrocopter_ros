enum MessageDirection : uint8_t {
    MULTIWII_TO_COM = '<',
    COM_TO_MULTIWII = '>'
};

enum MessageCode : uint8_t {
    MULTIWII_STATUS = 101,
    MULTIWII_IMU = 102,
    MULTIWII_SERVO = 103,
    MULTIWII_MOTOR = 104,
    MULTIWII_RC = 105,
    MULTIWII_GPS = 106,
    MULTIWII_ATTITUDE = 108,
    MULTIWII_ALTITUDE = 109,
    MULTIWII_MOTOR_SET = 214,
    MULTIWII_RC_SET = 200
};

enum MessageLength : uint8_t {
    REQUEST = 0,
    MULTIWII_STATUS_LENGTH = 11,
    MULTIWII_IMU_LENGTH = 18,
    MULTIWII_SERVO_LENGTH = 16,
    MULTIWII_MOTOR_LENGTH = 16,
    MULTIWII_RC_LENGTH = 16,
    MULTIWII_GPS_LENGTH = 16,
    MULTIWII_ATTITUDE_LENGTH = 6,
    MULTIWII_ALTITUDE_LENGTH = 6,
    MULTIWII_MOTOR_SET_LENGTH = 16,
    MULTIWII_RC_SET_LENGTH = 16
};

struct Payload {
    union {
        struct {
            uint16_t cycleTime;
            uint16_t i2c_errors_count;
            uint16_t sensor;
            uint32_t flag;
            uint8_t global_conf;
        } multiwii_status;

        struct {
            int16_t accx;
            int16_t accy;
            int16_t accz;
            int16_t gyrx;
            int16_t gyry;
            int16_t gyrz;
            int16_t magx;
            int16_t magy;
            int16_t magz;
        } multiwii_raw_imu;

        struct {
            uint16_t servo0;
            uint16_t servo1;
            uint16_t servo2;
            uint16_t servo3;
            uint16_t servo4;
            uint16_t servo5;
            uint16_t servo6;
            uint16_t servo7;
        } multiwii_servo;

        struct {
            uint16_t motor0;
            uint16_t motor1;
            uint16_t motor2;
            uint16_t motor3;
            uint16_t motor4;
            uint16_t motor5;
            uint16_t motor6;
            uint16_t motor7;
        } multiwii_motor;

        struct {
            uint16_t motor0;
            uint16_t motor1;
            uint16_t motor2;
            uint16_t motor3;
            uint16_t motor4;
            uint16_t motor5;
            uint16_t motor6;
            uint16_t motor7;
        } multiwii_motor_set;

        struct {
            uint16_t roll;
            uint16_t pitch;
            uint16_t yaw;
            uint16_t throttle;
            uint16_t aux1;
            uint16_t aux2;
            uint16_t aux3;
            uint16_t aux4;
        } multiwii_rc;

        struct {
            uint16_t roll;
            uint16_t pitch;
            uint16_t yaw;
            uint16_t throttle;
            uint16_t aux1;
            uint16_t aux2;
            uint16_t aux3;
            uint16_t aux4;
        } multiwii_rc_set;

        struct {
            uint8_t fix;
            uint8_t numSat;
            uint32_t coordLAT;
            uint32_t coordLON;
            uint16_t altitude;
            uint16_t speed;
            uint16_t ground_course;
        } multiwii_gps;

        struct {
            uint16_t roll;
            uint16_t pitch;
            uint16_t yaw;
        } multiwii_attitude;

        struct {
            uint32_t estAlt;
            uint16_t variation;
        } multiwii_altitude;
    };
};

struct Message {
    uint8_t msg_preamble;
    uint8_t msg_protocol;
    MessageDirection msg_direction;
    MessageLength msg_length;
    MessageCode msg_code;
    Payload msg_data;
    uint8_t checksum;
};

/*
void print_multiwii_message(Message* msg) {
    printf(" print_multiwii_message:\n");
    printf("   msg_header: %c %c %c\n", msg->msg_preamble, msg->msg_protocol, msg->msg_direction);
    printf("   msg_length: %i\n", msg->msg_length);
    printf("   msg_code: %i\n", msg->msg_code);
    switch (msg->msg_code){
        case MULTIWII_STATUS:
            printf("   msg_data: MULTIWII_STATUS\n");
            printf("        cycleTime:          %i\n", msg->msg_data.multiwii_status.cycleTime);
            printf("        i2c_errors_count:   %i\n", msg->msg_data.multiwii_status.i2c_errors_count);
            break;
        case MULTIWII_IMU:
            printf("   msg_data: MULTIWII_IMU\n");
            printf("        acc:    %i, %i, %i\n", msg->msg_data.multiwii_raw_imu.accx, msg->msg_data.multiwii_raw_imu.accy, msg->msg_data.multiwii_raw_imu.accz);
            printf("        gyro:   %i, %i, %i\n", msg->msg_data.multiwii_raw_imu.gyrx, msg->msg_data.multiwii_raw_imu.gyry, msg->msg_data.multiwii_raw_imu.gyrz);
            printf("        mag:    %i, %i, %i\n", msg->msg_data.multiwii_raw_imu.magx, msg->msg_data.multiwii_raw_imu.magy, msg->msg_data.multiwii_raw_imu.magz);
            break;
        case MULTIWII_RC:
            printf("   msg_data: MULTIWII_RC\n");
            printf("        roll:     %i\n", msg->msg_data.multiwii_rc.roll);
            printf("        pitch:    %i\n", msg->msg_data.multiwii_rc.pitch);
            printf("        yaw:      %i\n", msg->msg_data.multiwii_rc.yaw);
            printf("        throttle: %i\n", msg->msg_data.multiwii_rc.throttle);
            printf("        aux1:     %i\n", msg->msg_data.multiwii_rc.aux1);
            printf("        aux2:     %i\n", msg->msg_data.multiwii_rc.aux2);
            printf("        aux3:     %i\n", msg->msg_data.multiwii_rc.aux3);
            printf("        aux4:     %i\n", msg->msg_data.multiwii_rc.aux4);
            break;
        case MULTIWII_GPS:
            printf("   msg_data: MULTIWII_GPS\n");
            printf("        roll:     %i\n", msg->msg_data.multiwii_gps.fix);
            printf("        numSat:   %i\n", msg->msg_data.multiwii_gps.numSat);
            printf("        coordLAT: %i\n", msg->msg_data.multiwii_gps.coordLAT);
            printf("        coordLON: %i\n", msg->msg_data.multiwii_gps.coordLON);
            printf("        altitude: %i\n", msg->msg_data.multiwii_gps.altitude);
            printf("        speed:    %i\n", msg->msg_data.multiwii_gps.speed);
            break;
        default:
            break;
    }
}
*/
