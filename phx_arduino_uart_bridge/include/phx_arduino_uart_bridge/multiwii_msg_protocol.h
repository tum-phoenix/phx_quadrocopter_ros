enum MessageDirection : uint8_t {
    ERROR = '!',
    MULTIWII_TO_COM = '>',
    COM_TO_MULTIWII = '<'
};

enum MessageCode : uint8_t {
    MARVIC_CONTINUOUS_SENDING = 69,
    MARVIC_BATTERY = 66,
    MARVIC_AUTONOMOUS_FLIGHT = 67,
    MARVIC_SONAR = 68,
    MULTIWII_STATUS = 101,
    MULTIWII_IMU = 102,
    MULTIWII_SERVO = 103,
    MULTIWII_MOTOR = 104,
    MULTIWII_RC = 105,
    MULTIWII_GPS = 106,
    MULTIWII_ATTITUDE = 108,
    MULTIWII_ALTITUDE = 109,
    MULTIWII_MOTOR_SET = 214,           // setting motor
    MULTIWII_RC_SET = 200               // setting rc
};

enum MessageLength : uint8_t {
    REQUEST = 0,
    MARVIC_CONTINUOUS_SENDING_LENGTH = 1,
    MARVIC_BATTERY_LENGTH = 8,
    MARVIC_AUTONOMOUS_FLIGHT_LENGTH = 1,
    MARVIC_SONAR_LENGTH = 1,
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
            uint8_t on_off;
        } marvic_continuous_sending;

        struct {
            uint16_t cell1_mean;
            uint16_t cell2_mean;
            uint16_t cell3_mean;
            uint16_t cell4_mean;
        } marvic_battery;

        struct {
            uint8_t is_active;
        } marvic_autonomous;

        struct {
            uint16_t distance;
        } marvic_sonar;

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
            uint16_t aux5;
            uint16_t aux6;
            uint16_t aux7;
            uint16_t aux8;
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