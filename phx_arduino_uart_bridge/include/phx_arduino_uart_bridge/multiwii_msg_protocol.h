enum MessageDirection : uint8_t {
    ERROR = '!',
    MULTIWII_TO_COM = '>',
    COM_TO_MULTIWII = '<'
};

enum MessageProtocol : uint8_t {
    MULTIWII_PROTOCOL = 'M',
    PHOENIX_PROTOCOL = 'P'
};

enum MessageCode : uint8_t {
    MARVIC_LED_0 = 50,
    MARVIC_LED_1 = 51,
    MARVIC_LED_2 = 52,
    MARVIC_LED_3 = 53,
    MARVIC_SINGLE_LED = 54,
    MARVIC_BATTERY = 66,
    MARVIC_SONAR = 68,
    MARVIC_INFRA_RED = 69,
    MARVIC_LIDAR = 70,
    MARVIC_BAROMETER = 71,
    MULTIWII_STATUS = 101,
    MULTIWII_IMU = 102,
    MULTIWII_SERVO = 103,
    MULTIWII_MOTOR = 104,
    MULTIWII_RC = 105,
    MULTIWII_GPS = 106,
//    MULTIWII_PID = 112,
    MULTIWII_GPS_WP = 118,
    MULTIWII_ATTITUDE = 108,
    MULTIWII_ALTITUDE = 109,
    MULTIWII_SERVO_SET = 213,           // setting servos
    MULTIWII_MOTOR_SET = 214,           // setting motor
    MULTIWII_RC_SET = 200,              // setting rc
//    MULTIWII_PID_SET = 202,
    MULTIWII_GPS_WP_SET = 209           // setting gps way point
};

enum MessageLength : uint8_t {
    REQUEST = 0,
    MARVIC_BATTERY_LENGTH = 8,
    MARVIC_SONAR_LENGTH = 6,
    MARVIC_LIDAR_LENGTH = 6,
    MARVIC_INFRA_RED_LENGTH = 6,
    MARVIC_BAROMETER_LENGTH = 6,
    MARVIC_STRIP_LED_LENGTH = 30,
    MARVIC_SINGLE_LED_LENGTH = 5,
    MULTIWII_STATUS_LENGTH = 11,
    MULTIWII_IMU_LENGTH = 18,
    MULTIWII_SERVO_LENGTH = 16,
    MULTIWII_MOTOR_LENGTH = 16,
    MULTIWII_RC_LENGTH = 16,
    MULTIWII_GPS_LENGTH = 16,
    REQUEST_GPS_WP = 1,
    MULTIWII_GPS_WP_LENGTH = 18,
    MULTIWII_ATTITUDE_LENGTH = 6,
    MULTIWII_ALTITUDE_LENGTH = 6,
    MULTIWII_SERVO_SET_LENGTH = 36,
    MULTIWII_MOTOR_SET_LENGTH = 16,
    MULTIWII_RC_SET_LENGTH = 16,
    MULTIWII_GPS_WP_SET_LENGTH = 18
};

struct Payload {
    union {
        struct {
            uint8_t led_0_r;
            uint8_t led_0_g;
            uint8_t led_0_b;
            uint8_t led_1_r;
            uint8_t led_1_g;
            uint8_t led_1_b;
            uint8_t led_2_r;
            uint8_t led_2_g;
            uint8_t led_2_b;
            uint8_t led_3_r;
            uint8_t led_3_g;
            uint8_t led_3_b;
            uint8_t led_4_r;
            uint8_t led_4_g;
            uint8_t led_4_b;
            uint8_t led_5_r;
            uint8_t led_5_g;
            uint8_t led_5_b;
            uint8_t led_6_r;
            uint8_t led_6_g;
            uint8_t led_6_b;
            uint8_t led_7_r;
            uint8_t led_7_g;
            uint8_t led_7_b;
            uint8_t led_8_r;
            uint8_t led_8_g;
            uint8_t led_8_b;
            uint8_t led_9_r;
            uint8_t led_9_g;
            uint8_t led_9_b;
        } marvic_led_strip;                 // MARVIC_LED_0 = 50, MARVIC_LED_1 = 51, MARVIC_LED_2 = 52, MARVIC_LED_3 = 53

        struct {
            uint8_t led_id;
            uint8_t strip_index;
            uint8_t led_r;
            uint8_t led_g;
            uint8_t led_b;
        } marvic_led_single;                // MARVIC_SINGLE_LED = 54

        struct {
            uint32_t millisecond_time_stamp;
            uint16_t distance;
        } marvic_altitude;                  // MARVIC_SONAR = 68, MARVIC_INFRA_RED = 69, MARVIC_LIDAR = 70, MARVIC_BAROMETER = 71

        struct {
            uint16_t cell1_mean;
            uint16_t cell2_mean;
            uint16_t cell3_mean;
            uint16_t cell4_mean;
        } marvic_battery;

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
            uint16_t servo8;
            uint16_t servo9;
            uint16_t servo10;
            uint16_t servo11;
            uint16_t servo12;
            uint16_t servo13;
            uint16_t servo14;
            uint16_t servo15;
            uint16_t servo16;
            uint16_t servo17;
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
            int32_t coordLAT;
            int32_t coordLON;
            uint16_t altitude;
            uint16_t speed;
            uint16_t ground_course;
        } multiwii_gps;

        struct {
            uint8_t fix;
            uint8_t numSat;
            uint8_t coordLAT0;
            uint8_t coordLAT1;
            uint8_t coordLAT2;
            uint8_t coordLAT3;
            uint8_t coordLON0;
            uint8_t coordLON1;
            uint8_t coordLON2;
            uint8_t coordLON3;
            uint16_t altitude;
            uint16_t speed;
            uint16_t ground_course;
        } multiwii_gps_save;

        struct {
            uint8_t wp_number;
            int32_t coordLAT;
            int32_t coordLON;
            uint32_t altitude;
            uint16_t heading;
            uint16_t stay_time;
            uint8_t nav_flag;
        } multiwii_gps_way_point;

        struct {
            int16_t roll;
            int16_t pitch;
            int16_t yaw;
        } multiwii_attitude;

        struct {
            int32_t estAlt;
            uint16_t variation;
        } multiwii_altitude;

        struct {
            uint8_t wp_number;
            int32_t coordLAT;
            int32_t coordLON;
            uint32_t altitude;
            uint16_t heading;
            uint16_t stay_time;
            uint8_t nav_flag;
        } multiwii_gps_set_way_point;
    };
};

struct Message {
    uint8_t msg_preamble;
    MessageProtocol msg_protocol;
    MessageDirection msg_direction;
    MessageLength msg_length;
    MessageCode msg_code;
    Payload msg_data;
    uint8_t checksum;
};