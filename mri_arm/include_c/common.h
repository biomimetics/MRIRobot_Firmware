#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>

// for debugging
#define PRINT_USM 0
#define PRINT_ENCODER 0

// for communication
#define DMA_TX_BUFFER_SIZE 400 //256//(STM_BUFFER_SIZE) //200
#define DMA_RX_BUFFER_SIZE 400 //256//(STM_BUFFER_SIZE) //200

// constants
#define TWO_PI 6.28318
#define NSEC_TO_SEC 0.000000001
#define MSEC_TO_SEC 0.001

// USM constants
#define ARR_PERIOD 2000 //2000 //2000 //2000 //10000 would be great but still sawtooth-esque //65535 // 16-bit timer period for ARR (auto reload register)
#define PWM_RPM_MAX 250.0 // 250 RPM
#define PWM_RAD_PER_SEC_MAX 26.17993875 // 250 RPM = 26.1799... rad/s
#define PWM_DEG_PER_SEC_MAX 1500.0 // 250 RPM = 1500 deg/s

// safety limits
#define USM_MAX_DUTY_CYCLE 0.30 //0.20 //0.30 //0.06 //0.5 // only higher than 6% for PWM pin testing //0.06 // 6% duty cycle
#define USM_MIN_DUTY_CYCLE 0.000 //0.005 // if below this amount, we don't expect the motors to be able to move smoothly on their own based on bench testing.

#define MOTOR_EXP_FILTER_ALPHA 0.9
#define MOTOR_VELOCITY_DEADBAND_LIMIT 0.1309 // rad/s or 7.5 deg/s
#define MOTOR_VELOCITY_MAX_CHANGE 0.5236 // rad/s or 30 deg/s


// conversion for if inputs are in rad_per_sec
#define RAD_PER_SEC_TO_RPM 9.549297
#define RAD_PER_SEC_TO_DEG_PER_SEC 57.2958

#define RPM_TO_DEG_PER_SEC 6.0


/*
// [START][LENGTH][TYPE][DATA...][CHECKSUM]
typedef struct {
    uint8_t start;
    uint8_t length;
    uint8_t type;
    uint8_t data[UART_BUFFER_SIZE - PACKET_BYTE_OVERHEAD]; // excluding start, len, type, checksum
    uint16_t crc16_checksum;
} Packet;
*/

/*
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float motor_positions[DOF_NUMBER]; // expected positions of the motors, used for bounds checking
    float desired_motor_velocities[DOF_NUMBER]; // actual motor commands send to the USM.lf
    int force_disable_motors; // set to 1 to force the motors to be disabled. used when position setpoint is reached and commanded velocities
    int time_stamp;
    int message_index;
} CommandMessage;
#pragma pack(pop)
*/

/*
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float motor_positions[DOF_NUMBER];
    float motor_velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float sea_velocities[DOF_NUMBER];
    float echoed_desired_velocities[DOF_NUMBER];
    float duty_cycles[DOF_NUMBER];
    int time_stamp;
    int message_index;
} StateMessage;
#pragma pack(pop)
*/


// =====================
// TX/RX packet defintiions
// =====================

/*
#define DOF_NUMBER 7
#define EXTRA_LENGTH 21

#define PACKET_BYTE_OVERHEAD 5
#define UART_BUFFER_SIZE  (180 + PACKET_BYTE_OVERHEAD) // needs to be at least max(sizeof(CommandMessage), sizeof(StateMessage)) + 4
#define PACKET_START_BYTE 0xAA
#define CHECKSUM_MOD 256

#define FLOAT_PRINT_SCALE 1000
#define FLOAT_DECIMAL_SCALE 6 //3
#define USLEEP_TIME 100 // used during reads
#define LONG_USLEEP_TIME 1000 // used at end of logging loops

// [START][LENGTH][TYPE][DATA...][CHECKSUM]
typedef struct {
    uint8_t start;
    uint8_t length;
    uint8_t type;
    uint8_t data[UART_BUFFER_SIZE - PACKET_BYTE_OVERHEAD]; // excluding start, len, type, checksum
    uint8_t checksum;
} Packet;

typedef enum {
    PKT_TYPE_PING = 0x01,
    PKT_TYPE_DATA = 0x02,
    // ... add more as needed
} PacketType;

#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];
    float velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float extra[EXTRA_LENGTH];
    int time_stamp;
    int message_index;
} CommandMessage;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];
    float velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float extra[EXTRA_LENGTH];
    int time_stamp;
    int message_index;
} StateMessage;
#pragma pack(pop)
*/
