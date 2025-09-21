#ifndef STM_COMMS_H
#define STM_COMMS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>

// =====================
// Serial port definitions
// =====================

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 921600 //B115200

// =====================
// TX/RX packet defintiions
// =====================

#define UART_BUFFER_SIZE  120 // needs to be at least max(sizeof(SendData, ReceiveData) + 4
#define PACKET_START_BYTE 0xAA

#define USLEEP_TIME 1000
#define LONG_USLEEP_TIME 1000 //5000

typedef enum {
    PKT_TYPE_PING = 0x01,
    PKT_TYPE_DATA = 0x02,
    // ... add more as needed
} PacketType;

// [START][LENGTH][TYPE][DATA...][CHECKSUM]
typedef struct {
    uint8_t start;
    uint8_t length;
    uint8_t type;
    uint8_t data[UART_BUFFER_SIZE - 4]; // excluding start, len, type, checksum
    uint8_t checksum;
} Packet;

int open_serial(const char *port);
int build_packet(uint8_t *out_buf, PacketType type, uint8_t *data, uint8_t data_len);
int read_packet(int fd, uint8_t *buf, int max_len);

// =====================
// Sending/Receiving data structure definitions and helper functions
// =====================

#define DOF_NUMBER 7
#define EXTRA_LENGTH 1

#define FLOAT_PRINT_SCALE 1000

#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];
    float velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float extra[EXTRA_LENGTH];
    int time_stamp;
} Send_Data;
#pragma pack(pop)



void construct_send_data(
    Send_Data* packet,
    int behavior_mode,
    const float* positions,
    const float* velocities,
    const float* sea_positions,
    const float* extra,
    int time_stamp
);


#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];
    float velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float extra[EXTRA_LENGTH];
    int time_stamp;
} Receive_Data;
#pragma pack(pop)

void construct_receive_data(
    Receive_Data* packet,
    int behavior_mode,
    const float* positions,
    const float* velocities,
    const float* sea_positions,
    const float* extra,
    int time_stamp
);
void encode_receive_data_to_bytes(const Receive_Data *packet, uint8_t *buffer);
void decode_receive_bytes_to_data(uint8_t *data, size_t data_len);




// =====================
// Debug Utility
// =====================

/**
 * @brief Pretty-print the contents of a Send_Data to the console.
 *
 * @param packet Pointer to the packet to print
 */
void print_send_packet(const Send_Data *packet);

/**
 * @brief Pretty-print the contents of an Send_Data to the console, with floats presented as ints scaled by FLOAT_PRINT_SCALE.
 *
 * @param packet Pointer to the packet to print
 */
void print_send_packet_int(const Send_Data *packet);

/**
 * @brief Pretty-print the contents of a Receive_Data to the console.
 *
 * @param packet Pointer to the packet to print
 */
void print_receive_packet(const Receive_Data *packet);

/**
 * @brief Pretty-print the contents of an Receive_Data to the console, with floats presented as ints scaled by FLOAT_PRINT_SCALE.
 *
 * @param packet Pointer to the packet to print
 */
void print_receive_packet_int(const Receive_Data *packet);

#endif // STM_COMMS_H
