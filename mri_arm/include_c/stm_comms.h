#ifndef STM_COMMS_H
#define STM_COMMS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>

#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>
#include <stdbool.h>

// =====================
// Serial port definitions
// =====================

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 921600 //B115200

// =====================
// TX/RX packet defintiions
// =====================

#define DOF_NUMBER 7
#define EXTRA_LENGTH 21 // 1

#define COMMAND_MSG_SIZE (4 + 4 * DOF_NUMBER * 3 + 4 * EXTRA_LENGTH * 1 + 4)
#define STATE_MSG_SIZE (4 + 4 * DOF_NUMBER * 3 + 4 * EXTRA_LENGTH * 1 + 4)

#define PACKET_BYTE_OVERHEAD 4
#define UART_BUFFER_SIZE  (COMMAND_MSG_SIZE + PACKET_BYTE_OVERHEAD) // needs to be at least max(sizeof(CommandMessage), sizeof(StateMessage)) + 4
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
    uint8_t data[UART_BUFFER_SIZE - PACKET_BYTE_OVERHEAD]; // excluding start, len, type, checksum
    uint8_t checksum;
} Packet;

int build_packet(uint8_t *out_buf, PacketType type, uint8_t *data, uint8_t data_len);
int read_packet(int fd, uint8_t *buf, int max_iterations);

// =====================
// Command Message data structure definitions and helper functions
// =====================



#define FLOAT_PRINT_SCALE 1000

#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];
    float velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float extra[EXTRA_LENGTH];
    int time_stamp;
} CommandMessage;
#pragma pack(pop)



void construct_command_message(
    CommandMessage* msg,
    int behavior_mode,
    const float* positions,
    const float* velocities,
    const float* sea_positions,
    const float* extra,
    int time_stamp
);

size_t encode_command_message_to_data_buffer(const CommandMessage *msg, uint8_t *data_buffer);
bool decode_data_buffer_to_command_message(CommandMessage *msg, uint8_t *data_buffer, size_t data_buffer_len);
bool handle_command_message_packet(CommandMessage* msg, uint8_t *packet, size_t packet_len);


/**
 * @brief Pretty-print the contents of a Send_Data to the console.
 *
 * @param packet Pointer to the packet to print
 */
void print_command_message(const CommandMessage *msg);

/**
 * @brief Pretty-print the contents of an Send_Data to the console, with floats presented as ints scaled by FLOAT_PRINT_SCALE.
 *
 * @param packet Pointer to the packet to print
 */
void print_command_message_int(const CommandMessage *msg);


// =====================
// State Message data structure definitions and helper functions
// =====================

#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];
    float velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float extra[EXTRA_LENGTH];
    int time_stamp;
} StateMessage;
#pragma pack(pop)

void construct_state_message(
    StateMessage* msg,
    int behavior_mode,
    const float* positions,
    const float* velocities,
    const float* sea_positions,
    const float* extra,
    int time_stamp
);
int encode_state_message_to_data_buffer(const StateMessage *msg, uint8_t *data_buffer);
bool decode_data_buffer_to_state_message(StateMessage *msg, uint8_t *data_buffer, size_t data_buffer_len);
bool handle_state_message_packet(StateMessage* msg, uint8_t *packet, size_t packet_len);

/**
 * @brief Pretty-print the contents of a Receive_Data to the console.
 *
 * @param packet Pointer to the packet to print
 */
void print_state_message(const StateMessage *msg);

/**
 * @brief Pretty-print the contents of an Receive_Data to the console, with floats presented as ints scaled by FLOAT_PRINT_SCALE.
 *
 * @param packet Pointer to the packet to print
 */
void print_state_message_int(const StateMessage *msg);

#endif // STM_COMMS_H