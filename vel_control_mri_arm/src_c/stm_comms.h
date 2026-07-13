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
// Wire protocol version
// =====================
// Bumped whenever CommandMessage/StateMessage's layout changes. Checked on
// decode (see handle_command_message_packet/handle_state_message_packet) so
// a stale binary on either end of the link fails loudly instead of
// silently misinterpreting bytes -- the two copies of this file
// (vel_control_mri_arm/src_c and MRIRobot_ROS's mri_arm_hardware) have no
// shared build system to otherwise catch drift between them.
#define PROTOCOL_VERSION 4

// =====================
// TX/RX packet definitions
// =====================

#define DOF_NUMBER 7

// Framing: [START][LENGTH][TYPE][VERSION][DATA...][CRC_LO][CRC_HI]
// LENGTH is the byte count of (TYPE + VERSION + DATA) -- i.e. everything
// between LENGTH and the CRC, unambiguously: a decoder computes
// total_packet_size = 2 (start+length) + LENGTH + 2 (crc), no guessing.
// (The old protocol's LENGTH excluded a TYPE-sized byte from this count in
// a way that a previous read_packet_bulk() implementation got wrong -- see
// linux_comms.c's rewritten read_packet_bulk for how this resolves that.)
#define PACKET_START_BYTE 0xAA
#define PACKET_HEADER_SIZE 2   // START, LENGTH
#define PACKET_TYPE_VERSION_SIZE 2 // TYPE, VERSION -- included in LENGTH
#define PACKET_CRC_SIZE 2      // CRC_LO, CRC_HI
#define PACKET_OVERHEAD (PACKET_HEADER_SIZE + PACKET_TYPE_VERSION_SIZE + PACKET_CRC_SIZE) // 6

typedef enum {
    PKT_TYPE_PING = 0x01,
    PKT_TYPE_DATA = 0x02,
    // ... add more as needed
} PacketType;

// =====================
// Command Message data structure definitions and helper functions
// =====================
// Host -> STM32. Trimmed (protocol v4) to only the fields firmware still
// reads (State_Machine.lf's command_message reaction): behavior_mode and
// velocities, plus time_stamp/message_index for sequencing. The old
// position_deltas/position_offset/position_offset_sequence fields were
// dropped along with the pulse controllers and the position-offset restore
// path they fed (see State_Machine.lf), and are not carried forward.
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float velocities[DOF_NUMBER];       // rad/s, pass-through velocity command
    int time_stamp;
    int message_index;
} CommandMessage;
#pragma pack(pop)

void construct_command_message(
    CommandMessage* msg,
    int behavior_mode,
    const float* velocities,
    int time_stamp,
    int message_index
);
void zero_command_message(CommandMessage* msg);

int encode_command_message_to_data_buffer(const CommandMessage *msg, uint8_t *data_buffer);
bool decode_data_buffer_to_command_message(CommandMessage *msg, const uint8_t *data_buffer, size_t data_buffer_len);
bool handle_command_message_packet(CommandMessage* msg, const uint8_t *packet, size_t packet_len);

void print_command_message(const CommandMessage *msg);
void print_command_message_int(const CommandMessage *msg);

// =====================
// State Message data structure definitions and helper functions
// =====================
// STM32 -> host. positions/velocities/sea_positions are real measured
// motor-space feedback. Trimmed (protocol v4): the old sea_velocities,
// commanded_motor_velocity, and running_single_pulse_command fields were
// dropped along with the pulse controllers whose telemetry they carried.
// echoed_time_stamp/echoed_message_index carry back the time_stamp and
// message_index of the last CommandMessage the firmware received, letting
// the host estimate round-trip communication delay and align the command
// and state streams.
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];               // rad, measured motor position
    float velocities[DOF_NUMBER];               // rad/s, measured motor velocity
    float sea_positions[DOF_NUMBER];            // rad, measured SEA deflection
    int time_stamp;
    int message_index;
    int echoed_time_stamp; // the last time stamp we received from CommandMessage. Used for communication delay estimation on the host side.
    int echoed_message_index; // the last message index we received from CommandMessage. Also used for communication delay estimation and alignment.
} StateMessage;
#pragma pack(pop)

void construct_state_message(
    StateMessage* msg,
    int behavior_mode,
    const float* positions,
    const float* velocities,
    const float* sea_positions,
    int time_stamp,
    int message_index,
    int echoed_time_stamp,
    int echoed_message_index
);
void zero_state_message(StateMessage* msg);

int encode_state_message_to_data_buffer(const StateMessage *msg, uint8_t *data_buffer);
bool decode_data_buffer_to_state_message(StateMessage *msg, const uint8_t *data_buffer, size_t data_buffer_len);
bool handle_state_message_packet(StateMessage* msg, const uint8_t *packet, size_t len);

void print_state_message(const StateMessage *msg);
void print_state_message_int(const StateMessage *msg);

void write_state_message_csv_header(char *buffer, size_t size);
void serialize_state_message_csv(const StateMessage *msg, char *buffer, size_t size);

#define COMMAND_MSG_SIZE ((int) sizeof(CommandMessage))
#define STATE_MSG_SIZE ((int) sizeof(StateMessage))

// Buffer needs to hold the larger of the two messages, plus framing
// overhead -- StateMessage is the larger one today.
#define UART_BUFFER_SIZE ((STATE_MSG_SIZE > COMMAND_MSG_SIZE ? STATE_MSG_SIZE : COMMAND_MSG_SIZE) + PACKET_OVERHEAD)

#define FLOAT_PRINT_SCALE 1000
#define FLOAT_DECIMAL_SCALE 6
#define USLEEP_TIME 100 // used during reads
#define LONG_USLEEP_TIME 1000 // used at end of logging loops

// =====================
// Packet framing + CRC
// =====================

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no output
// xor. Bit-banged (no table) -- packets here are ~150 bytes at 10ms
// cadence, negligible cost even unoptimized, and a table would be one more
// thing that has to stay byte-identical between the two copies of this file.
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

void print_buffer(uint8_t *buffer, uint16_t length);

// Builds a full framed packet ([START][LENGTH][TYPE][VERSION][DATA...][CRC_LO][CRC_HI])
// into out_buf. Returns the total packet length, or -1 if data_len is too
// large for the buffer this is meant to fit in.
int build_packet(uint8_t *out_buf, PacketType type, const uint8_t *data, uint8_t data_len);

#endif // STM_COMMS_H
