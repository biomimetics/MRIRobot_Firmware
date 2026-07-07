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
#define PROTOCOL_VERSION 2

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
// Host -> STM32. Trimmed to only the fields firmware actually reads
// (State_Machine.lf's command_message reaction): behavior_mode, velocities,
// and now position_deltas. The old positions[]/sea_positions[]/extra[21]
// fields were confirmed dead (firmware never read them, host only ever
// wrote zeros) and are dropped entirely rather than carried forward.
// position_offset/position_offset_sequence: static per-joint offset for
// EncoderStateEstimator.lf (see that file and State_Machine.lf's
// position_offset output), for restoring the STM32's estimated position to
// a pre-known value after an STM32/FPGA restart. Expected to change at most
// once every several minutes, event-based rather than every cycle -- unlike
// every other field here, NOT threaded through construct_command_message
// below (would mean plumbing two rarely-used parameters through every
// existing call site for no benefit); host-side code that wants to set a
// new offset should assign msg.position_offset[i]/msg.position_offset_sequence
// directly on an already-constructed/zeroed message. position_offset_sequence
// is a plain increment-on-change counter (NOT a message index) -- host
// bumps it only when it actually wants the firmware to apply a new offset;
// State_Machine.lf compares it against the last value it saw to detect that
// edge rather than reapplying every message_index tick.
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float velocities[DOF_NUMBER];       // rad/s, pass-through velocity command
    float position_deltas[DOF_NUMBER];  // rad, remaining position error -> Small_DeltaP_Controller
                                         // (was the dead `positions[]` field)
    float position_offset[DOF_NUMBER];  // rad, static per-joint offset -- see comment above
    int position_offset_sequence;       // increment-on-change counter -- see comment above
    int time_stamp;
    int message_index;
} CommandMessage;
#pragma pack(pop)

void construct_command_message(
    CommandMessage* msg,
    int behavior_mode,
    const float* velocities,
    const float* position_deltas,
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
// motor-space feedback (unchanged). sea_velocities is new (QDEC's
// Encoder.lf already computes this internally as sea_vel_out, it just
// wasn't wired to State_Machine.lf before). commanded_motor_velocity
// replaces the old extra[21]'s tribal-knowledge layout (a velocity echo at
// extra[0:7), a dead always-zero placeholder at extra[7:14), USM duty cycle
// at extra[14:21)) with a single honestly-named field -- no
// Small_DeltaP_Controller-internal telemetry goes over the wire.
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float positions[DOF_NUMBER];               // rad, measured motor position
    float velocities[DOF_NUMBER];               // rad/s, measured motor velocity
    float sea_positions[DOF_NUMBER];            // rad, measured SEA deflection
    float sea_velocities[DOF_NUMBER];           // rad/s, measured SEA velocity
    float commanded_motor_velocity[DOF_NUMBER]; // rad/s, velocity actually commanded this cycle
    int time_stamp;
    int message_index;
} StateMessage;
#pragma pack(pop)

void construct_state_message(
    StateMessage* msg,
    int behavior_mode,
    const float* positions,
    const float* velocities,
    const float* sea_positions,
    const float* sea_velocities,
    const float* commanded_motor_velocity,
    int time_stamp,
    int message_index
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
