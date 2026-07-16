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
#define PROTOCOL_VERSION 5

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
//
// v5: velocities (float rad/s) became velocity_counts_per_sec (int32_t). The
// field is RENAMED, not just retyped, so a host built against v4 fails at
// COMPILE time rather than silently reinterpreting float bits as int -- the
// PROTOCOL_VERSION check only catches it at runtime, and the two copies of
// this file have no shared build system. See counts_domain_io_plan.md.
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    // counts/sec in the MANUFACTURER count space (5760 counts/rev -- the
    // drive's hardcoded 1440 CPR x4 assumption), NOT the real encoder's count
    // space that StateMessage reports back. USM.lf converts this to duty cycle
    // by a single divide by PWM_COUNTS_PER_SEC_MAX (24000 = 100% duty).
    // The host owns the real-encoder <-> manufacturer count mapping.
    int32_t velocity_counts_per_sec[DOF_NUMBER];
    int time_stamp;
    int message_index;
} CommandMessage;
#pragma pack(pop)

void construct_command_message(
    CommandMessage* msg,
    int behavior_mode,
    const int32_t* velocity_counts_per_sec,
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
// Encoder diagnostics
// =====================
// Per-channel health straight from the FPGA's own qdec blocks, carried inside
// StateMessage so the host can tell whether the counts in the same message are
// trustworthy. Layout mirrors read_fpga_example.c sections 4/6 (the
// authoritative protocol description) -- see that file before changing this.
//
// IMPORTANT, the reject counters SATURATE AT 4 BITS. The FPGA packs both into
// one wire byte (bits [7:4] illegal, bits [3:0] rate), so their range is 0-15
// and a value of 15 means "15 OR MORE events", NOT exactly 15. They are
// qualitative "is this channel unhappy" signals -- "pinned at max" is the
// meaningful reading, not the absolute number. Do not compute rates from them
// as if they were exact counts.
#pragma pack(push, 1)
typedef struct {
    // Frame-level link health, counted since boot (free-running, wraps).
    // Watch the DELTA between consecutive StateMessages: a nonzero packets_bad
    // delta means this cycle's per-channel fields below may be STALE (a frame
    // that fails magic/CRC leaves the previous parse in place rather than
    // zeroing), so interpret them against these two first.
    uint32_t encoder_packets_ok;
    uint32_t encoder_packets_bad;

    // status bits, per channel: bit0 = index correction applied this index
    // pulse, bit1 = large discrepancy between index-implied and tracked
    // position, bit2 = index pulse seen. Bits 3-7 always 0.
    uint8_t motor_status[DOF_NUMBER];
    uint8_t sea_status[DOF_NUMBER];

    // Saturating 0-15 reject counters (see the warning above).
    uint8_t motor_illegal_transition_rejects[DOF_NUMBER];
    uint8_t motor_rate_limit_rejects[DOF_NUMBER];
    uint8_t sea_illegal_transition_rejects[DOF_NUMBER];
    uint8_t sea_rate_limit_rejects[DOF_NUMBER];

    // FIRMWARE-side, not from the FPGA: Encoder.lf's glitch filter
    // (filter_channel_update) rejects readings that imply an impossible speed
    // and holds the previous value instead. 0 = the position/velocity in this
    // message is fresh for that channel; N > 0 = it has been HELD for N
    // consecutive cycles and is not a new measurement. Essential context for
    // reading the counts -- without it a held value is indistinguishable from a
    // genuinely stationary joint.
    uint8_t motor_reject_streak[DOF_NUMBER];
    uint8_t sea_reject_streak[DOF_NUMBER];
} EncoderDiagnostics;
#pragma pack(pop)

// Status bit masks for the *_status fields above.
#define ENCODER_STATUS_INDEX_CORRECTION_APPLIED 0x01
#define ENCODER_STATUS_LARGE_DISCREPANCY        0x02
#define ENCODER_STATUS_INDEX_SEEN               0x04

// Value at which the 4-bit reject counters pin. Reading one AT this value means
// "this many or more", so treat it as a saturation flag, not a count.
#define ENCODER_REJECT_COUNTER_SATURATED 15

// =====================
// State Message data structure definitions and helper functions
// =====================
// STM32 -> host. position_counts/velocity_counts_per_sec/sea_position_counts
// are real measured motor-space feedback. Trimmed (protocol v4): the old sea_velocities,
// commanded_motor_velocity, and running_single_pulse_command fields were
// dropped along with the pulse controllers whose telemetry they carried.
// echoed_time_stamp/echoed_message_index carry back the time_stamp and
// message_index of the last CommandMessage the firmware received, letting
// the host estimate round-trip communication delay and align the command
// and state streams.
//
// v5: positions/velocities/sea_positions carry RAW REAL-ENCODER COUNTS with no
// firmware-side conversion (fields renamed accordingly -- see CommandMessage's
// note on why renaming rather than silently retyping). "Real encoder" means the
// physical encoder on the joint (motor_configs[i]->qdec_cpr counts/rev for USM,
// sea_cpr counts/inch for SEA), which is a DIFFERENT count space from the
// manufacturer counts CommandMessage takes. Counts -> rad is the host's job now.
#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    int32_t position_counts[DOF_NUMBER];            // counts, measured motor position
    float velocity_counts_per_sec[DOF_NUMBER];      // counts/sec, measured motor velocity (fractional: FPGA's Q20.11 estimate)
    int32_t sea_position_counts[DOF_NUMBER];        // counts, measured SEA deflection (linear encoder)
    // Health of the readings above -- read this before trusting them.
    EncoderDiagnostics diagnostics;
    int time_stamp;
    int message_index;
    int echoed_time_stamp; // the last time stamp we received from CommandMessage. Used for communication delay estimation on the host side.
    int echoed_message_index; // the last message index we received from CommandMessage. Also used for communication delay estimation and alignment.
} StateMessage;
#pragma pack(pop)

void construct_state_message(
    StateMessage* msg,
    int behavior_mode,
    const int32_t* position_counts,
    const float* velocity_counts_per_sec,
    const int32_t* sea_position_counts,
    const EncoderDiagnostics* diagnostics, // NULL zeroes the block
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

// Prints only the channels reporting a fault or a held reading (a healthy block
// prints one "all clear" line). Called automatically by both print_state_message
// variants above.
void print_encoder_diagnostics(const EncoderDiagnostics *d);

// Both truncate safely if `size` is too small (see csv_append in stm_comms.c),
// but truncation silently loses columns -- size callers' buffers with these.
// The diagnostics block added ~58 columns, which pushed the header well past
// the 512-byte buffers that used to hold it.
#define STATE_MESSAGE_CSV_HEADER_BUFFER_SIZE 4096
#define STATE_MESSAGE_CSV_ROW_BUFFER_SIZE 1024

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
