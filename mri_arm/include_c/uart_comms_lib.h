#ifndef PACKET_H
#define PACKET_H

#include <stddef.h>
#include <stdint.h>

// ============================================================================
// Generic packet format
// ============================================================================
#define UART_MAX_PACKET 96

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE 921600 //B115200

#define DOF_NUMBER 7
#define EXTRA_LENGTH 7 // 1

#define COMMAND_MSG_SIZE (4 + 4 * DOF_NUMBER * 3 + 4 + 4)
#define STATE_MSG_SIZE (4 + 4 * DOF_NUMBER * 3 + 4 * EXTRA_LENGTH * 1 + 4 + 4)

#define PACKET_BYTE_OVERHEAD 3
#define UART_BUFFER_SIZE  (UART_MAX_PACKET + PACKET_BYTE_OVERHEAD)

#define FLOAT_PRINT_SCALE 1000
#define FLOAT_DECIMAL_SCALE 6 //3
#define USLEEP_TIME 100 // used during reads
#define LONG_USLEEP_TIME 1000 // used at end of logging loops

typedef enum {
    PACKET_TYPE_COMMAND = 1,
    PACKET_TYPE_STATE = 2
} PacketType;

typedef struct __attribute__((packed)) {
    uint8_t type;
    uint8_t payload[UART_MAX_PACKET - PACKET_BYTE_OVERHEAD];
    uint16_t crc;
} GenericPacket;

// ============================================================================
// Example user-defined packet payloads
// ============================================================================

#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float motor_positions[DOF_NUMBER]; // expected positions of the motors, used for bounds checking
    float desired_motor_velocities[DOF_NUMBER]; // actual motor commands send to the USM.lf
    float sea_positions[DOF_NUMBER]; // unused?
    int time_stamp;
    int message_index;
} CommandMessage;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    int behavior_mode;
    float motor_positions[DOF_NUMBER];
    float motor_velocities[DOF_NUMBER];
    float sea_positions[DOF_NUMBER];
    float extra[EXTRA_LENGTH];
    int time_stamp;
    int message_index;
} StateMessage;
#pragma pack(pop)

// ============================================================================
// CRC16-IBM (polynomial 0x8005, initial value 0xFFFF)
// ============================================================================
uint16_t crc16(const uint8_t *data, size_t len);

// ============================================================================
// COBS encode/decode
// ============================================================================
size_t cobsEncode(const void *data, size_t length, uint8_t *buffer);
size_t cobsDecode(const uint8_t *buffer, size_t length, void *data);

// ============================================================================
// UART simulation helpers
// ============================================================================
void uart_send_bytes(const uint8_t *data, size_t len);

// ============================================================================
// Generic encode/decode using CRC16
// ============================================================================
size_t encodePacket(PacketType type, const void *payload, size_t payloadLen,
                    uint8_t *encodedBuf);

int decodePacket(const uint8_t *encoded, size_t encodedLen,
                 PacketType *typeOut, void *payloadOut, size_t payloadMax,
                 size_t *payloadLenOut);

// ============================================================================
// Read fucntions
// ============================================================================
static int wait_for_data(int fd, int timeout_ms);

int read_packet_cobs_crc16(int fd, uint8_t *buf, size_t buf_size, int timeout_ms);

#endif // PACKET_H
