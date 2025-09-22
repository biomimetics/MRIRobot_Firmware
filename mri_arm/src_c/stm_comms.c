#include "../include_c/stm_comms.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>


// =====================
// TX/RX packet defintiions
// =====================

// --- Packet Builder ---
int build_packet(uint8_t *out_buf, PacketType type, uint8_t *data, uint8_t data_len) {
    if (data_len > UART_BUFFER_SIZE - 4) return -1;

    int idx = 0;
    out_buf[idx++] = PACKET_START_BYTE;
    out_buf[idx++] = data_len + 1; // type + data
    out_buf[idx++] = type;

    for (int i = 0; i < data_len; i++) {
        out_buf[idx++] = data[i];
    }

    uint8_t checksum = 0;
    for (int i = 1; i < idx; ++i) {
        checksum ^= out_buf[i];
    }

    out_buf[idx++] = checksum;

    return idx;
}

// --- Packet Reader (simple version) ---
int read_packet(int fd, uint8_t *buf, int max_iterations) {
    int state = 0;
    int idx = 0;
    uint8_t length = 0;

    for (int i = 0; i<max_iterations; i++){
        uint8_t byte;
        int n = read(fd, &byte, 1);
        if (n <= 0) {
            usleep(USLEEP_TIME); // Small delay
            continue;
        }

        //printf("read_packet state: %d, byte: %02X\r\n", state, byte);
        switch (state) {
            case 0:
                if (byte == PACKET_START_BYTE) {
                    buf[0] = byte;
                    idx = 1;
                    state = 1;
                }
                break;
            case 1:
                length = byte;
                buf[idx++] = byte;
                state = 2;
                break;
            case 2:
                buf[idx++] = byte;
                if (idx >= length + 2) {
                    state = 3;
                }
                break;
            case 3: {
                buf[idx++] = byte;
                uint8_t checksum = 0;
                for (int i = 1; i < idx - 1; ++i)
                    checksum ^= buf[i];
                if (checksum == byte) {
                    return idx; // Packet received
                } else {
                    fprintf(stderr, "Invalid checksum\n");
                    state = 0;
                    idx = 0;
                }
                break;
            }
        }
    }
}

// =====================
// Command message specific definitions
// =====================

void construct_command_message(CommandMessage* msg, int behavior_mode,
                        const float* positions, const float* velocities,
                        const float* sea_positions, const float* extra,
                        int time_stamp) {
    if (!msg) return;

    msg->behavior_mode = behavior_mode;

    memcpy(msg->positions, positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->velocities, velocities, sizeof(float) * DOF_NUMBER);
    memcpy(msg->sea_positions, sea_positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->extra, extra, sizeof(float) * EXTRA_LENGTH);

    msg->time_stamp = time_stamp;
}

size_t encode_command_message_to_data_buffer(const CommandMessage *msg, uint8_t *buffer) {
      size_t offset = 0;

      memcpy(buffer + offset, &msg->behavior_mode, sizeof(int));
      offset += sizeof(int);

      memcpy(buffer + offset, msg->positions, sizeof(float) * DOF_NUMBER);
      offset += sizeof(float) * DOF_NUMBER;

      memcpy(buffer + offset, msg->velocities, sizeof(float) * DOF_NUMBER);
      offset += sizeof(float) * DOF_NUMBER;

      memcpy(buffer + offset, msg->sea_positions, sizeof(float) * DOF_NUMBER);
      offset += sizeof(float) * DOF_NUMBER;

      memcpy(buffer + offset, msg->extra, sizeof(float) * EXTRA_LENGTH);
      offset += sizeof(float) * EXTRA_LENGTH;

      buffer[offset++] = msg->time_stamp;

      return offset;
}


bool decode_data_buffer_to_command_message(CommandMessage *msg, uint8_t *data_buffer, size_t data_buffer_len) {
    if (data_buffer_len != sizeof(CommandMessage)) {
        fprintf(stderr, "Unexpected STM_Packet size! Got %zu, expected %zu\n", data_buffer_len, sizeof(CommandMessage));
        return 0;
    }

    memcpy(msg, data_buffer, sizeof(data_buffer_len));
    return 1;
}

bool handle_command_message_packet(CommandMessage* msg, uint8_t *packet, size_t packet_len) {
    uint8_t type = packet[2];
    bool result = false;
    switch (type) {
        case PKT_TYPE_PING:
            //send_packet(PKT_TYPE_PING, NULL, 0); // Echo back
            break;

        case PKT_TYPE_DATA:
            uint8_t *data_buffer = &packet[3]; // After start, len, type
            size_t data_len = packet_len - 4; // Remove start, len, type, checksum
            result = decode_data_buffer_to_command_message(msg, data_buffer, data_len);
            break;
    }
    return result;
}

void print_command_message(const CommandMessage *msg) {
    printf("Command Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", msg->positions[i]);
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", msg->velocities[i]);
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", msg->sea_positions[i]);
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%.2f ", msg->extra[i]);
    printf("\n");

    printf("  Timestamp: %u\n", msg->time_stamp);
}

void print_command_message_int(const CommandMessage *msg) {
    printf("Command Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(msg->positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(msg->velocities[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(msg->sea_positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%d ", (int)(msg->extra[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Timestamp: %u\n", msg->time_stamp);
}


// =====================
// Receive Data specific definitions
// =====================

void construct_state_message(StateMessage* data, int behavior_mode, 
                            const float* positions, const float* velocities,
                            const float* sea_positions, const float* extra,
                            int time_stamp) {
    if (!data) return;

    data->behavior_mode = behavior_mode;

    memcpy(data->positions, positions, sizeof(float) * DOF_NUMBER);
    memcpy(data->velocities, velocities, sizeof(float) * DOF_NUMBER);
    memcpy(data->sea_positions, sea_positions, sizeof(float) * DOF_NUMBER);
    memcpy(data->extra, extra, sizeof(float) * EXTRA_LENGTH);

    data->time_stamp = time_stamp;
}

int encode_state_message_to_data_buffer(const StateMessage *data, uint8_t *buffer) {
      size_t offset = 0;

      memcpy(buffer + offset, &data->behavior_mode, sizeof(int));
      offset += sizeof(int);

      memcpy(buffer + offset, data->positions, sizeof(float) * DOF_NUMBER);
      offset += sizeof(float) * DOF_NUMBER;

      memcpy(buffer + offset, data->velocities, sizeof(float) * DOF_NUMBER);
      offset += sizeof(float) * DOF_NUMBER;

      memcpy(buffer + offset, data->sea_positions, sizeof(float) * DOF_NUMBER);
      offset += sizeof(float) * DOF_NUMBER;

      memcpy(buffer + offset, data->extra, sizeof(float) * EXTRA_LENGTH);
      offset += sizeof(float) * EXTRA_LENGTH;

      buffer[offset++] = sizeof(int); //data->time_stamp;

      return offset;
    }

// --- Parse the incomming packet's data and write it to the state structure
bool decode_data_buffer_to_state_message(StateMessage *msg, uint8_t *data_buffer, size_t data_buffer_len) {
    if (data_buffer_len != sizeof(StateMessage)) {
        fprintf(stderr, "Unexpected StateMessage size! Got %zu, expected %zu\n", data_buffer_len, sizeof(StateMessage));
        return 0;
    }

    memcpy(msg, data_buffer, sizeof(StateMessage));
    return 1;
}

bool handle_state_message_packet(StateMessage* msg, uint8_t *packet, size_t len) {
    uint8_t type = packet[2];

    bool result = false;
    switch (type) {
        case PKT_TYPE_PING:
            //send_packet(PKT_TYPE_PING, NULL, 0); // Echo back
            break;

        case PKT_TYPE_DATA:
            //printf("Got PKT_TYPE_DATA.\n");
            uint8_t *data_buffer = &packet[3]; // After start, len, type
            size_t data_len = len - 4; // Remove start, len, type, checksum
            //printf("Before decode_data_buffer_to_state_message");
            result = decode_data_buffer_to_state_message(msg, data_buffer, data_len);
            break;
    }
    return result;
}

void print_state_message(const StateMessage *msg) {
    printf("State Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", msg->positions[i]);
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", msg->velocities[i]);
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", msg->sea_positions[i]);
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%.2f ", msg->extra[i]);
    printf("\n");

    printf("  Timestamp: %u\n", msg->time_stamp);
}

void print_state_message_int(const StateMessage *msg) {
    printf("State Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(msg->positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(msg->velocities[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(msg->sea_positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%d ", (int)(msg->extra[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Timestamp: %u\n", msg->time_stamp);
}
