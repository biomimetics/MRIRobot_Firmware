#include "stm_comms.h"

// =====================
// Packet framing + CRC
// =====================

void print_buffer(uint8_t *buffer, uint16_t length) {
    printf("Received %d bytes:\n", length);
    for (uint16_t i = 0; i < length; i++) {
        printf("0x%02X ", buffer[i]);
    }
    printf("\n");
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t) data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (uint16_t) ((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t) (crc << 1);
            }
        }
    }
    return crc;
}

int build_packet(uint8_t *out_buf, PacketType type, const uint8_t *data, uint8_t data_len) {
    if ((size_t) data_len + PACKET_OVERHEAD > UART_BUFFER_SIZE) return -1;

    int idx = 0;
    out_buf[idx++] = PACKET_START_BYTE;
    out_buf[idx++] = (uint8_t) (data_len + PACKET_TYPE_VERSION_SIZE); // TYPE + VERSION + DATA
    out_buf[idx++] = (uint8_t) type;
    out_buf[idx++] = PROTOCOL_VERSION;

    for (int i = 0; i < data_len; i++) {
        out_buf[idx++] = data[i];
    }

    // CRC covers LENGTH through the end of DATA (bytes[1..idx-1]) -- same
    // range the old additive checksum covered, just a stronger check.
    uint16_t crc = crc16_ccitt(&out_buf[1], (size_t) (idx - 1));
    out_buf[idx++] = (uint8_t) (crc & 0xFF);        // CRC_LO
    out_buf[idx++] = (uint8_t) ((crc >> 8) & 0xFF); // CRC_HI

    return idx;
}

// =====================
// Command message specific definitions
// =====================

void zero_command_message(CommandMessage* msg) {
    memset(msg, 0, sizeof(CommandMessage));
}

void construct_command_message(CommandMessage* msg, int behavior_mode,
                        const float* velocities, const float* position_deltas,
                        int time_stamp, int message_index) {
    if (!msg) return;

    msg->behavior_mode = behavior_mode;

    memcpy(msg->velocities, velocities, sizeof(float) * DOF_NUMBER);
    memcpy(msg->position_deltas, position_deltas, sizeof(float) * DOF_NUMBER);

    msg->time_stamp = time_stamp;
    msg->message_index = message_index;
}

int encode_command_message_to_data_buffer(const CommandMessage *msg, uint8_t *buffer) {
    memcpy(buffer, msg, sizeof(CommandMessage));
    return (int) sizeof(CommandMessage);
}

bool decode_data_buffer_to_command_message(CommandMessage *msg, const uint8_t *data_buffer, size_t data_buffer_len) {
    if (data_buffer_len != sizeof(CommandMessage)) {
        fprintf(stderr, "Unexpected CommandMessage size! Got %u, expected %u\n",
                (unsigned)data_buffer_len, (unsigned)sizeof(CommandMessage));
        return false;
    }

    memcpy(msg, data_buffer, sizeof(CommandMessage));
    return true;
}

bool handle_command_message_packet(CommandMessage* msg, const uint8_t *packet, size_t packet_len) {
    uint8_t type = packet[2];
    uint8_t version = packet[3];
    bool result = false;

    if (version != PROTOCOL_VERSION) {
        fprintf(stderr, "CommandMessage packet protocol version mismatch! Got %d, expected %d\n",
                version, PROTOCOL_VERSION);
        return false;
    }

    switch (type) {
        case PKT_TYPE_PING:
            break;

        case PKT_TYPE_DATA: {
            const uint8_t *data_buffer = &packet[4]; // after start, length, type, version
            size_t data_len = packet_len - PACKET_OVERHEAD;
            result = decode_data_buffer_to_command_message(msg, data_buffer, data_len);
            break;
        }
    }
    return result;
}

void print_command_message(const CommandMessage *msg) {
    printf("Command Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);
    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->velocities[i]);
    printf("\n");

    printf("  Position Deltas: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->position_deltas[i]);
    printf("\n");

    printf("  Position Offset: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->position_offset[i]);
    printf(" (sequence %d)\n", msg->position_offset_sequence);

    printf("  Timestamp: %d\n", msg->time_stamp);
    printf("  Index: %d\n", msg->message_index);
}

void print_command_message_int(const CommandMessage *msg) {
    printf("Command Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);
    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->velocities[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Position Deltas: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->position_deltas[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Position Offset: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->position_offset[i] * FLOAT_PRINT_SCALE));
    printf(" (sequence %d)\n", msg->position_offset_sequence);

    printf("  Timestamp: %d\n", msg->time_stamp);
    printf("  Index: %d\n", msg->message_index);
}

// =====================
// State message specific definitions
// =====================

void zero_state_message(StateMessage* msg) {
    memset(msg, 0, sizeof(StateMessage));
}

void construct_state_message(StateMessage* msg, int behavior_mode,
                            const float* positions, const float* velocities,
                            const float* sea_positions, const float* sea_velocities,
                            const float* commanded_motor_velocity,
                            int time_stamp, int message_index) {
    if (!msg) return;

    msg->behavior_mode = behavior_mode;

    memcpy(msg->positions, positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->velocities, velocities, sizeof(float) * DOF_NUMBER);
    memcpy(msg->sea_positions, sea_positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->sea_velocities, sea_velocities, sizeof(float) * DOF_NUMBER);
    memcpy(msg->commanded_motor_velocity, commanded_motor_velocity, sizeof(float) * DOF_NUMBER);

    msg->time_stamp = time_stamp;
    msg->message_index = message_index;
}

int encode_state_message_to_data_buffer(const StateMessage *msg, uint8_t *buffer) {
    memcpy(buffer, msg, sizeof(StateMessage));
    return (int) sizeof(StateMessage);
}

bool decode_data_buffer_to_state_message(StateMessage *msg, const uint8_t *data_buffer, size_t data_buffer_len) {
    if (data_buffer_len != sizeof(StateMessage)) {
        fprintf(stderr, "Unexpected StateMessage size! Got %zu, expected %zu\n",
                data_buffer_len, sizeof(StateMessage));
        return false;
    }

    memcpy(msg, data_buffer, sizeof(StateMessage));
    return true;
}

bool handle_state_message_packet(StateMessage* msg, const uint8_t *packet, size_t len) {
    uint8_t type = packet[2];
    uint8_t version = packet[3];
    bool result = false;

    if (version != PROTOCOL_VERSION) {
        fprintf(stderr, "StateMessage packet protocol version mismatch! Got %d, expected %d\n",
                version, PROTOCOL_VERSION);
        return false;
    }

    switch (type) {
        case PKT_TYPE_PING:
            break;

        case PKT_TYPE_DATA: {
            const uint8_t *data_buffer = &packet[4]; // after start, length, type, version
            size_t data_len = len - PACKET_OVERHEAD;
            result = decode_data_buffer_to_state_message(msg, data_buffer, data_len);
            break;
        }
    }
    return result;
}

void print_state_message(const StateMessage *msg) {
    printf("State Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);

    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->positions[i]);
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->velocities[i]);
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->sea_positions[i]);
    printf("\n");

    printf("  SEA Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->sea_velocities[i]);
    printf("\n");

    printf("  Commanded Motor Velocity: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->commanded_motor_velocity[i]);
    printf("\n");

    printf("  Timestamp: %d\n", msg->time_stamp);
    printf("  Index: %d\n", msg->message_index);
}

void print_state_message_int(const StateMessage *msg) {
    printf("State Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);

    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->velocities[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->sea_positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  SEA Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->sea_velocities[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Commanded Motor Velocity: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->commanded_motor_velocity[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Timestamp: %d\n", msg->time_stamp);
    printf("  Index: %d\n", msg->message_index);
}

void write_state_message_csv_header(char *buffer, size_t size) {
    int written = 0;

    written += snprintf(buffer + written, size - written, "behavior_mode,");

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "position_%d,", i);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "velocity_%d,", i);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "sea_position_%d,", i);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "sea_velocity_%d,", i);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "commanded_motor_velocity_%d,", i);

    written += snprintf(buffer + written, size - written, "time_stamp,message_index");
}

void serialize_state_message_csv(const StateMessage *msg, char *buffer, size_t size) {
    int written = 0;

    written += snprintf(buffer + written, size - written, "%d,", msg->behavior_mode);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,", FLOAT_DECIMAL_SCALE, msg->positions[i]);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,", FLOAT_DECIMAL_SCALE, msg->velocities[i]);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,", FLOAT_DECIMAL_SCALE, msg->sea_positions[i]);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,", FLOAT_DECIMAL_SCALE, msg->sea_velocities[i]);

    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,", FLOAT_DECIMAL_SCALE, msg->commanded_motor_velocity[i]);

    written += snprintf(buffer + written, size - written, "%d,%d", msg->time_stamp, msg->message_index);
}
