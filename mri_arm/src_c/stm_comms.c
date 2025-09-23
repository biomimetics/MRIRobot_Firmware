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
        //checksum ^= out_buf[i]; // xor checksum
        checksum = (out_buf[i] + checksum) % CHECKSUM_MOD;
    }

    out_buf[idx++] = checksum;

    return idx;
}



// =====================
// Command message specific definitions
// =====================

void construct_command_message(CommandMessage* msg, int behavior_mode,
                        const float* positions, const float* velocities,
                        const float* sea_positions, const float* extra,
                        int time_stamp, int message_index) {
    if (!msg) return;

    msg->behavior_mode = behavior_mode;

    memcpy(msg->positions, positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->velocities, velocities, sizeof(float) * DOF_NUMBER);
    memcpy(msg->sea_positions, sea_positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->extra, extra, sizeof(float) * EXTRA_LENGTH);

    msg->time_stamp = time_stamp;
    msg->message_index = message_index;
}

int encode_command_message_to_data_buffer(const CommandMessage *msg, uint8_t *buffer) {
    int offset = 0;

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

    //buffer[offset] = &msg->time_stamp;
    memcpy(buffer + offset, &msg->time_stamp, sizeof(int));
    offset += sizeof(int);

    memcpy(buffer + offset, &msg->message_index, sizeof(int));
    offset += sizeof(int);
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
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->positions[i]);
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->velocities[i]);
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->sea_positions[i]);
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->extra[i]);
    printf("\n");

    printf("  Timestamp: %d\n", msg->time_stamp);

    printf("  Index: %d\n", msg->message_index);
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

    printf("  Timestamp: %d\n", msg->time_stamp);

    printf("  Index: %d\n", msg->message_index);
}


// =====================
// Receive Data specific definitions
// =====================

void construct_state_message(StateMessage* msg, int behavior_mode, 
                            const float* positions, const float* velocities,
                            const float* sea_positions, const float* extra,
                            int time_stamp, int message_index) {
    if (!msg) return;

    msg->behavior_mode = behavior_mode;

    memcpy(msg->positions, positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->velocities, velocities, sizeof(float) * DOF_NUMBER);
    memcpy(msg->sea_positions, sea_positions, sizeof(float) * DOF_NUMBER);
    memcpy(msg->extra, extra, sizeof(float) * EXTRA_LENGTH);

    msg->time_stamp = time_stamp;
    msg->message_index = message_index;
}

int encode_state_message_to_data_buffer(const StateMessage *msg, uint8_t *buffer) {
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

    //buffer[offset++] = sizeof(int); //data->time_stamp;
    memcpy(buffer + offset, &msg->time_stamp, sizeof(int));
    offset += sizeof(int);

    memcpy(buffer + offset, &msg->message_index, sizeof(int));
    offset += sizeof(int);

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
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->positions[i]);
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->velocities[i]);
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->sea_positions[i]);
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%.*f,",FLOAT_DECIMAL_SCALE, msg->extra[i]);
    printf("\n");

    printf("  Timestamp: %d\n", msg->time_stamp);

    printf("  Index: %d\n", msg->message_index);
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

    printf("  Timestamp: %d\n", msg->time_stamp);

    printf("  Index: %d\n", msg->message_index);
}


void write_state_message_csv_header(char *buffer, size_t size) {
    int written = 0;

    // Write fixed field: behavior_mode
    written += snprintf(buffer + written, size - written, "behavior_mode,");

    // Write position headers
    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "position_%d,", i);

    // Write velocity headers
    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "velocity_%d,", i);

    // Write sea_position headers
    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "sea_position_%d,", i);

    // Write extra headers
    for (int i = 0; i < EXTRA_LENGTH; ++i)
        written += snprintf(buffer + written, size - written, "extra_%d,", i);

    // Write final fixed fields: time_stamp and message_index
    written += snprintf(buffer + written, size - written, "time_stamp,message_index");
}

/**
 * Serializes a StateMessage to a CSV string.
 * 
 * @param msg    Pointer to the StateMessage to serialize
 * @param buffer Character buffer to write to
 * @param size   Size of the buffer
 */
void serialize_state_message_csv(const StateMessage *msg, char *buffer, size_t size) {
    int written = 0;

    // Write fixed fields: behavior_mode
    written += snprintf(buffer + written, size - written, "%d,", msg->behavior_mode);

    // Write positions
    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,",FLOAT_DECIMAL_SCALE, msg->positions[i]);

    // Write velocities
    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,",FLOAT_DECIMAL_SCALE, msg->velocities[i]);

    // Write sea_positions
    for (int i = 0; i < DOF_NUMBER; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,",FLOAT_DECIMAL_SCALE, msg->sea_positions[i]);

    // Write extra values
    for (int i = 0; i < EXTRA_LENGTH; ++i)
        written += snprintf(buffer + written, size - written, "%.*f,",FLOAT_DECIMAL_SCALE, msg->extra[i]);

    // Write timestamp and message index
    written += snprintf(buffer + written, size - written, "%d,%d", msg->time_stamp, msg->message_index);
}