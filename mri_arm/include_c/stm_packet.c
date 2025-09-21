#include "stm_packet.h"
#include <string.h>
#include <stdio.h>

void construct_stm_packet(
    STM_Packet* packet,
    int behavior_mode,
    float* positions,
    float* velocities,
    float* sea_positions,
    float* extra,
    uint8_t time_stamp
) {
    if (!packet) return;

    packet->behavior_mode = behavior_mode;

    memcpy(packet->positions, positions, sizeof(float) * DOF_NUMBER);
    memcpy(packet->velocities, velocities, sizeof(float) * DOF_NUMBER);
    memcpy(packet->sea_positions, sea_positions, sizeof(float) * DOF_NUMBER);
    memcpy(packet->extra, extra, sizeof(float) * EXTRA_LENGTH);

    packet->time_stamp = time_stamp;

    // Calculate checksum for everything before the checksum field
    packet->checksum = 0; // Zero it first to exclude it from CRC

    // Compute checksum
    packet->checksum = compute_crc16((uint8_t*)packet, sizeof(STM_Packet) - sizeof(uint16_t));
}


uint16_t compute_crc16(uint8_t *data_p, uint16_t length){
    unsigned char x;
    uint16_t crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x <<5)) ^ ((uint16_t)x);
    }
    return crc;
}

void serialize_packet(STM_Packet* packet, char* buffer) {
    char temp[64];
    size_t offset = 0;

    // Behavior Mode
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "m,%d,", packet->behavior_mode);

    // Positions
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "p,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%.*f,", FLOAT_PRECISION, packet->positions[i]);

    // Velocities
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "v,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%.*f,", FLOAT_PRECISION, packet->velocities[i]);

    // SEA Positions
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "s,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%.*f,", FLOAT_PRECISION, packet->sea_positions[i]);

    // Extra
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "e,");
    for (int i = 0; i < EXTRA_LENGTH; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%.*f,", FLOAT_PRECISION, packet->extra[i]);

    // Timestamp
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "/,%d,", packet->time_stamp);

    // Compute CRC
    uint16_t crc = compute_crc16((uint8_t*)buffer, offset);
    snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "?,0x%04X", crc);
}

void serialize_packet_int(STM_Packet* packet, char* buffer) {
    char temp[64];
    size_t offset = 0;

    // Behavior Mode
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "m,%d,", packet->behavior_mode);

    // Positions
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "p,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%d,", (int)(packet->positions[i] * FLOAT_PRINT_SCALE));

    // Velocities
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "v,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%d,", (int)(packet->velocities[i] * FLOAT_PRINT_SCALE));

    // SEA Positions
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "s,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%d,", (int)(packet->sea_positions[i] * FLOAT_PRINT_SCALE));

    // Extra
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "e,");
    for (int i = 0; i < EXTRA_LENGTH; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%d,", (int)(packet->extra[i] * FLOAT_PRINT_SCALE));

    // Timestamp
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "/,%d,", packet->time_stamp);

    // Compute CRC
    uint16_t crc = compute_crc16((uint8_t*)buffer, offset);
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "?,0x%04X", crc);

    // Add tail
    snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "\n");
}

/*
void serialize_packet( STM_Packet* packet, char* buffer) {
    char temp[64];
    size_t offset = 0;

    // Behavior Mode
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "m,%d,", packet->behavior_mode);

    // Positions
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "p,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%f,", packet->positions[i]);

    // Velocities
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "v,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%f,", packet->velocities[i]);

    // SEA Positions
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "s,");
    for (int i = 0; i < DOF_NUMBER; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%f,", packet->sea_positions[i]);

    // Extra
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "e,");
    for (int i = 0; i < EXTRA_LENGTH; ++i)
        offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "%f,", packet->extra[i]);

    // Timestamp
    offset += snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "/,%d,", packet->time_stamp);

    // Compute checksum (excluding itself)
    uint16_t crc = compute_crc16(( uint8_t*)buffer, offset);
    snprintf(buffer + offset, STM_BUFFER_SIZE - offset, "?,0x%04X", crc);
}
*/

bool deserialize_packet(char* buffer, STM_Packet* packet) {
    char buf_copy[STM_BUFFER_SIZE];
    strncpy(buf_copy, buffer, STM_BUFFER_SIZE - 1);
    buf_copy[STM_BUFFER_SIZE - 1] = '\0';

    // Locate and parse checksum
    char* checksum_marker = strstr(buf_copy, "?,");
    if (!checksum_marker) return false;

    uint16_t parsed_crc;
    if (sscanf(checksum_marker, "?,0x%hx", &parsed_crc) != 1) return false;
    *checksum_marker = '\0';  // Terminate buffer at checksum

    // Validate checksum
    uint16_t computed_crc = compute_crc16((uint8_t*)buf_copy, strlen(buf_copy));
    if (parsed_crc != computed_crc) {
        fprintf(stderr, "CRC mismatch: parsed=0x%04X, computed=0x%04X\r\n", parsed_crc, computed_crc);
        return false;
    }

    // Parsing
    char* token = strtok(buf_copy, ",");
    while (token != NULL) {
        if (strcmp(token, "m") == 0) {
            if (!(token = strtok(NULL, ","))) return false;
            packet->behavior_mode = atoi(token);
        } else if (strcmp(token, "p") == 0) {
            for (int i = 0; i < DOF_NUMBER; ++i) {
                token = strtok(NULL, ",");
                if (!token) {
                    fprintf(stderr, "Expected %d position values, got %d\r\n", DOF_NUMBER, i);
                    return false;
                }
                packet->positions[i] = strtof(token, NULL);
            }
        } else if (strcmp(token, "v") == 0) {
            for (int i = 0; i < DOF_NUMBER; ++i) {
                token = strtok(NULL, ",");
                if (!token) {
                    fprintf(stderr, "Expected %d velocity values, got %d\r\n", DOF_NUMBER, i);
                    return false;
                }
                packet->velocities[i] = strtof(token, NULL);
            }
        } else if (strcmp(token, "s") == 0) {
            for (int i = 0; i < DOF_NUMBER; ++i) {
                token = strtok(NULL, ",");
                if (!token) {
                    fprintf(stderr, "Expected %d sea_position values, got %d\r\n", DOF_NUMBER, i);
                    return false;
                }
                packet->sea_positions[i] = strtof(token, NULL);
            }
        } else if (strcmp(token, "e") == 0) {
            for (int i = 0; i < EXTRA_LENGTH; ++i) {
                token = strtok(NULL, ",");
                if (!token) {
                    fprintf(stderr, "Expected %d extra values, got %d\r\n", EXTRA_LENGTH, i);
                    return false;
                }
                packet->extra[i] = strtof(token, NULL);
            }
        } else if (strcmp(token, "/") == 0) {
            if (!(token = strtok(NULL, ","))) return false;
            packet->time_stamp = (uint8_t)atoi(token);
        }

        token = strtok(NULL, ",");
    }

    packet->checksum = parsed_crc;
    return true;
}

void print_packet( STM_Packet *p) {
    printf("STM_Packet:\r\n");
    printf("  Mode: %d\r\n", p->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->positions[i]);
    printf("\r\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->velocities[i]);
    printf("\r\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->sea_positions[i]);
    printf("\r\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%.2f ", p->extra[i]);
    printf("\r\n");

    printf("  Timestamp: %u\r\n", p->time_stamp);
    printf("  CRC16: 0x%04X\r\n", p->checksum);
}


void print_packet_int( STM_Packet *p) {
    printf("STM_Packet:\r\n");
    printf("  Mode: %d\r\n", p->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->positions[i] * FLOAT_PRINT_SCALE));
    printf("\r\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->velocities[i] * FLOAT_PRINT_SCALE));
    printf("\r\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->sea_positions[i] * FLOAT_PRINT_SCALE));
    printf("\r\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%d ", (int)(p->extra[i] * FLOAT_PRINT_SCALE));
    printf("\r\n");

    printf("  Timestamp: %u\r\n", p->time_stamp);
    printf("  CRC16: 0x%04X\r\n", p->checksum);
}