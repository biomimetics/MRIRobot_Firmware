#include "include_c/stm_comms.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
// =====================
// TX/RX packet defintiions
// =====================

// --- Serial Port Setup ---
int open_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("open_serial");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, set local mode
    options.c_cflag &= ~PARENB;           // No parity
    options.c_cflag &= ~CSTOPB;           // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;               // 8 data bits

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
    options.c_oflag &= ~OPOST;                          // raw output

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

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
// Send Data specific definitions
// =====================

void construct_send_data(Send_Data* data, int behavior_mode,
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

void encode_send_data_to_bytes(const Send_Data *data, uint8_t *buffer) {
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

      buffer[offset++] = data->time_stamp;
}


bool decode_bytes_to_send_data(Send_Data *data, uint8_t *packet, size_t packet_len) {
    if (packet_len != sizeof(Send_Data)) {
        fprintf(stderr, "Unexpected STM_Packet size! Got %zu, expected %zu\n", packet_len, sizeof(Send_Data));
        return 0;
    }

    memcpy(&data, packet, sizeof(packet_len));
    return 1;
}

void handle_send_data_packet(Send_Data* data, uint8_t *packet, uint8_t len) {
    uint8_t type = packet[2];

    switch (type) {
        case PKT_TYPE_PING:
            send_packet(PKT_TYPE_PING, NULL, 0); // Echo back
            break;

        case PKT_TYPE_DATA:
            uint8_t *data = &packet[3]; // After start, len, type
            size_t data_len = len - 4; // Remove start, len, type, checksum
            decode_bytes_to_send_data(data, packet, data_len);
            break;
    }
}

void print_send_packet(const Send_Data *p) {
    printf("STM_Packet:\n");
    printf("  Mode: %d\n", p->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->positions[i]);
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->velocities[i]);
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->sea_positions[i]);
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%.2f ", p->extra[i]);
    printf("\n");

    printf("  Timestamp: %u\n", p->time_stamp);
}

void print_send_packet_int(const Send_Data *p) {
    printf("STM_Packet:\n");
    printf("  Mode: %d\n", p->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->velocities[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->sea_positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%d ", (int)(p->extra[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Timestamp: %u\n", p->time_stamp);
}


// =====================
// Receive Data specific definitions
// =====================

void construct_receive_data(Receive_Data* data, int behavior_mode, 
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

int encode_receive_data_to_bytes(const Receive_Data *data, uint8_t *buffer) {
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
bool decode_bytes_to_receive_data(Receive_Data *data, uint8_t *packet, size_t packet_len) {
    if (packet_len != sizeof(Receive_Data)) {
        fprintf(stderr, "Unexpected Receive_Data size! Got %zu, expected %zu\n", packet_len, sizeof(Receive_Data));
        return 0;
    }

    memcpy(&data, packet, sizeof(Receive_Data));
    return 1;
}

void handle_receive_data_packet(Receive_Data* data, uint8_t *packet, uint8_t len) {
    uint8_t type = packet[2];

    switch (type) {
        case PKT_TYPE_PING:
            send_packet(PKT_TYPE_PING, NULL, 0); // Echo back
            break;

        case PKT_TYPE_DATA:
            uint8_t *data = &packet[3]; // After start, len, type
            size_t data_len = len - 4; // Remove start, len, type, checksum
            decode_bytes_to_receive_data(data, packet, data_len);
            break;
    }
}

void print_receive_packet(const Receive_Data *p) {
    printf("Recieve Packet:\n");
    printf("  Mode: %d\n", p->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->positions[i]);
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->velocities[i]);
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.2f ", p->sea_positions[i]);
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%.2f ", p->extra[i]);
    printf("\n");

    printf("  Timestamp: %u\n", p->time_stamp);
}

void print_receive_packet_int(const Receive_Data *p) {
    printf("STM_Packet:\n");
    printf("  Mode: %d\n", p->behavior_mode);
    printf("  Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Velocities: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->velocities[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  SEA Positions: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int)(p->sea_positions[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Extra Values: ");
    for (int i = 0; i < EXTRA_LENGTH; ++i) printf("%d ", (int)(p->extra[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  Timestamp: %u\n", p->time_stamp);
}
