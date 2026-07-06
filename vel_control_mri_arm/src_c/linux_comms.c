#include "stm_comms.h"
#include "linux_comms.h"
#include <termios.h>

static int wait_for_data(int fd, int timeout_ms) {
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int result = select(fd + 1, &read_fds, NULL, NULL, &timeout);
    if (result < 0) {
        perror("select");
    }

    return result; // >0 = ready, 0 = timeout, <0 = error
}

// --- Serial Port Setup ---
int open_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("open_serial");
        return -1;
    }

    return fd;
}

int configure_serial_port(int fd) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        perror("Error from tcgetattr");
        return -1;
    }

    // ========== Input Flags (c_iflag) ==========
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control (XON/XOFF)
    tty.c_iflag &= ~(ICRNL | INLCR);        // Disable CR-to-NL mapping
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INPCK); // Misc

    // ========== Output Flags (c_oflag) ==========
    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)

    // ========== Control Flags (c_cflag) ==========
    tty.c_cflag |= (CLOCAL | CREAD);    // Enable receiver, ignore modem control lines
    tty.c_cflag &= ~CSIZE;              // Clear data size bits
    tty.c_cflag |= CS8;                 // 8 data bits
    tty.c_cflag &= ~PARENB;             // No parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // Disable hardware flow control

    // ========== Local Flags (c_lflag) ==========
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode (non-canonical)

    // ========== Control Characters (c_cc) ==========
    tty.c_cc[VMIN] = 1;   // Minimum number of characters to read
    tty.c_cc[VTIME] = 1;  // Timeout in deciseconds (0.5s)

    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return -1;
    }

    return 0;
}

// --- Packet Reader ---
// Rewritten from scratch, replacing an earlier version whose total-packet-size
// calculation had an unresolved off-by-N bug (the comment above it literally
// read "how does this work in the other code but not here???"). LENGTH's
// meaning is now unambiguous by construction (see stm_comms.h's framing
// comment), so total_packet_size is computed once, correctly, with no guessing.
int read_packet_bulk(int fd, uint8_t *buf, size_t buf_size) {
    if (!buf || buf_size < PACKET_OVERHEAD) {
        fprintf(stderr, "Invalid buffer\n");
        return -1;
    }

    uint8_t byte;
    int n;

    // Step 1: find PACKET_START_BYTE.
    while (1) {
        if (wait_for_data(fd, SELECT_TIMEOUT_MS) <= 0)
            return -1;

        n = read(fd, &byte, 1);
        if (n == 1 && byte == PACKET_START_BYTE) {
            buf[0] = byte;
            break;
        }
    }

    // Step 2: read LENGTH (byte count of TYPE + VERSION + DATA).
    if (wait_for_data(fd, SELECT_TIMEOUT_MS) <= 0)
        return -1;

    n = read(fd, &byte, 1);
    if (n != 1)
        return -1;

    uint8_t length = byte;
    buf[1] = length;

    size_t total_packet_size = PACKET_HEADER_SIZE + (size_t) length + PACKET_CRC_SIZE;

    if (total_packet_size > buf_size) {
        fprintf(stderr, "Packet too large: %zu bytes, buffer is %zu\n", total_packet_size, buf_size);
        return -1;
    }

    // Step 3: read the rest (TYPE + VERSION + DATA + CRC_LO + CRC_HI).
    size_t remaining = total_packet_size - PACKET_HEADER_SIZE;
    size_t offset = PACKET_HEADER_SIZE;

    while (remaining > 0) {
        if (wait_for_data(fd, SELECT_TIMEOUT_MS) <= 0)
            return -1;

        n = read(fd, &buf[offset], remaining);
        if (n <= 0)
            continue;

        offset += (size_t) n;
        remaining -= (size_t) n;
    }

    // Step 4: verify CRC over LENGTH..end-of-DATA (buf[1 .. total_packet_size - PACKET_CRC_SIZE - 1]).
    size_t crc_range_len = total_packet_size - PACKET_CRC_SIZE - 1; // excludes start byte and the 2 crc bytes
    uint16_t computed_crc = crc16_ccitt(&buf[1], crc_range_len);

    uint16_t received_crc = (uint16_t) buf[total_packet_size - 2] |
                             ((uint16_t) buf[total_packet_size - 1] << 8);

    if (computed_crc != received_crc) {
        fprintf(stderr, "CRC mismatch: expected 0x%04X, got 0x%04X\n", computed_crc, received_crc);
        return -1;
    }

    return (int) total_packet_size;
}

int send_command_message(int port_id, CommandMessage* msg) {
    uint8_t tx_buff[UART_BUFFER_SIZE];
    memset(tx_buff, 0, UART_BUFFER_SIZE);

    uint8_t data_buff[COMMAND_MSG_SIZE];
    memset(data_buff, 0, COMMAND_MSG_SIZE);

    int msg_len = encode_command_message_to_data_buffer(msg, data_buff);
    int pkt_len = build_packet(tx_buff, PKT_TYPE_DATA, data_buff, (uint8_t) msg_len);

    if (pkt_len == -1) {
        printf("Error building transmission packet!\r\n");
        return 0;
    }

    write(port_id, tx_buff, pkt_len);
    return 1;
}

int read_state_message(int port_id, StateMessage* state_msg) {
    uint8_t rx_buff[UART_BUFFER_SIZE];
    memset(rx_buff, 0, UART_BUFFER_SIZE);

    int rx_len = read_packet_bulk(port_id, rx_buff, UART_BUFFER_SIZE);

    if (rx_len > 0) {
        return (int) handle_state_message_packet(state_msg, rx_buff, (size_t) rx_len);
    }
    return 0;
}
