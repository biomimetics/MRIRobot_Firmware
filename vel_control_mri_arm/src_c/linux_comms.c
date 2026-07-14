#include "stm_comms.h"
#include "linux_comms.h"
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

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

    // Ask the USB-serial driver for low-latency delivery. Without this, the
    // FTDI adapter batches RX data (~45ms clumps of 4-5 state packets measured
    // on 2026-07-13 despite the sysfs latency_timer reading 16ms), so the
    // 100Hz control loop saw a fresh packet on only ~23% of cycles and
    // drain-to-newest discarded the rest -- the direct cause of the "stale/
    // quantized ArmStatus telemetry" issue in mri_arm_estimator/docs/TODO.md.
    // ASYNC_LOW_LATENCY makes ftdi_sio set its latency timer to 1ms (verified:
    // arrivals become a clean 10ms cadence, zero clumps). The flag does not
    // persist across replug, so it must be set on every open. Best-effort:
    // non-FTDI adapters may not support TIOCSSERIAL, and a warning beats
    // refusing to run.
    struct serial_struct serial;
    if (ioctl(fd, TIOCGSERIAL, &serial) == 0) {
        serial.flags |= ASYNC_LOW_LATENCY;
        if (ioctl(fd, TIOCSSERIAL, &serial) != 0) {
            perror("configure_serial_port: TIOCSSERIAL (low latency not applied)");
        }
    } else {
        perror("configure_serial_port: TIOCGSERIAL (low latency not applied)");
    }

    return 0;
}

// --- Packet Reader ---
// Parses packets out of a persistent carry buffer instead of consuming bytes
// from the fd one at a time. This exists to fix a resync amplification bug:
// the previous reader consumed START+LENGTH from the fd before validating
// them, so locking onto a false 0xAA inside a packet's payload (payload bytes
// are not stuffed/escaped, so ~1-in-3 packets contain one) permanently
// discarded bytes that included the *real* packet start, cascading a single
// corrupted byte on the wire into multi-second bursts of "CRC mismatch" /
// "Packet too large" while the scanner chased false starts. Here, a rejected
// candidate (implausible LENGTH, unknown TYPE, wrong VERSION, or bad CRC)
// drops exactly ONE byte -- the false START -- and rescans the bytes already
// in hand, so at most one real packet is ever lost per corrupted byte.
//
// Sized to hold a few packets of backlog; must be >= UART_BUFFER_SIZE so an
// aligned, length-valid packet always fits (guaranteed by the LENGTH check
// against buf_size <= RX_CARRY_SIZE).
#define RX_CARRY_SIZE 512

static uint8_t rx_carry[RX_CARRY_SIZE];
static size_t rx_carry_len = 0;

// Drop the first n bytes of the carry buffer (n <= rx_carry_len).
static void carry_drop(size_t n) {
    memmove(rx_carry, rx_carry + n, rx_carry_len - n);
    rx_carry_len -= n;
}

int serial_rx_buffered_bytes(void) {
    return (int) rx_carry_len;
}

void serial_rx_reset(void) {
    rx_carry_len = 0;
}

int read_packet_bulk(int fd, uint8_t *buf, size_t buf_size) {
    if (!buf || buf_size < PACKET_OVERHEAD || buf_size > RX_CARRY_SIZE) {
        fprintf(stderr, "Invalid buffer\n");
        return -1;
    }

    while (1) {
        // Step 1: align the carry buffer to a candidate START byte, discarding
        // leading non-START bytes (those can never begin a packet).
        size_t start = 0;
        while (start < rx_carry_len && rx_carry[start] != PACKET_START_BYTE)
            start++;
        if (start > 0)
            carry_drop(start);

        // Step 2: validate the candidate as its header bytes become available.
        // Each check either accepts, rejects (drop ONE byte, rescan -- the real
        // start may be anywhere in the bytes we already hold), or falls through
        // to read more.
        if (rx_carry_len >= PACKET_HEADER_SIZE) {
            uint8_t length = rx_carry[1];
            size_t total_packet_size = PACKET_HEADER_SIZE + (size_t) length + PACKET_CRC_SIZE;

            if (total_packet_size > buf_size) {
                fprintf(stderr, "Packet too large: %zu bytes, buffer is %zu -- resyncing\n",
                        total_packet_size, buf_size);
                carry_drop(1);
                continue;
            }

            if (rx_carry_len >= PACKET_HEADER_SIZE + PACKET_TYPE_VERSION_SIZE) {
                uint8_t type = rx_carry[2];
                uint8_t version = rx_carry[3];
                if ((type != PKT_TYPE_PING && type != PKT_TYPE_DATA) ||
                    version != PROTOCOL_VERSION) {
                    fprintf(stderr, "Bad packet header (type 0x%02X, version %u) -- resyncing\n",
                            type, version);
                    carry_drop(1);
                    continue;
                }

                if (rx_carry_len >= total_packet_size) {
                    // Step 3: verify CRC over LENGTH..end-of-DATA
                    // (rx_carry[1 .. total_packet_size - PACKET_CRC_SIZE - 1]).
                    size_t crc_range_len = total_packet_size - PACKET_CRC_SIZE - 1;
                    uint16_t computed_crc = crc16_ccitt(&rx_carry[1], crc_range_len);
                    uint16_t received_crc = (uint16_t) rx_carry[total_packet_size - 2] |
                                            ((uint16_t) rx_carry[total_packet_size - 1] << 8);

                    if (computed_crc != received_crc) {
                        fprintf(stderr, "CRC mismatch: expected 0x%04X, got 0x%04X -- resyncing\n",
                                computed_crc, received_crc);
                        carry_drop(1);
                        continue;
                    }

                    memcpy(buf, rx_carry, total_packet_size);
                    carry_drop(total_packet_size);
                    return (int) total_packet_size;
                }
            }
        }

        // Step 4: need more bytes. Batch-read whatever the kernel has (draining
        // it faster than the old one-byte-per-syscall scan, which widened the
        // window for TTY buffer overruns under scheduling jitter). A timeout
        // returns -1 but keeps the partial bytes for the next call.
        if (rx_carry_len == RX_CARRY_SIZE)
            carry_drop(1); // defensive; unreachable given the LENGTH check above

        if (wait_for_data(fd, SELECT_TIMEOUT_MS) <= 0)
            return -1;

        ssize_t n = read(fd, rx_carry + rx_carry_len, RX_CARRY_SIZE - rx_carry_len);
        if (n <= 0) {
            if (n < 0)
                perror("read_packet_bulk: read");
            return -1;
        }
        rx_carry_len += (size_t) n;
    }
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

    ssize_t n_written = write(port_id, tx_buff, pkt_len);
    if (n_written != (ssize_t) pkt_len) {
        if (n_written < 0) {
            perror("send_command_message: write");
        } else {
            fprintf(stderr, "send_command_message: short write (%zd of %d bytes)\n",
                    n_written, pkt_len);
        }
        return 0;
    }
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
