#include "stm_comms.h"
#include "linux_comms.h"
#include <termios.h>


// --- Serial Port Setup ---
int open_serial(const char *port) {
    //int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    int fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        perror("open_serial");
        return -1;
    }

    return fd;
}

// these settings works better?
int configure_serial_port(int fd) {
    struct termios tty;

    // Read current serial port settings
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
    //tty.c_cc[VTIME] = 5;  // Timeout in deciseconds (0.5s)
    tty.c_cc[VTIME] = 1;  // Timeout in deciseconds (0.5s)

    // Set input and output baud rates
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    // Apply the settings
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        return -1;
    }

    return 0;
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
        //printf("idx: %d", idx);
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
                if (idx >= length + 2) { // length doesn't include start byte and checksum
                    state = 3;
                }
                break;
            case 3: {
                buf[idx++] = byte;
                uint8_t checksum = 0;
                for (int i = 1; i < idx - 1; ++i)
                    //checksum ^= buf[i];
                    checksum = (buf[i] + checksum) % CHECKSUM_MOD;
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

// how does this work in the other code but not here???
int read_packet_bulk(int fd, uint8_t *buf, size_t buf_size) {
    if (!buf || buf_size < 4) {  // PACKET_BYTE_OVERHEAD = 4
        fprintf(stderr, "Invalid buffer\n");
        return -1;
    }

    uint8_t byte;
    int n;

    // Step 1: Read PACKET_START_BYTE
    while (1) {
        if (wait_for_data(fd, SELECT_TIMEOUT_MS) <= 0)
            return -1;

        n = read(fd, &byte, 1);
        if (n == 1 && byte == PACKET_START_BYTE) {
            buf[0] = byte;
            break;
        }
    }

    // Step 2: Read LENGTH byte
    if (wait_for_data(fd, SELECT_TIMEOUT_MS) <= 0)
        return -1;

    n = read(fd, &byte, 1);
    if (n != 1)
        return -1;

    uint8_t length = byte;
    buf[1] = length;

    // Full packet = start(1) + length(1) + length bytes + checksum(1)
    size_t total_packet_size = 2 + length + 1; // = length + 3
    //size_t total_packet_size = 2 + length + 3; // changing this made it work briefly but it's still not correct
    //size_t total_packet_size = 2 + length + 3; // changing this made it work briefly but it's still not correct

    //printf("read_packet_bulk: total_packet_size: %d\n", total_packet_size);

    if (total_packet_size > buf_size) {
        fprintf(stderr, "Packet too large: %zu bytes, buffer is %zu\n", total_packet_size, buf_size);
        return -1;
    }

    // Step 3: Read rest of the packet (type + data + checksum)
    size_t remaining = total_packet_size - 2;
    size_t offset = 2;

    //printf("read_packet_bulk: remaining: %d\n", remaining);
    while (remaining > 0) {
        //printf("read_packet_bulk: remaining in loop: %d\n", remaining);
        if (wait_for_data(fd, SELECT_TIMEOUT_MS) <= 0)
            return -1;

        n = read(fd, &buf[offset], remaining);
        if (n <= 0)
            continue;

        offset += n;
        remaining -= n;
    }

        // Step 4: Compute checksum: sum of buf[1] to buf[total_packet_size - 2]
    uint8_t computed_checksum = 0;
    for (size_t i = 1; i < total_packet_size - 1; i++) {
        computed_checksum = (computed_checksum + buf[i]) % CHECKSUM_MOD;
    }
    //uint8_t checksum = 0;
    //for (int i = 1; i < total_packet_size; ++i) {computed_checksum = (computed_checksum + buf[i]) & 0xFF;}


    uint8_t received_checksum = buf[total_packet_size - 1];

    

    if (computed_checksum != received_checksum) {
        fprintf(stderr, "Checksum mismatch: expected 0x%02X, got 0x%02X\n",
                computed_checksum, received_checksum);

        //printf("Received packet (%zu bytes):\n", total_packet_size);
        //for (size_t i = 0; i < total_packet_size; ++i) {
        //    printf(" %02X", buf[i]);
        //}
        //printf("\n");
        printf("Checksum calculation range: buf[1] to buf[%zu]\n", total_packet_size - 2);
        printf("Calculated checksum: 0x%02X, Received checksum: 0x%02X\n",
                computed_checksum, received_checksum);
        return -1;
    }

    return total_packet_size;
}


/*
typedef struct {
    uint8_t start;
    uint8_t length;
    uint8_t type;
    uint8_t data[UART_BUFFER_SIZE - PACKET_BYTE_OVERHEAD]; // excluding start, len, type, checksum
    uint8_t checksum;
} Packet;
*/
// --- Packet Reader (simple version) -


int send_command_message(int port_id, CommandMessage* msg){

    uint8_t tx_buff[UART_BUFFER_SIZE];
    memset(tx_buff, 0, UART_BUFFER_SIZE);

    uint8_t state_data_buff[COMMAND_MSG_SIZE];
    memset(state_data_buff, 0, COMMAND_MSG_SIZE);

    int msg_len = encode_command_message_to_data_buffer(msg, state_data_buff);

    int pkt_len = build_packet(tx_buff, PKT_TYPE_DATA, state_data_buff, msg_len);
    
    if (pkt_len == -1){
        printf("Error building transmission packet!\r\n");
        return 0;
    }
    else{
        write(port_id, tx_buff, pkt_len);
    }
    return 1;
}

int read_state_message(int port_id, StateMessage* state_msg){
    uint8_t rx_buff[UART_BUFFER_SIZE];
    memset(rx_buff, 0, UART_BUFFER_SIZE);

    //int rx_len = read_packet(port_id, rx_buff, UART_BUFFER_SIZE);
    int rx_len = read_packet_bulk(port_id, rx_buff, UART_BUFFER_SIZE);

    //printf("read_state_message: rx_len: %d\n", rx_len);
    if (0 < rx_len){
        return  (int)handle_state_message_packet(state_msg, rx_buff, rx_len);
    }
    else {
        return 0;
    }
};

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
