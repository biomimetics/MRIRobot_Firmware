#include "../src/stm_comms.h"
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

    int rx_len = read_packet(port_id, rx_buff, UART_BUFFER_SIZE);

    if (0 < rx_len){
        return  (int)handle_state_message_packet(state_msg, rx_buff, rx_len);
    }
    else {
        return 0;
    }
};