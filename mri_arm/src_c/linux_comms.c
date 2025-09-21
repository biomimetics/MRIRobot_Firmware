#include <termios.h>
#include "../include_c/stm_comms.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

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

