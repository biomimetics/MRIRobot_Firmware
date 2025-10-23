#include "../src/stm_comms.h"
#include "../src/linux_comms.h"

#define LONG_USLEEP_TIME 8000
int main() {
    int fd = open_serial(SERIAL_PORT);
    int res = configure_serial_port(fd);
    if (fd < 0) return 1;

    while (true){

        StateMessage state_msg;
        int res = read_state_message(fd, &state_msg);
        if (res){
        print_state_message_int(&state_msg);
        }
        else{
            printf("Could not read state message from serial port %s at baurd rate %ld... \n", SERIAL_PORT, BAUDRATE);
        }

        usleep(LONG_USLEEP_TIME);
    }

    printf("Exiting!\r\n");
    close(fd);
    return 0;
}