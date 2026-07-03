#include "../include_c/stm_comms.h"
#include "../include_c/linux_comms.h"

#define LONG_USLEEP_TIME 1000
int main() {
    int fd = open_serial(SERIAL_PORT);
    int res = configure_serial_port(fd);
    if (fd < 0) return 1;

    FILE *fpt;
    fpt = fopen("uart_log.csv", "w+");
    char header_buffer[512];  // Adjust size as needed
    write_state_message_csv_header(header_buffer, sizeof(header_buffer));
    fprintf(fpt,"%s\n", header_buffer);
    fclose(fpt);

    while (true){

        StateMessage state_msg;
        int res = read_state_message(fd, &state_msg);
        print_state_message_int(&state_msg);

        fpt = fopen("uart_log.csv", "a+");
        char buffer[512];  // Make sure this is large enough
        serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
        fprintf(fpt,"%s\n", buffer);
        fclose(fpt);

        usleep(LONG_USLEEP_TIME);
    }

    printf("Exiting!\r\n");
    close(fd);
    return 0;
}