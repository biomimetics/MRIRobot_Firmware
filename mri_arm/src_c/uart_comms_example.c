#include "../include_c/stm_comms.h"
#include "../include_c/linux_comms.h"



#define LONG_USLEEP_TIME 10000
int main() {
    int fd = open_serial(SERIAL_PORT);
    int res = configure_serial_port(fd);
    if (fd < 0) return 1;

    printf("About to start reading...\n");
    uint8_t rx_buf[UART_BUFFER_SIZE];
    uint8_t tx_buf[UART_BUFFER_SIZE];

    float pos_fake[DOF_NUMBER];
    float vel_fake[DOF_NUMBER];
    float sea_fake[DOF_NUMBER];
    for (int i = 0; i<DOF_NUMBER; i++){
        pos_fake[i] = 0.0;
        vel_fake[i] = 0.0;
        sea_fake[i] = 0.0;
    }

    float extra_fake[EXTRA_LENGTH];
    for (int i = 0; i<EXTRA_LENGTH; i++) extra_fake[i] = ((float)i) * ((float) i) / 1000.0;

    
    printf("sizeof(StateMessage): %d\n", sizeof(StateMessage));
    
    int message_index = 0;
    for (int i = 0; i<1000; i++){

        StateMessage state_msg;
        int res = read_state_message(fd, &state_msg);
        print_state_message_int(&state_msg);

        // --- Send a STM_State Packet
        CommandMessage transmit_data;

        construct_command_message(&transmit_data, 2,
                            pos_fake, vel_fake,
                            sea_fake, extra_fake,
                            0, message_index++ % 256);

        print_command_message_int(&transmit_data);

        send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);

        usleep(LONG_USLEEP_TIME);
    }

    /**/
    // --- Send a STM_State Packet
        CommandMessage transmit_data;

        construct_command_message(&transmit_data, 0,
                            pos_fake, vel_fake,
                            sea_fake, extra_fake,
                            0, message_index++ % 256);

        print_command_message_int(&transmit_data);



    printf("Exiting!\r\n");
    close(fd);
    return 0;
}