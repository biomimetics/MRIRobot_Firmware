#include "stm_comms.h"
#include "linux_comms.h"



#define LONG_USLEEP_TIME 8000
int main() {
    int fd = open_serial(SERIAL_PORT);
    int res = configure_serial_port(fd);
    if (fd < 0) return 1;

    printf("About to start reading...\n");
    uint8_t rx_buf[UART_BUFFER_SIZE];
    uint8_t tx_buf[UART_BUFFER_SIZE];

    float vel_fake[DOF_NUMBER];
    float position_deltas_fake[DOF_NUMBER];
    for (int i = 0; i<DOF_NUMBER; i++){
        vel_fake[i] = 0.0;
        position_deltas_fake[i] = 0.0;
    }


    //printf("sizeof(StateMessage): %d\n", sizeof(StateMessage));
    
    int message_index = 0;

    for (int i = 0; i<1000; i++){
        /*
        int n = read(fd, &rx_buf, UART_BUFFER_SIZE);
        printf("Read %d bytes, expected %d.\n", n, UART_BUFFER_SIZE);

        print_buffer(rx_buf, n);

        StateMessage state_msg;

        
        int res = handle_state_message_packet(&state_msg, rx_buf, n);
        if (res){
            print_state_message_int(&state_msg);
        }
        else{
            printf("failed to parse it, res was %d\n", res);
        }
        */

        

        
        //printf("sizeof(Int): %zu\n", sizeof(int)); // just making sure I wasn't going crazy
        StateMessage state_msg;
        int res = read_state_message(fd, &state_msg);
        print_state_message_int(&state_msg);

        // --- Send a STM_State Packet
        CommandMessage transmit_data;

        construct_command_message(&transmit_data, 1,
                            vel_fake, position_deltas_fake,
                            0, message_index++ % 256);

        print_command_message_int(&transmit_data);

        send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);
        
        usleep(LONG_USLEEP_TIME);
    }

    /**/
    // --- Send a STM_State Packet
    CommandMessage transmit_data;

    construct_command_message(&transmit_data, 0,
                        vel_fake, position_deltas_fake,
                        0, message_index++ % 256);

    print_command_message_int(&transmit_data);

    send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);



    printf("Exiting!\r\n");
    close(fd);
    return 0;
}