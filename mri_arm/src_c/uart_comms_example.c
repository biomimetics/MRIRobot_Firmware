#include "../include_c/stm_comms.h"

int main() {
    int fd = open_serial(SERIAL_PORT);
    if (fd < 0) return 1;

    printf("About to start reading...\n");
    uint8_t rx_buf[UART_BUFFER_SIZE];
    uint8_t tx_buf[UART_BUFFER_SIZE];


    for (int i = 0; i<1000; i++){

        // receive a state message packet from the stm32    
        int rx_len = read_packet(fd, rx_buf, UART_BUFFER_SIZE);
        StateMessage state_msg;
        handle_state_message_packet(&state_msg, rx_buf, rx_len);
        printf("Received state message:\r\n");
        print_state_message(&state_msg);
        
        // --- Send a STM_State Packet
        CommandMessage transmit_data = {
            .behavior_mode = 0, 
            .positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            .velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            .sea_positions = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f},
            .extra = {1.234},
            .time_stamp = 0
        };

        // send a command message packet to the stm32
        uint8_t state_data_buff[100];
        encode_command_message_to_data_buffer(&transmit_data, state_data_buff);
        int pkt_len = build_packet(tx_buf, PKT_TYPE_DATA, state_data_buff, sizeof(CommandMessage));

        if (pkt_len == -1){
            printf("Error building transmission packet!\r\n");
        }
        else{
            write(fd, tx_buf, pkt_len);
        }
        
        usleep(LONG_USLEEP_TIME);
    }

    printf("Exiting!\r\n");
    close(fd);
    return 0;
}