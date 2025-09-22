#include "../include_c/stm_comms.h"
#include "../include_c/linux_comms.h"

int main() {
    int fd = open_serial(SERIAL_PORT);
    if (fd < 0) return 1;

    printf("About to start reading...\n");
    uint8_t rx_buf[UART_BUFFER_SIZE];
    uint8_t tx_buf[UART_BUFFER_SIZE];

    printf("sizeof(StateMessage): %d\n", sizeof(StateMessage));
    for (int i = 0; i<1000; i++){

        // receive a state message packet from the stm32    
        //printf("Before read_packet\n");
        int rx_len = read_packet(fd, rx_buf, UART_BUFFER_SIZE);
        StateMessage state_msg;
        //printf("STATE_MSG_SIZE: %d", STATE_MSG_SIZE);
        //printf("PACKET_BYTE_OVERHEAD: %d", PACKET_BYTE_OVERHEAD);
        //printf("Before handle_state_message_packet, got rx_len of %d, expecting packet size of %d\n", rx_len, STATE_MSG_SIZE + PACKET_BYTE_OVERHEAD);
        bool result = handle_state_message_packet(&state_msg, rx_buf, rx_len);
        if (result){
            printf("Received state message:\n");
            print_state_message(&state_msg);
        }
        else{
            printf("Failed to read state message!\n");
        }
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
        uint8_t state_data_buff[200];
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