#include "../include_c/stm_comms.h"
#include "../include_c/linux_comms.h"

#define LONG_USLEEP_TIME 7000
int main() {
    int fd = open_serial(SERIAL_PORT);
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

    StateMessage state_msg;
    printf("sizeof(StateMessage): %d\n", sizeof(StateMessage));
    int message_index = 0;
    for (int i = 0; i<1000; i++){

        // receive a state message packet from the stm32    
        //printf("Before read_packet\n");
        int rx_len = read_packet(fd, rx_buf, UART_BUFFER_SIZE);
        
        //printf("STATE_MSG_SIZE: %d", STATE_MSG_SIZE);
        //printf("PACKET_BYTE_OVERHEAD: %d", PACKET_BYTE_OVERHEAD);
        //printf("Before handle_state_message_packet, got rx_len of %d, expecting packet size of %d\n", rx_len, STATE_MSG_SIZE + PACKET_BYTE_OVERHEAD);
        bool result = handle_state_message_packet(&state_msg, rx_buf, rx_len);
        if (result){
            printf("Received state message:\n");
            print_state_message_int(&state_msg);
        }
        else{
            printf("Failed to read state message!\n");
        }

        // --- Send a STM_State Packet
        CommandMessage transmit_data;

        construct_command_message(&transmit_data, 2,
                            pos_fake, vel_fake,
                            sea_fake, extra_fake,
                            0, message_index++ % 256);

        print_command_message_int(&transmit_data);
        // send a command message packet to the stm32
        uint8_t state_data_buff[200];
        //printf("Before encode_command_message_to_data_buffer\n");
        int msg_len = encode_command_message_to_data_buffer(&transmit_data, state_data_buff);
        //printf("Before build_packet\n");
        int pkt_len = build_packet(tx_buf, PKT_TYPE_DATA, state_data_buff, msg_len);
        //printf("After build_packet with pkt_len: %d", pkt_len);

        if (pkt_len == -1){
            printf("Error building transmission packet!\r\n");
        }
        else{
            write(fd, tx_buf, pkt_len);
        }
        
        usleep(LONG_USLEEP_TIME);
    }

    // --- Send a STM_State Packet
        CommandMessage transmit_data;

        construct_command_message(&transmit_data, 0,
                            pos_fake, vel_fake,
                            sea_fake, extra_fake,
                            0, message_index++ % 256);

        print_command_message_int(&transmit_data);
        // send a command message packet to the stm32
        uint8_t state_data_buff[200];
        //printf("Before encode_command_message_to_data_buffer\n");
        int msg_len = encode_command_message_to_data_buffer(&transmit_data, state_data_buff);
        //printf("Before build_packet\n");
        int pkt_len = build_packet(tx_buf, PKT_TYPE_DATA, state_data_buff, msg_len);
        //printf("After build_packet with pkt_len: %d", pkt_len);

        if (pkt_len == -1){
            printf("Error building transmission packet!\r\n");
        }
        else{
            write(fd, tx_buf, pkt_len);
        }

    printf("Exiting!\r\n");
    close(fd);
    return 0;
}