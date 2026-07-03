#include "../include_c/uart_comms_lib.h"

#define SELECT_TIMEOUT_MS 100
#define UART_BUF_SIZE 512

#define LONG_USLEEP_TIME 8000
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

    
    //printf("sizeof(StateMessage): %d\n", sizeof(StateMessage));
    
    int message_index = 0;

    while (1) {
        int len = read_packet_cobs_crc16(fd, rx_buf, sizeof(rx_buf), SELECT_TIMEOUT_MS);
        if (len > 0) {
            printf("Received valid packet (%d bytes): ", len);
            for (int i = 0; i < len; i++)
                printf("%02X ", rx_buf[i]);
            printf("\n");


            // Decode application struct here...
            PacketType type;
            TelemetryPacket decodedTelemetry;
            size_t payloadLen;

            int res = decodePacket(encoded, len, &type, &decodedTelemetry, sizeof(decodedTelemetry), &payloadLen);
            if (res == 0 && type == PACKET_TYPE_TELEMETRY)
            {
                printf("Decoded TELEMETRY packet:\n");
                printf("  ID: %u\n", decodedTelemetry.id);
                printf("  Temp: %.2f °C\n", decodedTelemetry.temperature);
                printf("  Volt: %.2f V\n", decodedTelemetry.voltage);
                printf("  Status: 0x%02X\n", decodedTelemetry.status);
            }
            else
            {
                printf("Decode error: %d\n", res);
            }
        }
        else if (len == -2) {
            fprintf(stderr, "CRC error, skipping frame\n");
        }
        else {
            // Timeout or error — could continue
        }
    }

    /*
    for (int i = 0; i<1000; i++){
        
        //printf("sizeof(Int): %zu\n", sizeof(int)); // just making sure I wasn't going crazy
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
    */
    // --- Send a STM_State Packet
        CommandMessage transmit_data;

        construct_command_message(&transmit_data, 0,
                            pos_fake, vel_fake,
                            sea_fake, extra_fake,
                            0, message_index++ % 256);

        print_command_message_int(&transmit_data);

        send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);



    printf("Exiting!\r\n");
    close(fd);
    return 0;
}
