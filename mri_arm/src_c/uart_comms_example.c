#include "c_include/stm_comms.h"

int main() {
    int fd = open_serial(SERIAL_PORT);
    if (fd < 0) return 1;

    printf("About to start reading...\n");
    uint8_t rx_buf[UART_BUFFER_SIZE];
    uint8_t tx_buf[UART_BUFFER_SIZE];

    for (int i = 0; i<1000; i++){

        
        int rx_len = read_packet(fd, rx_buf, UART_BUFFER_SIZE);

        
        if (rx_len > 0) {
            printf("Received packet (%d bytes): ", rx_len);
            //for (int i = 0; i < rx_len; ++i) {
            //    printf("%02X ", rx_buf[i]);
            //    //printf("%s ", rx_buf[i]);
            //}
            //printf("\n");

            uint8_t type = rx_buf[2];
            if (type == PKT_TYPE_PING) {
                printf("Received PING reply from device.\n");
            }

            if (type == PKT_TYPE_DATA) {
                uint8_t *data = &rx_buf[3]; // After start, len, type
                size_t data_len = rx_len - 4; // Remove start, len, type, checksum

                parse_packet_data(data, data_len);
            }

        }


        int timestamp_ms = 100; // placeholder

        // --- Send a STM_State Packet
        Send_Data transmit_data = {
            .behavior_mode = 0, 
            .positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            .velocities = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
            .sea_positions = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f},
            .extra = {1.234},
            .time_stamp = timestamp_ms
        };

        //PacketType type, uint8_t *data, uint8_t len
        uint8_t state_data_buff[100];
        encode_packet_to_bytes(&transmit_data, state_data_buff);
        int pkt_len = build_packet(tx_buf, PKT_TYPE_DATA, state_data_buff, sizeof(Send_Data));

        if (pkt_len == -1){
            printf("Error building transmission packet!\r\n");
        }
        else{
            write(fd, tx_buf, pkt_len);
        }
        
        usleep(LONG_USLEEP_TIME);

        struct timespec remaining, request = { 5, 100 };

        printf("Taking a nap...\n");
        int response = nanosleep(&request, &remaining);
    }

    printf("Exiting!\r\n");
    close(fd);
    return 0;
}