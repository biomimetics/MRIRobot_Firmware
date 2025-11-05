#include "stm_comms.h"
#include "linux_comms.h"

#include <math.h>


#define LONG_USLEEP_TIME 8000 //8000
int main() {
    int fd = open_serial(SERIAL_PORT);
    int res = configure_serial_port(fd);
    if (fd < 0) return 1;

    printf("About to start reading...\n");
    uint8_t rx_buf[UART_BUFFER_SIZE];
    uint8_t tx_buf[UART_BUFFER_SIZE];

    float pos_fake[DOF_NUMBER];
    float vel_fake[DOF_NUMBER];
    float vel_cmd[DOF_NUMBER];
    float sea_fake[DOF_NUMBER];
    for (int i = 0; i<DOF_NUMBER; i++){
        pos_fake[i] = 0.0;
        vel_fake[i] = 0.0;
        sea_fake[i] = 0.0;
        vel_cmd[i] = 0.0;
    }

    float extra_fake[EXTRA_LENGTH];
    for (int i = 0; i<EXTRA_LENGTH; i++) extra_fake[i] = ((float)i) * ((float) i) / 1000.0;

    FILE *fpt;
    fpt = fopen("csv_data/uart_log.csv", "w+");
    char header_buffer[512];  // Adjust size as needed
    write_state_message_csv_header(header_buffer, sizeof(header_buffer));
    fprintf(fpt,"%s\n", header_buffer);
    fclose(fpt);

    //printf("sizeof(StateMessage): %d\n", sizeof(StateMessage));
    
    int message_index = 0;

    int iter_num = 500; //5000; //1000;
    int rep_num = 4;
    float iter_float = (float) iter_num;

    float y_offset = 0.0; //1.01; //0.55;
    int last_message_index = 0;
    for (int rep = 0; rep<rep_num; rep++){
        for (int i = 0; i<iter_num; i++){

            vel_cmd[0] = sin(2.0 * 3.1416 * (((float) i) / iter_float)) + y_offset;
            //printf("sizeof(Int): %zu\n", sizeof(int)); // just making sure I wasn't going crazy
            StateMessage state_msg;
            int res = read_state_message(fd, &state_msg);
            //int res = read_packet_bulk(int fd, uint8_t *buf, size_t buf_size);
            //print_state_message_int(&state_msg);

            
            if (state_msg.message_index != last_message_index){ // only save if we get new data
                fpt = fopen("csv_data/uart_log.csv", "a+");
                char buffer[512];  // Make sure this is large enough
                serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                fprintf(fpt,"%s\n", buffer);
                fclose(fpt);
            }
            last_message_index = state_msg.message_index;

            // --- Send a STM_State Packet
            CommandMessage transmit_data;

            construct_command_message(&transmit_data, 1,
                                pos_fake, vel_cmd,
                                sea_fake, extra_fake,
                                0, message_index++ % 256);

            //print_command_message_int(&transmit_data);

            send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);
            
            

            usleep(LONG_USLEEP_TIME);
        }
    }

    iter_num = 1000; //5000; //1000;
    iter_float = (float) iter_num;

    for (int rep = 0; rep<rep_num; rep++){
        for (int i = 0; i<iter_num; i++){

            vel_cmd[0] = sin(2.0 * 3.1416 * (((float) i) / iter_float)) + y_offset;
            //printf("sizeof(Int): %zu\n", sizeof(int)); // just making sure I wasn't going crazy
            StateMessage state_msg;
            int res = read_state_message(fd, &state_msg);
            //int res = read_packet_bulk(int fd, uint8_t *buf, size_t buf_size);
            //print_state_message_int(&state_msg);

            if (state_msg.message_index != last_message_index){ // only save if we get new data
                fpt = fopen("csv_data/uart_log.csv", "a+");
                char buffer[512];  // Make sure this is large enough
                serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                fprintf(fpt,"%s\n", buffer);
                fclose(fpt);
            }
            last_message_index = state_msg.message_index;

            // --- Send a STM_State Packet
            CommandMessage transmit_data;

            construct_command_message(&transmit_data, 1,
                                pos_fake, vel_cmd,
                                sea_fake, extra_fake,
                                0, message_index++ % 256);

            //print_command_message_int(&transmit_data);

            send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);
            
            

            usleep(LONG_USLEEP_TIME);
        }
    }

    iter_num = 2000; //5000; //1000;
    iter_float = (float) iter_num;

    for (int rep = 0; rep<rep_num; rep++){
        for (int i = 0; i<iter_num; i++){

            vel_cmd[0] = sin(2.0 * 3.1416 * (((float) i) / iter_float)) + y_offset;
            //printf("sizeof(Int): %zu\n", sizeof(int)); // just making sure I wasn't going crazy
            StateMessage state_msg;
            int res = read_state_message(fd, &state_msg);
            //int res = read_packet_bulk(int fd, uint8_t *buf, size_t buf_size);
            //print_state_message_int(&state_msg);

            if (state_msg.message_index != last_message_index){ // only save if we get new data
                fpt = fopen("csv_data/uart_log.csv", "a+");
                char buffer[512];  // Make sure this is large enough
                serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                fprintf(fpt,"%s\n", buffer);
                fclose(fpt);
            }
            last_message_index = state_msg.message_index;

            // --- Send a STM_State Packet
            CommandMessage transmit_data;

            construct_command_message(&transmit_data, 1,
                                pos_fake, vel_cmd,
                                sea_fake, extra_fake,
                                0, message_index++ % 256);

            //print_command_message_int(&transmit_data);

            send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);
            
            

            usleep(LONG_USLEEP_TIME);
        }
    }

    iter_num = 5000; //5000; //1000;
    iter_float = (float) iter_num;

    for (int rep = 0; rep<rep_num; rep++){
        for (int i = 0; i<iter_num; i++){

            vel_cmd[0] = sin(2.0 * 3.1416 * (((float) i) / iter_float)) + y_offset;
            //printf("sizeof(Int): %zu\n", sizeof(int)); // just making sure I wasn't going crazy
            StateMessage state_msg;
            int res = read_state_message(fd, &state_msg);
            //int res = read_packet_bulk(int fd, uint8_t *buf, size_t buf_size);
            //print_state_message_int(&state_msg);

            if (state_msg.message_index != last_message_index){ // only save if we get new data
                fpt = fopen("csv_data/uart_log.csv", "a+");
                char buffer[512];  // Make sure this is large enough
                serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                fprintf(fpt,"%s\n", buffer);
                fclose(fpt);
            }
            last_message_index = state_msg.message_index;

            // --- Send a STM_State Packet
            CommandMessage transmit_data;

            construct_command_message(&transmit_data, 1,
                                pos_fake, vel_cmd,
                                sea_fake, extra_fake,
                                0, message_index++ % 256);

            //print_command_message_int(&transmit_data);

            send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);
            
            

            usleep(LONG_USLEEP_TIME);
        }
    }

    /**/
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