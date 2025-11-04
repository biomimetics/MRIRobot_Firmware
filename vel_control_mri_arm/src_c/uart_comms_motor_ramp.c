#include "stm_comms.h"
#include "linux_comms.h"

#include <math.h>

#define ROT_PER_MIN_TO_RAD_PER_SECOND 0.10472
#define LONG_USLEEP_TIME 8000 //8000
#define USE_HARDWARE 1

float _lerp(float lb, float ub, float percent){
    return (ub - lb) * percent + lb;
};

float lerp(float lb, float ub, float percent){
    return (ub * percent) +  (lb * (1.0 - percent));
};


int main() {

    // settings
    float rpm_velocity_setpoint_max = 32.0f;
    const int rpm_velocity_setpoint_num = 9;

    int hold_iters = 3000; //3000;
    int ramp_iters = 1000; // 1000;

    // init
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
    fpt = fopen("csv_data/uart_ramp.csv", "w+");
    char header_buffer[512];  // Adjust size as needed
    write_state_message_csv_header(header_buffer, sizeof(header_buffer));
    fprintf(fpt,"%s\n", header_buffer);
    fclose(fpt);


    // compute the rpm velocity sequence
    float rpm_velocity_setpoints[rpm_velocity_setpoint_num];// = {0.0f, 4.0f, 8.0f, 12.0f, 16.0f, 20.0f, 24.0f, 28.0f, 32.0f};
    float rpm_velocity_setpoint_step = rpm_velocity_setpoint_max / ((float) rpm_velocity_setpoint_num - 1.0f);
    float rad_velocity_setpoints[rpm_velocity_setpoint_num];
    printf("rpm_velocity_setpoint_step: %f\n", rpm_velocity_setpoint_step);
    for (int i = 0; i<rpm_velocity_setpoint_num; i++){
        rpm_velocity_setpoints[i] = rpm_velocity_setpoint_step * ((float) i);
        printf("rpm_velocity_setpoints[%d]: %f\n", i, rpm_velocity_setpoint_step * ((float) i));
        rad_velocity_setpoints[i] = rpm_velocity_setpoints[i] * ROT_PER_MIN_TO_RAD_PER_SECOND;
    }

    

    int message_index = 0;
    int last_message_index = 0;


    for (int i = 1; i<rpm_velocity_setpoint_num; i++){
        printf("Going up! step %d, holding speed of %f.\n", i, rpm_velocity_setpoints[i-1]);
        printf("Holding at speed of %f...\n", rad_velocity_setpoints[i-1]);
        for (int j = 0; j<hold_iters; j++){

            // --- Send a CommandMessage Packet
            CommandMessage transmit_data;
            vel_cmd[0] = rad_velocity_setpoints[i-1];
            if (!USE_HARDWARE) printf("(%d, %d) vel_cmd[0]: %f\n", i, j, vel_cmd[0]);

            if (USE_HARDWARE){
                construct_command_message(&transmit_data, 1,
                                    pos_fake, vel_cmd,
                                    sea_fake, extra_fake,
                                    0, message_index++ % 256);

                send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);

                StateMessage state_msg;
                int res = read_state_message(fd, &state_msg);
                
                if (state_msg.message_index != last_message_index){ // only save if we get new data
                    fpt = fopen("csv_data/uart_ramp.csv", "a+");
                    char buffer[512];  // Make sure this is large enough
                    serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                    fprintf(fpt,"%s\n", buffer);
                    fclose(fpt);
                }
                last_message_index = state_msg.message_index;
                usleep(LONG_USLEEP_TIME);
            }
        }

        printf("End of hold, ramping to speed of %f...\n", rad_velocity_setpoints[i]);
        for (int j = 0; j<ramp_iters; j++){
            // --- Send a CommandMessage Packet
            CommandMessage transmit_data;

            float percent = ((float) j) / ((float) ramp_iters);
            vel_cmd[0] = lerp(rad_velocity_setpoints[i-1], rad_velocity_setpoints[i], percent);
            if (!USE_HARDWARE) printf("(%d, %d) vel_cmd[0]: %f\n", i, j, vel_cmd[0]);
            if (USE_HARDWARE){
                construct_command_message(&transmit_data, 1,
                                    pos_fake, vel_cmd,
                                    sea_fake, extra_fake,
                                    0, message_index++ % 256);

                send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);

                StateMessage state_msg;
                int res = read_state_message(fd, &state_msg);
                
                if (state_msg.message_index != last_message_index){ // only save if we get new data
                    fpt = fopen("csv_data/uart_ramp.csv", "a+");
                    char buffer[512];  // Make sure this is large enough
                    serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                    fprintf(fpt,"%s\n", buffer);
                    fclose(fpt);
                }
                last_message_index = state_msg.message_index;
                usleep(LONG_USLEEP_TIME);
            }
        }
    } 

    for (int j = 0; j<hold_iters; j++){
        //printf("Hold at the top: %f...\n", rad_velocity_setpoints[rpm_velocity_setpoint_num-1]);
        // --- Send a CommandMessage Packet
        CommandMessage transmit_data;
        vel_cmd[0] = rad_velocity_setpoints[rpm_velocity_setpoint_num-1];
        
        if (USE_HARDWARE){
            construct_command_message(&transmit_data, 1,
                                pos_fake, vel_cmd,
                                sea_fake, extra_fake,
                                0, message_index++ % 256);

            send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);

            StateMessage state_msg;
            int res = read_state_message(fd, &state_msg);
            
            if (state_msg.message_index != last_message_index){ // only save if we get new data
                fpt = fopen("csv_data/uart_ramp.csv", "a+");
                char buffer[512];  // Make sure this is large enough
                serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                fprintf(fpt,"%s\n", buffer);
                fclose(fpt);
            }
            last_message_index = state_msg.message_index;
            usleep(LONG_USLEEP_TIME);
        }
    }
    
    for (int i = rpm_velocity_setpoint_num - 1; 0 < i; i--){
        printf("Going down! step %d, holding speed of %f.\n", i, rpm_velocity_setpoints[i]);

        printf("Holding at speed of %f...\n", rad_velocity_setpoints[i]);
        for (int j = 0; j<hold_iters; j++){
            
            // --- Send a CommandMessage Packet
            CommandMessage transmit_data;
            vel_cmd[0] = rad_velocity_setpoints[i];
            if (!USE_HARDWARE) printf("(%d, %d) vel_cmd[0]: %f\n", i, j, vel_cmd[0]);
            if (USE_HARDWARE){
                construct_command_message(&transmit_data, 1,
                                    pos_fake, vel_cmd,
                                    sea_fake, extra_fake,
                                    0, message_index++ % 256);

                send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);

                StateMessage state_msg;
                int res = read_state_message(fd, &state_msg);
                
                if (state_msg.message_index != last_message_index){ // only save if we get new data
                    fpt = fopen("csv_data/uart_ramp.csv", "a+");
                    char buffer[512];  // Make sure this is large enough
                    serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                    fprintf(fpt,"%s\n", buffer);
                    fclose(fpt);
                }
                last_message_index = state_msg.message_index;
                usleep(LONG_USLEEP_TIME);
            }
        }

        printf("Ramping to speed of %f...\n", rad_velocity_setpoints[i-1]);
        for (int j = 0; j<ramp_iters; j++){
            // --- Send a CommandMessage Packet
            CommandMessage transmit_data;

            float percent = ((float) j) / ((float) ramp_iters);
            //printf("rad_velocity_setpoints[i] %f,  rad_velocity_setpoints[i-1]: %f, percent: %f\n", rad_velocity_setpoints[i], rad_velocity_setpoints[i-1], percent);
            vel_cmd[0] = (rad_velocity_setpoints[i-1] - rad_velocity_setpoints[i]) * percent + rad_velocity_setpoints[i]; //lerp(rad_velocity_setpoints[i], rad_velocity_setpoints[i-1], percent);


            if (!USE_HARDWARE) printf("(%d, %d) vel_cmd[0]: %f\n", i, j, vel_cmd[0]);
            if (USE_HARDWARE){
                construct_command_message(&transmit_data, 1,
                                    pos_fake, vel_cmd,
                                    sea_fake, extra_fake,
                                    0, message_index++ % 256);

                send_command_message(fd, &transmit_data);//, state_data_buff, tx_buf);

                StateMessage state_msg;
                int res = read_state_message(fd, &state_msg);
                
                if (state_msg.message_index != last_message_index){ // only save if we get new data
                    fpt = fopen("csv_data/uart_ramp.csv", "a+");
                    char buffer[512];  // Make sure this is large enough
                    serialize_state_message_csv(&state_msg, buffer, sizeof(buffer));
                    fprintf(fpt,"%s\n", buffer);
                    fclose(fpt);
                }
                last_message_index = state_msg.message_index;
                usleep(LONG_USLEEP_TIME);
            }
        }
        

        
    } 


    
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