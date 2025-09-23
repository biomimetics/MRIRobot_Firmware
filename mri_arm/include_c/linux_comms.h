#include "stm_comms.h"

int open_serial(const char *port);
int configure_serial_port(int fd);
int read_packet(int fd, uint8_t *buf, int max_iterations);

// wrapper functions
int init_serial(const char *port);
int send_command_message(int port_id, CommandMessage* msg);
int read_state_message(int port_id, StateMessage* state_msg);