#include "stm_comms.h"

#include <sys/select.h> // select()
#define SELECT_TIMEOUT_MS 5   // Timeout per select() call (adjust as needed)

int open_serial(const char *port);
int configure_serial_port(int fd);
int read_packet(int fd, uint8_t *buf, int max_iterations);

// wrapper functions
int init_serial(const char *port);
int send_command_message(int port_id, CommandMessage* msg);
int read_state_message(int port_id, StateMessage* state_msg);

static int wait_for_data(int fd, int timeout_ms);
int read_packet_bulk(int fd, uint8_t *buf, size_t buf_size);

