#ifndef LINUX_COMMS_H
#define LINUX_COMMS_H

#include "stm_comms.h"

#include <sys/select.h> // select()
#define SELECT_TIMEOUT_MS 5   // Timeout per select() call (adjust as needed)

// --- Serial port setup ---
int open_serial(const char *port);
int configure_serial_port(int fd);

// Reads one framed packet ([START][LENGTH][TYPE][VERSION][DATA...][CRC_LO][CRC_HI])
// from fd into buf (capacity buf_size), using select()-gated reads so a
// stalled/slow link can't hang this indefinitely (SELECT_TIMEOUT_MS per
// step). Returns the total packet length on success, or -1 on timeout,
// oversized length byte, or CRC/version mismatch. This is the only packet
// reader now -- an earlier, simpler byte-state-machine reader
// (read_packet()) existed alongside this one but was never actually used by
// send_command_message/read_state_message, so it's been dropped rather than
// kept as an unused second implementation.
int read_packet_bulk(int fd, uint8_t *buf, size_t buf_size);

// --- Wrapper functions ---
int send_command_message(int port_id, CommandMessage* msg);
int read_state_message(int port_id, StateMessage* state_msg);

#endif // LINUX_COMMS_H
