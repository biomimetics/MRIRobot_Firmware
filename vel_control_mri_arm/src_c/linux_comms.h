#ifndef LINUX_COMMS_H
#define LINUX_COMMS_H

#include "stm_comms.h"

#include <sys/select.h> // select()
#define SELECT_TIMEOUT_MS 5   // Timeout per select() call (adjust as needed)

// --- Serial port setup ---
int open_serial(const char *port);
int configure_serial_port(int fd);

// Reads one framed packet ([START][LENGTH][TYPE][VERSION][DATA...][CRC_LO][CRC_HI])
// from fd into buf (capacity buf_size), using select()-gated batch reads so a
// stalled/slow link can't hang this indefinitely (SELECT_TIMEOUT_MS per
// read). Returns the total packet length on success, or -1 on timeout or
// read error. Malformed candidates (oversized LENGTH, unknown TYPE, wrong
// VERSION, CRC mismatch) do NOT return -1; the parser drops one byte and
// resyncs within the same call, keeping unconsumed bytes in a persistent
// carry buffer across calls so a false start can't discard a real packet's
// bytes (see linux_comms.c for the resync-amplification bug this fixes).
// NOTE: the carry buffer is static -- one serial link per process.
int read_packet_bulk(int fd, uint8_t *buf, size_t buf_size);

// Bytes currently held in read_packet_bulk's carry buffer (already consumed
// from the fd but not yet parsed into a packet). Callers deciding whether
// another packet is pending must add this to the kernel's FIONREAD count.
int serial_rx_buffered_bytes(void);

// Discard the carry buffer -- pair with tcflush when (re)opening the port so
// stale bytes from a previous session can't poison the first parse.
void serial_rx_reset(void);

// --- Wrapper functions ---
int send_command_message(int port_id, CommandMessage* msg);
int read_state_message(int port_id, StateMessage* state_msg);

#endif // LINUX_COMMS_H
