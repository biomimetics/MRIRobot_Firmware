#include "stm_comms.h"
#include <stdarg.h>

// =====================
// CSV helpers
// =====================

// snprintf returns the length it WOULD have written, not what it did. Naively
// accumulating that (`written += snprintf(buf + written, size - written, ...)`)
// lets `written` run PAST `size` once the content overflows -- after which
// `buffer + written` points outside the caller's buffer and `size - written`
// underflows (both size_t) into a huge value, so the next snprintf happily
// writes out of bounds. That is a live stack smash in uart_comms_logger.c,
// whose buffers are fixed-size locals.
//
// This clamps instead: once the buffer is full, further appends are dropped and
// the result stays NUL-terminated and merely truncated. Callers that care about
// truncation should size their buffer per the *_CSV_BUFFER_SIZE hints in
// stm_comms.h.
static void csv_append(char *buffer, size_t size, size_t *written, const char *fmt, ...) {
    if (buffer == NULL || size == 0 || *written >= size - 1) return;

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buffer + *written, size - *written, fmt, ap);
    va_end(ap);

    if (n < 0) return; // encoding error; leave `written` alone
    *written += (size_t) n;
    if (*written >= size) *written = size - 1; // vsnprintf truncated; stay in bounds
}

// =====================
// Packet framing + CRC
// =====================

void print_buffer(uint8_t *buffer, uint16_t length) {
    printf("Received %d bytes:\n", length);
    for (uint16_t i = 0; i < length; i++) {
        printf("0x%02X ", buffer[i]);
    }
    printf("\n");
}

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= ((uint16_t) data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000) {
                crc = (uint16_t) ((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t) (crc << 1);
            }
        }
    }
    return crc;
}

int build_packet(uint8_t *out_buf, PacketType type, const uint8_t *data, uint8_t data_len) {
    if ((size_t) data_len + PACKET_OVERHEAD > UART_BUFFER_SIZE) return -1;

    int idx = 0;
    out_buf[idx++] = PACKET_START_BYTE;
    out_buf[idx++] = (uint8_t) (data_len + PACKET_TYPE_VERSION_SIZE); // TYPE + VERSION + DATA
    out_buf[idx++] = (uint8_t) type;
    out_buf[idx++] = PROTOCOL_VERSION;

    for (int i = 0; i < data_len; i++) {
        out_buf[idx++] = data[i];
    }

    // CRC covers LENGTH through the end of DATA (bytes[1..idx-1]) -- same
    // range the old additive checksum covered, just a stronger check.
    uint16_t crc = crc16_ccitt(&out_buf[1], (size_t) (idx - 1));
    out_buf[idx++] = (uint8_t) (crc & 0xFF);        // CRC_LO
    out_buf[idx++] = (uint8_t) ((crc >> 8) & 0xFF); // CRC_HI

    return idx;
}

// =====================
// Command message specific definitions
// =====================

void zero_command_message(CommandMessage* msg) {
    memset(msg, 0, sizeof(CommandMessage));
}

void construct_command_message(CommandMessage* msg, int behavior_mode,
                        const int32_t* velocity_counts_per_sec,
                        int time_stamp, int message_index) {
    if (!msg) return;

    msg->behavior_mode = behavior_mode;

    memcpy(msg->velocity_counts_per_sec, velocity_counts_per_sec, sizeof(int32_t) * DOF_NUMBER);

    msg->time_stamp = time_stamp;
    msg->message_index = message_index;
}

int encode_command_message_to_data_buffer(const CommandMessage *msg, uint8_t *buffer) {
    memcpy(buffer, msg, sizeof(CommandMessage));
    return (int) sizeof(CommandMessage);
}

bool decode_data_buffer_to_command_message(CommandMessage *msg, const uint8_t *data_buffer, size_t data_buffer_len) {
    if (data_buffer_len != sizeof(CommandMessage)) {
        fprintf(stderr, "Unexpected CommandMessage size! Got %u, expected %u\n",
                (unsigned)data_buffer_len, (unsigned)sizeof(CommandMessage));
        return false;
    }

    memcpy(msg, data_buffer, sizeof(CommandMessage));
    return true;
}

bool handle_command_message_packet(CommandMessage* msg, const uint8_t *packet, size_t packet_len) {
    uint8_t type = packet[2];
    uint8_t version = packet[3];
    bool result = false;

    if (version != PROTOCOL_VERSION) {
        fprintf(stderr, "CommandMessage packet protocol version mismatch! Got %d, expected %d\n",
                version, PROTOCOL_VERSION);
        return false;
    }

    switch (type) {
        case PKT_TYPE_PING:
            break;

        case PKT_TYPE_DATA: {
            const uint8_t *data_buffer = &packet[4]; // after start, length, type, version
            size_t data_len = packet_len - PACKET_OVERHEAD;
            result = decode_data_buffer_to_command_message(msg, data_buffer, data_len);
            break;
        }
    }
    return result;
}

// Commands are integral counts/sec now, so this and print_command_message_int
// below print identically -- the _int variant existed only to avoid float
// formatting. Both are kept so callers don't have to change.
void print_command_message(const CommandMessage *msg) {
    printf("Command Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);
    printf("  Velocities [counts/sec]: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%ld,", (long) msg->velocity_counts_per_sec[i]);
    printf("\n");

    printf("  Timestamp: %d\n", msg->time_stamp);
    printf("  Index: %d\n", msg->message_index);
}

void print_command_message_int(const CommandMessage *msg) {
    print_command_message(msg);
}

// =====================
// State message specific definitions
// =====================

void zero_state_message(StateMessage* msg) {
    memset(msg, 0, sizeof(StateMessage));
}

void construct_state_message(StateMessage* msg, int behavior_mode,
                            const int32_t* position_counts, const float* velocity_counts_per_sec,
                            const int32_t* sea_position_counts,
                            const EncoderDiagnostics* diagnostics,
                            int time_stamp, int message_index,
                            int echoed_time_stamp, int echoed_message_index) {
    if (!msg) return;

    msg->behavior_mode = behavior_mode;

    memcpy(msg->position_counts, position_counts, sizeof(int32_t) * DOF_NUMBER);
    memcpy(msg->velocity_counts_per_sec, velocity_counts_per_sec, sizeof(float) * DOF_NUMBER);
    memcpy(msg->sea_position_counts, sea_position_counts, sizeof(int32_t) * DOF_NUMBER);

    // Zero rather than leave indeterminate when the caller has no diagnostics:
    // an all-zero block reads as "no faults, nothing held", which is the honest
    // default for a caller that isn't tracking them (e.g. host-side test tools).
    if (diagnostics) {
        msg->diagnostics = *diagnostics;
    } else {
        memset(&msg->diagnostics, 0, sizeof(msg->diagnostics));
    }

    msg->time_stamp = time_stamp;
    msg->message_index = message_index;

    msg->echoed_time_stamp = echoed_time_stamp;
    msg->echoed_message_index = echoed_message_index;
}

int encode_state_message_to_data_buffer(const StateMessage *msg, uint8_t *buffer) {
    memcpy(buffer, msg, sizeof(StateMessage));
    return (int) sizeof(StateMessage);
}

bool decode_data_buffer_to_state_message(StateMessage *msg, const uint8_t *data_buffer, size_t data_buffer_len) {
    if (data_buffer_len != sizeof(StateMessage)) {
        fprintf(stderr, "Unexpected StateMessage size! Got %zu, expected %zu\n",
                data_buffer_len, sizeof(StateMessage));
        return false;
    }

    memcpy(msg, data_buffer, sizeof(StateMessage));
    return true;
}

bool handle_state_message_packet(StateMessage* msg, const uint8_t *packet, size_t len) {
    uint8_t type = packet[2];
    uint8_t version = packet[3];
    bool result = false;

    if (version != PROTOCOL_VERSION) {
        fprintf(stderr, "StateMessage packet protocol version mismatch! Got %d, expected %d\n",
                version, PROTOCOL_VERSION);
        return false;
    }

    switch (type) {
        case PKT_TYPE_PING:
            break;

        case PKT_TYPE_DATA: {
            const uint8_t *data_buffer = &packet[4]; // after start, length, type, version
            size_t data_len = len - PACKET_OVERHEAD;
            result = decode_data_buffer_to_state_message(msg, data_buffer, data_len);
            break;
        }
    }
    return result;
}

// Prints the diagnostics block, but only the channels with something to say --
// a per-channel dump of 14 mostly-zero rows every message buries the one row
// that matters. A fully healthy block prints a single "all clear" line.
void print_encoder_diagnostics(const EncoderDiagnostics *d) {
    if (!d) return;

    printf("  Encoder diagnostics: packets ok=%lu bad=%lu\n",
           (unsigned long) d->encoder_packets_ok, (unsigned long) d->encoder_packets_bad);

    int noisy = 0;
    for (int i = 0; i < DOF_NUMBER; ++i) {
        const struct { const char *name; uint8_t status, illegal, rate, streak; } ch[2] = {
            {"USM", d->motor_status[i], d->motor_illegal_transition_rejects[i],
                    d->motor_rate_limit_rejects[i], d->motor_reject_streak[i]},
            {"SEA", d->sea_status[i], d->sea_illegal_transition_rejects[i],
                    d->sea_rate_limit_rejects[i], d->sea_reject_streak[i]},
        };
        for (int g = 0; g < 2; ++g) {
            // status bit2 (index seen) is normal operation, not a fault, so it
            // alone does not make a channel interesting.
            bool faulted = ch[g].illegal || ch[g].rate || ch[g].streak ||
                           (ch[g].status & (ENCODER_STATUS_INDEX_CORRECTION_APPLIED |
                                            ENCODER_STATUS_LARGE_DISCREPANCY));
            if (!faulted) continue;
            noisy++;
            printf("    %s ch%d: status=0x%02X%s%s%s illegal=%u%s rate=%u%s held=%u\n",
                   ch[g].name, i, ch[g].status,
                   (ch[g].status & ENCODER_STATUS_INDEX_CORRECTION_APPLIED) ? " [idx-corrected]" : "",
                   (ch[g].status & ENCODER_STATUS_LARGE_DISCREPANCY) ? " [DISCREPANCY]" : "",
                   (ch[g].status & ENCODER_STATUS_INDEX_SEEN) ? " [idx-seen]" : "",
                   (unsigned) ch[g].illegal,
                   ch[g].illegal >= ENCODER_REJECT_COUNTER_SATURATED ? "(SAT,>=15)" : "",
                   (unsigned) ch[g].rate,
                   ch[g].rate >= ENCODER_REJECT_COUNTER_SATURATED ? "(SAT,>=15)" : "",
                   (unsigned) ch[g].streak);
        }
    }
    if (!noisy) printf("    all channels clear\n");
}

void print_state_message(const StateMessage *msg) {
    printf("State Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);

    printf("  Positions [counts]: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%ld,", (long) msg->position_counts[i]);
    printf("\n");

    printf("  Velocities [counts/sec]: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%.*f,", FLOAT_DECIMAL_SCALE, msg->velocity_counts_per_sec[i]);
    printf("\n");

    printf("  SEA Positions [counts]: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%ld,", (long) msg->sea_position_counts[i]);
    printf("\n");

    printf("  Timestamp: %d\n", msg->time_stamp);
    printf("  Index: %d\n", msg->message_index);
    printf("  Echoed Timestamp: %d\n", msg->echoed_time_stamp);
    printf("  Echoed Index: %d\n", msg->echoed_message_index);
    print_encoder_diagnostics(&msg->diagnostics);
}

// Positions are integral counts now, so only the velocity line still needs the
// milli-int treatment to stay float-formatting-free.
void print_state_message_int(const StateMessage *msg) {
    printf("State Message:\n");
    printf("  Mode: %d\n", msg->behavior_mode);

    printf("  Positions [counts]: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%ld ", (long) msg->position_counts[i]);
    printf("\n");

    printf("  Velocities [milli-counts/sec]: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%d ", (int) (msg->velocity_counts_per_sec[i] * FLOAT_PRINT_SCALE));
    printf("\n");

    printf("  SEA Positions [counts]: ");
    for (int i = 0; i < DOF_NUMBER; ++i) printf("%ld ", (long) msg->sea_position_counts[i]);
    printf("\n");

    printf("  Timestamp: %d\n", msg->time_stamp);
    printf("  Index: %d\n", msg->message_index);
    printf("  Echoed Timestamp: %d\n", msg->echoed_time_stamp);
    printf("  Echoed Index: %d\n", msg->echoed_message_index);
    print_encoder_diagnostics(&msg->diagnostics);
}

// Column order here and value order in serialize_state_message_csv below MUST
// stay in lockstep -- they are two hand-maintained lists describing one row.
void write_state_message_csv_header(char *buffer, size_t size) {
    size_t written = 0;

    csv_append(buffer, size, &written, "behavior_mode,");

    // Column names carry the unit explicitly: these are raw real-encoder counts
    // now, not rad/rad-per-sec, and old CSVs on disk are in the old units.
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "position_counts_%d,", i);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "velocity_counts_per_sec_%d,", i);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "sea_position_counts_%d,", i);

    csv_append(buffer, size, &written, "encoder_packets_ok,encoder_packets_bad,");

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "motor_status_%d,", i);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "motor_illegal_transition_rejects_%d,", i);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "motor_rate_limit_rejects_%d,", i);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "motor_reject_streak_%d,", i);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "sea_status_%d,", i);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "sea_illegal_transition_rejects_%d,", i);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "sea_rate_limit_rejects_%d,", i);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "sea_reject_streak_%d,", i);

    csv_append(buffer, size, &written, "time_stamp,message_index,echoed_time_stamp,echoed_message_index");
}

void serialize_state_message_csv(const StateMessage *msg, char *buffer, size_t size) {
    size_t written = 0;
    const EncoderDiagnostics *d = &msg->diagnostics;

    csv_append(buffer, size, &written, "%d,", msg->behavior_mode);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%ld,", (long) msg->position_counts[i]);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%.*f,", FLOAT_DECIMAL_SCALE, msg->velocity_counts_per_sec[i]);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%ld,", (long) msg->sea_position_counts[i]);

    csv_append(buffer, size, &written, "%lu,%lu,",
               (unsigned long) d->encoder_packets_ok, (unsigned long) d->encoder_packets_bad);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->motor_status[i]);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->motor_illegal_transition_rejects[i]);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->motor_rate_limit_rejects[i]);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->motor_reject_streak[i]);

    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->sea_status[i]);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->sea_illegal_transition_rejects[i]);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->sea_rate_limit_rejects[i]);
    for (int i = 0; i < DOF_NUMBER; ++i)
        csv_append(buffer, size, &written, "%u,", (unsigned) d->sea_reject_streak[i]);

    csv_append(buffer, size, &written, "%d,%d,%d,%d",
               msg->time_stamp, msg->message_index,
               msg->echoed_time_stamp, msg->echoed_message_index);
}
