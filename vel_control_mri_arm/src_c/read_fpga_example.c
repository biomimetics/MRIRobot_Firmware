/*
 * Example STM32-side parsing code for the encoder burst protocol
 * implemented by mri_encoder_reader.v (the actual top-level module --
 * fpga_top.v is an older, unused leftover and should not be used as a
 * reference for this protocol).
 *
 * ============================================================================
 * 1. PHYSICAL LINK
 * ============================================================================
 * The STM32 talks to the FPGA over a single UART pair (STM32_IN/STM32_OUT
 * on the FPGA side), handled by comm_blk.v's STM_uart instance
 * (uart_transmitter.v / uart_receiver.v). Framing is standard 8N1 at a
 * baud rate hardwired inside comm_blk.v to 921600 -- not the BAUD_RATE
 * parameter on mri_encoder_reader.v's module header, which is actually
 * unused dead code: the USB-facing debug UART (see section on
 * debug_status_reporter.v) hardcodes its own baud rate (115200) directly
 * rather than referencing it, so BAUD_RATE doesn't affect anything.
 * Bits within each byte go out LSB-first, which is ordinary UART framing
 * and transparent to byte-oriented code -- nothing special to handle here.
 * The one thing that IS easy to get wrong is BYTE order within each
 * multi-byte field; see section 4.
 *
 * ============================================================================
 * 2. COMMAND PROTOCOL (STM32 -> FPGA)
 * ============================================================================
 * The STM32 sends single command bytes. mri_encoder_reader.v compares the
 * most-recently-received byte (STM_data) directly against these values:
 *
 *   0x01  Reset (synchronous; see mri_encoder_reader.v's `assign reset =
 *         BTN_sync[0] || (STM_data == 8'h01);`). This resets ALL encoder
 *         state (both groups' position counters, velocity estimators,
 *         status bytes, and qdec_enable).
 *   0x02  Disable encoder counting (qdec_enable <= 0). Position/velocity/
 *         diagnostics freeze at their current values; A/B/I transitions
 *         are ignored until re-enabled.
 *   0x03  Enable encoder counting (qdec_enable <= 1). This is the
 *         power-up default (qdec_en's REGISTER_R has INIT=1'b1), so you
 *         only need to send this after having sent 0x02 or after a 0x01
 *         reset if you want counting to resume without an external
 *         re-enable step -- reset does NOT re-enable counting on its own,
 *         it just clears qdec_enable back to its register's post-reset
 *         state, which is 1 (enabled) here as well.
 *   0x04  Fire the SEA group's burst readout (see section 3).
 *   0x05  Fire the USM group's burst readout (see section 3).
 *   other No defined effect. STM_data reflects whatever byte was last
 *         received, but nothing in mri_encoder_reader.v compares against
 *         other values.
 *
 * Internally, STM_data is only actually valid for a single FPGA clock
 * cycle per received byte (comm_blk.v's uart_receiver ties data_out_ready
 * high, so the "byte available" pulse self-clears the next cycle) -- but
 * this is invisible from the STM32 side. As far as this code is
 * concerned, "send a command" just means "write one byte."
 *
 * ============================================================================
 * 3. BURST RESPONSE PROTOCOL (FPGA -> STM32)
 * ============================================================================
 * Firing 0x04 or 0x05 starts that group's qdec_channel_bank 14-word burst
 * readout state machine (shift_counter14 inside qdec_channel_bank.v).
 * Each fire produces exactly one 112-byte payload (14 words x 8 bytes)
 * for that group's own 7 channels. That payload is NOT sent raw --
 * packet_framer.v (inserted between the serializer and the UART, see
 * section 6) appends a CRC-8 and COBS-encodes the result with a 0x00
 * delimiter, so the host reads a variable-length framed packet, not a
 * fixed 112-byte run. This is what replaced the earlier "there is no way
 * to resynchronize mid-burst" behavior: a corrupted or desynced frame is
 * now detected (via the CRC-8, and/or by a missing delimiter within the
 * expected max frame size) and the host can always resync on the next
 * 0x00 rather than staying permanently misaligned. The per-channel index
 * echo (section 5) is now a secondary, redundant self-check on top of
 * that -- the CRC-8 is the primary defense against silent corruption.
 *
 * SEA and USM are independently triggered and NEVER combined into one
 * burst -- there is no FPGA-side command that fires both groups at once,
 * and qdec_arbiter.v is a pass-through priority mux (only one group is
 * ever actively producing words during a single fire), not an
 * interleaver. To read the full state of all 14 encoders, fire 0x04 and
 * read its framed response, THEN fire 0x05 and read another -- exactly
 * what read_full_encoder_state() below does.
 *
 * Word order within a 112-byte payload (qdec_channel_bank.v's cntr_val
 * counts 0..13; even values select count_arr[cntr_val>>1], odd values
 * select metadata_arr[cntr_val>>1]):
 *
 *   word 0  = channel 0 count      word 1  = channel 0 metadata
 *   word 2  = channel 1 count      word 3  = channel 1 metadata
 *   ...
 *   word 12 = channel 6 count      word 13 = channel 6 metadata
 *
 * i.e. 16-byte records per channel, count word first, metadata word
 * second -- matching ENCODER_BYTES_PER_CHANNEL/parse_encoder_channel_report
 * below. The channel_index embedded in each metadata word (section 5) is
 * always local to its own group (0-6) -- it does NOT continue 7-13 for
 * USM, so the two 112-byte payloads must be parsed as two separate
 * 7-channel groups, never as one contiguous 14-channel stream.
 *
 * ============================================================================
 * 4. BYTE ORDER -- little-endian, NOT big-endian
 * ============================================================================
 * This section describes the byte order WITHIN THE DECODED 112-byte
 * payload (i.e. after cobs_decode() in section 6 has already stripped
 * the COBS framing) -- packet_framer.v's framing sits on top of, and is
 * independent of, this word-level byte order.
 *
 * Every 64-bit word (count or metadata) is serialized onto the wire
 * LEAST SIGNIFICANT BYTE FIRST. This comes from comm_blk.v's serializer,
 * VectorToSingle.sv: it slices the 64-bit FIFO word into 8 bytes indexed
 * 0-7 (index 0 = bits[7:0], the LSB byte; index 7 = bits[63:56], the MSB
 * byte) and transmits index 0 first, index 7 last.
 *
 * Practical effect: the FIRST byte you read off the wire for any 64-bit
 * field is that field's least significant byte, and the 8th byte is its
 * most significant byte. Reconstruct with
 *     v |= (uint64_t)byte[i] << (8 * i)
 * for i = 0..7 -- NOT the reverse. This was previously implemented
 * backwards in this file (a read_be64() that treated the first byte as
 * the MOST significant byte) -- if STM32-side code copied that
 * assumption, every count/velocity/status field comes out byte-reversed,
 * which looks like consistently-wrong-but-not-obviously-corrupt data
 * rather than random noise -- exactly the kind of "packets don't parse
 * right" symptom this header is here to prevent.
 *
 * ============================================================================
 * 5. WORD CONTENTS
 * ============================================================================
 * Count word: raw 64-bit two's-complement signed position counter
 * (qdec_channel.v's `count` register). +1/-1 per accepted quadrature
 * step, plus an optional signed index-correction nudge (index_corrector.v)
 * when index correction applies a discrepancy fix. Not scaled or
 * normalized to counts/revolution -- it's the raw accumulated count.
 *
 * Metadata word bit layout (MSB to LSB), from qdec_channel_bank.v's
 * metadata_arr packing
 * (`{velocity, illegal_transition_count, rate_reject_count, status, 5'b0, ch[2:0]}`):
 *
 *   bits [63:32]  velocity      signed Q20.11 fixed-point counts/sec.
 *                               Must match qdec_params.vh's
 *                               QDEC_VELOCITY_FRAC_BITS (11) -- see
 *                               ENCODER_VELOCITY_FRAC_BITS below. Holds
 *                               its last nonzero value between accepted
 *                               steps but is forced to exactly 0 once
 *                               QDEC_VELOCITY_STALL_TIMEOUT_CYCLES have
 *                               elapsed with no new accepted step (see
 *                               velocity_estimator.v), so a channel that
 *                               has genuinely stopped moving reads 0
 *                               rather than a stale nonzero value.
 *   bits [31:24]  illegal_transition_count   saturating 8-bit diagnostic
 *                               counter (invalid Gray-code A/B transition).
 *   bits [23:16]  rate_reject_count          saturating 8-bit diagnostic
 *                               counter (step arrived faster than
 *                               QDEC_MIN_STEP_INTERVAL_CYCLES allows).
 *   bits [15:8]   status        bit0: index correction applied this
 *                               index pulse; bit1: large discrepancy
 *                               detected between the index-implied and
 *                               tracked position; bit2: index pulse seen.
 *                               All three bits are sticky until the next
 *                               qualified index pulse (qdec_channel.v's
 *                               status_reg only updates when index_seen
 *                               fires).
 *   bits [7:3]    always 0      padding.
 *   bits [2:0]    channel_index 0-6, local to this group (SEA or USM) --
 *                               frame self-check field; see
 *                               parse_encoder_channel_report() below.
 *
 * Corrected relative to the plan doc's original snippet: velocity is
 * Q20.11 fixed-point counts/sec, not Q16.16 -- the format was rebalanced
 * after the plan doc was written, to fix an overflow bug (the old Q16.16
 * format silently wrapped for any accepted-step interval below ~3815
 * cycles, which is faster than rate_limiter's default 2500-cycle minimum
 * legal interval). See qdec/velocity_estimator.v and qdec_params.vh's
 * QDEC_VELOCITY_FRAC_BITS for the current authoritative format.
 *
 * ============================================================================
 * 6. PACKET FRAMING -- COBS + CRC-8 (see packet_framer.v, TODO.md)
 * ============================================================================
 * Motivation: without any packet-boundary marker or checksum, a burst
 * used to be just a fixed-length 112-byte run of raw bytes -- if any byte
 * was dropped, duplicated, or corrupted, everything after that point
 * silently shifted and got parsed as if it were valid, with only the
 * per-channel index echo (section 5) available to notice something was
 * wrong *after the fact*, and no way to find the next real packet
 * boundary. packet_framer.v (inserted into comm_blk.v between
 * VectorToSingle's serializer and the UART TX) fixes this:
 *
 *   1. Appends a CRC-8 (poly 0x07, init 0x00, computed byte-serially over
 *      the 112 payload bytes) as a 113th byte.
 *   2. COBS-encodes (Consistent Overhead Byte Stuffing) that 113-byte
 *      block: 0x00 is reserved exclusively as a packet-boundary marker,
 *      and every 0x00 that would otherwise appear in the payload/CRC is
 *      "stuffed away" behind a code byte instead.
 *   3. Terminates the encoded block with a single literal 0x00 delimiter.
 *
 * Because 113 (payload + CRC) is well under COBS's 254-byte max
 * single-code-byte span, packet_framer.v never needs COBS's "0xFF forced
 * split" case -- cobs_decode() below doesn't special-case code==0xFF
 * either. Frame length on the wire is variable: as few as 114 bytes (if
 * the payload happens to be all zero, the most-stuffed case) up to 115
 * bytes (if the payload has zero 0x00 bytes at all, the least-stuffed
 * case) -- ENCODER_FRAME_MAX_BYTES below has margin above that.
 *
 * Receive algorithm (see read_encoder_group() below): read bytes one at a
 * time until a 0x00 delimiter is seen (or ENCODER_FRAME_MAX_BYTES is
 * exceeded, treated as a desync/failure), COBS-decode the result, verify
 * the CRC-8, and only then hand the recovered 112-byte payload to
 * parse_encoder_burst() (unchanged, still does the per-channel index
 * self-check on top). On any failure -- decode error, wrong decoded
 * length, CRC mismatch, or no delimiter within the max -- treat the read
 * as failed; because framing is now explicit, resyncing is simply "start
 * looking for the next 0x00" on the following read, instead of staying
 * permanently misaligned the way the old fixed-length-read protocol did.
 *
 * crc8_compute() below MUST exactly match packet_framer.v's crc8_update
 * function (same poly 0x07, same init 0x00) bit-for-bit -- if it doesn't,
 * every single frame will fail its CRC check and be rejected as
 * corrupted, even when the link itself is perfectly clean.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define ENCODER_CHANNELS_PER_GROUP   7u
#define ENCODER_BYTES_PER_CHANNEL    16u   /* 8B count word + 8B metadata word */
#define ENCODER_BYTES_PER_BURST      (ENCODER_CHANNELS_PER_GROUP * ENCODER_BYTES_PER_CHANNEL)

/* Packet framing (section 6): 112 payload bytes + 1 CRC-8 byte, before
 * COBS encoding. Must match packet_framer.v's PAYLOAD_BYTES (112). */
#define ENCODER_PAYLOAD_BYTES        (ENCODER_BYTES_PER_BURST + 1u)

/* Worst case on-wire frame size (excluding the 0x00 delimiter itself) is
 * 1 code byte + 113 data bytes = 114 bytes (payload has zero 0x00 bytes,
 * so nothing gets stuffed away); rounded up with margin. */
#define ENCODER_FRAME_MAX_BYTES      128u

/* Total across both groups (7 SEA + 7 USM). Indices 0-6 are SEA,
 * indices 7-13 are USM, in read_full_encoder_state()'s output -- this
 * global numbering only exists on the STM32 side, assembled from two
 * separate 112-byte payloads; it is not how the FPGA labels channels. */
#define ENCODER_TOTAL_CHANNELS       (2u * ENCODER_CHANNELS_PER_GROUP)

/* Command bytes understood by mri_encoder_reader.v (see STM_data
 * handling there -- section 2 above). */
#define ENCODER_CMD_RESET            0x01u
#define ENCODER_CMD_DISABLE_COUNTING 0x02u
#define ENCODER_CMD_ENABLE_COUNTING  0x03u
#define ENCODER_CMD_FIRE_SEA         0x04u
#define ENCODER_CMD_FIRE_USM         0x05u

/* Must match qdec_params.vh's QDEC_VELOCITY_FRAC_BITS (11 -> Q20.11). If
 * that macro is ever retuned, this needs to change to match. */
#define ENCODER_VELOCITY_FRAC_BITS   11
#define ENCODER_VELOCITY_SCALE       ((float)(1 << ENCODER_VELOCITY_FRAC_BITS))

typedef struct {
    int64_t count;                        /* raw position, same semantics as today */
    int32_t velocity_q20_11;              /* signed Q20.11 fixed-point counts/sec (raw) */
    uint8_t illegal_transition_rejects;   /* saturating diagnostic counter */
    uint8_t rate_limit_rejects;           /* saturating diagnostic counter */
    uint8_t index_status;                 /* bit0: correction applied, bit1: large discrepancy, bit2: index seen */
    uint8_t channel_index;                /* echoed 0-6 *within its own group* (SEA or USM), for frame self-check */
} encoder_channel_report_t;

/* Converts the raw Q20.11 fixed-point value to a floating-point
 * counts/sec reading. */
static float encoder_velocity_counts_per_sec(int32_t velocity_q20_11) {
    return (float)velocity_q20_11 / ENCODER_VELOCITY_SCALE;
}

/* Reassembles a 64-bit field from 8 wire bytes in little-endian order --
 * byte[0] is the least significant byte, byte[7] the most significant.
 * See section 4 above: the FPGA's serializer (VectorToSingle.sv) sends
 * every 64-bit word LSB-byte-first. Do NOT swap this back to big-endian;
 * that was the bug this file previously had. */
static uint64_t read_le64(const uint8_t *p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) {
        v |= ((uint64_t)p[i]) << (8 * i);
    }
    return v;
}

/* Byte-serial CRC-8, poly 0x07, init 0x00. Must exactly match
 * packet_framer.v's crc8_update -- see section 6. */
static uint8_t crc8_compute(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* Standard COBS decode of `in` (in_len bytes, NOT including the 0x00
 * delimiter) into `out` (capacity out_cap). Returns the decoded length,
 * or 0 on a malformed frame (a code byte pointing past the end of `in`
 * or `out`, or an unexpected literal 0x00 in the frame body). See
 * section 6 -- packet_framer.v never produces COBS's 0xFF forced-split
 * case, so this decoder doesn't special-case code==0xFF either. */
static size_t cobs_decode(const uint8_t *in, size_t in_len, uint8_t *out, size_t out_cap) {
    size_t read_pos = 0;
    size_t write_pos = 0;

    while (read_pos < in_len) {
        uint8_t code = in[read_pos++];
        if (code == 0) {
            return 0;
        }
        size_t run = (size_t)code - 1u;
        if (read_pos + run > in_len || write_pos + run > out_cap) {
            return 0;
        }
        for (size_t i = 0; i < run; i++) {
            out[write_pos++] = in[read_pos++];
        }
        if (read_pos < in_len) {
            /* More bytes remain, which only happens if a real payload
             * zero was stuffed away here -- put it back. */
            if (write_pos >= out_cap) {
                return 0;
            }
            out[write_pos++] = 0x00;
        }
    }
    return write_pos;
}

/* Parses one 16-byte channel record (count word + metadata word) starting at `bytes`.
 * Returns false if the embedded channel-index byte doesn't match `expected_channel`,
 * signaling that the byte stream has desynced from the expected framing. */
static bool parse_encoder_channel_report(const uint8_t *bytes, uint8_t expected_channel,
                                          encoder_channel_report_t *out) {
    uint64_t count_word    = read_le64(bytes);
    uint64_t metadata_word = read_le64(bytes + 8);

    out->count                      = (int64_t)count_word;
    out->velocity_q20_11            = (int32_t)(metadata_word >> 32);
    out->illegal_transition_rejects = (uint8_t)(metadata_word >> 24);
    out->rate_limit_rejects         = (uint8_t)(metadata_word >> 16);
    out->index_status               = (uint8_t)(metadata_word >> 8);
    out->channel_index              = (uint8_t)(metadata_word);

    return out->channel_index == expected_channel;
}

/* Parses one 112-byte burst (7 channels, local indices 0-6) from either
 * group. Returns the number of channels parsed before a framing mismatch
 * was detected (== ENCODER_CHANNELS_PER_GROUP on a fully successful
 * parse). */
static int parse_encoder_burst(const uint8_t *burst,
                                encoder_channel_report_t out[ENCODER_CHANNELS_PER_GROUP]) {
    for (int ch = 0; ch < (int)ENCODER_CHANNELS_PER_GROUP; ch++) {
        const uint8_t *record = burst + ch * ENCODER_BYTES_PER_CHANNEL;
        if (!parse_encoder_channel_report(record, (uint8_t)ch, &out[ch])) {
            return ch;
        }
    }
    return (int)ENCODER_CHANNELS_PER_GROUP;
}

/* Hooks the caller's UART driver must provide -- not implemented here,
 * since the actual transport is part of the STM32 firmware project, not
 * this repo. stm32_read_byte should block until one byte has arrived (or
 * a timeout elapses), returning false on failure -- frames are
 * variable-length now (section 6), so there's no fixed byte count to
 * read in one shot the way there used to be. */
extern void stm32_send_command_byte(uint8_t cmd);
extern bool stm32_read_byte(uint8_t *b);

/* Fires one group (SEA or USM), reads its COBS-framed response up to the
 * 0x00 delimiter, decodes + CRC-8-verifies it (section 6), and parses
 * the recovered 112-byte payload into out[0..ENCODER_CHANNELS_PER_GROUP-1]. */
static bool read_encoder_group(uint8_t fire_cmd,
                                encoder_channel_report_t out[ENCODER_CHANNELS_PER_GROUP]) {
    uint8_t frame[ENCODER_FRAME_MAX_BYTES];
    uint8_t decoded[ENCODER_PAYLOAD_BYTES];
    size_t frame_len = 0;

    stm32_send_command_byte(fire_cmd);

    for (;;) {
        if (frame_len >= ENCODER_FRAME_MAX_BYTES) {
            return false; /* no delimiter within the expected max -- desynced */
        }
        uint8_t b;
        if (!stm32_read_byte(&b)) {
            return false;
        }
        if (b == 0x00) {
            break;
        }
        frame[frame_len++] = b;
    }

    size_t decoded_len = cobs_decode(frame, frame_len, decoded, sizeof(decoded));
    if (decoded_len != ENCODER_PAYLOAD_BYTES) {
        return false; /* malformed frame */
    }

    uint8_t crc = crc8_compute(decoded, ENCODER_BYTES_PER_BURST);
    if (crc != decoded[ENCODER_BYTES_PER_BURST]) {
        return false; /* corrupted -- caller can just retry, which resyncs on the next 0x00 */
    }

    return parse_encoder_burst(decoded, out) == (int)ENCODER_CHANNELS_PER_GROUP;
}

/* Reads the full state (count + velocity, all diagnostics) of all 14
 * encoders by firing SEA then USM in turn and concatenating their
 * reports -- out[0..6] = SEA channels 0-6, out[7..13] = USM channels
 * 0-6. This is two independent framed transfers on the wire (112-byte
 * payload each, ~114-115 bytes once COBS-encoded -- section 6), not one
 * combined 224-byte-payload burst; the FPGA has no single command that
 * fires both groups at once. */
bool read_full_encoder_state(encoder_channel_report_t out[ENCODER_TOTAL_CHANNELS]) {
    if (!read_encoder_group(ENCODER_CMD_FIRE_SEA, &out[0])) {
        return false;
    }
    if (!read_encoder_group(ENCODER_CMD_FIRE_USM, &out[ENCODER_CHANNELS_PER_GROUP])) {
        return false;
    }
    return true;
}
