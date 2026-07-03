/*
 * Example STM32-side parsing code for the encoder burst protocol
 * implemented by mri_encoder_reader.v (the actual top-level module --
 * fpga_top.v is an older, unused leftover and should not be used as a
 * reference for this protocol).
 *
 * THIS IS A REFERENCE SPEC, NOT YET WIRED INTO Encoder.lf. This file
 * documents the new single-fire, fixed-frame protocol precisely enough
 * to implement the corresponding Encoder.lf changes directly from it (a
 * separate, later pass -- see the FPGA repo's plan history). It replaces
 * the OLD two-independent-fires (0x04=SEA / 0x05=USM) COBS+CRC-8
 * protocol this file used to document.
 *
 * ============================================================================
 * 0. WHY THIS CHANGED
 * ============================================================================
 * The old protocol fired SEA and USM independently, back-to-back every
 * cycle, merged downstream by qdec_arbiter.v's runtime priority mux and
 * a single-buffered COBS+CRC-8 packet_framer.v. That design produced two
 * classes of hard-to-diagnose bugs on real hardware (see
 * fpga_comms_potential_issues.md in the firmware repo for full worked
 * examples): a stable +4 rotation in each record's channel_index
 * self-check byte, and an occasional whole-burst swap where firing USM
 * immediately after SEA returned a CRC-valid but exact copy of SEA's
 * data. Both trace to the same root pattern: two independent triggered
 * bursts racing to share one arbiter/framer/UART pipeline.
 *
 * The fix is architectural: ONE unified fire now atomically snapshots
 * and streams all 14 channels as a single fixed packet (see
 * qdec_burst_sequencer.v in the FPGA repo). qdec_arbiter.v is deleted.
 * There is no longer a way for one bank's data to substitute for the
 * other's, and no per-record self-check label is needed since wire
 * position is authoritative again by construction.
 *
 * ============================================================================
 * 1. PHYSICAL LINK
 * ============================================================================
 * The STM32 talks to the FPGA over a single UART pair (STM32_IN/STM32_OUT
 * on the FPGA side), handled by comm_blk.v's STM_uart instance
 * (uart_transmitter.v / uart_receiver.v). Framing is standard 8N1 at
 * 2,500,000 baud (raised from the old 921600 -- comm_blk.v hardwires
 * this internally; it divides the FPGA's 125MHz system clock exactly,
 * 125_000_000/2_500_000 = 50 cycles/bit with zero rounding). On the
 * STM32 side (huart3, per Encoder.lf), matching this requires
 * UART_OVERSAMPLING_8 -- OVER16 tops out around 2.625Mbps on this
 * board's ~42MHz APB1/PCLK1, too little margin against 2.5Mbps. After
 * setting this, verify HAL's actual achieved baud is within a couple
 * percent of nominal.
 *
 * At 2.5Mbps, the new 144-byte fixed frame (section 3) takes roughly
 * 144 * 10 bits / 2,500,000 =~ 576us to transmit -- comfortably inside a
 * <1-2ms round-trip target even with STM32-side processing overhead.
 *
 * ============================================================================
 * 2. COMMAND PROTOCOL (STM32 -> FPGA)
 * ============================================================================
 * The STM32 sends single command bytes. mri_encoder_reader.v compares the
 * most-recently-received byte (STM_data) directly against these values:
 *
 *   0x01  Reset (synchronous). Resets ALL encoder state (both groups'
 *         position counters, velocity estimators, status bytes, and
 *         qdec_enable).
 *   0x02  Disable encoder counting (qdec_enable <= 0). Position/velocity/
 *         diagnostics freeze at their current values; A/B/I transitions
 *         are ignored until re-enabled.
 *   0x03  Enable encoder counting (qdec_enable <= 1). Power-up default.
 *   0x04  Fire the unified burst readout -- atomically snapshots and
 *         streams ALL 14 channels (SEA 0-6 then USM 0-6) as one fixed
 *         frame (section 3). REPURPOSED from the old "fire SEA only"
 *         meaning.
 *   0x05  RETIRED. The old "fire USM only" command no longer exists --
 *         0x04 now reads everything in one shot. mri_encoder_reader.v
 *         does not special-case 0x05; sending it has no defined effect
 *         (same as any other undefined byte).
 *   other No defined effect. STM_data reflects whatever byte was last
 *         received, but nothing in mri_encoder_reader.v compares against
 *         other values.
 *
 * Internally, STM_data is only actually valid for a single FPGA clock
 * cycle per received byte -- invisible from the STM32 side. As far as
 * this code is concerned, "send a command" just means "write one byte."
 *
 * ============================================================================
 * 3. BURST RESPONSE PROTOCOL (FPGA -> STM32) -- fixed 144-byte frame
 * ============================================================================
 * Firing 0x04 triggers qdec_burst_sequencer.v: on the fire cycle, it
 * atomically snapshots all 14 channels' current count/velocity/status/
 * diagnostics (every channel in the burst reflects the same instant,
 * even if the underlying counts are still ticking during the walk that
 * follows), then streams the snapshot out as ONE fixed-length frame --
 * no separate SEA/USM triggers, no arbitration between them.
 *
 * Frame layout (always exactly 144 bytes on the wire, no delimiter, no
 * length byte -- both sides already know the length):
 *
 *   bytes   0-1    magic header: 0xAA, 0x55
 *   bytes   2-141  payload: 14 channel records x 10 bytes each (below)
 *   bytes 142-143  CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), over
 *                  the payload ONLY (bytes 2-141) -- LOW byte first
 *                  (byte 142), then HIGH byte (byte 143)
 *
 * Channel order within the payload is FIXED: record 0 = SEA channel 0,
 * record 1 = SEA channel 1, ..., record 6 = SEA channel 6, record 7 =
 * USM channel 0, ..., record 13 = USM channel 6. This is wire position,
 * not a label -- there is no per-record channel_index self-check byte
 * in this format (unlike the old protocol), because with the old
 * arbiter's race eliminated there is nothing left to rotate. Trust wire
 * position directly.
 *
 * ============================================================================
 * 4. PER-CHANNEL RECORD LAYOUT -- 10 bytes, all little-endian
 * ============================================================================
 *   bytes 0-3  count      int32, signed, little-endian. Truncated from
 *                         the FPGA's internal 64-bit position counter --
 *                         real values are nowhere near +-2^31, only the
 *                         wire copy narrows.
 *   bytes 4-7  velocity   int32, signed, little-endian, Q20.11
 *                         fixed-point counts/sec -- SAME format as the
 *                         old protocol, unchanged. Must match
 *                         qdec_params.vh's QDEC_VELOCITY_FRAC_BITS (11).
 *                         See ENCODER_VELOCITY_FRAC_BITS below.
 *   byte  8    status     bit0: index correction applied this index
 *                         pulse; bit1: large discrepancy detected
 *                         between the index-implied and tracked
 *                         position; bit2: index pulse seen. Bits 3-7
 *                         always 0. Same meaning as the old protocol's
 *                         status byte, unchanged.
 *   byte  9    diag       bits [7:4] = illegal_transition_count,
 *                         SATURATED to 4 bits (0-15, not the old 8-bit
 *                         range). bits [3:0] = rate_reject_count,
 *                         likewise saturated to 4 bits. These are
 *                         already saturating/qualitative diagnostic
 *                         counters on the FPGA side ("pinned at max" is
 *                         the meaningful signal), so 4 bits of
 *                         resolution on the wire is sufficient -- do NOT
 *                         assume a value of 15 here means "exactly 15
 *                         events," it means "15 or more."
 *
 * Byte reconstruction: v |= (uint32_t)byte[i] << (8*i) for i = 0..3 --
 * little-endian, same convention the old protocol used for its 64-bit
 * words (this file previously got that backwards once; see the
 * read_le32 comment below for the same warning restated).
 *
 * ============================================================================
 * 5. FRAME VALIDATION AND RESYNC
 * ============================================================================
 * Read exactly 144 bytes (a single fixed-length blocking read is
 * sufficient and simpler than the old protocol's byte-at-a-time
 * delimiter-hunting loop, since the length is now known a priori).
 * Validate:
 *   1. bytes[0:1] == 0xAA, 0x55
 *   2. CRC-16/CCITT-FALSE over bytes[2:141] == bytes[142:143] (low byte
 *      first)
 *
 * On EITHER check failing, do not just drop the frame -- the stream may
 * have desynced (e.g. a dropped byte shifted everything after it). Byte-
 * scan forward for the next 0xAA, 0x55 occurrence and attempt to read
 * the remaining 142 bytes from there, re-validating. This replaces the
 * old protocol's COBS delimiter-hunting (0x00 byte) -- there is no
 * delimiter in this format, so resync is driven by the magic header
 * instead.
 *
 * crc16_compute() below MUST exactly match packet_framer_fixed.v's
 * crc16_update function (same poly 0x1021, same init 0xFFFF) bit-for-bit
 * -- if it doesn't, every single frame will fail its CRC check and be
 * rejected as corrupted, even when the link itself is perfectly clean.
 *
 * ============================================================================
 * 6. CARRYING FORWARD EXISTING STM32-SIDE MITIGATIONS
 * ============================================================================
 * When Encoder.lf is updated to use this protocol, two existing
 * mitigations should carry over UNCHANGED -- they address failure modes
 * this redesign does not claim to fix:
 *
 *   - filter_channel_update()'s reject-streak filter: keep it. It also
 *     catches genuine encoder-line electrical-noise glitches
 *     (rate-limiter saturation) that are unrelated to the arbiter/framer
 *     race this redesign eliminates.
 *   - ENCODER_DEBUG_SWAP_SEA_USM_LAST_THREE: keep it, exactly as-is,
 *     unless/until separately confirmed fixed. The channels-4-6
 *     cross-routing anomaly it works around is NOT confirmed to share a
 *     root cause with the whole-burst swap bug this redesign targets --
 *     test on hardware post-rollout whether it happens to also resolve;
 *     if so, remove the flag in a later, SEPARATE change.
 *
 * The old channel_index-based placement workaround (trusting an
 * embedded self-check byte over word position, to work around the old
 * protocol's +4 rotation bug) has no equivalent in this format and
 * should simply be deleted -- there is no channel_index field anymore,
 * and none is needed.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define ENCODER_TOTAL_CHANNELS       14u  /* 7 SEA (indices 0-6) + 7 USM (indices 7-13) */
#define ENCODER_BYTES_PER_CHANNEL    10u
#define ENCODER_PAYLOAD_BYTES        (ENCODER_TOTAL_CHANNELS * ENCODER_BYTES_PER_CHANNEL) /* 140 */

#define ENCODER_MAGIC_BYTE_0         0xAAu
#define ENCODER_MAGIC_BYTE_1         0x55u

/* Total on-wire frame size: 2 magic + 140 payload + 2 CRC-16 = 144.
 * Fixed -- no delimiter, no length byte, both sides already know this. */
#define ENCODER_FRAME_BYTES          (2u + ENCODER_PAYLOAD_BYTES + 2u)

/* Command bytes understood by mri_encoder_reader.v (see section 2). */
#define ENCODER_CMD_RESET            0x01u
#define ENCODER_CMD_DISABLE_COUNTING 0x02u
#define ENCODER_CMD_ENABLE_COUNTING  0x03u
#define ENCODER_CMD_FIRE_ALL         0x04u  /* was "fire SEA" in the old protocol */

/* Must match qdec_params.vh's QDEC_VELOCITY_FRAC_BITS (11 -> Q20.11). If
 * that macro is ever retuned, this needs to change to match. */
#define ENCODER_VELOCITY_FRAC_BITS   11
#define ENCODER_VELOCITY_SCALE       ((float)(1 << ENCODER_VELOCITY_FRAC_BITS))

typedef struct {
    int32_t count;               /* raw position, truncated to 32 bits on the wire */
    int32_t velocity_q20_11;     /* signed Q20.11 fixed-point counts/sec (raw) */
    uint8_t status;              /* bit0: correction applied, bit1: large discrepancy, bit2: index seen */
    uint8_t illegal_transition_rejects; /* saturated to 4 bits (0-15) on the wire */
    uint8_t rate_limit_rejects;         /* saturated to 4 bits (0-15) on the wire */
} encoder_channel_report_t;

/* Converts the raw Q20.11 fixed-point value to a floating-point
 * counts/sec reading. */
static float encoder_velocity_counts_per_sec(int32_t velocity_q20_11) {
    return (float)velocity_q20_11 / ENCODER_VELOCITY_SCALE;
}

/* Reassembles a 32-bit field from 4 wire bytes in little-endian order --
 * byte[0] is the least significant byte, byte[3] the most significant.
 * Same convention the old protocol used for its 64-bit words -- do NOT
 * assume big-endian here, that was a real bug in an earlier version of
 * this file (see section 4). */
static uint32_t read_le32(const uint8_t *p) {
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        v |= ((uint32_t)p[i]) << (8 * i);
    }
    return v;
}

/* Byte-serial CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no
 * reflection, xorout 0. Must exactly match packet_framer_fixed.v's
 * crc16_update -- see section 5. */
static uint16_t crc16_compute(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Parses one 10-byte channel record starting at `bytes`. Channel
 * identity is the caller's wire-position index -- there is no
 * self-check field in this record to cross-validate against (see
 * section 3: with the old arbiter's race eliminated, wire position is
 * authoritative by construction). */
static void parse_encoder_channel_report(const uint8_t *bytes, encoder_channel_report_t *out) {
    out->count                      = (int32_t)read_le32(bytes);
    out->velocity_q20_11            = (int32_t)read_le32(bytes + 4);
    out->status                     = bytes[8];
    out->illegal_transition_rejects = (uint8_t)(bytes[9] >> 4);
    out->rate_limit_rejects         = (uint8_t)(bytes[9] & 0x0F);
}

/* Parses the 140-byte payload (already validated -- see
 * validate_and_parse_frame below) into all 14 channels: out[0..6] = SEA
 * channels 0-6, out[7..13] = USM channels 0-6, straight from wire
 * position. */
static void parse_encoder_payload(const uint8_t *payload,
                                   encoder_channel_report_t out[ENCODER_TOTAL_CHANNELS]) {
    for (unsigned ch = 0; ch < ENCODER_TOTAL_CHANNELS; ch++) {
        parse_encoder_channel_report(payload + ch * ENCODER_BYTES_PER_CHANNEL, &out[ch]);
    }
}

/* Validates a captured ENCODER_FRAME_BYTES-byte frame (magic + CRC-16,
 * section 5) and, if valid, parses it into out[]. Returns false on any
 * validation failure -- caller should byte-scan for the next magic
 * header occurrence and retry (section 5), not just drop and re-read
 * blindly, since a byte may have been dropped/inserted somewhere in the
 * stream. */
static bool validate_and_parse_frame(const uint8_t *frame,
                                      encoder_channel_report_t out[ENCODER_TOTAL_CHANNELS]) {
    if (frame[0] != ENCODER_MAGIC_BYTE_0 || frame[1] != ENCODER_MAGIC_BYTE_1) {
        return false;
    }

    const uint8_t *payload = frame + 2;
    uint16_t crc = crc16_compute(payload, ENCODER_PAYLOAD_BYTES);
    uint16_t frame_crc = (uint16_t)payload[ENCODER_PAYLOAD_BYTES] |
                          ((uint16_t)payload[ENCODER_PAYLOAD_BYTES + 1] << 8);
    if (crc != frame_crc) {
        return false;
    }

    parse_encoder_payload(payload, out);
    return true;
}

/* Hooks the caller's UART driver must provide -- not implemented here,
 * since the actual transport is part of the STM32 firmware project, not
 * this repo. stm32_read_bytes should block until `len` bytes have
 * arrived (or a timeout elapses), returning false on failure. Unlike the
 * old variable-length COBS protocol, frames here are always exactly
 * ENCODER_FRAME_BYTES, so a single fixed-length blocking read suffices
 * for the fast path (see stm32_scan_for_magic below for the resync
 * path). */
extern void stm32_send_command_byte(uint8_t cmd);
extern bool stm32_read_bytes(uint8_t *buf, size_t len);
extern bool stm32_read_byte(uint8_t *b);

/* Resync path: on a validation failure, shift a 1-byte sliding window
 * forward looking for the next magic-header occurrence, then read the
 * remaining ENCODER_FRAME_BYTES-2 bytes from there and re-validate.
 * Bounded by max_scan_bytes to avoid scanning forever on a dead/garbage
 * link. */
static bool read_encoder_frame_with_resync(uint8_t frame[ENCODER_FRAME_BYTES],
                                            encoder_channel_report_t out[ENCODER_TOTAL_CHANNELS],
                                            size_t max_scan_bytes) {
    size_t scanned = 0;
    uint8_t b0 = frame[0], b1 = frame[1];

    while (scanned < max_scan_bytes) {
        if (b0 == ENCODER_MAGIC_BYTE_0 && b1 == ENCODER_MAGIC_BYTE_1) {
            uint8_t candidate[ENCODER_FRAME_BYTES];
            candidate[0] = b0;
            candidate[1] = b1;
            if (!stm32_read_bytes(&candidate[2], ENCODER_FRAME_BYTES - 2)) {
                return false;
            }
            if (validate_and_parse_frame(candidate, out)) {
                return true;
            }
            /* CRC failed even after finding a magic-shaped header --
             * false positive in the payload data; keep scanning from
             * just past this false match. */
            b0 = candidate[ENCODER_FRAME_BYTES - 2];
            b1 = candidate[ENCODER_FRAME_BYTES - 1];
            scanned += ENCODER_FRAME_BYTES;
            continue;
        }
        b0 = b1;
        if (!stm32_read_byte(&b1)) {
            return false;
        }
        scanned++;
    }
    return false;
}

/* Fires the unified burst (0x04), reads the fixed 144-byte frame,
 * validates it, and on failure falls back to the resync scan. Populates
 * out[0..13] = SEA channels 0-6 then USM channels 0-6, straight from
 * wire position -- no group-by-group split, no channel_index
 * cross-check, unlike the old two-fire protocol this replaces. */
bool read_full_encoder_state(encoder_channel_report_t out[ENCODER_TOTAL_CHANNELS]) {
    uint8_t frame[ENCODER_FRAME_BYTES];

    stm32_send_command_byte(ENCODER_CMD_FIRE_ALL);

    if (!stm32_read_bytes(frame, ENCODER_FRAME_BYTES)) {
        return false;
    }
    if (validate_and_parse_frame(frame, out)) {
        return true;
    }

    /* Fast path failed -- resync scan, bounded to a couple of frames'
     * worth of bytes so a persistently desynced/dead link fails fast
     * rather than hanging. */
    return read_encoder_frame_with_resync(frame, out, ENCODER_FRAME_BYTES * 3);
}
