// Standalone sanity check for stm_comms.c's packet framing/CRC + message
// encode/decode round trip -- pure C, no LF reactor, no cmake integration.
// Not part of the firmware build; compile and run directly:
//
//   cd vel_control_mri_arm/src_c
//   gcc -o /tmp/comms_test stm_comms_test.c stm_comms.c -lm
//   /tmp/comms_test
#include "stm_comms.h"
#include <math.h>

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); g_failures++; } \
    else { printf("PASS: %s\n", msg); } \
  } while (0)

static bool floats_equal(float a, float b) {
    return fabsf(a - b) < 1e-6f;
}

static void test_command_message_round_trip(void) {
    float velocities[DOF_NUMBER] = {0.1f, -0.2f, 0.3f, -0.4f, 0.5f, -0.6f, 0.7f};
    float position_deltas[DOF_NUMBER] = {0.01f, 0.02f, -0.03f, 0.04f, -0.05f, 0.06f, -0.07f};

    CommandMessage msg;
    zero_command_message(&msg);
    construct_command_message(&msg, 1, velocities, position_deltas, 12345, 7);

    uint8_t data_buf[COMMAND_MSG_SIZE];
    int data_len = encode_command_message_to_data_buffer(&msg, data_buf);
    CHECK(data_len == COMMAND_MSG_SIZE, "CommandMessage encodes to COMMAND_MSG_SIZE bytes");

    uint8_t packet[UART_BUFFER_SIZE];
    int pkt_len = build_packet(packet, PKT_TYPE_DATA, data_buf, (uint8_t) data_len);
    CHECK(pkt_len == data_len + PACKET_OVERHEAD, "build_packet adds exactly PACKET_OVERHEAD bytes");

    CommandMessage decoded;
    bool ok = handle_command_message_packet(&decoded, packet, (size_t) pkt_len);
    CHECK(ok, "handle_command_message_packet succeeds on a well-formed packet");

    bool fields_match = decoded.behavior_mode == 1 && decoded.time_stamp == 12345 && decoded.message_index == 7;
    for (int i = 0; i < DOF_NUMBER; i++) {
        fields_match = fields_match
            && floats_equal(decoded.velocities[i], velocities[i])
            && floats_equal(decoded.position_deltas[i], position_deltas[i]);
    }
    CHECK(fields_match, "decoded CommandMessage fields match the originals");
}

static void test_state_message_round_trip(void) {
    float positions[DOF_NUMBER] = {0, 1, 2, 3, 4, 5, 6};
    float velocities[DOF_NUMBER] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f};
    float sea_positions[DOF_NUMBER] = {-0.1f, -0.2f, -0.3f, -0.4f, -0.5f, -0.6f, -0.7f};
    float sea_velocities[DOF_NUMBER] = {0.01f, 0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.07f};
    float commanded_motor_velocity[DOF_NUMBER] = {1, 1, 1, 1, 1, 1, 1};

    StateMessage msg;
    zero_state_message(&msg);
    construct_state_message(&msg, 1, positions, velocities, sea_positions, sea_velocities,
                             commanded_motor_velocity, 999, 3);

    uint8_t data_buf[STATE_MSG_SIZE];
    int data_len = encode_state_message_to_data_buffer(&msg, data_buf);

    uint8_t packet[UART_BUFFER_SIZE];
    int pkt_len = build_packet(packet, PKT_TYPE_DATA, data_buf, (uint8_t) data_len);

    StateMessage decoded;
    bool ok = handle_state_message_packet(&decoded, packet, (size_t) pkt_len);
    CHECK(ok, "handle_state_message_packet succeeds on a well-formed packet");

    bool fields_match = decoded.behavior_mode == 1 && decoded.time_stamp == 999 && decoded.message_index == 3;
    for (int i = 0; i < DOF_NUMBER; i++) {
        fields_match = fields_match
            && floats_equal(decoded.positions[i], positions[i])
            && floats_equal(decoded.velocities[i], velocities[i])
            && floats_equal(decoded.sea_positions[i], sea_positions[i])
            && floats_equal(decoded.sea_velocities[i], sea_velocities[i])
            && floats_equal(decoded.commanded_motor_velocity[i], commanded_motor_velocity[i]);
    }
    CHECK(fields_match, "decoded StateMessage fields match the originals");
}

static void test_crc_rejects_corruption(void) {
    float zeros[DOF_NUMBER] = {0};
    CommandMessage msg;
    zero_command_message(&msg);
    construct_command_message(&msg, 0, zeros, zeros, 0, 0);

    uint8_t data_buf[COMMAND_MSG_SIZE];
    int data_len = encode_command_message_to_data_buffer(&msg, data_buf);

    uint8_t packet[UART_BUFFER_SIZE];
    int pkt_len = build_packet(packet, PKT_TYPE_DATA, data_buf, (uint8_t) data_len);

    // Flip a bit in the middle of the DATA section.
    packet[10] ^= 0x01;

    CommandMessage decoded;
    bool ok = handle_command_message_packet(&decoded, packet, (size_t) pkt_len);
    // handle_command_message_packet itself doesn't check CRC (that's
    // read_packet_bulk's job on the receiving side) -- so exercise the CRC
    // check directly here, the way read_packet_bulk does internally.
    uint16_t computed = crc16_ccitt(&packet[1], (size_t) (pkt_len - 1 - PACKET_CRC_SIZE));
    uint16_t received = (uint16_t) packet[pkt_len - 2] | ((uint16_t) packet[pkt_len - 1] << 8);
    CHECK(computed != received, "CRC detects a single flipped bit in DATA");
    (void) ok;
}

static void test_version_mismatch_rejected(void) {
    float zeros[DOF_NUMBER] = {0};
    CommandMessage msg;
    zero_command_message(&msg);
    construct_command_message(&msg, 0, zeros, zeros, 0, 0);

    uint8_t data_buf[COMMAND_MSG_SIZE];
    int data_len = encode_command_message_to_data_buffer(&msg, data_buf);

    uint8_t packet[UART_BUFFER_SIZE];
    int pkt_len = build_packet(packet, PKT_TYPE_DATA, data_buf, (uint8_t) data_len);

    packet[3] = PROTOCOL_VERSION + 1; // corrupt the VERSION byte
    // Recompute CRC so this test isolates the version check, not CRC failure.
    uint16_t crc = crc16_ccitt(&packet[1], (size_t) (pkt_len - 1 - PACKET_CRC_SIZE));
    packet[pkt_len - 2] = (uint8_t) (crc & 0xFF);
    packet[pkt_len - 1] = (uint8_t) ((crc >> 8) & 0xFF);

    CommandMessage decoded;
    bool ok = handle_command_message_packet(&decoded, packet, (size_t) pkt_len);
    CHECK(!ok, "handle_command_message_packet rejects a protocol version mismatch");
}

int main(void) {
    printf("---- stm_comms round-trip sanity check ----\n");
    test_command_message_round_trip();
    test_state_message_round_trip();
    test_crc_rejects_corruption();
    test_version_mismatch_rejected();

    printf(g_failures == 0 ? "ALL PASS\n" : "SOME FAILED\n");
    return g_failures == 0 ? 0 : 1;
}
