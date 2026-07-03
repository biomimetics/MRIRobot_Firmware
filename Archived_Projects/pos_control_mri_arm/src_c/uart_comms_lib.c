#include "../include_c/uart_comms_lib.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/select.h>
#include <sys/time.h>
#include <errno.h>

// ============================================================================
// CRC16-IBM (polynomial 0x8005, initial value 0xFFFF)
// ============================================================================
uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--)
    {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x8005 : (crc << 1);
    }
    return crc;
}

// ============================================================================
// COBS encode/decode
// ============================================================================
size_t cobsEncode(const void *data, size_t length, uint8_t *buffer)
{
    assert(data && buffer);
    uint8_t *encode = buffer;
    uint8_t *codep = encode++;
    uint8_t code = 1;

    for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte)
    {
        if (*byte)
        {
            *encode++ = *byte;
            ++code;
        }
        if (!*byte || code == 0xFF)
        {
            *codep = code;
            code = 1;
            codep = encode;
            if (!*byte || length)
                ++encode;
        }
    }
    *codep = code;
    return (size_t)(encode - buffer);
}

size_t cobsDecode(const uint8_t *buffer, size_t length, void *data)
{
    assert(buffer && data);
    const uint8_t *byte = buffer;
    uint8_t *decode = (uint8_t *)data;

    for (uint8_t code = 0xFF, block = 0; byte < buffer + length; --block)
    {
        if (block)
        {
            *decode++ = *byte++;
        }
        else
        {
            block = *byte++;
            if (block && (code != 0xFF))
                *decode++ = 0;
            code = block;
            if (!code)
                break;
        }
    }
    return (size_t)(decode - (uint8_t *)data);
}

// ============================================================================
// UART simulation helpers
// ============================================================================
void uart_send_bytes(const uint8_t *data, size_t len)
{
    printf("UART TX (%zu bytes): ", len);
    for (size_t i = 0; i < len; i++)
        printf("%02X ", data[i]);
    printf("\n");
}

// ============================================================================
// Generic encode/decode using CRC16
// ============================================================================
size_t encodePacket(PacketType type, const void *payload, size_t payloadLen,
                    uint8_t *encodedBuf)
{
    uint8_t raw[UART_MAX_PACKET];
    assert(payloadLen + 3 <= sizeof(raw)); // type + payload + CRC16

    raw[0] = (uint8_t)type;
    memcpy(&raw[1], payload, payloadLen);

    uint16_t crc = crc16(raw, 1 + payloadLen);

    raw[1 + payloadLen]     = (uint8_t)(crc & 0xFF);   // CRC low byte
    raw[1 + payloadLen + 1] = (uint8_t)(crc >> 8);     // CRC high byte

    size_t encLen = cobsEncode(raw, 1 + payloadLen + 2, encodedBuf);
    encodedBuf[encLen++] = 0x00; // frame delimiter
    return encLen;
}

int decodePacket(const uint8_t *encoded, size_t encodedLen,
                 PacketType *typeOut, void *payloadOut, size_t payloadMax,
                 size_t *payloadLenOut)
{
    uint8_t raw[UART_MAX_PACKET];
    size_t decLen = cobsDecode(encoded, encodedLen - 1, raw); // remove delimiter

    if (decLen < 4)
        return -1; // too short

    uint16_t crcRx = (uint16_t)raw[decLen - 2] | ((uint16_t)raw[decLen - 1] << 8);
    uint16_t crcCalc = crc16(raw, decLen - 2);
    if (crcRx != crcCalc)
        return -2; // CRC mismatch

    *typeOut = (PacketType)raw[0];
    *payloadLenOut = decLen - 3; // exclude type + CRC16

    if (*payloadLenOut > payloadMax)
        return -3;

    memcpy(payloadOut, &raw[1], *payloadLenOut);
    return 0;
}



// =========================================================================
// Wait for UART data (same as before)
// =========================================================================
static int wait_for_data(int fd, int timeout_ms)
{
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int result = select(fd + 1, &read_fds, NULL, NULL, &timeout);
    if (result < 0)
        perror("select");

    return result; // >0 = ready, 0 = timeout, <0 = error
}

// =========================================================================
// Read COBS + CRC16 packet (no start byte, 0x00 delimiter only)
// =========================================================================
int read_packet_cobs_crc16(int fd, uint8_t *buf, size_t buf_size, int timeout_ms)
{
    if (!buf || buf_size < 8) {
        fprintf(stderr, "Invalid buffer\n");
        return -1;
    }

    uint8_t byte;
    size_t offset = 0;
    int n;

    // --- Step 1: Collect bytes until delimiter (0x00) ---
    while (1)
    {
        if (wait_for_data(fd, timeout_ms) <= 0)
            return -1; // timeout or error

        n = read(fd, &byte, 1);
        if (n <= 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                continue;
            perror("read");
            return -1;
        }

        if (byte == 0x00)
        {
            // End of frame
            break;
        }

        if (offset >= buf_size)
        {
            fprintf(stderr, "Packet overflow (> %zu bytes)\n", buf_size);
            return -1;
        }

        buf[offset++] = byte;
    }

    if (offset == 0)
        return -1; // empty frame

    // --- Step 2: Decode COBS ---
    uint8_t decoded[1024];
    size_t decodedLen = cobsDecode(buf, offset, decoded);

    if (decodedLen < 3)
    {
        fprintf(stderr, "COBS decode too short (%zu)\n", decodedLen);
        return -1;
    }

    // --- Step 3: Verify CRC16 ---
    uint16_t crc_rx = (uint16_t)decoded[decodedLen - 2] |
                      ((uint16_t)decoded[decodedLen - 1] << 8);
    uint16_t crc_calc = crc16(decoded, decodedLen - 2);

    if (crc_rx != crc_calc)
    {
        fprintf(stderr, "CRC mismatch: calc=0x%04X, recv=0x%04X\n",
                crc_calc, crc_rx);

        fprintf(stderr, "Decoded (%zu bytes): ", decodedLen);
        for (size_t i = 0; i < decodedLen; i++)
            fprintf(stderr, "%02X ", decoded[i]);
        fprintf(stderr, "\n");

        return -2;
    }

    // --- Step 4: Copy payload (excluding CRC16) ---
    size_t payloadLen = decodedLen - 2;
    if (payloadLen > buf_size)
        payloadLen = buf_size;

    memcpy(buf, decoded, payloadLen);
    return (int)payloadLen;
}