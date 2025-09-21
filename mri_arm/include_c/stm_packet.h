#ifndef STM_PACKET_H
#define STM_PACKET_H

#include <stdint.h>
#include <stddef.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// =====================
// Constants and Macros
// =====================

/**
 * @brief Number of Degrees of Freedom for the packet data.
 */
#define DOF_NUMBER 7

/**
 * @brief Length of 'extra' float line used in the packet data for debugging information.
 */
#define EXTRA_LENGTH 21


#define STM_BUFFER_SIZE 400 //256

/**
 * @brief Scale for printing floats on STM32.
 */
#define FLOAT_PRINT_SCALE 1000

#define FLOAT_PRECISION 3

#ifdef __cplusplus
extern "C" {
#endif

// =====================
// Data Structures
// =====================

/**
 * @brief Structure representing a packet to be sent/received via UART.
 */
typedef struct {
    int behavior_mode;                     ///< Operation behavior_mode
    float positions[DOF_NUMBER];           ///< Joint/motor positions
    float velocities[DOF_NUMBER];          ///< Joint/motor velocities
    float sea_positions[DOF_NUMBER];       ///< SEA (Series Elastic Actuator) positions
    float extra[EXTRA_LENGTH];             ///< Extra values, used for debugging info.
    uint8_t time_stamp;                    ///< Timestamp
} STM_Packet;


void construct_stm_packet(
    STM_Packet* packet,
    int behavior_mode,
     float* positions,
     float* velocities,
     float* sea_positions,
     float* extra,
    uint8_t time_stamp
);

// =====================
// CRC Utility
// =====================

/**
 * @brief Compute CRC-16 (XModem/CCITT) for a data buffer.
 * 
 * @param data Pointer to the input data
 * @param length Length of data in bytes
 * @return Computed 16-bit CRC
 */
uint16_t compute_crc16( uint8_t *data, uint16_t length);

bool deserialize_packet( char* buffer, STM_Packet* packet);

void serialize_packet( STM_Packet* packet, char* buffer);
// =====================
// Debug Utility
// =====================

/**
 * @brief Pretty-print the contents of an STM_Packet to the console.
 *
 * @param packet Pointer to the packet to print
 */
void print_packet( STM_Packet *packet);

/**
 * @brief Pretty-print the contents of an STM_Packet to the console, with floats presented as ints scaled by 1000.
 *
 * @param packet Pointer to the packet to print
 */
void print_packet_int( STM_Packet *packet);

#ifdef __cplusplus
}
#endif

#endif // STM_PACKET_H
