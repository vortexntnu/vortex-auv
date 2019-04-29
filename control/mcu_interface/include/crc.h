#ifndef DEF_LIBCRC_CHECKSUM_H
#define DEF_LIBCRC_CHECKSUM_H

#include <stdint.h>

/*
 * #define CRC_POLY_xxxx
 *
 * The constants of the form CRC_POLY_xxxx define the polynomials for some well
 * known CRC calculations.
 */

#define	CRC_POLY_16 0xA001

/*
 * #define CRC_START_xxxx
 *
 * The constants of the form CRC_START_xxxx define the values that are used for
 * initialization of a CRC value for common used calculation methods.
 */

#define CRC_START_16 0x0000
#define CRC_START_MODBUS 0xFFFF

/*
 * Prototype list of global functions
 */

unsigned char *	checksum_NMEA( const unsigned char *input_str, unsigned char *result );
uint16_t crc_16( const unsigned char *input_str, size_t num_bytes );
uint16_t crc_modbus( const unsigned char *input_str, size_t num_bytes );
uint16_t update_crc_16( uint16_t crc, unsigned char c );

/*
 * Global CRC lookup tables
 */

extern const uint32_t crc_tab32[];
extern const uint64_t crc_tab64[];

#endif  // DEF_LIBCRC_CHECKSUM_H
