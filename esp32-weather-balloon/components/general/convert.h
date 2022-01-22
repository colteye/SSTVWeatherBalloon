#ifndef _CONVERT_H
#define _CONVERT_H

// Convert from two uint8_ts into a uint16_t.
#define U8S_2_U16(b0, b1) (((uint16_t)b0 << 8) | b1)

// Convert from four uint8_ts into a uint32_t.
#define U8S_2_U32(b0, b1, b2, b3) (((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3)

// Convert from a uint16_t to alternate form (Helpful for the BMP280).
#define U16_2_U16_LE(b) ((b >> 8) | (b << 8))

#endif // _CONVERT_H