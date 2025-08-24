#ifndef BITSTREAM_ROM_H
#define BITSTREAM_ROM_H

#include <stdint.h>

// Bitstream info structure
struct bitstream_info {
    const char* name;
    const uint8_t* data;
    uint32_t size;
};

// Bitstream data (defined in raw_bitstreams.c)
extern const struct bitstream_info bitstreams[];
extern const int bitstream_count;

#endif // BITSTREAM_ROM_H