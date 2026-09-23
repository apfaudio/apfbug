#ifndef BITSTREAM_ROM_H
#define BITSTREAM_ROM_H

#include <stdint.h>

struct bitstream_info {
    const char* name;
    const uint8_t* data;
    uint32_t compressed_size;
    uint32_t original_size;
};

extern const struct bitstream_info bitstreams[];
extern const int bitstream_count;

#endif // BITSTREAM_ROM_H
