#include "ecp5_jtag.h"

// Example raw bitstream storage - replace with actual bitstream data
// To generate: extract raw .bit files and convert to C arrays

static const uint8_t bitstream1_data[] = {
    // Placeholder data - replace with actual ECP5 bitstream
    0x00, 0x09, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
    0x0F, 0xF0, 0x00, 0x00, 0x01, 0x61, 0x00, 0x0D,
    // ... actual bitstream data would go here
    // This is just a minimal placeholder
};

static const uint8_t bitstream2_data[] = {
    // Placeholder data for second bitstream
    0x00, 0x09, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
    0x0F, 0xF0, 0x00, 0x00, 0x01, 0x62, 0x00, 0x0E,
    // ... actual bitstream data would go here
};

static const uint8_t bitstream3_data[] = {
    // Placeholder data for third bitstream
    0x00, 0x09, 0x0F, 0xF0, 0x0F, 0xF0, 0x0F, 0xF0,
    0x0F, 0xF0, 0x00, 0x00, 0x01, 0x63, 0x00, 0x0F,
    // ... actual bitstream data would go here
};

const struct bitstream_info bitstreams[] = {
    {"BITSTREAM1", bitstream1_data, sizeof(bitstream1_data)},
    {"BITSTREAM2", bitstream2_data, sizeof(bitstream2_data)},
    {"BITSTREAM3", bitstream3_data, sizeof(bitstream3_data)},
};

const int bitstream_count = sizeof(bitstreams) / sizeof(bitstreams[0]);