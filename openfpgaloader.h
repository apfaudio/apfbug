// Minimal port of the ECP5 SRAM loading path from openFPGALoader
// (https://github.com/trabucayre/openFPGALoader, Apache 2.0).

#ifndef OPENFPGALOADER_H
#define OPENFPGALOADER_H

#include <stdint.h>
#include <stdbool.h>

#include "pio_jtag.h"

void jtag_init(pio_jtag_inst_t* jtag);

uint32_t ecp5_jtag_read_id(void);

void ecp5_jtag_load_bitstream(const uint8_t* bitstream_data, uint32_t size);

uint32_t ecp5_jtag_read_status(void);

#endif // OPENFPGALOADER_H
