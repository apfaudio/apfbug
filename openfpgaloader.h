// This is a minimal, embedded port of the business logic from
// openFPGALoader. It's sufficient to load a bitstream to the
// ECP5's SRAM using the dirtyJtag adapter embedded in this firmware.
//
// Original project: https://github.com/trabucayre/openFPGALoader/tree/master
// Original license: Apache 2.0

#ifndef OPENFPGALOADER_H
#define OPENFPGALOADER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef UNIT_TEST
// Stub definitions for unit testing
typedef struct {
    void* dummy;
} pio_jtag_inst_t;
#else
#include "pio_jtag.h"
#endif

// Initialize static local to this module that stores the jtag handle, so
// we don't have to snake it through every single function in the module.
void jtag_init(pio_jtag_inst_t* jtag);

// Read the attached ECP5's device ID.
uint32_t ecp5_jtag_read_id(void);

// Load an entire bitstream into SRAM and run it. This performs all the
// steps needed (e.g. erase, enable config, clock in the data, disable config
// and refresh).
void ecp5_jtag_load_bitstream(const uint8_t* bitstream_data, uint32_t size);

// Read the ECP5's status register
uint64_t ecp5_jtag_read_status(void);

#ifdef UNIT_TEST
// Used for more granular unit testing
bool ecp5_jtag_check_busy(void);
void ecp5_jtag_enable_config(void);
void ecp5_jtag_disable_config(void);
void ecp5_jtag_erase(void);
void ecp5_jtag_refresh(void);
#endif

#endif // OPENFPGALOADER_H
