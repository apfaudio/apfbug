#ifndef ECP5_JTAG_H
#define ECP5_JTAG_H

#include <stdint.h>
#include <stdbool.h>
#include "pio_jtag.h"
#include "lattice_cmds.h"

typedef enum {
    STATE_TEST_LOGIC_RESET = 0,
    STATE_RUN_TEST_IDLE = 1,
    STATE_SELECT_DR_SCAN = 2,
    STATE_CAPTURE_DR = 3,
    STATE_SHIFT_DR = 4,
    STATE_EXIT1_DR = 5,
    STATE_PAUSE_DR = 6,
    STATE_EXIT2_DR = 7,
    STATE_UPDATE_DR = 8,
    STATE_SELECT_IR_SCAN = 9,
    STATE_CAPTURE_IR = 10,
    STATE_SHIFT_IR = 11,
    STATE_EXIT1_IR = 12,
    STATE_PAUSE_IR = 13,
    STATE_EXIT2_IR = 14,
    STATE_UPDATE_IR = 15
} jtag_tap_state_t;

struct bitstream_info {
    const char* name;
    const uint8_t* data;
    uint32_t size;
};

extern const struct bitstream_info bitstreams[];
extern const int bitstream_count;

void ecp5_jtag_goto_state(pio_jtag_inst_t* jtag, jtag_tap_state_t target_state);
uint32_t ecp5_jtag_read_id(pio_jtag_inst_t* jtag);
bool ecp5_jtag_check_busy(pio_jtag_inst_t* jtag);
void ecp5_jtag_enable_config(pio_jtag_inst_t* jtag);
void ecp5_jtag_disable_config(pio_jtag_inst_t* jtag);
void ecp5_jtag_erase(pio_jtag_inst_t* jtag);
void ecp5_jtag_load_bitstream(pio_jtag_inst_t* jtag, const uint8_t* bitstream_data, uint32_t size);
void ecp5_jtag_refresh(pio_jtag_inst_t* jtag);
bool ecp5_load_bitstream_by_name(pio_jtag_inst_t* jtag, const char* name);

#endif // ECP5_JTAG_H
