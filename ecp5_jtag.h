#ifndef ECP5_JTAG_H
#define ECP5_JTAG_H

#include <stdint.h>
#include <stdbool.h>
#include "pio_jtag.h"

enum lattice_ecp5_cmd {
    ISC_NOOP = 0xFF,
    READ_ID = 0xE0,
    USERCODE = 0xC0,
    LSC_READ_STATUS = 0x3C,
    LSC_CHECK_BUSY = 0xF0,
    LSC_REFRESH = 0x79,
    ISC_ENABLE = 0xC6,
    ISC_ENABLE_X = 0x74,
    ISC_DISABLE = 0x26,
    ISC_PROGRAM_USERCODE = 0xC2,
    ISC_ERASE = 0x0E,
    ISC_PROGRAM_DONE = 0x5E,
    ISC_PROGRAM_SECURITY = 0xCE,
    LSC_INIT_ADDRESS = 0x46,
    LSC_WRITE_ADDRESS = 0xB4,
    LSC_BITSTREAM_BURST = 0x7A,
    LSC_PROG_INCR_RTI = 0x82,
    LSC_PROG_INCR_ENC = 0xB6,
    LSC_PROG_INCR_CMP = 0xB8,
    LSC_PROG_INCR_CNE = 0xBA,
    LSC_VERIFY_INCR_RTI = 0x6A,
    LSC_PROG_CTRL0 = 0x22,
    LSC_READ_CTRL0 = 0x20,
    LSC_RESET_CRC = 0x3B,
    LSC_READ_CRC = 0x60,
    LSC_PROG_SED_CRC = 0xA2,
    LSC_READ_SED_CRC = 0xA4,
    LSC_PROG_PASSWORD = 0xF1,
    LSC_READ_PASSWORD = 0xF2,
    LSC_SHIFT_PASSWORD = 0xBC,
    LSC_PROG_CIPHER_KEY = 0xF3,
    LSC_READ_CIPHER_KEY = 0xF4,
    LSC_PROG_FEATURE = 0xE4,
    LSC_READ_FEATURE = 0xE7,
    LSC_PROG_FEABITS = 0xF8,
    LSC_READ_FEABITS = 0xFB,
    LSC_PROG_OTP = 0xF9,
    LSC_READ_OTP = 0xFA,
};

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

struct device_id_pair {
    const char* device_name;
    uint32_t device_id;
};

extern const struct device_id_pair ecp5_devices[];
extern const int ecp5_device_count;

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