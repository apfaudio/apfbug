#ifndef OPENFPGALOADER_H
#define OPENFPGALOADER_H

#include <stdint.h>
#include <stdbool.h>
#include "lattice_cmds.h"

#ifdef UNIT_TEST
// Stub definitions for unit testing
typedef struct {
    void* dummy;
} pio_jtag_inst_t;
#else
#include "pio_jtag.h"
#endif

// Flash erase modes (for ISC_ERASE)
#define FLASH_ERASE_SRAM        (1<<0)
#define FLASH_ERASE_FEATURE     (1<<1)
#define FLASH_ERASE_CFG         (1<<2)
#define FLASH_ERASE_UFM         (1<<3)
#define FLASH_ERASE_ALL         0x0F

// Status register bits
#define REG_STATUS_DONE         (1 << 8)   /* Flash or SRAM Done Flag */
#define REG_STATUS_ISC_EN       (1 << 9)   /* Enable Configuration Interface */
#define REG_STATUS_BUSY         (1 << 12)  /* Busy Flag */
#define REG_STATUS_FAIL         (1 << 13)  /* Fail Flag */

// Bitstream info structure
struct bitstream_info {
    const char* name;
    const uint8_t* data;
    uint32_t size;
};

// JTAG TAP States
typedef enum {
    TAP_TEST_LOGIC_RESET = 0,
    TAP_RUN_TEST_IDLE = 1,
    TAP_SELECT_DR_SCAN = 2,
    TAP_CAPTURE_DR = 3,
    TAP_SHIFT_DR = 4,
    TAP_EXIT1_DR = 5,
    TAP_PAUSE_DR = 6,
    TAP_EXIT2_DR = 7,
    TAP_UPDATE_DR = 8,
    TAP_SELECT_IR_SCAN = 9,
    TAP_CAPTURE_IR = 10,
    TAP_SHIFT_IR = 11,
    TAP_EXIT1_IR = 12,
    TAP_PAUSE_IR = 13,
    TAP_EXIT2_IR = 14,
    TAP_UPDATE_IR = 15,
} tap_state_t;

// High-level Lattice ECP5 operations (replaces old ecp5_jtag.h interface)
uint32_t ecp5_jtag_read_id(pio_jtag_inst_t* jtag);
bool ecp5_jtag_check_busy(pio_jtag_inst_t* jtag);
void ecp5_jtag_enable_config(pio_jtag_inst_t* jtag);
void ecp5_jtag_disable_config(pio_jtag_inst_t* jtag);
void ecp5_jtag_erase(pio_jtag_inst_t* jtag);
void ecp5_jtag_load_bitstream(pio_jtag_inst_t* jtag, const uint8_t* bitstream_data, uint32_t size);
void ecp5_jtag_refresh(pio_jtag_inst_t* jtag);

// Core implementation functions
bool lattice_wr_rd(pio_jtag_inst_t* jtag, uint8_t cmd, 
                   const uint8_t* tx, int tx_len, 
                   uint8_t* rx, int rx_len);
bool lattice_poll_busy_flag(pio_jtag_inst_t* jtag);
bool jtag_shift_ir(pio_jtag_inst_t* jtag, uint8_t instruction, tap_state_t end_state);
bool jtag_shift_dr(pio_jtag_inst_t* jtag, const uint8_t* tx_data, uint8_t* rx_data, 
                   int bits, tap_state_t end_state);
void jtag_go_idle_clocks(pio_jtag_inst_t* jtag, int clocks);

// Bitstream data (defined in raw_bitstreams.c)
extern const struct bitstream_info bitstreams[];
extern const int bitstream_count;

#endif // OPENFPGALOADER_H