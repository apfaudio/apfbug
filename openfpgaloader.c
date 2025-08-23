#include "openfpgaloader.h"

#ifdef UNIT_TEST
// Stub definitions for unit testing
uint32_t cmd_handle(pio_jtag_inst_t* jtag, uint8_t* rxbuf, uint32_t count, uint8_t* tx_buf, bool local_host);
enum SignalIdentifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};
#else
#include "cmd.h"
#endif

#include <string.h>
#include <stdlib.h>

// DirtyJTAG command definitions (from dirtyJtag.cpp)
enum dirtyJtagCmd {
    CMD_STOP = 0x00,
    CMD_INFO = 0x01,
    CMD_FREQ = 0x02,
    CMD_XFER = 0x03,
    CMD_SETSIG = 0x04,
    CMD_GETSIG = 0x05,
    CMD_CLK = 0x06
};

enum CommandModifier {
    EXTEND_LENGTH = 0x40,
    NO_READ = 0x80
};

// Signal definitions (from cmd.h)
// SIG_TCK = 1 << 1, SIG_TDI = 1 << 2, SIG_TDO = 1 << 3, SIG_TMS = 1 << 4

// Core JTAG function: shift instruction register
// Based on openFPGALoader's shiftIR implementation
bool jtag_shift_ir(pio_jtag_inst_t* jtag, uint8_t instruction, tap_state_t end_state) {
    // Use cmd_xfer to shift 8-bit instruction into IR
    uint8_t cmd_buf[3];
    cmd_buf[0] = CMD_XFER;
    cmd_buf[1] = 8;  // 8 bits
    cmd_buf[2] = instruction;
    
    uint8_t dummy_response[16];
    uint32_t result = cmd_handle(jtag, cmd_buf, 3, dummy_response, true);
    
    return result > 0;
}

// Core JTAG function: shift data register
// Based on dirtyJtag.cpp writeTDI implementation
bool jtag_shift_dr(pio_jtag_inst_t* jtag, const uint8_t* tx_data, uint8_t* rx_data, 
                   int bits, tap_state_t end_state) {
    if (bits == 0) return true;
    
    uint32_t real_bit_len = bits - (end_state != TAP_SHIFT_DR ? 1 : 0);
    uint32_t kRealByteLen = (bits + 7) / 8;
    
    uint8_t* tx_cpy = malloc(kRealByteLen);
    uint8_t* tx_buf = malloc(512);
    uint8_t* rx_buf = malloc(512);
    if (!tx_cpy || !tx_buf || !rx_buf) {
        free(tx_cpy); free(tx_buf); free(rx_buf);
        return false;
    }
    
    // Prepare TX data with proper bit ordering
    if (tx_data) {
        memcpy(tx_cpy, tx_data, kRealByteLen);
    } else {
        memset(tx_cpy, 0, kRealByteLen);
    }
    
    // Process main bits (all but potentially the last)
    if (real_bit_len > 0) {
        tx_buf[0] = CMD_XFER | (rx_data ? 0 : NO_READ);
        uint16_t bit_to_send = real_bit_len;
        size_t byte_to_send = (bit_to_send + 7) / 8;
        
        // Handle bit length encoding (simplified for <= 255 bits)
        tx_buf[1] = bit_to_send & 0xFF;
        size_t header_offset = 2;
        
        // Convert to MSB-first bit order as expected by dirtyJtag
        memset(tx_buf + header_offset, 0, byte_to_send);
        for (int i = 0; i < bit_to_send; i++) {
            if (tx_cpy[i >> 3] & (1 << (i & 0x07))) {
                tx_buf[header_offset + (i >> 3)] |= (0x80 >> (i & 0x07));
            }
        }
        
        uint32_t result = cmd_handle(jtag, tx_buf, byte_to_send + header_offset, rx_data ? rx_buf : NULL, true);
        if (result == 0) {
            free(tx_cpy); free(tx_buf); free(rx_buf);
            return false;
        }
        
        // Convert received data back from MSB-first to LSB-first
        if (rx_data) {
            for (int i = 0; i < bit_to_send; i++) {
                rx_data[i >> 3] = (rx_data[i >> 3] >> 1) |
                    (((rx_buf[i >> 3] << (i&0x07)) & 0x80));
            }
        }
    }
    
    // Handle last bit with TMS transition (if needed)
    if (end_state != TAP_SHIFT_DR) {
        int pos = bits - 1;
        uint8_t last_bit = (tx_cpy[pos >> 3] & (1 << (pos & 0x07))) ? SIG_TDI : 0;
        uint8_t mask = SIG_TMS | SIG_TDI;
        uint8_t val = SIG_TMS | last_bit;
        
        if (rx_data) {
            // Use CMD_SETSIG + CMD_GETSIG for last bit with read
            mask |= SIG_TCK;
            uint8_t cmd_buf[8] = {
                CMD_SETSIG, mask, val,
                CMD_SETSIG, mask, val | SIG_TCK,
                CMD_GETSIG,
                CMD_STOP
            };
            
            uint8_t sig;
            uint32_t result = cmd_handle(jtag, cmd_buf, sizeof(cmd_buf), &sig, true);
            if (result > 0) {
                rx_data[pos >> 3] >>= 1;
                if (sig & SIG_TDO) {
                    rx_data[pos >> 3] |= (1 << (pos & 0x07));
                }
            }
        } else {
            // Use CMD_CLK for last bit without read
            uint8_t cmd_buf[4] = {
                CMD_CLK, SIG_TMS | last_bit, 1,
                CMD_STOP
            };
            cmd_handle(jtag, cmd_buf, sizeof(cmd_buf), NULL, true);
        }
    }
    
    free(tx_cpy); free(tx_buf); free(rx_buf);
    return true;
}

// Generate idle clocks
void jtag_go_idle_clocks(pio_jtag_inst_t* jtag, int clocks) {
    uint8_t cmd_buf[4];
    cmd_buf[0] = CMD_CLK;
    cmd_buf[1] = 0;  // TMS=0, TDI=0 (idle state)
    
    while (clocks > 0) {
        int clk_chunk = (clocks > 255) ? 255 : clocks;
        cmd_buf[2] = clk_chunk;
        cmd_buf[3] = CMD_STOP;
        
        uint8_t dummy[4];
        cmd_handle(jtag, cmd_buf, 4, dummy, true);
        clocks -= clk_chunk;
    }
}

// Core Lattice function: write/read operation
// Direct port of openFPGALoader's Lattice::wr_rd function
bool lattice_wr_rd(pio_jtag_inst_t* jtag, uint8_t cmd, 
                   const uint8_t* tx, int tx_len, 
                   uint8_t* rx, int rx_len) {
    int kXferLen = rx_len;
    if (tx_len > rx_len)
        kXferLen = tx_len;

    uint8_t* xfer_tx = malloc(kXferLen);
    uint8_t* xfer_rx = malloc(kXferLen);
    if (!xfer_tx || !xfer_rx) {
        free(xfer_tx);
        free(xfer_rx);
        return false;
    }
    
    memset(xfer_tx, 0, kXferLen);
    if (tx != NULL && tx_len > 0) {
        for (int i = 0; i < tx_len; i++)
            xfer_tx[i] = tx[i];
    }

    // Step 1: shiftIR(&cmd, NULL, 8, Jtag::PAUSE_IR)
    if (!jtag_shift_ir(jtag, cmd, TAP_PAUSE_IR)) {
        free(xfer_tx);
        free(xfer_rx);
        return false;
    }
    
    // Step 2: shiftDR(xfer_tx, xfer_rx, 8 * kXferLen, Jtag::PAUSE_DR)
    if (rx || tx) {
        if (!jtag_shift_dr(jtag, xfer_tx, (rx) ? xfer_rx : NULL, 8 * kXferLen, TAP_PAUSE_DR)) {
            free(xfer_tx);
            free(xfer_rx);
            return false;
        }
    }
    
    // Copy response data
    if (rx) {
        for (int i = 0; i < rx_len; i++)
            rx[i] = xfer_rx[i];
    }
    
    free(xfer_tx);
    free(xfer_rx);
    return true;
}

// Poll busy flag - direct port of openFPGALoader's pollBusyFlag
bool lattice_poll_busy_flag(pio_jtag_inst_t* jtag) {
    uint8_t rx;
    int timeout = 0;
    do {
        if (!lattice_wr_rd(jtag, LSC_CHECK_BUSY, NULL, 0, &rx, 1))
            return false;
            
        jtag_go_idle_clocks(jtag, 100);  // Some idle clocks
        
        if (timeout == 100000) {
            return false;  // timeout
        } else {
            timeout++;
        }
    } while (rx != 0);

    return true;
}

// High-level ECP5 functions - direct ports from openFPGALoader

uint32_t ecp5_jtag_read_id(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::idCode()
    uint8_t device_id[4];
    if (!lattice_wr_rd(jtag, READ_ID, NULL, 0, device_id, 4))
        return 0;
        
    return device_id[3] << 24 |
           device_id[2] << 16 |
           device_id[1] << 8  |
           device_id[0];
}

bool ecp5_jtag_check_busy(pio_jtag_inst_t* jtag) {
    // Direct port of pollBusyFlag for single check
    uint8_t rx;
    if (!lattice_wr_rd(jtag, LSC_CHECK_BUSY, NULL, 0, &rx, 1))
        return true;  // Assume busy on error
    return (rx & 1) != 0;
}

void ecp5_jtag_enable_config(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::EnableISC(0x00)
    uint8_t flash_mode = 0x00;
    lattice_wr_rd(jtag, ISC_ENABLE, &flash_mode, 1, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}

void ecp5_jtag_disable_config(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::DisableISC()
    lattice_wr_rd(jtag, ISC_DISABLE, NULL, 0, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}

void ecp5_jtag_erase(pio_jtag_inst_t* jtag) {
    // Direct port of SRAM erase from openFPGALoader
    uint8_t erase_op = FLASH_ERASE_SRAM;  // Erase SRAM only
    lattice_wr_rd(jtag, ISC_ERASE, &erase_op, 1, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}

// Helper function to reverse bits in a byte (port of ConfigBitstreamParser::reverseByte)
static uint8_t reverse_byte(uint8_t b) {
    uint8_t reversed = 0;
    for (int i = 0; i < 8; i++) {
        reversed = (reversed << 1) | (b & 1);
        b >>= 1;
    }
    return reversed;
}

void ecp5_jtag_load_bitstream(pio_jtag_inst_t* jtag, const uint8_t* bitstream_data, uint32_t size) {
    // Direct port of openFPGALoader's corrected bitstream loading implementation
    
    // Step 1: LSC_INIT_ADDRESS (0x46) - Initialize address pointer
    if (!lattice_wr_rd(jtag, 0x46, NULL, 0, NULL, 0))
        return;
    jtag_go_idle_clocks(jtag, 1000);
    
    // Step 2: LSC_BITSTREAM_BURST (0x7A) - Enter bitstream mode
    if (!lattice_wr_rd(jtag, LSC_BITSTREAM_BURST, NULL, 0, NULL, 0))
        return;
    jtag_go_idle_clocks(jtag, 2);
    
    // Step 3: Send bitstream data in chunks with byte reversal
    const uint32_t chunk_size = 1024;  // Use 1024 bytes like openFPGALoader
    uint32_t bytes_sent = 0;
    uint8_t* tmp_buffer = malloc(chunk_size);
    if (!tmp_buffer) return;
    
    while (bytes_sent < size) {
        uint32_t bytes_to_send = (size - bytes_sent > chunk_size) ? chunk_size : (size - bytes_sent);
        tap_state_t next_state = (bytes_sent + bytes_to_send >= size) ? TAP_RUN_TEST_IDLE : TAP_SHIFT_DR;
        
        // Apply byte reversal like openFPGALoader: reverseByte(data[i+ii])
        for (uint32_t ii = 0; ii < bytes_to_send; ii++) {
            tmp_buffer[ii] = reverse_byte(bitstream_data[bytes_sent + ii]);
        }
        
        // Send chunk to DR with appropriate end state
        jtag_shift_dr(jtag, tmp_buffer, NULL, bytes_to_send * 8, next_state);
        bytes_sent += bytes_to_send;
    }
    
    free(tmp_buffer);
    
    // Step 4: Final idle clocks and return to RUN_TEST_IDLE
    jtag_go_idle_clocks(jtag, 1000);
}

void ecp5_jtag_refresh(pio_jtag_inst_t* jtag) {
    // Direct port of Lattice::loadConfiguration()
    lattice_wr_rd(jtag, LSC_REFRESH, NULL, 0, NULL, 0);
    
    jtag_go_idle_clocks(jtag, 1000);
    lattice_poll_busy_flag(jtag);
}
