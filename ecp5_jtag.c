#include "ecp5_jtag.h"
#include "pio_jtag.h"
#include "cmd.h"
#include <string.h>
#include <stdlib.h>

// Use dirtyJtag command infrastructure for all JTAG operations

// Send JTAG instruction (8 bits to IR)
static void ecp5_send_instruction(pio_jtag_inst_t* jtag, uint8_t instruction) {
    // Reset TAP and go to SHIFT_IR using cmd_clk
    uint8_t clk_cmd[3];
    uint8_t dummy_response;
    
    // 5 clocks with TMS=1 to get to TEST_LOGIC_RESET
    clk_cmd[1] = SIG_TMS; // signals = TMS high
    clk_cmd[2] = 5;       // 5 clock pulses
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    // Navigate to SHIFT_IR: IDLE(0) -> SELECT_DR(1) -> SELECT_IR(1) -> CAPTURE_IR(0) -> SHIFT_IR(0)
    clk_cmd[1] = 0;       // TMS=0 to go to RUN_TEST_IDLE
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = SIG_TMS; // TMS=1 to SELECT_DR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = SIG_TMS; // TMS=1 to SELECT_IR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = 0;       // TMS=0 to CAPTURE_IR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = 0;       // TMS=0 to SHIFT_IR  
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    // Send instruction using cmd_xfer
    uint8_t xfer_cmd[3];  // cmd + length + data
    xfer_cmd[1] = 8;      // 8 bits
    xfer_cmd[2] = instruction;
    cmd_xfer(jtag, xfer_cmd, false, true, NULL); // no_read = true
    
    // Exit to RUN_TEST_IDLE
    clk_cmd[1] = SIG_TMS; // TMS=1 to EXIT1_IR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = SIG_TMS; // TMS=1 to UPDATE_IR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = 0;       // TMS=0 to RUN_TEST_IDLE
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
}

// Send data to DR (variable length)
static void ecp5_send_data(pio_jtag_inst_t* jtag, const uint8_t* data, uint32_t bits, uint8_t* response) {
    uint8_t clk_cmd[3];
    uint8_t dummy_response;
    
    // Navigate to SHIFT_DR: IDLE(1) -> SELECT_DR(0) -> CAPTURE_DR(0) -> SHIFT_DR
    clk_cmd[1] = SIG_TMS; // TMS=1 to SELECT_DR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = 0;       // TMS=0 to CAPTURE_DR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = 0;       // TMS=0 to SHIFT_DR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    // Send data using cmd_xfer (handles multi-byte data properly)
    uint32_t bytes = (bits + 7) / 8;
    uint8_t* xfer_cmd = malloc(2 + bytes);
    if (!xfer_cmd) return;
    
    xfer_cmd[1] = bits & 0xFF;  // Lower 8 bits of bit count
    // Note: cmd_xfer uses extend_length flag for bits > 255
    memcpy(&xfer_cmd[2], data, bytes);
    
    bool extend_length = bits > 255;
    bool no_read = (response == NULL);
    cmd_xfer(jtag, xfer_cmd, extend_length, no_read, response);
    
    free(xfer_cmd);
    
    // Exit to RUN_TEST_IDLE
    clk_cmd[1] = SIG_TMS; // TMS=1 to EXIT1_DR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = SIG_TMS; // TMS=1 to UPDATE_DR
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    clk_cmd[1] = 0;       // TMS=0 to RUN_TEST_IDLE
    clk_cmd[2] = 1;
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
}

uint32_t ecp5_jtag_read_id(pio_jtag_inst_t* jtag) {
    // Send READ_ID instruction
    ecp5_send_instruction(jtag, READ_ID);
    
    // Read 32-bit ID from DR
    uint8_t dummy_data[4] = {0};
    uint8_t id_bytes[4];
    ecp5_send_data(jtag, dummy_data, 32, id_bytes);
    
    // Convert to uint32_t (little-endian)
    uint32_t id = (id_bytes[3] << 24) | (id_bytes[2] << 16) | (id_bytes[1] << 8) | id_bytes[0];
    return id;
}

bool ecp5_jtag_check_busy(pio_jtag_inst_t* jtag) {
    // Send LSC_CHECK_BUSY instruction
    ecp5_send_instruction(jtag, LSC_CHECK_BUSY);
    
    // Read 1 bit busy flag from DR
    uint8_t dummy_data = 0;
    uint8_t busy_bit;
    ecp5_send_data(jtag, &dummy_data, 1, &busy_bit);
    
    return (busy_bit & 1) != 0;
}

void ecp5_jtag_enable_config(pio_jtag_inst_t* jtag) {
    // Send ISC_ENABLE instruction
    ecp5_send_instruction(jtag, ISC_ENABLE);
    
    // Send 24 bits of dummy data to DR
    uint8_t dummy_data[3] = {0x00, 0x00, 0x00};
    ecp5_send_data(jtag, dummy_data, 24, NULL);
    
    // Add some idle clocks
    uint8_t clk_cmd[3];
    uint8_t dummy_response;
    clk_cmd[1] = 0; // TMS=0, TDI=0
    clk_cmd[2] = 2; // 2 clock pulses
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
}

void ecp5_jtag_disable_config(pio_jtag_inst_t* jtag) {
    // Send ISC_DISABLE instruction
    ecp5_send_instruction(jtag, ISC_DISABLE);
    
    // Add some idle clocks
    uint8_t clk_cmd[3];
    uint8_t dummy_response;
    clk_cmd[1] = 0; // TMS=0, TDI=0
    clk_cmd[2] = 2; // 2 clock pulses
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
}

void ecp5_jtag_erase(pio_jtag_inst_t* jtag) {
    // Send ISC_ERASE instruction
    ecp5_send_instruction(jtag, ISC_ERASE);
    
    // Send erase data (erase configuration memory)
    uint8_t erase_data[3] = {0x01, 0x00, 0x00};
    ecp5_send_data(jtag, erase_data, 24, NULL);
    
    // Add some idle clocks
    uint8_t clk_cmd[3];
    uint8_t dummy_response;
    clk_cmd[1] = 0; // TMS=0, TDI=0
    clk_cmd[2] = 2; // 2 clock pulses
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
    
    // Wait for erase to complete
    while (ecp5_jtag_check_busy(jtag)) {
        clk_cmd[2] = 100; // 100 idle clocks
        cmd_clk(jtag, clk_cmd, false, &dummy_response);
    }
}

void ecp5_jtag_load_bitstream(pio_jtag_inst_t* jtag, const uint8_t* bitstream_data, uint32_t size) {
    // Send LSC_BITSTREAM_BURST instruction
    ecp5_send_instruction(jtag, LSC_BITSTREAM_BURST);
    
    // Send bitstream data to DR in chunks
    const uint32_t chunk_size = 256; // bytes
    uint32_t bytes_sent = 0;
    
    while (bytes_sent < size) {
        uint32_t bytes_to_send = (size - bytes_sent) > chunk_size ? chunk_size : (size - bytes_sent);
        
        // Send chunk using ecp5_send_data helper
        ecp5_send_data(jtag, &bitstream_data[bytes_sent], bytes_to_send * 8, NULL);
        bytes_sent += bytes_to_send;
        
        // Add idle clocks between chunks if not the last chunk
        if (bytes_sent < size) {
            uint8_t clk_cmd[3];
            uint8_t dummy_response;
            clk_cmd[1] = 0; // TMS=0, TDI=0
            clk_cmd[2] = 10; // 10 idle clocks
            cmd_clk(jtag, clk_cmd, false, &dummy_response);
        }
    }
    
    // Add final idle clocks
    uint8_t clk_cmd[3];
    uint8_t dummy_response;
    clk_cmd[1] = 0; // TMS=0, TDI=0  
    clk_cmd[2] = 100; // 100 idle clocks
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
}

void ecp5_jtag_refresh(pio_jtag_inst_t* jtag) {
    // Send LSC_REFRESH instruction
    ecp5_send_instruction(jtag, LSC_REFRESH);
    
    // Add some idle clocks
    uint8_t clk_cmd[3];
    uint8_t dummy_response;
    clk_cmd[1] = 0; // TMS=0, TDI=0
    clk_cmd[2] = 2; // 2 clock pulses
    cmd_clk(jtag, clk_cmd, false, &dummy_response);
}

bool ecp5_load_bitstream_by_name(pio_jtag_inst_t* jtag, const char* name) {
    for (int i = 0; i < bitstream_count; i++) {
        if (strcmp(bitstreams[i].name, name) == 0) {
            ecp5_jtag_enable_config(jtag);
            ecp5_jtag_erase(jtag);
            ecp5_jtag_load_bitstream(jtag, bitstreams[i].data, bitstreams[i].size);
            ecp5_jtag_disable_config(jtag);
            ecp5_jtag_refresh(jtag);
            return true;
        }
    }
    return false;
}