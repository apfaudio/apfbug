#include "ecp5_jtag.h"
#include "pio_jtag.h"
#include "cmd.h"
#include <string.h>
#include <stdlib.h>

// TODO: deduplicate

enum CommandIdentifier {
  CMD_STOP = 0x00,
  CMD_INFO = 0x01,
  CMD_FREQ = 0x02,
  CMD_XFER = 0x03,
  CMD_SETSIG = 0x04,
  CMD_GETSIG = 0x05,
  CMD_CLK = 0x06,
  CMD_SETVOLTAGE = 0x07,
  CMD_GOTOBOOTLOADER = 0x08
};

enum CommandModifier
{
  // CMD_XFER
  NO_READ = 0x80,
  EXTEND_LENGTH = 0x40,
  // CMD_CLK
  READOUT = 0x80,
};

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
    // Match exactly what openFPGALoader does - complete SETSIG sequence + 2 XFERs

    /*
[BUF] 01 00                                                                                                                             
[INFO] Device ID: 0x41111043                                                                                                                 
[BUF] 02 17 70 00 
[BUF] 04 12 10 04 12 12 04 12 10 04 12 12 04 12 10 04 12 12 04 12 10 04 12 12 04 12 10 0
[BUF] 04 12 00 04 12 02 04 12 10 04 12 12 04 12 00 04 12 02 04 12 00 04 12 02 04 12 00 0
[BUF] 03 20 FF FF FF FF 
[XFER] 32b -> C2 08 88 82
[BUF] 03 20 FF FF FF FF 
[XFER] 32b -> FF FF FF FF
[BUF] 04 12 10 04 12 12 04 12 10 04 12 12 04 12 10 04 12 12 04 12 10 04 12 12 04 12 10 0
*/

    // First SETSIG sequence - exactly matching openFPGALoader
    uint8_t setsig1[] = {
        0x04, 0x12, 0x10, 0x04, 0x12, 0x12, 0x04, 0x12, 0x10, 0x04, 0x12, 0x12, 
        0x04, 0x12, 0x10, 0x04, 0x12, 0x12, 0x04, 0x12, 0x10, 0x04, 0x12, 0x12, 
        0x04, 0x12, 0x10, 0x04, 0x12, 0x12, 0x04, 0x12, 0x10, 0x04, 0x12, 0x12, 
        0x04, 0x12, 0x10, 0x00
    };
    uint8_t dummy1[16];
    cmd_handle(jtag, setsig1, sizeof(setsig1), dummy1, true);
    
    // Second SETSIG sequence - exactly matching openFPGALoader  
    uint8_t setsig2[] = {
        0x04, 0x12, 0x00, 0x04, 0x12, 0x02, 0x04, 0x12, 0x10, 0x04, 0x12, 0x12, 
        0x04, 0x12, 0x00, 0x04, 0x12, 0x02, 0x04, 0x12, 0x00, 0x04, 0x12, 0x02, 
        0x04, 0x12, 0x00, 0x00
    };
    uint8_t dummy2[16];
    cmd_handle(jtag, setsig2, sizeof(setsig2), dummy2, true);
    
    // First XFER - this should return the IDCODE (C2 08 88 82)
    uint8_t xfer1[] = {0x03, 0x20, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t response1[16];
    cmd_handle(jtag, xfer1, sizeof(xfer1), response1, true);
    
    // Second XFER - openFPGALoader does this too
    uint8_t xfer2[] = {0x03, 0x20, 0xFF, 0xFF, 0xFF, 0xFF};  
    uint8_t response2[16];
    cmd_handle(jtag, xfer2, sizeof(xfer2), response2, true);
    
    // Bit-reverse each byte since JTAG shifts LSB-first
    uint8_t reversed[4];
    for (int i = 0; i < 4; i++) {
        uint8_t byte = response1[i];
        uint8_t rev = 0;
        for (int j = 0; j < 8; j++) {
            rev = (rev << 1) | (byte & 1);
            byte >>= 1;
        }
        reversed[i] = rev;
    }
    
    // Now assemble the properly bit-reversed IDCODE
    uint32_t idcode = (reversed[0] << 0) | 
                      (reversed[1] << 8) | 
                      (reversed[2] << 16) | 
                      (reversed[3] << 24);
    
    return idcode;
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
