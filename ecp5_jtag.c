#include "ecp5_jtag.h"
#include "pio_jtag.h"
#include <string.h>
#include <hardware/gpio.h>

static jtag_tap_state_t current_state = STATE_TEST_LOGIC_RESET;

static const uint8_t tms_transitions[] = {
    0x01, 0xFD, 0xE3, 0xE7, 0xEF, 0x0F, 0xBF, 0x0F,
    0xFD, 0xFF, 0x3F, 0x7F, 0x7F, 0xFF, 0x7F, 0xFD
};

void ecp5_jtag_goto_state(pio_jtag_inst_t* jtag, jtag_tap_state_t target_state) {
    while (current_state != target_state) {
        uint8_t transition = tms_transitions[current_state];
        bool tms = (transition & (1 << target_state)) ? true : false;
        
        pio_jtag_write_tms_blocking(jtag, false, tms, 1);
        
        if (tms) {
            current_state = (transition >> 4) & 0x0F;
        } else {
            current_state = transition & 0x0F;
        }
    }
}

uint32_t ecp5_jtag_read_id(pio_jtag_inst_t* jtag) {
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_IR);
    
    uint8_t read_id_cmd = READ_ID;
    jtag_transfer(jtag, 8, &read_id_cmd, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_DR);
    
    uint32_t id = 0;
    uint8_t id_bytes[4] = {0};
    jtag_transfer(jtag, 32, id_bytes, id_bytes);
    
    id = (id_bytes[3] << 24) | (id_bytes[2] << 16) | (id_bytes[1] << 8) | id_bytes[0];
    
    ecp5_jtag_goto_state(jtag, STATE_RUN_TEST_IDLE);
    return id;
}

bool ecp5_jtag_check_busy(pio_jtag_inst_t* jtag) {
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_IR);
    
    uint8_t check_busy_cmd = LSC_CHECK_BUSY;
    jtag_transfer(jtag, 8, &check_busy_cmd, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_DR);
    
    uint8_t busy_bit = 0;
    jtag_transfer(jtag, 1, &busy_bit, &busy_bit);
    
    ecp5_jtag_goto_state(jtag, STATE_RUN_TEST_IDLE);
    return (busy_bit & 1) != 0;
}

void ecp5_jtag_enable_config(pio_jtag_inst_t* jtag) {
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_IR);
    
    uint8_t enable_cmd = ISC_ENABLE;
    jtag_transfer(jtag, 8, &enable_cmd, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_DR);
    
    uint8_t dummy_data[3] = {0x00, 0x00, 0x00};
    jtag_transfer(jtag, 24, dummy_data, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_RUN_TEST_IDLE);
    
    for (int i = 0; i < 2; i++) {
        pio_jtag_write_tms_blocking(jtag, false, false, 1);
    }
}

void ecp5_jtag_disable_config(pio_jtag_inst_t* jtag) {
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_IR);
    
    uint8_t disable_cmd = ISC_DISABLE;
    jtag_transfer(jtag, 8, &disable_cmd, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_RUN_TEST_IDLE);
    
    for (int i = 0; i < 2; i++) {
        pio_jtag_write_tms_blocking(jtag, false, false, 1);
    }
}

void ecp5_jtag_erase(pio_jtag_inst_t* jtag) {
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_IR);
    
    uint8_t erase_cmd = ISC_ERASE;
    jtag_transfer(jtag, 8, &erase_cmd, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_DR);
    
    uint8_t erase_data[3] = {0x01, 0x00, 0x00};  // Erase configuration memory
    jtag_transfer(jtag, 24, erase_data, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_RUN_TEST_IDLE);
    
    for (int i = 0; i < 2; i++) {
        pio_jtag_write_tms_blocking(jtag, false, false, 1);
    }
    
    while (ecp5_jtag_check_busy(jtag)) {
        for (int i = 0; i < 1000; i++) {
            pio_jtag_write_tms_blocking(jtag, false, false, 1);
        }
    }
}

void ecp5_jtag_load_bitstream(pio_jtag_inst_t* jtag, const uint8_t* bitstream_data, uint32_t size) {
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_IR);
    
    uint8_t bitstream_burst_cmd = LSC_BITSTREAM_BURST;
    jtag_transfer(jtag, 8, &bitstream_burst_cmd, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_DR);
    
    const uint32_t chunk_size = 256;
    uint32_t bytes_sent = 0;
    
    while (bytes_sent < size) {
        uint32_t bytes_to_send = (size - bytes_sent) > chunk_size ? chunk_size : (size - bytes_sent);
        
        jtag_transfer(jtag, bytes_to_send * 8, (uint8_t*)&bitstream_data[bytes_sent], NULL);
        bytes_sent += bytes_to_send;
        
        if (bytes_sent < size) {
            for (int i = 0; i < 10; i++) {
                pio_jtag_write_tms_blocking(jtag, false, false, 1);
            }
        }
    }
    
    ecp5_jtag_goto_state(jtag, STATE_RUN_TEST_IDLE);
    
    for (int i = 0; i < 100; i++) {
        pio_jtag_write_tms_blocking(jtag, false, false, 1);
    }
}

void ecp5_jtag_refresh(pio_jtag_inst_t* jtag) {
    ecp5_jtag_goto_state(jtag, STATE_SHIFT_IR);
    
    uint8_t refresh_cmd = LSC_REFRESH;
    jtag_transfer(jtag, 8, &refresh_cmd, NULL);
    
    ecp5_jtag_goto_state(jtag, STATE_RUN_TEST_IDLE);
    
    for (int i = 0; i < 2; i++) {
        pio_jtag_write_tms_blocking(jtag, false, false, 1);
    }
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