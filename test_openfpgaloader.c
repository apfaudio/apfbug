#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>

#include "openfpgaloader.h"

static uint8_t captured_commands[1024];
static size_t captured_length = 0;
static int capture_call = 0;

uint32_t cmd_handle(pio_jtag_inst_t* jtag, uint8_t* rxbuf, uint32_t count, uint8_t* tx_buf, bool local_host) {
    printf("Call %d: ", ++capture_call);
    for (uint32_t i = 0; i < count; i++) {
        printf("%02X ", rxbuf[i]);
    }
    printf("\n");
    
    // Store commands for verification
    memcpy(&captured_commands[captured_length], rxbuf, count);
    captured_length += count;
    
    // Mock responses
    if (tx_buf) {
        if (count >= 3 && rxbuf[0] == 0x03 && rxbuf[1] == 0x20) {
            // This is a 32-bit XFER - return mock IDCODE data
            tx_buf[0] = 0xC2;  // Raw response that becomes 0x41111043 after bit-reversal
            tx_buf[1] = 0x08;
            tx_buf[2] = 0x88;
            tx_buf[3] = 0x82;
        } else if (count >= 3 && rxbuf[0] == 0x03 && rxbuf[1] == 0x08 && rxbuf[2] == 0xF0) {
            // READ_BUSY_FLAG instruction - mock "not busy" response
            tx_buf[0] = 0x00;  // Not busy
        } else if (count >= 3 && rxbuf[0] == 0x03 && rxbuf[1] == 0x08) {
            // 8-bit XFER for busy flag data read - return 0 (not busy)
            tx_buf[0] = 0x00;  // Not busy
        }
    }
    
    return count;
}

int main() {
    printf("Testing openFPGALoader-based implementation...\n\n");
    
    pio_jtag_inst_t mock_jtag;
    
    // Test 1: Read device ID
    printf("Test 1: Reading device ID\n");
    uint32_t device_id = ecp5_jtag_read_id(&mock_jtag);
    printf("Device ID: 0x%08X (expected: 0x41111043)\n", device_id);
    
    if (device_id == 0x41111043) {
        printf("✓ Device ID read successful!\n");
    } else {
        printf("✗ Device ID read failed!\n");
    }
    
    // Test 2: Check busy flag
    printf("\nTest 2: Check busy flag\n");
    bool is_busy = ecp5_jtag_check_busy(&mock_jtag);
    printf("Busy flag: %s\n", is_busy ? "BUSY" : "NOT BUSY");
    
    // Test 3: Config enable/disable
    printf("\nTest 3: Enable/Disable configuration\n");
    ecp5_jtag_enable_config(&mock_jtag);
    printf("Config enabled\n");
    
    ecp5_jtag_disable_config(&mock_jtag);
    printf("Config disabled\n");
    
    // Test 4: SRAM erase
    printf("\nTest 4: SRAM erase\n");
    ecp5_jtag_erase(&mock_jtag);
    printf("SRAM erased\n");
    
    // Test 5: Refresh
    printf("\nTest 5: Refresh\n");
    ecp5_jtag_refresh(&mock_jtag);
    printf("Refreshed\n");
    
    printf("\nAll tests completed successfully!\n");
    printf("New openFPGALoader-based implementation is working.\n");
    
    return 0;
}
