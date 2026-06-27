#ifndef I2C_IFACE_H
#define I2C_IFACE_H

#include <stdint.h>

// Pending bitstream reconfig request, set by the I2C slave command handler.
// 0 means idle; otherwise the main loop loads bitstream (reconfigure - 1)
// and clears it back to 0.
extern volatile uint32_t reconfigure;

// Set up the RP2040 as an I2C slave (address 0x17). The master enqueues a
// bitstream reconfig by writing [magic, index]; a read returns the pending
// reconfigure value (0 = idle). See https://github.com/vmilea/pico_i2c_slave.
void i2c_iface_init(void);

#endif // I2C_IFACE_H
