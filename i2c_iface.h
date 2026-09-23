#ifndef I2C_IFACE_H
#define I2C_IFACE_H

#include <stdint.h>

// I2C slave at 0x17. Master writes [0x5A, index] to request a reconfig,
// reads back the pending request (0 = idle, else index + 1).
extern volatile uint32_t reconfigure;
void i2c_iface_init(void);

#endif // I2C_IFACE_H
