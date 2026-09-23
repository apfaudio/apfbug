#include "i2c_iface.h"

#include <pico/stdlib.h>
#include <hardware/gpio.h>
#include <hardware/i2c.h>
#include <i2c_fifo.h>
#include <i2c_slave.h>

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000;

static const uint I2C_SLAVE_SDA_PIN = 28;
static const uint I2C_SLAVE_SCL_PIN = 29;

#define RECONFIG_MAGIC 0x5A

volatile uint32_t reconfigure = 0;

static bool magic_seen = false;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            uint8_t byte = i2c_read_byte(i2c);
            if (magic_seen) {
                reconfigure = byte + 1;
                magic_seen = false;
            } else {
                magic_seen = (byte == RECONFIG_MAGIC);
            }
            break;
        }
        case I2C_SLAVE_REQUEST:
            i2c_write_byte(i2c, (uint8_t)reconfigure);
            break;
        case I2C_SLAVE_FINISH:
            magic_seen = false;
            break;
        default:
            break;
    }
}

void i2c_iface_init(void) {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}
