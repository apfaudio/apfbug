#include <stdio.h>
#include <hardware/adc.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pio_jtag.h"
#include "cdc_uart.h"
#include "led.h"
#include "bsp/board.h"
#include "tusb.h"
#include "cmd.h"
#include "get_serial.h"

#include "dirtyJtagConfig.h"

#include <i2c_fifo.h>
#include <i2c_slave.h>

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

static const uint I2C_SLAVE_SDA_PIN = 28;
static const uint I2C_SLAVE_SCL_PIN = 29;

//#define MULTICORE

static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    bool mem_address_written;
} context;

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: // master has written some data
            if (!context.mem_address_written) {
                // writes always start with the memory address
                context.mem_address = i2c_read_byte(i2c);
                context.mem_address_written = true;
            } else {
                // save into memory
                context.mem[context.mem_address] = i2c_read_byte(i2c);
                context.mem_address++;
            }
            break;
        case I2C_SLAVE_REQUEST: // master is requesting data
            // load from memory
            i2c_write_byte(i2c, context.mem[context.mem_address]);
            context.mem_address++;
            break;
        case I2C_SLAVE_FINISH: // master has signalled Stop / Restart
            context.mem_address_written = false;
            break;
        default:
            break;
    }
}

static void setup_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    // configure I2C0 for slave mode
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

void init_pins()
{
    bi_decl(bi_4pins_with_names(PIN_TCK, "TCK", PIN_TDI, "TDI", PIN_TDO, "TDO", PIN_TMS, "TMS"));
    #if !( BOARD_TYPE == BOARD_QMTECH_RP2040_DAUGHTERBOARD )
    bi_decl(bi_2pins_with_names(PIN_RST, "RST", PIN_TRST, "TRST"));
    #endif
}

pio_jtag_inst_t jtag = {
            .pio = pio0,
            .sm = 0
};

void djtag_init()
{
    init_pins();
    #if !( BOARD_TYPE == BOARD_QMTECH_RP2040_DAUGHTERBOARD )
    init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, PIN_RST, PIN_TRST);
    #else
    init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, 255, 255);
    #endif
}
typedef uint8_t cmd_buffer[64];
static uint wr_buffer_number = 0;
static uint rd_buffer_number = 0; 
typedef struct buffer_info
{
    volatile uint8_t count;
    volatile uint8_t busy;
    cmd_buffer buffer;
} buffer_info;

#define n_buffers (4)

buffer_info buffer_infos[n_buffers];

static cmd_buffer tx_buf;

void jtag_main_task()
{
#ifdef MULTICORE
    if (multicore_fifo_rvalid())
    {
        //some command processing has been done
        uint rx_num = multicore_fifo_pop_blocking();
        buffer_info* bi = &buffer_infos[rx_num];
        bi->busy = false;

    }
#endif
    if ((buffer_infos[wr_buffer_number].busy == false)) 
    {
        //If tud_task() is called and tud_vendor_read isn't called immediately (i.e before calling tud_task again)
        //after there is data available, there is a risk that data from 2 BULK OUT transaction will be (partially) combined into one
        //The DJTAG protocol does not tolerate this. 
        tud_task();// tinyusb device task
        if (tud_vendor_available())
        {
            led_rx( 1 );
            uint bnum = wr_buffer_number;
            uint count = tud_vendor_read(buffer_infos[wr_buffer_number].buffer, 64);
            if (count != 0)
            {
                buffer_infos[bnum].count = count;
                buffer_infos[bnum].busy = true;
                wr_buffer_number = wr_buffer_number + 1; //switch buffer
                if (wr_buffer_number == n_buffers)
                {
                    wr_buffer_number = 0; 
                }
#ifdef MULTICORE
                multicore_fifo_push_blocking(bnum);
#endif
            }
            led_rx( 0 );
        } else {
#if ( USB_CDC_UART_BRIDGE )           
            cdc_uart_task();
#endif
        }
    }
}

void jtag_task()
{
#ifndef MULTICORE
    jtag_main_task();
#endif
}

#ifdef MULTICORE
void core1_entry() {

    djtag_init();
    while (1)
    {
        uint rx_num = multicore_fifo_pop_blocking();
        buffer_info* bi = &buffer_infos[rx_num];
        assert (bi->busy);
        cmd_handle(&jtag, bi->buffer, bi->count, tx_buf);
        multicore_fifo_push_blocking(rx_num);
    }
 
}
#endif

void fetch_command()
{
#ifndef MULTICORE
    if (buffer_infos[rd_buffer_number].busy)
    {
        cmd_handle(&jtag, buffer_infos[rd_buffer_number].buffer, buffer_infos[rd_buffer_number].count, tx_buf, false);
        buffer_infos[rd_buffer_number].busy = false;
        rd_buffer_number++; //switch buffer
        if (rd_buffer_number == n_buffers)
        {
            rd_buffer_number = 0; 
        }
    }
#endif
}

//this is to work around the fact that tinyUSB does not handle setup request automatically
//Hence this boiler plate code
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;
    return false;
}

extern uint32_t reconfigure;

int main()
{
    board_init();
    usb_serial_init();
    tusb_init();

    gpio_init(PIN_VBUS);
    gpio_set_dir(PIN_VBUS, GPIO_IN);

    led_init( LED_INVERTED, PIN_LED_TX, PIN_LED_RX, PIN_LED_ERROR );
#if ( USB_CDC_UART_BRIDGE )
    cdc_uart_init( PIN_UART0, PIN_UART0_RX, PIN_UART0_TX );
    #if (PIN_UART_INTF_COUNT == 2)
        cdc_uart_init( PIN_UART1, PIN_UART1_RX, PIN_UART1_TX );
    #endif
#endif

#ifdef MULTICORE
    multicore_launch_core1(core1_entry);
#else 
    djtag_init();
#endif

    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);

    gpio_init(1);
    gpio_set_dir(1, GPIO_IN);
    gpio_init(2);
    gpio_set_dir(2, GPIO_IN);
    gpio_init(3);
    gpio_set_dir(3, GPIO_IN);
    gpio_init(4);
    gpio_set_dir(4, GPIO_IN);

    setup_slave();

    int mod = 0;
    while (1) {
        jtag_main_task();
        fetch_command();//for unicore implementation
        ++mod;
        if (mod > 100000) {
            uint16_t raw = adc_read();
            const float conversion_factor = 3.3f / (1<<12);
            float result = raw * conversion_factor;
            float temp = 27 - (result - 0.706)/0.001721;
            char buf[128];
            sprintf(buf, "%fdegC, gpio: %d%d%d%d mem: %x:%x:%x:%x\r\n", temp,
                    gpio_get(1),
                    gpio_get(2),
                    gpio_get(3),
                    gpio_get(4),
                    context.mem[0],
                    context.mem[1],
                    context.mem[2],
                    context.mem[3]
                    );
            tud_cdc_n_write(0, buf, strlen(buf));
            tud_cdc_n_write_flush(0);
            mod = 0;
        }
        if (reconfigure != 0) {
            uint32_t size_out = 0;
            uint8_t* buffer_out = NULL;
            if (cmd_data_for_index((reconfigure-1), &size_out, &buffer_out)) {
                replay_compressed_jtag_sequence(size_out, buffer_out);
            }
            reconfigure = 0;
        }
    }
}
