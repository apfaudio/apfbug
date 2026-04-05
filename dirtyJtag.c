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
#include "openfpgaloader.h"

#include "dirtyJtagConfig.h"

static const uint I2C_SLAVE_ADDRESS = 0x17;
static const uint I2C_BAUDRATE = 100000; // 100 kHz

static const uint I2C_SLAVE_SDA_PIN = 28;
static const uint I2C_SLAVE_SCL_PIN = 29;

//#define MULTICORE

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

// MS OS 1.0 Extended Compat ID descriptor — associates WINUSB with interface 0
static const uint8_t ms_os_10_compat_id[] = {
    // Header (16 bytes)
    0x28, 0x00, 0x00, 0x00,  // dwLength = 40
    0x00, 0x01,              // bcdVersion = 1.00
    0x04, 0x00,              // wIndex = extended compat ID
    0x01,                    // bCount = 1 function
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // reserved
    // Function 0 (24 bytes)
    0x00,                    // bFirstInterfaceNumber = 0
    0x01,                    // bReserved (must be 1)
    'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,  // compatibleID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // subCompatibleID
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // reserved
};

// MS OS 1.0 Extended Properties descriptor (empty, like Glasgow)
static const uint8_t ms_os_10_properties[] = {
    0x0A, 0x00, 0x00, 0x00,  // dwLength = 10
    0x00, 0x01,              // bcdVersion = 1.00
    0x05, 0x00,              // wIndex = extended properties
    0x00, 0x00,              // wCount = 0
};

// Vendor code 0xEE, matching the bMS_VendorCode in the MS OS 1.0 string descriptor (index 0xEE)
#define MS_OS_10_VENDOR_CODE 0xEE
#define WEBUSB_VENDOR_CODE   0xD0

extern const uint8_t webusb_url_descriptor[];

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
    if (stage != CONTROL_STAGE_SETUP) return true;

    // MS OS 1.0 descriptor requests
    if (request->bRequest == MS_OS_10_VENDOR_CODE) {
        if (request->wIndex == 0x0004) {
            tud_control_xfer(rhport, request, (void *)ms_os_10_compat_id, sizeof(ms_os_10_compat_id));
            return true;
        }
        if (request->wIndex == 0x0005) {
            tud_control_xfer(rhport, request, (void *)ms_os_10_properties, sizeof(ms_os_10_properties));
            return true;
        }
    }

    // WebUSB URL request
    if (request->bRequest == WEBUSB_VENDOR_CODE &&
        request->wIndex == 0x0002 &&  // GET_URL
        request->wValue == 0x0001) {  // iLandingPage
        tud_control_xfer(rhport, request, (void *)webusb_url_descriptor, webusb_url_descriptor[0]);
        return true;
    }

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

    jtag_init(&jtag);

    while (1) {
        jtag_main_task();
        fetch_command();//for unicore implementation
        if (reconfigure != 0) {
            uint32_t device_id;
            uint32_t status;
            bool success = load_bitstream_by_number(&jtag, reconfigure-1, &device_id, &status);
            char device_id_msg[120];
            if (success) {
                sprintf(device_id_msg, "Device ID: 0x%08X loaded bitstream %u, Status: 0x%08X\r\n", device_id, reconfigure, status);
            } else {
                sprintf(device_id_msg, "Failed to load bitstream %u (Device ID: 0x%08X)\r\n", reconfigure, device_id);
            }
            tud_cdc_n_write(0, device_id_msg, strlen(device_id_msg));
            tud_cdc_n_write_flush(0);
            reconfigure = 0;
        }
    }
}
