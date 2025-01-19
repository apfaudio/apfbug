/*
 * This file is part of the Black Magic Debug project.
 *
 * Based on work that is Copyright (C) 2017 Black Sphere Technologies Ltd.
 * Copyright (C) 2017 Dave Marples <dave@marples.net>
 * Copyright (C) 2023 Patrick H Dussud
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.	 If not, see <http://www.gnu.org/licenses/>.
 */
#include "dirtyJtagConfig.h"

#if ( USB_CDC_UART_BRIDGE )

#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include "led.h"
#include "tusb.h"
#include "cdc_uart.h"
#include "cmd.h"

static struct uart_device
{
	uint index;
	uart_inst_t *inst;
	uint8_t *tx_buf;
	volatile uint8_t rx_buf[RX_BUFFER_SIZE];
	uint rx_dma_channel;
	uint tx_dma_channel;
	volatile uint8_t *tx_write_address;
	uint8_t *rx_read_address;
	uint n_checks;
	uint is_connected;
} uart_devices[2];

static uint8_t tx_bufs[2][TX_BUFFER_SIZE] __attribute__((aligned(TX_BUFFER_SIZE)));
static bool cdc_stopped = true; // is CDC connected over USB

// Rx/Tx Buffer handling functions (depend on USB connection state)
static void dma_handler();
static void handle_rx_buffer(struct uart_device *uart);
static void handle_tx_buffer(struct uart_device *uart);
static void process_connected_rx(struct uart_device *uart, uint32_t available_space);
static void process_disconnected_rx(struct uart_device *uart, uint32_t available_space);
static void update_read_pointer(struct uart_device *uart, uint32_t increment);

// Reboot intercept TODO cleanup
static char *code = "BITSTREAM";
static size_t code_len   = 9;
static size_t code_index = 0;
static bool next_is_cmd = false;
uint32_t reconfigure = 0;

static uint n_bits(uint n)
{
	int i;
	for (i = 0; i < 31; i++)
	{
		n >>= 1;
		if (n == 0)
		break;
	}
	return i+1;
}

uint setup_usart_tx_dma(uart_inst_t *uart, void *tx_address, uint buffer_size)
{
	uint dma_chan = dma_claim_unused_channel(true);
	// Tell the DMA to raise IRQ line 0 when the channel finishes a block
	dma_channel_set_irq1_enabled(dma_chan, true);
	// enable DMA TX
	hw_write_masked(&uart_get_hw(uart)->dmacr, 1 << UART_UARTDMACR_TXDMAE_LSB, UART_UARTDMACR_TXDMAE_BITS);

	dma_channel_config c = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, true);
	channel_config_set_write_increment(&c, false);
	channel_config_set_dreq(&c, uart_get_dreq(uart, true));
	channel_config_set_ring(&c, false, n_bits(buffer_size - 1));
	dma_channel_configure(
		dma_chan,
		&c,
		&uart_get_hw(uart)->dr, // Write Ad
		tx_address,             // Read Address
		0,                      // transfer count
		false                   // start
	);
	return dma_chan;
}

uint setup_usart_rx_dma(uart_inst_t *uart, volatile void *rx_address, irq_handler_t handler, uint buffer_size)
{
	uint dma_chan = dma_claim_unused_channel(true);
	// Tell the DMA to raise IRQ line 0 when the channel finishes a block
	dma_channel_set_irq1_enabled(dma_chan, true);

	// Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
	irq_add_shared_handler(DMA_IRQ_1, handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
	irq_set_enabled(DMA_IRQ_1, true);
	// enable DMA RX
	hw_write_masked(&uart_get_hw(uart)->dmacr, 1 << UART_UARTDMACR_RXDMAE_LSB, UART_UARTDMACR_RXDMAE_BITS);

	dma_channel_config c = dma_channel_get_default_config(dma_chan);
	channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
	channel_config_set_read_increment(&c, false);
	channel_config_set_write_increment(&c, true);
	channel_config_set_dreq(&c, uart_get_dreq(uart, false));
	hw_clear_bits(&uart_get_hw(uart)->rsr, UART_UARTRSR_BITS); // clear
	dma_channel_configure(
		dma_chan,
		&c,
		rx_address,				// Write Address
		&uart_get_hw(uart)->dr, // Read Address
		buffer_size,			// transfer count
		true					// start
	);
	return dma_chan;
}

void cdc_uart_init( uart_inst_t *const uart_, int uart_rx_pin, int uart_tx_pin )  {
	uint uart_index;
	uart_index = uart_get_index(uart_);
    struct uart_device *uart;
	uart = &uart_devices[uart_index];

	gpio_set_function(uart_tx_pin, GPIO_FUNC_UART);
	gpio_set_function(uart_rx_pin, GPIO_FUNC_UART);
	gpio_set_pulls(uart_tx_pin, 1, 0);
	gpio_set_pulls(uart_rx_pin, 1, 0);

	uart->inst = uart_;
	uart->index = uart_index;
	uart_init(uart->inst, USBUSART_BAUDRATE);
	uart_set_hw_flow(uart->inst, false, false);
	uart_set_format(uart->inst, 8, 1, UART_PARITY_NONE);
	uart_set_fifo_enabled(uart->inst, true);
	uart->tx_buf = tx_bufs[uart_index];
	uart->tx_dma_channel = setup_usart_tx_dma(uart->inst, uart->tx_buf, TX_BUFFER_SIZE);
	uart->rx_dma_channel = setup_usart_rx_dma(uart->inst, &uart->rx_buf[0], dma_handler, RX_BUFFER_SIZE);
	uart->tx_write_address = uart->tx_buf;
	uart->rx_read_address = (uint8_t *)&uart->rx_buf[0];
	uart->n_checks = 0; 
}

// shared between rx_dma_channel and tx_dma_channel

static void dma_handler()
{
	volatile uint32_t ints = dma_hw->ints1;
	struct uart_device *uart;

	for (size_t i = 0; i < PIN_UART_INTF_COUNT; i++)
	{
		uart = &uart_devices[i];
		if (ints & (1 << uart->rx_dma_channel)) //dma_channel_hw_addr(rx_dma_channel)->transfer_count == 0) // dma_channel_get_irq1_status(rx_dma_channel))
		{
			dma_channel_set_write_addr(uart->rx_dma_channel, &uart->rx_buf[0], true);
		}
		if (ints & (1 << uart->tx_dma_channel)) //(dma_channel_hw_addr(tx_dma_channel)->transfer_count == 0) // (dma_channel_get_irq1_status(tx_dma_channel))
		{
			uint8_t *ra = (uint8_t *)(dma_channel_hw_addr(uart->tx_dma_channel)->read_addr);
			// cdc_uart_task can modify uart->tx_write_address. cache it locally
			volatile uint8_t *l_tx_write_address = uart->tx_write_address;
			size_t space = (l_tx_write_address >= ra) ? (l_tx_write_address - ra) : (l_tx_write_address + TX_BUFFER_SIZE - ra);
			if (space > 0)
				dma_channel_set_trans_count(uart->tx_dma_channel, space, true);
		}
	}
	dma_hw->ints1 = ints;
}

void intercept_uart(struct uart_device *uart, volatile uint8_t *buffer, uint32_t size)
{
    for (size_t i = 0; i != size; ++i) {
        if (next_is_cmd) {
            reconfigure = 1 + (buffer[i] - '0');
            char *hello = "\r\n[REBOOT]\r\n";
            tud_cdc_n_write(uart->index, hello, strlen(hello));
            tud_cdc_n_write(uart->index, &buffer[i], 1);
            tud_cdc_n_write_flush(uart->index);
            next_is_cmd = false;
        }
        if (buffer[i] == code[code_index]) {
            if (code_index == (code_len-1)) {
                next_is_cmd = true;
                code_index = 0;
            } else {
                ++code_index;
            }
        } else {
            code_index = 0;
        }
    }
}

void handle_rx_buffer(struct uart_device *uart) {
    volatile uint8_t *write_addr = (uint8_t*)(dma_channel_hw_addr(uart->rx_dma_channel)->write_addr);
    if (write_addr == &uart->rx_buf[RX_BUFFER_SIZE]) {
        write_addr = &uart->rx_buf[0];
    }

    uint32_t available_space = (write_addr >= uart->rx_read_address) ? 
        (write_addr - uart->rx_read_address) : 
        (write_addr + RX_BUFFER_SIZE - uart->rx_read_address);

    uart->n_checks++;

    bool should_process = (available_space >= FULL_SWO_PACKET) || 
        (available_space != 0 && uart->n_checks > 4);

    if (!should_process) return;

    uart->n_checks = 0;

    if (uart->is_connected) {
        process_connected_rx(uart, available_space);
    } else {
        process_disconnected_rx(uart, available_space);
    }
}

void process_connected_rx(struct uart_device *uart, uint32_t available_space) {
    led_tx(1);
    uint32_t cdc_capacity = tud_cdc_n_write_available(uart->index);
    uint32_t transfer_size = MIN(available_space, cdc_capacity);

    if (cdc_capacity >= FULL_SWO_PACKET) {
        uint32_t written = tud_cdc_n_write(uart->index, uart->rx_read_address, transfer_size);
        if (available_space < FULL_SWO_PACKET) {
            tud_cdc_n_write_flush(uart->index);
        }
        tud_task();
        intercept_uart(uart, uart->rx_read_address, written);
        update_read_pointer(uart, written);
    }
    led_tx(0);
}

void process_disconnected_rx(struct uart_device *uart, uint32_t available_space) {
    intercept_uart(uart, uart->rx_read_address, available_space);
    update_read_pointer(uart, available_space);
}

void handle_tx_buffer(struct uart_device *uart) {
    uint32_t available = tud_cdc_n_available(uart->index);
    size_t max_write = &uart->tx_buf[TX_BUFFER_SIZE] - uart->tx_write_address;
    size_t watermark = MIN(available, max_write);

    if (watermark == 0) return;

    led_rx(1);
    size_t tx_len = tud_cdc_n_read(uart->index, (void*)uart->tx_write_address, watermark);

    // Update write pointer
    volatile uint8_t *new_write_addr = uart->tx_write_address + tx_len;
    uart->tx_write_address = (new_write_addr >= &uart->tx_buf[TX_BUFFER_SIZE]) ? 
        new_write_addr - TX_BUFFER_SIZE : new_write_addr;

    // Restart DMA if needed
    if (!dma_channel_is_busy(uart->tx_dma_channel)) {
        uint8_t *read_addr = (uint8_t *)(dma_channel_hw_addr(uart->tx_dma_channel)->read_addr);
        size_t space = (uart->tx_write_address >= read_addr) ? 
            (uart->tx_write_address - read_addr) : 
            (uart->tx_write_address + TX_BUFFER_SIZE - read_addr);

        if (space > 0) {
            dma_channel_set_trans_count(uart->tx_dma_channel, space, true);
        }
    }
    led_rx(0);
}

void update_read_pointer(struct uart_device *uart, uint32_t increment) {
    uart->rx_read_address += increment;
    if (uart->rx_read_address >= &uart->rx_buf[RX_BUFFER_SIZE]) {
        uart->rx_read_address -= RX_BUFFER_SIZE;
    }
}

void cdc_uart_task(void) {
    for (size_t i = 0; i < PIN_UART_INTF_COUNT; i++) {
        struct uart_device *uart = &uart_devices[i];
        uart->is_connected = tud_cdc_n_connected(uart->index) && !cdc_stopped;

        if (!uart->is_connected) {
            tud_cdc_n_write_clear(uart->index);
        }

        handle_rx_buffer(uart);

        if (uart->is_connected) {
            handle_tx_buffer(uart);
        }
    }
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* line_coding)
{
	struct uart_device *uart;

    /* FIXME: line speed changes breaks bitstream reboot/keyword
     * detection in specific case where terminal was opened and closed
     * but USB remains connected (disconnected USB and connected usb
     * w/terminal open still work).
     *
     * https://github.com/apfaudio/apfbug/issues/2
     *
     * To sidestep this bug, disallow baudrate changes for now.
     */
    /*
	for (size_t i = 0; i < PIN_UART_INTF_COUNT; i++)
	{
		uart = &uart_devices[i];
		if (uart->index == itf)
		{
			uart_deinit(uart->inst);
			tud_cdc_n_write_clear(itf);
			tud_cdc_n_read_flush(itf);
			uart_init(uart->inst, line_coding->bit_rate);
		}
	}
    */

    // 1200 baud bootloader reentry magic
    if (line_coding->bit_rate == 1200) {
        rom_reset_usb_boot(0, 0);
    }
}

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts)
{
    if (dtr) {
        cdc_stopped = false;
    } else {
        cdc_stopped = true;
    }
}


#endif // USB_CDC_UART_BRIDGE
