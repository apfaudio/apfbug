/*
  Copyright (c) 2017 Jean THOMAS.
  Copyright (c) 2020-2022 Patrick Dussud
  
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software
  is furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <pico/stdlib.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>

#include "jtag.pio.h"
#include "tusb.h"
#include "pio_jtag.h"
#include "cmd.h"
#include "cdc_uart.h"

#include "heatshrink_encoder.h"
#include "heatshrink_decoder.h"


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

enum SignalIdentifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};

#define CMD_BUFFER_SZ 3*32768
static uint8_t cmd_buffer[CMD_BUFFER_SZ];
static uint32_t cmd_buffer_index = 0;
static uint32_t cmd_buffer_sunk = 0;
static uint32_t cmd_buffer_count = 0;
static bool hse_init = false;

/**
 * @brief Handle CMD_INFO command
 *
 * CMD_INFO returns a string to the host software. This
 * could be used to check DirtyJTAG firmware version
 * or supported commands.
 *
 * @param usbd_dev USB device
 */
static uint32_t  cmd_info(uint8_t *buffer);

/**
 * @brief Handle CMD_FREQ command
 *
 * CMD_FREQ sets the clock frequency on the probe.
 * Currently this does not changes anything.
 *
 * @param commands Command data
 */
static void cmd_freq(pio_jtag_inst_t* jtag, const uint8_t *commands);

/**
 * @brief Handle CMD_XFER command
 *
 * CMD_XFER reads and writes data simultaneously.
 *
 * @param usbd_dev USB device
 * @param commands Command data
 */
static uint32_t cmd_xfer(pio_jtag_inst_t* jtag, const uint8_t *commands, bool extend_length, bool no_read, uint8_t* tx_buf);

/**
 * @brief Handle CMD_SETSIG command
 *
 * CMD_SETSIG set the logic state of the JTAG signals.
 *
 * @param commands Command data
 */
static void cmd_setsig(pio_jtag_inst_t* jtag, const uint8_t *commands);

/**
 * @brief Handle CMD_GETSIG command
 *
 * CMD_GETSIG gets the current signal state.
 * 
 * @param usbd_dev USB device
 */
static uint32_t cmd_getsig(pio_jtag_inst_t* jtag, uint8_t *buffer);

/**
 * @brief Handle CMD_CLK command
 *
 * CMD_CLK sends clock pulses with specific TMS and TDI state.
 *
 * @param usbd_dev USB device
 * @param commands Command data
 * @param readout Enable TDO readout
 */
static uint32_t cmd_clk(pio_jtag_inst_t *jtag, const uint8_t *commands, bool readout, uint8_t *buffer);
/**
 * @brief Handle CMD_SETVOLTAGE command
 *
 * CMD_SETVOLTAGE sets the I/O voltage for devices that support this feature.
 *
 * @param commands Command data
 */
static void cmd_setvoltage(const uint8_t *commands);

/**
 * @brief Handle CMD_GOTOBOOTLOADER command
 *
 * CMD_GOTOBOOTLOADER resets the MCU and enters its bootloader (if installed)
 */
static void cmd_gotobootloader(void);

static heatshrink_encoder hse;
static heatshrink_decoder hsd;

void uputc(char c) {
    uint uart_index = uart_get_index(PIN_UART1);
    while (0 == tud_cdc_n_write(uart_index, &c, 1)) {
        tud_task();
    }
}

void uputs(const char *c) {
    while (*c != '\0') {
        uputc(*c);
        ++c;
    }
}


uint32_t cmd_handle(pio_jtag_inst_t* jtag, uint8_t* rxbuf, uint32_t count, uint8_t* tx_buf, bool local_host) {

  uint8_t *commands= (uint8_t*)rxbuf;
  uint8_t *output_buffer = tx_buf;

  if (!local_host) {
      if (!hse_init) {
        heatshrink_encoder_reset(&hse);
        hse_init = true;
      }

      if ((cmd_buffer_index + count) < CMD_BUFFER_SZ) {
        size_t n_sunk = 0;
        size_t total_sunk = 0;
        size_t n_polled = 0;
        while (total_sunk != count) {
            heatshrink_encoder_sink(&hse, &rxbuf[total_sunk], count-total_sunk, &n_sunk);
            total_sunk += n_sunk;
            cmd_buffer_sunk += n_sunk;
            heatshrink_encoder_poll(&hse, &cmd_buffer[cmd_buffer_index],
                                    CMD_BUFFER_SZ-cmd_buffer_index, &n_polled);
            cmd_buffer_index += n_polled;
        }
      } else {
        cmd_buffer_index = (CMD_BUFFER_SZ - 1);
      }

      cmd_buffer_count += count;
  }

  while ((commands < (rxbuf + count)) && (*commands != CMD_STOP))
  {
    switch ((*commands)&0x0F) {
    case CMD_INFO:
    {
      if (!local_host) {
          uint32_t trbytes = cmd_info(output_buffer);
          output_buffer += trbytes;
      }
      break;
    }
    case CMD_FREQ:
      cmd_freq(jtag, commands);
      commands += 2;
      break;

    case CMD_XFER:
    {
      bool no_read = *commands & NO_READ;
      uint32_t trbytes = cmd_xfer(jtag, commands, *commands & EXTEND_LENGTH, no_read, output_buffer);
      commands += 1 + trbytes;
      output_buffer += (no_read ? 0 : trbytes);
      break;
    }
    case CMD_SETSIG:
      cmd_setsig(jtag, commands);
      commands += 2;
      break;

    case CMD_GETSIG:
    {
      uint32_t trbytes = cmd_getsig(jtag, output_buffer);
      output_buffer += trbytes;
      break;
    }
    case CMD_CLK:
    {
      uint32_t trbytes = cmd_clk(jtag, commands, !!(*commands & READOUT), output_buffer);
      output_buffer += trbytes;
      commands += 2;
      break;
    }
    case CMD_SETVOLTAGE:
      cmd_setvoltage(commands);
      commands += 1;
      break;

    case CMD_GOTOBOOTLOADER:
      cmd_gotobootloader();
      break;

    default:
      uputc(*commands);
      break;
    }

    commands++;
  }

  /* Send the transfer response back to host */
  if ((tx_buf != output_buffer) && !local_host)
  {
    tud_vendor_write(tx_buf, output_buffer - tx_buf);
    tud_vendor_flush();
  }

  // Count CMD_STOP as a recieved command.
  if (*commands == CMD_STOP) {
      ++commands;
  }

  return commands - rxbuf;
}

static const unsigned char base64_table[65] =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void base64_print(
    const unsigned char *src, size_t len)
{
	const unsigned char *end, *in;
	int line_len;

	end = src + len;
	in = src;
	line_len = 0;
	while (end - in >= 3) {
		uputc(base64_table[in[0] >> 2]);
		uputc(base64_table[((in[0] & 0x03) << 4) | (in[1] >> 4)]);
		uputc(base64_table[((in[1] & 0x0f) << 2) | (in[2] >> 6)]);
		uputc(base64_table[in[2] & 0x3f]);
		in += 3;
		line_len += 4;
		if (line_len >= 72) {
			uputc('\r');
			uputc('\n');
			line_len = 0;
		}
	}

	if (end - in) {
		uputc(base64_table[in[0] >> 2]);
		if (end - in == 1) {
			uputc(base64_table[(in[0] & 0x03) << 4]);
			uputc('=');
		} else {
			uputc(base64_table[((in[0] & 0x03) << 4) |
					      (in[1] >> 4)]);
			uputc(base64_table[(in[1] & 0x0f) << 2]);
		}
		uputc('=');
		line_len += 4;
	}

	if (line_len) {
        uputc('\r');
        uputc('\n');
    }
}

#define DECOMPRESSION_BUF_SZ 2048
#define FAKE_TX_BUF_SZ       1024
#define HANDLE_WATER         1024
#define HANDLE_MAX           512

extern pio_jtag_inst_t jtag;

uint8_t decompression_buf[DECOMPRESSION_BUF_SZ];
uint8_t fake_tx_buf[FAKE_TX_BUF_SZ];

static void decode() {

    if (cmd_buffer_sunk < 1000) {
        return;
    }

    uputs("\n\rattempt decode + writeback ... ");

    heatshrink_decoder_reset(&hsd);
    uint32_t compressed_size = cmd_buffer_sunk;
    size_t sunk = 0;
    size_t bytes_in_decomp = 0;
    size_t bytes_total_decomp = 0;
    size_t total_handled = 0;
    while (sunk < compressed_size) {

        // Read from saved command buffer
        size_t scount = 0;
        heatshrink_decoder_sink(&hsd, &cmd_buffer[sunk],
                                compressed_size - sunk, &scount);
        sunk += scount;

        if (sunk == compressed_size) {
            heatshrink_decoder_finish(&hsd);
        }

        // Decompress as much as we can from what we just sunk.
        HSD_poll_res pres;
        do {
            size_t pcount = 0;
            pres = heatshrink_decoder_poll(
                &hsd, &decompression_buf[bytes_in_decomp],
                DECOMPRESSION_BUF_SZ - bytes_in_decomp, &pcount);
            bytes_in_decomp += pcount;
            bytes_total_decomp += pcount;

        } while ((pres == HSDR_POLL_MORE) && (bytes_in_decomp < DECOMPRESSION_BUF_SZ));

        // If we have a lot of decompressed commands pending, handle them until it's less than HANDLE_WATER
        while (bytes_in_decomp >= HANDLE_WATER) {
            size_t n_handled = cmd_handle(&jtag, &decompression_buf[0], HANDLE_MAX, fake_tx_buf, true);

            // Can't rely on memcpy copy order!
            for (int i = 0; i < (bytes_in_decomp - n_handled); i++) {
                (decompression_buf)[i] = (decompression_buf + n_handled)[i];
            }

            bytes_in_decomp -= n_handled;
            total_handled += n_handled;
        }

    }

    while (bytes_in_decomp > 0) {
        size_t n_handled = cmd_handle(&jtag, &decompression_buf[0], bytes_in_decomp, fake_tx_buf, true);

        for (int i = 0; i < (bytes_in_decomp - n_handled); i++) {
            (decompression_buf)[i] = (decompression_buf + n_handled)[i];
        }

        bytes_in_decomp -= n_handled;
        total_handled += n_handled;
    }

    uputs("\n\rdecompressed bytes: ");
    base64_print((uint8_t*)&bytes_total_decomp, 4);

    uputs("\n\rhandled commands: ");
    base64_print((uint8_t*)&total_handled, 4);
}

static uint32_t cmd_info(uint8_t *buffer) {
  char info_string[10] = "DJTAG2\n";
  memcpy(buffer, info_string, 10);

  while(HSER_FINISH_MORE == heatshrink_encoder_finish(&hse)) {
    size_t n_polled = 0;
    heatshrink_encoder_poll(&hse, &cmd_buffer[cmd_buffer_index],
                            CMD_BUFFER_SZ-cmd_buffer_index, &n_polled);
    cmd_buffer_index += n_polled;
  }

  uputs("\r\n*** START base64 ***\r\n");
  base64_print(cmd_buffer, cmd_buffer_index);
  uputs("\r\n*** END base64 ***\r\n");
  uputs("\n\rcompressed bytes: ");
  base64_print((uint8_t*)&cmd_buffer_index, 4);
  uputs("\n\runcompressed bytes: ");
  base64_print((uint8_t*)&cmd_buffer_sunk, 4);
  uputs("\n\rcommand bytes: ");
  base64_print((uint8_t*)&cmd_buffer_count, 4);

  decode();

  cmd_buffer_index = 0;
  cmd_buffer_sunk = 0;
  cmd_buffer_count = 0;
  hse_init = false;

  return 10;
}

static void cmd_freq(pio_jtag_inst_t* jtag, const uint8_t *commands) {
  jtag_set_clk_freq(jtag, (commands[1] << 8) | commands[2]);
}

//static uint8_t output_buffer[64];

static uint32_t cmd_xfer(pio_jtag_inst_t* jtag, const uint8_t *commands, bool extend_length, bool no_read, uint8_t* tx_buf) {
  uint16_t transferred_bits;
  uint8_t* output_buffer = 0;
  transferred_bits = commands[1];
  if (extend_length)
  {
    transferred_bits += 256;
  }
  // Ensure we don't do over-read
  if (transferred_bits > 62 * 8)
  {
    transferred_bits = 62 * 8;
  }

  /* Fill the output buffer with zeroes */
  if (!no_read)
  {
    output_buffer = tx_buf;
    memset(output_buffer, 0, (transferred_bits + 7) / 8);
  }

  jtag_transfer(jtag, transferred_bits, commands+2, output_buffer);

  return (transferred_bits + 7) / 8;
}

static void cmd_setsig(pio_jtag_inst_t* jtag, const uint8_t *commands) {
  uint8_t signal_mask, signal_status;

  signal_mask = commands[1];
  signal_status = commands[2];

  if (signal_mask & SIG_TCK) {
    jtag_set_clk(jtag,signal_status & SIG_TCK);
  }

  if (signal_mask & SIG_TDI) {
    jtag_set_tdi(jtag, signal_status & SIG_TDI);
  }

  if (signal_mask & SIG_TMS) {
    jtag_set_tms(jtag, signal_status & SIG_TMS);
  }
  
  if (signal_mask & SIG_TRST) {
    jtag_set_trst(jtag, signal_status & SIG_TRST);
  }

  if (signal_mask & SIG_SRST) {
    jtag_set_rst(jtag, signal_status & SIG_SRST);
  }
}

static uint32_t cmd_getsig(pio_jtag_inst_t* jtag, uint8_t *buffer)
{
  uint8_t signal_status = 0;
  
  if (jtag_get_tdo(jtag)) {
    signal_status |= SIG_TDO;
  }
  buffer[0] = signal_status;
  return 1;
}

static uint32_t cmd_clk(pio_jtag_inst_t *jtag, const uint8_t *commands, bool readout, uint8_t *buffer)
{
  uint8_t signals, clk_pulses;
  signals = commands[1];
  clk_pulses = commands[2];
  uint8_t readout_val = jtag_strobe(jtag, clk_pulses, signals & SIG_TMS, signals & SIG_TDI);

  if (readout)
  {
    buffer[0] = readout_val;
  }
  return readout ? 1 : 0;
}

static void cmd_setvoltage(const uint8_t *commands) {
  (void)commands;
}

static void cmd_gotobootloader(void) {

}
