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

#include "openfpgaloader.h"
#include "lattice_cmds.h"
#include "bitstream_rom.h"
#include "heatshrink_decoder.h"

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

/**
 * @brief Handle CMD_SETSIG command
 *
 * CMD_SETSIG set the logic state of the JTAG signals.
 *
 * @param commands Command data
 */

/**
 * @brief Handle CMD_GETSIG command
 *
 * CMD_GETSIG gets the current signal state.
 * 
 * @param usbd_dev USB device
 */
uint32_t cmd_getsig(pio_jtag_inst_t* jtag, uint8_t *buffer);

/**
 * @brief Handle CMD_CLK command
 *
 * CMD_CLK sends clock pulses with specific TMS and TDI state.
 *
 * @param usbd_dev USB device
 * @param commands Command data
 * @param readout Enable TDO readout
 */
uint32_t cmd_clk(pio_jtag_inst_t *jtag, const uint8_t *commands, bool readout, uint8_t *buffer);
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

uint32_t cmd_handle(pio_jtag_inst_t* jtag, uint8_t* rxbuf, uint32_t count, uint8_t* tx_buf, bool local_host) {
  uint8_t *commands= (uint8_t*)rxbuf;
  uint8_t *output_buffer = tx_buf;
  
  while ((commands < (rxbuf + count)) && (*commands != CMD_STOP))
  {
    uint8_t cmd_byte = *commands;
    uint8_t cmd_type = cmd_byte & 0x0F;
    
    
    switch (cmd_type) {
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
      bool extend_length = *commands & EXTEND_LENGTH;
      uint16_t bits = commands[1];
      if (extend_length) bits += 256;
      uint32_t data_bytes = (bits + 7) / 8;
      uint32_t trbytes = cmd_xfer(jtag, commands, extend_length, no_read, output_buffer);
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

extern pio_jtag_inst_t jtag;

static uint8_t decompressed_buf[128 * 1024];

static bool heatshrink_decompress(const uint8_t *compressed, uint32_t compressed_size,
                                   uint8_t *output, uint32_t original_size) {
    heatshrink_decoder hsd;
    heatshrink_decoder_reset(&hsd);

    size_t sink_offset = 0;
    size_t out_offset = 0;

    while (out_offset < original_size) {
        if (sink_offset < compressed_size) {
            size_t sunk = 0;
            HSD_sink_res sres = heatshrink_decoder_sink(&hsd,
                (uint8_t *)&compressed[sink_offset], compressed_size - sink_offset, &sunk);
            if (sres < 0) return false;
            sink_offset += sunk;
        }

        HSD_poll_res pres;
        do {
            size_t polled = 0;
            pres = heatshrink_decoder_poll(&hsd,
                &output[out_offset], original_size - out_offset, &polled);
            if (pres < 0) return false;
            out_offset += polled;
            if (out_offset >= original_size) break;
        } while (pres == HSDR_POLL_MORE);

        if (sink_offset >= compressed_size && pres == HSDR_POLL_EMPTY) {
            heatshrink_decoder_finish(&hsd);
        }
    }

    return out_offset == original_size;
}

bool load_bitstream_by_number(pio_jtag_inst_t* jtag, uint32_t bitstream_number, uint32_t* device_id_out, uint64_t* status_out) {
    const struct bitstream_info* bitstream = &bitstreams[bitstream_number];

    jtag_set_clk_freq(jtag, 60000);

    uint32_t device_id = ecp5_jtag_read_id();
    if (device_id_out) {
        *device_id_out = device_id;
    }
    bool device_found = false;
    for (int i = 0; i < ecp_device_count; i++) {
        if (ecp_devices[i].device_id == device_id) {
            device_found = true;
            break;
        }
    }
    if (!device_found) {
        return false;
    }

    if (bitstream->original_size > sizeof(decompressed_buf)) {
        return false;
    }

    if (!heatshrink_decompress(bitstream->data, bitstream->compressed_size,
                                decompressed_buf, bitstream->original_size)) {
        return false;
    }

    ecp5_jtag_load_bitstream(decompressed_buf, bitstream->original_size);

    // Read status register after bitstream loading
    uint64_t status = ecp5_jtag_read_status();
    if (status_out) {
        *status_out = status;
    }
    return true;
}


static uint32_t cmd_info(uint8_t *buffer) {
  char info_string[10] = "DJTAG2\n";
  memcpy(buffer, info_string, 10);
  return 10;
}

static void cmd_freq(pio_jtag_inst_t* jtag, const uint8_t *commands) {
  jtag_set_clk_freq(jtag, (commands[1] << 8) | commands[2]);
}

//static uint8_t output_buffer[64];

uint32_t cmd_xfer(pio_jtag_inst_t* jtag, const uint8_t *commands, bool extend_length, bool no_read, uint8_t* tx_buf) {
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

void cmd_setsig(pio_jtag_inst_t* jtag, const uint8_t *commands) {
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

uint32_t cmd_getsig(pio_jtag_inst_t* jtag, uint8_t *buffer)
{
  uint8_t signal_status = 0;
  
  if (jtag_get_tdo(jtag)) {
    signal_status |= SIG_TDO;
  }
  buffer[0] = signal_status;
  return 1;
}

uint32_t cmd_clk(pio_jtag_inst_t *jtag, const uint8_t *commands, bool readout, uint8_t *buffer)
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
