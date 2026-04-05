/*
  Copyright (c) 2017 Jean THOMAS.
  
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

/**
 * @brief Handle a DirtyJTAG command
 *
 * @param usbd_dev USB device
 * @param transfer Received packet
 * @return Command needs to send data back to host
 */

#include "pio_jtag.h"
#include "dirtyjtag_protocol.h"

bool load_bitstream_by_number(pio_jtag_inst_t* jtag, uint32_t bitstream_number, uint32_t* device_id_out, uint32_t* status_out);
uint32_t cmd_handle(pio_jtag_inst_t* jtag, uint8_t* rxbuf, uint32_t count, uint8_t* tx_buf, bool local_host);

// Expose individual command functions for direct use
uint32_t cmd_xfer(pio_jtag_inst_t* jtag, const uint8_t *commands, bool extend_length, bool no_read, uint8_t* tx_buf);
void cmd_setsig(pio_jtag_inst_t* jtag, const uint8_t *commands);
uint32_t cmd_getsig(pio_jtag_inst_t* jtag, uint8_t *buffer);
uint32_t cmd_clk(pio_jtag_inst_t *jtag, const uint8_t *commands, bool readout, uint8_t *buffer);
