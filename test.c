#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "out.c"

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

static uint32_t cmd_xfer(const uint8_t *commands, bool extend_length) {
  uint16_t transferred_bits;
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

  return (transferred_bits + 7) / 8;
}

uint32_t cmd_handle(uint8_t* rxbuf, uint32_t count) {

  uint8_t *commands= (uint8_t*)rxbuf;

  while ((commands < (rxbuf + count)) && (*commands != CMD_STOP))
  {
    switch ((*commands)&0x0F) {
    case CMD_INFO:
    {
      printf("cmd_info\n");
      break;
    }
    case CMD_FREQ:
      printf("cmd_freq\n");
      commands += 2;
      break;

    case CMD_XFER:
    {
      printf("cmd_xfer - ");
      bool no_read = *commands & NO_READ;
      uint32_t trbytes = cmd_xfer(commands, *commands & EXTEND_LENGTH);
      commands += 1 + trbytes;
      printf("%i\n", 1 + trbytes);
      break;
    }
    case CMD_SETSIG:
      printf("cmd_setsig\n");
      commands += 2;
      break;

    case CMD_GETSIG:
    {
      printf("cmd_getsig\n");
      break;
    }
    case CMD_CLK:
    {
      printf("cmd_clk\n");
      commands += 2;
      break;
    }
    case CMD_SETVOLTAGE:
      printf("cmd_setvoltage\n");
      commands += 1;
      break;

    case CMD_GOTOBOOTLOADER:
      printf("cmd_gotobootloader\n");
      commands += 1;
      break;

    default:
      printf("UNKNOWN\n");
      break;
    }

    commands++;
  }

  // Count CMD_STOP as a recieved command.
  if (*commands == CMD_STOP) {
      ++commands;
  }

  return commands - rxbuf;
}

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

#define DECOMPRESSION_BUF_SZ 2048
#define FAKE_TX_BUF_SZ       1024
#define HANDLE_WATER         256
#define HANDLE_MAX           128

static heatshrink_decoder hsd;

uint8_t decompression_buf[DECOMPRESSION_BUF_SZ];

static void decode() {

    heatshrink_decoder_reset(&hsd);
    uint32_t compressed_size = cmd_buffer_sunk;
    size_t sunk = 0;
    size_t bytes_in_decomp = 0;
    size_t bytes_total_decomp = 0;
    size_t total_handled = 0;
    bool abort = false;
    while (sunk < compressed_size && !abort) {

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
        } while (pres == HSDR_POLL_MORE);

        // If we have a lot of decompressed commands pending, handle them until it's less than HANDLE_WATER
        while (bytes_in_decomp >= HANDLE_WATER) {
            size_t n_handled = cmd_handle(&decompression_buf[0], HANDLE_MAX);

            if (n_handled > bytes_in_decomp) {
                printf("abort\n");
                abort = true;
                break;
            }

            // Can't rely on memcpy copy order!
            for (int i = 0; i < (bytes_in_decomp - n_handled); i++) {
                (decompression_buf)[i] = (decompression_buf + n_handled)[i];
            }


            bytes_in_decomp -= n_handled;
            total_handled += n_handled;
        }

    }

    while (bytes_in_decomp > 0 && !abort) {
        size_t n_handled = cmd_handle(&decompression_buf[0], bytes_in_decomp);

        for (int i = 0; i < (bytes_in_decomp - n_handled); i++) {
            (decompression_buf)[i] = (decompression_buf + n_handled)[i];
        }

        bytes_in_decomp -= n_handled;
        total_handled += n_handled;

    }

    printf("n_handled: %i\n", total_handled);
    printf("n_decompressed: %i\n", bytes_total_decomp);
}

int main() {
    decode();
    /*
    uint8_t *decompression_buf = malloc(1024*1024);
    FILE *ptr;
    ptr = fopen("outx.bin","rb");
    fseek(ptr, 0L, SEEK_END);
    size_t sz = ftell(ptr);
    fseek(ptr, 0L, SEEK_SET);
    fread(decompression_buf,sz,1,ptr);

    size_t bytes_in_decomp = sz;
    size_t total_handled = 0;
    while (bytes_in_decomp > 0) {
        size_t n_handled = cmd_handle(&decompression_buf[0], MIN(512, bytes_in_decomp));
        printf("-- consumed %i\n", n_handled);
        memcpy(decompression_buf, decompression_buf + n_handled, bytes_in_decomp - n_handled);
        bytes_in_decomp -= n_handled;
        total_handled += n_handled;
    }

    printf("handled: %i\n", total_handled);
    */
}
