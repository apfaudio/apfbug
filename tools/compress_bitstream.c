#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../heatshrink/heatshrink_encoder.h"

static int compress_file(const char *inpath, const char *outpath) {
    FILE *fin = fopen(inpath, "rb");
    if (!fin) {
        fprintf(stderr, "Error: cannot open %s\n", inpath);
        return 1;
    }

    fseek(fin, 0, SEEK_END);
    long insize = ftell(fin);
    fseek(fin, 0, SEEK_SET);

    uint8_t *inbuf = malloc(insize);
    if (!inbuf) {
        fprintf(stderr, "Error: malloc failed\n");
        fclose(fin);
        return 1;
    }
    fread(inbuf, 1, insize, fin);
    fclose(fin);

    /* Worst case: compressed is larger than input */
    size_t outbuf_sz = insize + (insize / 2) + 256;
    uint8_t *outbuf = malloc(outbuf_sz);
    if (!outbuf) {
        fprintf(stderr, "Error: malloc failed\n");
        free(inbuf);
        return 1;
    }

    heatshrink_encoder hse;
    heatshrink_encoder_reset(&hse);

    size_t sink_offset = 0;
    size_t out_offset = 0;

    while (sink_offset < (size_t)insize) {
        size_t sunk = 0;
        HSE_sink_res sres = heatshrink_encoder_sink(&hse,
            &inbuf[sink_offset], insize - sink_offset, &sunk);
        if (sres < 0) {
            fprintf(stderr, "Error: sink failed\n");
            free(inbuf);
            free(outbuf);
            return 1;
        }
        sink_offset += sunk;

        HSE_poll_res pres;
        do {
            size_t polled = 0;
            pres = heatshrink_encoder_poll(&hse,
                &outbuf[out_offset], outbuf_sz - out_offset, &polled);
            if (pres < 0) {
                fprintf(stderr, "Error: poll failed\n");
                free(inbuf);
                free(outbuf);
                return 1;
            }
            out_offset += polled;
        } while (pres == HSER_POLL_MORE);
    }

    HSE_finish_res fres;
    do {
        fres = heatshrink_encoder_finish(&hse);
        if (fres < 0) {
            fprintf(stderr, "Error: finish failed\n");
            free(inbuf);
            free(outbuf);
            return 1;
        }

        size_t polled = 0;
        HSE_poll_res pres;
        do {
            pres = heatshrink_encoder_poll(&hse,
                &outbuf[out_offset], outbuf_sz - out_offset, &polled);
            if (pres < 0) {
                fprintf(stderr, "Error: poll failed\n");
                free(inbuf);
                free(outbuf);
                return 1;
            }
            out_offset += polled;
        } while (pres == HSER_POLL_MORE);
    } while (fres == HSER_FINISH_MORE);

    FILE *fout = fopen(outpath, "wb");
    if (!fout) {
        fprintf(stderr, "Error: cannot open %s for writing\n", outpath);
        free(inbuf);
        free(outbuf);
        return 1;
    }

    /* Write 4-byte little-endian original size header, then compressed data */
    uint8_t hdr[4];
    hdr[0] = (insize >>  0) & 0xff;
    hdr[1] = (insize >>  8) & 0xff;
    hdr[2] = (insize >> 16) & 0xff;
    hdr[3] = (insize >> 24) & 0xff;
    fwrite(hdr, 1, 4, fout);
    fwrite(outbuf, 1, out_offset, fout);
    fclose(fout);

    fprintf(stderr, "%s: %ld -> %zu bytes (%.1f%%)\n",
        inpath, insize, out_offset, 100.0 * out_offset / insize);

    free(inbuf);
    free(outbuf);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3 || (argc % 2) != 1) {
        fprintf(stderr, "Usage: %s <input.bit> <output.bit.hs> [...]\n", argv[0]);
        return 1;
    }

    for (int i = 1; i < argc; i += 2) {
        if (compress_file(argv[i], argv[i + 1]) != 0) {
            return 1;
        }
    }
    return 0;
}
