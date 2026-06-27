#!/usr/bin/env python3
"""
Tool to convert ECP5 bitstream files to C arrays for inclusion in the firmware.

Usage:
    python3 genrom.py input1.bit input2.bit ... > rom.c

Each input is compressed with the upstream heatshrink CLI (using the same
window/lookahead parameters as the firmware's static decoder config) and
emitted as a C array. The original (uncompressed) size is taken from the
input file itself, so no intermediate .hs files or size header are needed.

The heatshrink CLI is expected at ../heatshrink/heatshrink (build it with
`make -C heatshrink heatshrink`), or pointed to via the HEATSHRINK env var.
"""

import sys
import os
import subprocess

# Must match the firmware's static decoder config (heatshrink_config.h /
# HEATSHRINK_STATIC_WINDOW_BITS, HEATSHRINK_STATIC_LOOKAHEAD_BITS).
WINDOW_BITS = 8
LOOKAHEAD_BITS = 4

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(SCRIPT_DIR)
HEATSHRINK = os.environ.get(
    "HEATSHRINK", os.path.join(REPO_ROOT, "heatshrink", "heatshrink"))


def heatshrink_compress(raw):
    """Compress bytes with the heatshrink CLI via stdin/stdout."""
    if not (os.path.isfile(HEATSHRINK) and os.access(HEATSHRINK, os.X_OK)):
        sys.exit("Error: heatshrink CLI not found at %s\n"
                 "Build it with: make -C heatshrink heatshrink\n"
                 "(or set the HEATSHRINK env var to its path)" % HEATSHRINK)
    proc = subprocess.run(
        [HEATSHRINK, "-e", "-w", str(WINDOW_BITS), "-l", str(LOOKAHEAD_BITS)],
        input=raw, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.returncode != 0:
        sys.exit("Error: heatshrink failed: %s" % proc.stderr.decode(errors="replace"))
    return proc.stdout


def bitstream_to_c_array(filename, var_name):
    """Compress a bitstream file and convert it to a C array declaration."""
    with open(filename, 'rb') as f:
        raw = f.read()

    original_size = len(raw)
    compressed_data = heatshrink_compress(raw)

    lines = ['static const uint8_t %s_data[] = {' % var_name]

    for i in range(0, len(compressed_data), 16):
        chunk = compressed_data[i:i+16]
        hex_values = ['0x%02x' % b for b in chunk]
        line = '    ' + ', '.join(hex_values)
        if i + 16 < len(compressed_data):
            line += ','
        lines.append(line)

    lines.append('};')
    lines.append('')

    return '\n'.join(lines), len(compressed_data), original_size

def generate_header():
    return '''#include "rom.h"

// Auto-generated compressed bitstream data
// Generated with genrom.py from heatshrink-compressed inputs

'''

def generate_footer(bitstreams):
    lines = ['const struct bitstream_info bitstreams[] = {']

    max_original = 0
    for name, var_name, compressed_size, original_size in bitstreams:
        lines.append('    {"%s", %s_data, %d, %d},' % (
            name, var_name, compressed_size, original_size))
        if original_size > max_original:
            max_original = original_size

    lines.append('};')
    lines.append('')
    lines.append('const int bitstream_count = sizeof(bitstreams) / sizeof(bitstreams[0]);')
    lines.append('')
    lines.append('_Static_assert(%d <= 128 * 1024,' % max_original)
    lines.append('    "largest bitstream (%d bytes) exceeds decompression buffer (128KB)");' % max_original)

    return '\n'.join(lines)

def main():
    if len(sys.argv) < 2:
        print("Usage: %s <bitstream1.bit> [bitstream2.bit] [...]" % sys.argv[0], file=sys.stderr)
        print("\nCompresses ECP5 bitstream files (via the heatshrink CLI) into C arrays.", file=sys.stderr)
        sys.exit(1)

    print(generate_header())

    bitstreams = []

    for i, filename in enumerate(sys.argv[1:], 1):
        if not os.path.exists(filename):
            print("Error: File not found: %s" % filename, file=sys.stderr)
            sys.exit(1)

        var_name = 'bitstream%d' % i
        display_name = 'BITSTREAM%d' % i

        array_code, compressed_size, original_size = bitstream_to_c_array(filename, var_name)
        print(array_code)

        bitstreams.append((display_name, var_name, compressed_size, original_size))

        print("// %s: %d bytes compressed, %d bytes original from %s" % (
            display_name, compressed_size, original_size, filename))
        print()

    print(generate_footer(bitstreams))

if __name__ == '__main__':
    main()
