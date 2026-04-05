#!/usr/bin/env python3
"""
Tool to convert heatshrink-compressed ECP5 bitstream files to C arrays for
inclusion in the firmware.

Usage:
    python3 bitstream_to_c.py input1.bit.hs input2.bit.hs ... > bitstream_rom.c

Each .hs file is expected to have a 4-byte little-endian original size header
followed by the compressed data (as produced by compress_bitstream).
"""

import sys
import os
import struct

def bitstream_to_c_array(filename, var_name):
    """Convert a compressed bitstream file to a C array declaration."""
    with open(filename, 'rb') as f:
        raw = f.read()

    original_size = struct.unpack('<I', raw[:4])[0]
    compressed_data = raw[4:]

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
    return '''#include "bitstream_rom.h"

// Auto-generated compressed bitstream data
// Generated with bitstream_to_c.py from heatshrink-compressed inputs

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
        print("Usage: %s <bitstream1.bit.hs> [bitstream2.bit.hs] [...]" % sys.argv[0], file=sys.stderr)
        print("\nConverts heatshrink-compressed ECP5 bitstream files to C arrays.", file=sys.stderr)
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
