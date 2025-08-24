#!/usr/bin/env python3
"""
Tool to convert raw ECP5 bitstream files to C arrays for inclusion in the firmware.

Usage:
    python3 bitstream_to_c.py input1.bit input2.bit input3.bit > raw_bitstreams.c

This generates a raw_bitstreams.c file with the bitstreams stored as ROM constants.
"""

import sys
import os

def bitstream_to_c_array(filename, var_name):
    """Convert a bitstream file to a C array declaration."""
    with open(filename, 'rb') as f:
        data = f.read()
    
    # Generate C array
    lines = ['static const uint8_t %s_data[] = {' % var_name]
    
    # Split data into 16-byte lines for readability
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_values = ['0x%02x' % b for b in chunk]
        line = '    ' + ', '.join(hex_values)
        if i + 16 < len(data):
            line += ','
        lines.append(line)
    
    lines.append('};')
    lines.append('')
    
    return '\n'.join(lines), len(data)

def generate_header():
    """Generate the file header."""
    return '''#include "bitstream_rom.h"

// Auto-generated raw bitstream data
// Generated with bitstream_to_c.py

'''

def generate_footer(bitstreams):
    """Generate the bitstream info array and count."""
    lines = ['const struct bitstream_info bitstreams[] = {']
    
    for name, var_name, size in bitstreams:
        lines.append('    {"%s", %s_data, %d},' % (name, var_name, size))
    
    lines.append('};')
    lines.append('')
    lines.append('const int bitstream_count = sizeof(bitstreams) / sizeof(bitstreams[0]);')
    
    return '\n'.join(lines)

def main():
    if len(sys.argv) < 2:
        print("Usage: %s <bitstream1.bit> [bitstream2.bit] [...]" % sys.argv[0], file=sys.stderr)
        print("\nConverts raw ECP5 bitstream files to C arrays.", file=sys.stderr)
        sys.exit(1)
    
    print(generate_header())
    
    bitstreams = []
    
    for i, filename in enumerate(sys.argv[1:], 1):
        if not os.path.exists(filename):
            print("Error: File not found: %s" % filename, file=sys.stderr)
            sys.exit(1)
        
        # Generate variable name from filename
        base_name = os.path.splitext(os.path.basename(filename))[0]
        var_name = 'bitstream%d' % i
        display_name = 'BITSTREAM%d' % i
        
        array_code, size = bitstream_to_c_array(filename, var_name)
        print(array_code)
        
        bitstreams.append((display_name, var_name, size))
        
        print("// %s: %d bytes from %s" % (display_name, size, filename))
        print()
    
    print(generate_footer(bitstreams))

if __name__ == '__main__':
    main()