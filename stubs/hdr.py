#!/bin/python3

import sys
import base64

with open(f"stub_data.c", "w") as f_out:

    for src_path in sys.argv[1:]:

        started = False

        content_raw = ''

        with open(src_path, "r") as f:
            for line in f:
                if "* START base64" in line:
                    started = True
                    continue
                if "* END base64" in line:
                    break
                content_raw += line.strip()

        content_decoded = base64.b64decode(content_raw)

        """
        print(content_decoded)
        with open("out.bin", "wb") as f:
            f.write(content_decoded)
        """

        f_out.write('\n\n')
        f_out.write(f"uint32_t {src_path}_cmd_buffer_sunk = {len(content_decoded)};\n")
        f_out.write(f"uint8_t {src_path}_cmd_buffer[{len(content_decoded)}]")
        f_out.write(" = {\n")
        line = ''
        for byte in content_decoded:
            line += "0x{:02x}".format(byte)
            line += ','
            if len(line) >= 80:
                f_out.write('\t' + line + '\n')
                line = ''
            else:
                line += ' '
        f_out.write('\t' + line)
        f_out.write('\n};')

    f_out.write('\n\n')
    f_out.write("bool cmd_data_for_index(size_t ix, uint32_t *size_out, uint8_t **buffer_out) {\n")
    f_out.write("\tswitch (ix) {\n")
    for n, src_path in enumerate(sys.argv[1:]):
        f_out.write(f"\t\tcase {n}:\n")
        f_out.write(f"\t\t\t*size_out = {src_path}_cmd_buffer_sunk;\n")
        f_out.write(f"\t\t\t*buffer_out = {src_path}_cmd_buffer;\n")
        f_out.write("\t\t\tbreak;\n")
    f_out.write(f"\t\tdefault:\n")
    f_out.write("\t\t\treturn false;\n")
    f_out.write("\t\t\tbreak;\n")
    f_out.write("\t}\n")
    f_out.write("\treturn true;\n")
    f_out.write('\n};')
