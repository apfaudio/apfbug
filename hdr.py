#!/bin/python3

import base64

started = False

content_raw = ''

with open("img.txt", "r") as f:
    for line in f:
        if "* START base64" in line:
            started = True
            continue
        if "* END base64" in line:
            break
        content_raw += line.strip()

content_decoded = base64.b64decode(content_raw)
print(content_decoded)

with open("out.bin", "wb") as f:
    f.write(content_decoded)

with open("out.c", "w") as f:
    f.write(f"uint32_t cmd_buffer_sunk = {len(content_decoded)};\n")
    f.write(f"uint8_t cmd_buffer[{len(content_decoded)}]")
    f.write(" = {\n")
    line = ''
    for byte in content_decoded:
        line += "0x{:02x}".format(byte)
        line += ','
        if len(line) >= 80:
            f.write('\t' + line + '\n')
            line = ''
        else:
            line += ' '
    f.write('\t' + line)
    f.write('\n};')
