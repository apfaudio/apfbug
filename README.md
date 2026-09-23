# `apfbug` - RP2040 debug probe with offline JTAG replay

<sup>WARN: 🚧 under construction! 🚧 - this is in active development</sup>

## What is this?

- Firmware for the RP2040-based JTAG debug probe built into [Tiliqua](https://github.com/apfaudio/tiliqua).
- `apfbug` started as a fork of the [pico-dirtyJtag](https://github.com/phdussud/pico-dirtyJtag) project.

## How do I install/use this?

- See the Tiliqua documentation: https://apfaudio.github.io/tiliqua/bootloader.html

## What has been added?

`apfbug` includes the same features as `pico-dirtyjtag` (USB-JTAG and USB-UART bridge), with some additions:
- UART traffic is inspected to look for keywords.
- If a keyword is encountered e.g. `BITSTREAM1`, a compressed bitstream stored on the RP2040's SPI flash is decompressed and sent to the ECP5 over JTAG, using the ECP5's own configuration protocol.

## Bitstream ROM

ECP5 allows bitstreams to jump to arbitrary addresses - however it is not possible to issue a JTAG command to jump to an arbitrary address. So, we store N 'bootstub' bitsreams, each of which does nothing except A) reboot immediately to B) the target arbitrary address. Unfortunately these are 'normal' bitstreams, so they are quite large. To keep the bitstream switch time low, each 'bootstub' is compressed with heatshrink to about 10KiB each (yes this is much smaller than even a compressed bitstream from ecppack, so we are compressing twice!) - the double-compressed bitstream is stored in a generated `rom.c`. On bitstream selection, this bitstream is heatshrink-decompressed and sent over JTAG to the ECP5. The ECP5 is then decompressing the `ecppack`-compressed bitstream into a full-size bitstream. Interestingly, this whole process is much faster than sending non-compressed bitstreams straight from SPI flash via RP2040, as flash read speed is the bottleneck.

In `bitstream/src/*.bit` you find the source bitstreams generated from the Tiliqua repository using `scripts/bootstubs.sh` - basically a set of commands like `pdm bootstub build --name=bootstub4 --bootaddr=0x400000`.

In this repository, to regenerate `bitstream/rom.c` from these bootstubs in `bitstream/src/*.bit`:

```
# Build heatshrink CLI
make -C heatshrink heatshrink
# `genrom` wraps the heatshrink CLI
python3 bitstream/genrom.py bitstream/src/bootstub*.bit > bitstream/rom.c
```

## Future

I have tried and failed on a few occasions to figure out a way to have the RP2040 dynamically rewrite the packed bitstream to insert an arbitrary BOOTADDR, rather than needing N separately packed bitstreams. I'm sure it's possible, I just haven't figured out how yet.
