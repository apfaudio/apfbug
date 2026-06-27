#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cc -DHEATSHRINK_DYNAMIC_ALLOC=0 -o "$SCRIPT_DIR/compress_bitstream" "$SCRIPT_DIR/compress_bitstream.c" "$ROOT_DIR/heatshrink/heatshrink_encoder.c" -I"$ROOT_DIR/heatshrink"
