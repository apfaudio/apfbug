#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cc -o "$SCRIPT_DIR/compress_bitstream" "$SCRIPT_DIR/compress_bitstream.c" "$ROOT_DIR/heatshrink_encoder.c" -I"$ROOT_DIR"
