#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="${SCRIPT_DIR}/transponder"

echo "Compiling code, please wait"
arduino-cli compile --fqbn esp32:esp32:esp32-poe-iso "${SKETCH_DIR}"

echo "Uploading code, please wait"
arduino-cli upload \
  -p /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --fqbn esp32:esp32:esp32-poe-iso \
  "${SKETCH_DIR}"

echo "Done"
