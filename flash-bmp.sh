#!/usr/bin/env bash
set -euo pipefail
BMP_SERIAL="${1:-/dev/cu.usbmodem97B71C141}"

if [[ ! -f "build/zephyr/zephyr.elf" ]]; then
  echo "zephyr.elf not found. Build first with: west build -b nrf52dk_nrf52810 ."
  exit 1
fi

echo "Flashing via BMP on ${BMP_SERIAL}..."
arm-none-eabi-gdb build/zephyr/zephyr.elf   -ex "target extended-remote ${BMP_SERIAL}"   -ex "monitor swdp_scan"   -ex "attach 1"   -ex "load"   -ex "monitor reset"   -ex "quit"
