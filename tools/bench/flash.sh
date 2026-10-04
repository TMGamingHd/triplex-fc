#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Flash a built firmware image to a Nucleo through the ST-LINK with openocd.
#   tools/bench/flash.sh [build_dir]      default: build/nucleo_g474re
# Build first:  . firmware/env.sh && west build -p always -b nucleo_g474re firmware/app -d build/nucleo_g474re
set -euo pipefail
cd "$(dirname "$0")/../.."
DIR="${1:-build/nucleo_g474re}"
[ -f "$DIR/zephyr/zephyr.elf" ] || { echo "no $DIR/zephyr/zephyr.elf: build it first (see the header of this script)" >&2; exit 1; }
if command -v lsusb >/dev/null && ! lsusb | grep -qi "0483:"; then
  echo "no ST-LINK on USB: plug the Nucleo in (and check the cable carries data)" >&2
  exit 1
fi
# shellcheck disable=SC1091
. firmware/env.sh
west flash -d "$DIR" --runner openocd
echo "flashed $DIR. The console is on the ST-LINK's serial port (115200 8N1), e.g. screen /dev/ttyACM0 115200"
