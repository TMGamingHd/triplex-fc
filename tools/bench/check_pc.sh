#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Is this PC ready for the bench? Prints one line per check and what to do about a missing one; the exit status is the number
# of missing items. Read-only: changes nothing. Run it before the parts arrive and again with the hardware plugged in.
cd "$(dirname "$0")/../.." || exit 2
missing=0
ok()   { printf '  [ ok ]  %s\n' "$1"; }
miss() { printf '  [MISS]  %s\n          -> %s\n' "$1" "$2"; missing=$((missing + 1)); }
note() { printf '  [ -- ]  %s\n' "$1"; }

echo "Build and flash"
[ -x ../.venv/bin/west ] && ok "west (../.venv)" || miss "west is not in ../.venv" "see firmware/README.md, one-time setup"
ls -d "$HOME"/zephyr-sdk-* >/dev/null 2>&1 && ok "Zephyr SDK ($(ls -d "$HOME"/zephyr-sdk-* | head -1))" || miss "Zephyr SDK not found in ~" "firmware/README.md, one-time setup"
command -v openocd >/dev/null && ok "openocd $(openocd --version 2>&1 | head -1 | awk '{print $4}')" || miss "openocd" "sudo apt install openocd"
grep -qs "0483" /usr/lib/udev/rules.d/*openocd* /etc/udev/rules.d/*openocd* 2>/dev/null && ok "ST-LINK udev rule (openocd package)" || miss "no udev rule for the ST-LINK" "sudo cp /usr/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/ && sudo udevadm control --reload"
id -nG | grep -qw plugdev && ok "user is in plugdev" || miss "user is not in plugdev" "sudo usermod -aG plugdev \$USER, then log in again"
id -nG | grep -qw dialout && ok "user is in dialout (serial ports)" || miss "user is not in dialout" "sudo usermod -aG dialout \$USER, then log in again"
if command -v lsusb >/dev/null; then
  lsusb | grep -qi "0483:" && ok "an ST device is plugged in (ST-LINK)" || note "no ST-LINK plugged in right now"
fi

echo "CAN"
ip -brief link show vcan0 >/dev/null 2>&1 && ok "vcan0 exists" || miss "vcan0 does not exist (it does not survive a reboot)" "./sim/scripts/setup_vcan.sh"
modinfo gs_usb >/dev/null 2>&1 && ok "gs_usb kernel module available (USB-CAN adapter)" || miss "gs_usb module not available" "a kernel with CONFIG_CAN_GS_USB"
command -v candump >/dev/null && ok "can-utils (candump)" || miss "can-utils" "sudo apt install can-utils"
ip -brief link show can0 >/dev/null 2>&1 && ok "can0 present (adapter plugged in): $(ip -brief link show can0 | awk '{print $2}')" || note "no can0 right now (plug in the adapter, then tools/bench/can_up.sh)"

echo "Python tools"
python3 -c "import serial" 2>/dev/null && ok "pyserial (Pico client)" || miss "pyserial" "python3 -m pip install pyserial"
[ -x build/host/tfc_replay ] && ok "build/host/tfc_replay" || miss "tfc_replay is not built" "cmake -S . -B build/host -G Ninja -DTFC_SANITIZE=ON && cmake --build build/host"
[ -x build/native_sim/zephyr/zephyr.exe ] && ok "native_sim firmware" || miss "native_sim firmware is not built" "west build -p always -b native_sim/native/64 firmware/app -d build/native_sim"
[ -f build/nucleo_g474re/zephyr/zephyr.elf ] && ok "nucleo_g474re firmware" || miss "nucleo_g474re firmware is not built" "west build -p always -b nucleo_g474re firmware/app -d build/nucleo_g474re"

echo "Not checked here"
note "logic-analyser software for the Kingst LA1010 (vendor program or sigrok): check before the device arrives"
note "Pico: no tool needed to flash a UF2 (drag it onto the drive); Pico SDK needed to build Pico firmware (SW-20, SW-21)"

echo
[ "$missing" -eq 0 ] && echo "Ready." || echo "$missing item(s) missing."
exit "$missing"
