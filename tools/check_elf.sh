#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
# Fails if the flight binary contains anything the coding standard forbids (docs/CODING_STANDARD.md):
# dynamic memory, C++ exceptions, RTTI or virtual dispatch. Usage: tools/check_elf.sh ELF [NM]
set -euo pipefail
elf="${1:?usage: check_elf.sh ELF [NM]}"
nm="${2:-${NM:-$(ls "$HOME"/zephyr-sdk-*/gnu/arm-zephyr-eabi/bin/arm-zephyr-eabi-nm 2>/dev/null | head -1)}}"
nm="${nm:-nm}"
forbidden='(^| )(malloc|calloc|realloc|free|_Znwj|_Znaj|_Znwm|_Znam|_ZdlPv|_ZdaPv|__cxa_throw|__cxa_allocate_exception|__cxa_begin_catch|__gxx_personality_v0|__dynamic_cast)( |$)|^[0-9a-f]+ [A-Za-z] (_ZTV|_ZTI|_ZTS)'
syms=$("$nm" "$elf") || { echo "check_elf: $nm could not read $elf (wrong binutils for this binary?)" >&2; exit 2; }
if hits=$(grep -E "$forbidden" <<<"$syms"); then
  echo "FORBIDDEN symbols in $elf (heap, exceptions, RTTI or vtables):"
  echo "$hits"
  exit 1
fi
echo "ok: $elf has no heap, exception, RTTI or vtable symbols"
