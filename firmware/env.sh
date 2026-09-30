# Source this (". firmware/env.sh") before using west. Paths are overridable:
#   TFC_VENV      Python venv that has west installed   (default: ../.venv next to the repo)
#   ZEPHYR_SDK_INSTALL_DIR  Zephyr SDK location          (default: ~/zephyr-sdk-1.0.1)
_tfc_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
. "${TFC_VENV:-$_tfc_root/../.venv}/bin/activate"
export ZEPHYR_BASE="$_tfc_root/zephyr"
export ZEPHYR_SDK_INSTALL_DIR="${ZEPHYR_SDK_INSTALL_DIR:-$HOME/zephyr-sdk-1.0.1}"
unset _tfc_root
