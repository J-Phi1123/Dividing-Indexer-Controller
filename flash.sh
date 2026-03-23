#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
UPLOAD_PORT="${UPLOAD_PORT:-/dev/ttyUSB0}"
UPLOAD_SPEED="${UPLOAD_SPEED:-115200}"

if [[ ! -f "${VENV_DIR}/bin/activate" ]]; then
  echo "Missing virtualenv activate script at ${VENV_DIR}/bin/activate" >&2
  exit 1
fi

if [[ ${EUID} -ne 0 ]]; then
  exec sudo UPLOAD_PORT="${UPLOAD_PORT}" bash "$0" "$@"
fi

cd "${SCRIPT_DIR}"
source "${VENV_DIR}/bin/activate"

echo "Using upload port: ${UPLOAD_PORT}"
echo "Using upload speed: ${UPLOAD_SPEED}"
echo "Hold BOOT on the ESP32, then press Enter to start upload."
read -r

if [[ "${UPLOAD_SPEED}" != "115200" ]]; then
  echo "Note: flash.sh does not pass upload speed on the CLI; set upload_speed in platformio.ini if you need a different value." >&2
fi

exec pio run -t upload --upload-port "${UPLOAD_PORT}" "$@"
