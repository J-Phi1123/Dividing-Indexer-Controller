#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${SCRIPT_DIR}/.venv"
UPLOAD_PORT="${UPLOAD_PORT:-/dev/ttyUSB0}"

if [[ ! -f "${VENV_DIR}/bin/activate" ]]; then
  echo "Missing virtualenv activate script at ${VENV_DIR}/bin/activate" >&2
  exit 1
fi

if [[ ${EUID} -ne 0 ]]; then
  exec sudo UPLOAD_PORT="${UPLOAD_PORT}" bash "$0" "$@"
fi

cd "${SCRIPT_DIR}"
source "${VENV_DIR}/bin/activate"

exec pio run -t upload --upload-port "${UPLOAD_PORT}" "$@"
