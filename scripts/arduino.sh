#!/bin/bash
set -euo pipefail

root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
sketch="$root/firmware/SyringePumpFirmware"
fqbn="arduino:avr:mega:cpu=atmega2560"
action="${1:-}"

case "$action" in
  compile|ports)
    ;;
  upload)
    port="${2:-}"
    if [[ "${3:-}" != "UPLOAD" ]]; then
      printf '%s\n' 'Upload cancelled. Explicit approval is required.' >&2
      exit 1
    fi
    if [[ "$port" != /dev/cu.* || "${port#/dev/cu.}" == */* || ! -c "$port" ]]; then
      printf '%s\n' 'Enter the full /dev/cu.* device path for the Mega from the port detection task.' >&2
      exit 1
    fi
    ;;
  *)
    printf '%s\n' 'Usage: bash scripts/arduino.sh {compile|ports|upload /dev/cu.PORT UPLOAD}' >&2
    exit 2
    ;;
esac

bundled_cli="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
if [[ -n "${ARDUINO_CLI:-}" ]]; then
  cli="$ARDUINO_CLI"
elif command -v arduino-cli >/dev/null 2>&1; then
  cli="$(command -v arduino-cli)"
elif [[ -x "$bundled_cli" ]]; then
  cli="$bundled_cli"
else
  printf '%s\n' 'Arduino CLI is unavailable. Install it or set ARDUINO_CLI to its executable path.' >&2
  exit 1
fi

if [[ "$action" == ports ]]; then
  exec "$cli" board list
fi

mkdir -p "$root/.build"
export ARDUINO_BUILD_CACHE_PATH="${ARDUINO_BUILD_CACHE_PATH:-"$root/.build/cache"}"
build_dir="$(mktemp -d "$root/.build/mega2560.XXXXXXXX")"
trap 'rm -rf -- "$build_dir"' EXIT

"$cli" compile --fqbn "$fqbn" --warnings all --build-path "$build_dir" "$sketch"

if [[ "$action" == upload ]]; then
  "$cli" upload --fqbn "$fqbn" --port "$port" --input-dir "$build_dir" --verify "$sketch"
fi
