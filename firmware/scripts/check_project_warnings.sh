#!/usr/bin/env bash
set -euo pipefail

env_name="${1:-adafruit_feather_nrf52840}"
tmp_log="$(mktemp)"
trap 'rm -f "$tmp_log"' EXIT

echo "[warn-gate] Building env: ${env_name}"
if ! pio run -e "${env_name}" 2>&1 | tee "$tmp_log"; then
  echo "[warn-gate] Build failed"
  exit 1
fi

echo "[warn-gate] Checking warning lines (excluding vendor paths)..."
warn_lines="$(grep -E ": warning:" "$tmp_log" | grep -vE "src/drivers/|\.pio/libdeps/" || true)"
if [[ -n "$warn_lines" ]]; then
  echo "$warn_lines"
  echo "[warn-gate] FAILED: non-vendor warnings detected"
  exit 1
fi

echo "[warn-gate] PASSED"
