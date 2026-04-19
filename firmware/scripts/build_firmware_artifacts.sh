#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
FIRMWARE_DIR=$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)

cd "$FIRMWARE_DIR"

if command -v pio >/dev/null 2>&1; then
  PIO_CMD="pio"
elif command -v platformio >/dev/null 2>&1; then
  PIO_CMD="platformio"
elif [ -n "${USERPROFILE:-}" ] && [ -x "$USERPROFILE/.platformio/penv/Scripts/platformio.exe" ]; then
  PIO_CMD="$USERPROFILE/.platformio/penv/Scripts/platformio.exe"
else
  echo "[build-artifacts] PlatformIO CLI not found (pio/platformio)." >&2
  echo "[build-artifacts] Install PlatformIO CLI or add it to PATH." >&2
  exit 1
fi

echo "[build-artifacts] Building debug firmware..."
"$PIO_CMD" run -e debug

echo "[build-artifacts] Building release firmware..."
"$PIO_CMD" run -e release

copy_artifacts() {
  ENV_NAME="$1"
  SRC_DIR="$FIRMWARE_DIR/.pio/build/$ENV_NAME"
  DST_DIR="$FIRMWARE_DIR/builds/$ENV_NAME"

  mkdir -p "$DST_DIR"

  for artifact in firmware.elf firmware.hex firmware.zip; do
    if [ -f "$SRC_DIR/$artifact" ]; then
      cp -f "$SRC_DIR/$artifact" "$DST_DIR/$artifact"
      echo "[build-artifacts] Copied $SRC_DIR/$artifact -> $DST_DIR/$artifact"
    fi
  done
}

copy_artifacts "debug"
copy_artifacts "release"

echo "[build-artifacts] Artifact sync complete."
