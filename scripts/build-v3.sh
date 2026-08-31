#!/bin/sh
# Build UV-K5 V3 (PY32F071) Serial Bridge firmware and copy firmware-v3.bin.
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
DIR="$ROOT/vendor/uv-k5-v3"
PATCH="$ROOT/patches/v3-serial-bridge.patch"
OVERLAY="$ROOT/v3-overlay"
OUT_DIR="$ROOT/compiled-firmware"
TOOLCHAIN_BIN="$ROOT/.toolchain/bin"

. "$ROOT/scripts/fetch-v3.sh"

git -C "$DIR" checkout --force 3bd3ebba2ceb553edc88c3f087ce0c7f420433b2
git -C "$DIR" clean -fd -- App CMakePresets.json

cp -R "$OVERLAY/." "$DIR/"
git -C "$DIR" apply "$PATCH"

if [ -d "$TOOLCHAIN_BIN" ]; then
	PATH="$TOOLCHAIN_BIN:$PATH"
	export PATH
fi

cmake --preset SerialBridge -S "$DIR"
cmake --build "$DIR/build/SerialBridge"

mkdir -p "$OUT_DIR"
cp -f "$DIR/build/SerialBridge/firmware-v3.bin" "$OUT_DIR/firmware-v3.bin"
echo "V3 firmware: $OUT_DIR/firmware-v3.bin  (flash with uvtools2, raw .bin)"
