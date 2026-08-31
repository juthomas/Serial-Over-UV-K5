#!/bin/sh
# Clone F4HWN UV-K5 V3 firmware at the pinned tag if vendor/uv-k5-v3 is missing.
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
DIR="$ROOT/vendor/uv-k5-v3"
URL="https://github.com/armel/uv-k1-k5v3-firmware-custom.git"
TAG="v5.7.0"
PIN="3bd3ebba2ceb553edc88c3f087ce0c7f420433b2"

mkdir -p "$ROOT/vendor"

if [ ! -d "$DIR/.git" ]; then
	if [ -f "$ROOT/.gitmodules" ] && command -v git >/dev/null 2>&1; then
		git -C "$ROOT" submodule update --init --depth 1 vendor/uv-k5-v3 || true
	fi
fi

if [ ! -d "$DIR/.git" ]; then
	git clone --depth 1 --branch "$TAG" "$URL" "$DIR"
fi

git -C "$DIR" checkout --force "$PIN"
