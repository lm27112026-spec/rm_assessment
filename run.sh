#!/bin/sh
set -eu

TARGET="${1:?Usage: $0 <target> [Debug|Release]}"
CONFIG="${2:-Debug}"
ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$ROOT/build"

# --- configure (first time only) ---
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    cmake -S "$ROOT" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE="$CONFIG"
fi

# --- build ---
cmake --build "$BUILD_DIR" --target "$TARGET" -- -j"$(nproc)"

# --- locate executable (handles both single-config and multi-config generators) ---
EXE="$BUILD_DIR/$TARGET"
if [ -f "$BUILD_DIR/$CONFIG/$TARGET" ]; then
    EXE="$BUILD_DIR/$CONFIG/$TARGET"
fi

if [ ! -f "$EXE" ]; then
    echo "Error: executable not found: $EXE" >&2
    exit 1
fi

exec "$EXE"
