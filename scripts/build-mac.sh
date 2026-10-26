#!/bin/bash
# Configure and build Drape on a Mac (Phase B spec 3.3). Usage: scripts/build-mac.sh [--release]
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
preset=mac
[[ "${1:-}" == "--release" ]] && preset=mac-release
# Compiler and tool temp files go next to the build, not on the (small) system drive.
export TMPDIR="$repo/build/tmp"
mkdir -p "$TMPDIR"
cd "$repo"
cmake --preset "$preset"
cmake --build --preset "$preset"
