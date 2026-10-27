#!/bin/bash
# Configure and build Drape on a Mac (Phase B spec 3.3). Usage: scripts/build-mac.sh [--release]
set -euo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
preset=mac
[[ "${1:-}" == "--release" ]] && preset=mac-release
# Compiler and tool temp files go next to the build, not on the (small) system drive.
export TMPDIR="$repo/build/tmp"
mkdir -p "$TMPDIR"
export xcrun_db="$TMPDIR/xcrun_db"  # xcrun's lookup cache, otherwise in /var/folders on the system drive
# Qt: Homebrew (in the preset), or an official Qt from aqtinstall at DRAPE_QT_DIR (default: the newest
# tools/qt/6.*/macos next to the repo), for Macs whose system drive has no room for Homebrew Qt.
qt="${DRAPE_QT_DIR:-}"
if [[ -z "$qt" ]]; then
  for d in "$repo"/../tools/qt/6.*/macos; do [[ -d "$d" ]] && qt="$d"; done
fi
if [[ -n "$qt" ]]; then
  export CMAKE_PREFIX_PATH="$(cd "$qt" && pwd)${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi
cd "$repo"
cmake --preset "$preset"
cmake --build --preset "$preset"
