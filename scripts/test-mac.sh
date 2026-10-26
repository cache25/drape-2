#!/bin/bash
# Run the Mac test suite (Phase B spec 7.2). Usage: scripts/test-mac.sh [--release]
#   default:   every test except perf, in build/mac (portable, Metal physics/parity, render)
#   --release: the perf gates, in build/mac-release
# Prints ALL PASSED, or the step that failed, last.
set -uo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
export TMPDIR="$repo/build/tmp"
mkdir -p "$TMPDIR"
cd "$repo"
fail() { echo "FAILED: $1"; exit 1; }
if [[ "${1:-}" == "--release" ]]; then
  ctest --preset mac-release -L perf || fail "ctest perf (mac-release)"
else
  ctest --preset mac -LE perf || fail "ctest (mac)"
fi
echo "ALL PASSED"
