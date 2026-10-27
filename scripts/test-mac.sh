#!/bin/bash
# Run the Mac test suite (Phase B spec 7.2). Usage: scripts/test-mac.sh [--release]
#   default:   every test except perf, in build/mac (portable, Metal physics/parity, render)
#   --release: the perf gates, in build/mac-release
# Prints ALL PASSED, or the step that failed, last.
set -uo pipefail
repo="$(cd "$(dirname "$0")/.." && pwd)"
export TMPDIR="$repo/build/tmp"
mkdir -p "$TMPDIR"
export xcrun_db="$TMPDIR/xcrun_db"  # xcrun's lookup cache, otherwise in /var/folders on the system drive
cd "$repo"
fail() { echo "FAILED: $1"; exit 1; }
if [[ "${1:-}" == "--release" ]]; then
  ctest --preset mac-release -L perf || fail "ctest perf (mac-release)"
  echo "ALL PASSED"
  exit 0
fi
ctest --preset mac -LE perf || fail "ctest (mac)"

# App self-test (Phase B spec 7.2, 7.3): drape the tee live in the window, then check the gates.
app="build/mac/apps/drape/Drape.app/Contents/MacOS/Drape"
[[ -x "$app" ]] || fail "app self-test: $app not built (is Qt installed?)"
out="$("$app" --selftest build/mac/selftest.png)" || { echo "$out"; fail "app self-test exited non-zero"; }
echo "$out"
value() { echo "$out" | sed -n "s/.*$1=\([^ ]*\).*/\1/p" | tail -1; }
[[ "$(value backend)" == "metal" ]] || fail "self-test did not run on Metal"
awk -v v="$(value launch_ms)" 'BEGIN { exit !(v != "" && v < 3000) }' || fail "launch_ms $(value launch_ms) >= 3000"
awk -v v="$(value fps_asleep)" 'BEGIN { exit !(v != "" && v >= 59.5) }' || fail "fps_asleep $(value fps_asleep) < 60"
# Spec 7.2's 5-40% band is for an 800x1000 portrait render. The app frames the same body height in a 1280x800
# landscape window, twice the area, so the tee's share halves: 2.5-20%.
awk -v v="$(value tee_coverage)" 'BEGIN { exit !(v != "" && v >= 0.025 && v <= 0.20) }' \
  || fail "tee_coverage $(value tee_coverage) outside [0.025, 0.20]"
echo "ALL PASSED"
