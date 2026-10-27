#!/bin/bash
# Run the flight-code harness on a snapshot (see snapshot.sh). Run it INSIDE WSL: dronekit
# lives in the repo's .venv there.
#
# Takes:   flight_snapshot.sh LABEL [BASE_DIR]      (BASE_DIR default: <repo>/.test_snapshots)
#          env PY=<python>  to override (default: <repo>/.venv/bin/python)
# Does:    multi_drone_sim_test (default field, then "dense") and one_drone_sim_test on the copy;
#          full logs go to <snapshot>/md_default.log, md_dense.log, one.log.
# Returns: summary in <snapshot>/flight_results.txt (also printed); takes ~15-20 min, so run it
#          in the background.
#
# From Git Bash:
#   MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -- bash /mnt/c/Users/harpe/Multirotor/IARC-10/tools/testing/flight_snapshot.sh LABEL
LABEL="${1:?usage: flight_snapshot.sh LABEL [BASE_DIR]}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASE="${2:-$REPO/.test_snapshots}"
SNAP="$BASE/$LABEL"
PY="${PY:-$REPO/.venv/bin/python}"
[ -d "$SNAP" ] || { echo "no snapshot at $SNAP (run snapshot.sh $LABEL first)"; exit 1; }
[ -x "$PY" ] || PY="$SNAP/../../.venv/bin/python"
[ -x "$PY" ] || { echo "no python at $PY; set PY=..."; exit 1; }
OUT="$SNAP/flight_results.txt"
cd "$SNAP" || exit 1
export PYTHONPATH="$SNAP"
: > "$OUT"
for a in "" "dense"; do
  timeout 1200 "$PY" -m integration_tests.multi_drone_sim_test $a > "$SNAP/md_${a:-default}.log" 2>&1
  echo "== multi ${a:-default} exit=$?" >> "$OUT"
  grep -v 'state.json' "$SNAP/md_${a:-default}.log" | grep -E 'SIMULATION|reported|agree|RESULT|Traceback|ABORT' >> "$OUT"
done
timeout 900 "$PY" -m integration_tests.one_drone_sim_test > "$SNAP/one.log" 2>&1
echo "== one exit=$?" >> "$OUT"
tail -n 6 "$SNAP/one.log" >> "$OUT"
echo "== DONE" >> "$OUT"
cat "$OUT"
