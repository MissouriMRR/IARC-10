#!/bin/bash
# Freeze the working tree into a folder, so tests can run on it while you keep editing.
#
# Takes:   snapshot.sh LABEL [BASE_DIR]     (BASE_DIR default: <repo>/.test_snapshots)
# Does:    copies code and configs (flight/ state_machine/ interdrone_communication/
#          integration_tests/ tests/ tools/, vision/ *.py and *.json only, root *.py *.json),
#          minus __pycache__; writes SNAPSHOT_TIME.txt and SNAPSHOT_DIFFSTAT.txt into it.
# Returns: prints the snapshot folder. Exit 1 on bad usage.
#
# Then run tests on the copy (Git Bash / Linux / WSL):
#   cd <snapshot> && PYTHONPATH="$PWD" python -m flight.pathfinding.pathnode.test_pathnode
#   bash tools/testing/flight_snapshot.sh LABEL        (flight harness, WSL, ~15-20 min)
# Add .test_snapshots/ to .gitignore so the copies are never committed.
set -e
LABEL="${1:?usage: snapshot.sh LABEL [BASE_DIR]}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BASE="${2:-$REPO/.test_snapshots}"
DEST="$BASE/$LABEL"
rm -rf "$DEST"
mkdir -p "$DEST"
cd "$REPO"
for d in flight state_machine interdrone_communication integration_tests tests tools; do
  [ -e "$d" ] && cp -r "$d" "$DEST/"
done
# vision: code and configs only (the model files are large and the tests do not load them)
find vision \( -name "*.py" -o -name "*.json" \) -not -path "*/__pycache__/*" | while read -r f; do
  mkdir -p "$DEST/$(dirname "$f")"; cp "$f" "$DEST/$f"
done
cp ./*.py ./*.json "$DEST/" 2>/dev/null || true
find "$DEST" -name __pycache__ -type d -prune -exec rm -rf {} +
git diff --stat HEAD > "$DEST/SNAPSHOT_DIFFSTAT.txt" 2>/dev/null || true
date > "$DEST/SNAPSHOT_TIME.txt"
echo "$DEST"
