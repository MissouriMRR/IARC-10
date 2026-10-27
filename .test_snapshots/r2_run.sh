#!/bin/bash
# usage: r2_run.sh SNAP OUTFILE ARGS...   (run inside WSL)
SNAP="$1"; OUT="$2"; shift 2
cd /mnt/c/Users/harpe/Multirotor/IARC-10/.test_snapshots/$SNAP || exit 1
PYTHONPATH=$PWD timeout 330 /mnt/c/Users/harpe/Multirotor/IARC-10/.venv/bin/python -m integration_tests.multi_drone_sim_test "$@" > "$OUT" 2>&1
echo "exit $?" >> "$OUT"
