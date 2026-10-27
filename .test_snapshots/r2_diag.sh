#!/bin/bash
# usage: r2_diag.sh SNAP OUT args...
SNAP="$1"; OUT="$2"; shift 2
cd /mnt/c/Users/harpe/Multirotor/IARC-10/.test_snapshots/$SNAP || exit 1
cp ../r2_diag.py .
PYTHONPATH=$PWD timeout 330 /mnt/c/Users/harpe/Multirotor/IARC-10/.venv/bin/python r2_diag.py "$@" > "$OUT" 2>&1
echo "exit $?" >> "$OUT"
