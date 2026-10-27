"""Shared bootstrap for the tools in this folder.

Does: puts the repo root (two folders above this file) on sys.path, so the tools run from
      any directory and on any snapshot copy of the repo, with no PYTHONPATH needed.
Returns: REPO (pathlib.Path of the repo root the tools are running against).
"""
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
