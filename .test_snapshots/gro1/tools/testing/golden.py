"""Record a behavioural fingerprint of the simulator (to prove a refactor changed nothing).

Takes: OUT.json, optionally case names (default: all 17). --list shows the case names.
Does:  runs each named mission and saves every drone's exact photo sequence, the reported
       route, discovered mines, tick count and planner stats, plus a short hash per case.
Returns: writes OUT.json; prints "<case> <hash> <failure> <photos>" per case.

Workflow for a pure refactor:
  1. python tools/testing/golden.py before.json      (on the old code, e.g. a snapshot)
  2. ...refactor...
  3. python tools/testing/golden.py after.json
  4. python tools/testing/golden_compare.py before.json after.json    -> "17 identical"

Examples:
  python tools/testing/golden.py base.json
  python tools/testing/golden.py base.json d1_s3 walled
"""
import argparse
import hashlib
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402,F401

from flight.pathfinding.pathnode.minefield import generate_true_minefield as g  # noqa: E402
from flight.pathfinding.pathnode.sim import Mission  # noqa: E402

CASES = {}
for _d in (1, 2, 3, 4):
    for _seed in (3, 11, 76):
        CASES[f"d{_d}_s{_seed}"] = dict(mines=90, seed=_seed, drones=_d, kw={})
CASES["loss_s100"] = dict(mines=90, seed=100, drones=4, kw=dict(drop=0.5, delay=3, seed=100))
CASES["loss_s5"] = dict(mines=60, seed=5, drones=4, kw=dict(drop=0.3, delay=5, seed=5))
CASES["yaw_s1"] = dict(mines=30, seed=1, drones=2, kw=dict(photo_yaw_deg="random", max_ticks=1200))
CASES["walled"] = dict(mines=120, seed=13, drones=1, kw={})
CASES["band_s2"] = dict(mines=70, seed=2, drones=4, kw=dict(coverage="band"))


def _pt(p):
    return [round(float(p[0]), 4), round(float(p[1]), 4)]


def record(case):
    """Takes: a CASES entry. Returns: its fingerprint dict (with a 'hash' field)."""
    m = Mission(g(case["mines"], case["seed"]), drones=case["drones"], **case["kw"]).run()
    route = m.report_route()
    s = m.summary()
    rec = {
        "failure": m.failure, "tick": m.tick,
        "photos": {str(d): [_pt(p) for p in v] for d, v in m.photos.items()},
        "route": [_pt(p) for p in route] if route else None,
        "discovered": sorted(_pt(p) for p in m.discovered),
        "stats": {k: v for k, v in s.items() if isinstance(v, (int, float, str)) or v is None},
    }
    rec["hash"] = hashlib.sha256(json.dumps(rec, sort_keys=True).encode()).hexdigest()[:16]
    return rec, s["photos"]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("out", nargs="?", help="output .json file")
    ap.add_argument("cases", nargs="*", help="case names (default: all)")
    ap.add_argument("--list", action="store_true", help="print the case names and exit")
    args = ap.parse_args()
    if args.list:
        print("\n".join(CASES))
        return 0
    if not args.out:
        ap.error("OUT.json is required")
    unknown = [c for c in args.cases if c not in CASES]
    if unknown:
        ap.error(f"unknown case(s) {unknown}; see --list")
    result = {}
    for name in args.cases or CASES:
        rec, photos = record(CASES[name])
        result[name] = rec
        print(name, rec["hash"], rec["failure"], photos, flush=True)
    with open(args.out, "w") as f:
        json.dump(result, f, indent=1, sort_keys=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
