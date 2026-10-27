"""Compare two golden fingerprints (from golden.py).

Takes: BASE.json NEW.json [key ...]. Either file may be a comma-separated list of files
       (they are merged). Extra keys are ignored in the comparison (e.g. tick, or stats keys).
Does:  compares failure, tick, photos, route, discovered and every stats value per case.
Returns: prints "<case> DIFF [what changed]" for each difference and a final
         "N identical, M different"; exit code 1 if anything differs or is missing.

Examples:
  python tools/testing/golden_compare.py base.json after.json
  python tools/testing/golden_compare.py base.json after.json tick
"""
import argparse
import json
import sys


def load(spec):
    data = {}
    for path in spec.split(","):
        with open(path) as f:
            data.update(json.load(f))
    return data


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    ap.add_argument("base")
    ap.add_argument("new")
    ap.add_argument("ignore", nargs="*", help="keys to ignore")
    args = ap.parse_args()
    base, new, ignore = load(args.base), load(args.new), set(args.ignore)
    same = diff = 0
    for name in sorted(base):
        if name not in new:
            print(name, "MISSING")
            diff += 1
            continue
        b, n = base[name], new[name]
        bad = [k for k in ("failure", "tick", "photos", "route", "discovered")
               if k not in ignore and b[k] != n[k]]
        bad += [f"stats.{k}={b['stats'][k]}->{n['stats'].get(k)}" for k in b["stats"]
                if k not in ignore and b["stats"][k] != n["stats"].get(k)]
        if bad:
            diff += 1
            print(name, "DIFF", bad)
        else:
            same += 1
    print(f"{same} identical, {diff} different")
    return 1 if diff else 0


if __name__ == "__main__":
    sys.exit(main())
