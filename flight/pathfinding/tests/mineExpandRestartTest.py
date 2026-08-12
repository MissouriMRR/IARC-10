"""
Experimental verification for Pathfinder.repair_path_after_expansion -- the
"restart the gambler/assistant loop, now with more mine data" repair
ExpandNodes runs after growing every known mine's safety radius for the
final safety margin (see state_machine/states/impl/expand_nodes_impl.py).

Field.expandField keeps every obstacle-to-obstacle graph edge globally
consistent when it grows several mines at once (see its own docstring for
why one merge-then-rewire pass over every affected obstacle together is
enough for that) -- but it never re-validates a floating-node-to-floating-
node edge (start/end/helper nodes), which is wired once at that node's own
creation time and never revisited afterward. That's the gap
repair_path_after_expansion is meant to close.

This reuses droneWorkflowTest's own simulated discover-as-you-fly harness
(build_empty_pathfinder/generate_true_minefield/simulate_one_pair_maze) so
the mines and the committed route come from an actual scan, not an
artificial hand-placed field -- "the previously existing mines for this
run" -- then expands the mine radius and checks whether the already-
committed route (pf.get_maze_path()) still avoids every mine, BOTH with
and without the repair, across many random minefield seeds. Checking both
is the point: if the no-repair case never actually breaks anything, the
repair would be solving a problem that doesn't occur in practice, and the
comparison is what proves it isn't vacuous.
"""

import random

from flight.pathfinding.tests.droneWorkflowTest import (
    build_empty_pathfinder,
    generate_true_minefield,
    simulate_one_pair_maze,
    _bad_edge_count,
)

# Matches expand_nodes_impl.MINE_RADIUS_EXPANSION_FT -- the real value this
# would run with in the state machine.
EXPANSION_FT = 2.0


def _run_one_seed(seed: int, n_mines: int, expansion_ft: float = EXPANSION_FT) -> dict:
    pf = build_empty_pathfinder()
    true_mines = generate_true_minefield(n=n_mines, seed=seed)
    result = simulate_one_pair_maze(pf, true_mines, max_steps=500, max_waypoints=8000)

    bad_before = _bad_edge_count(pf, pf.get_maze_path())

    pf.increase_radius(expansion_ft)
    bad_no_repair = _bad_edge_count(pf, pf.get_maze_path())

    repair_passes = pf.repair_path_after_expansion()
    bad_after_repair = _bad_edge_count(pf, pf.get_maze_path())

    return {
        "seed": seed,
        "n_mines": n_mines,
        "discovered": len(pf.protoMines),
        "hit_cap": result["hit_cap"],
        "bad_before": bad_before,
        "bad_no_repair": bad_no_repair,
        "bad_after_repair": bad_after_repair,
        "repair_passes": repair_passes,
    }


def sweep(seeds, n_mines: int, expansion_ft: float = EXPANSION_FT) -> bool:
    rows = [_run_one_seed(seed, n_mines, expansion_ft) for seed in seeds]

    baseline_unsafe = sum(1 for r in rows if r["bad_before"] > 0)
    broke_on_expand = sum(1 for r in rows if r["bad_no_repair"] > 0)
    still_broken_after_repair = sum(1 for r in rows if r["bad_after_repair"] > 0)
    hit_cap = sum(1 for r in rows if r["hit_cap"])
    max_passes = max((r["repair_passes"] for r in rows), default=0)
    multi_pass = sum(1 for r in rows if r["repair_passes"] > 1)

    ok = baseline_unsafe == 0 and still_broken_after_repair == 0
    print(
        f"sweep(n_mines={n_mines}, expansion_ft={expansion_ft}, seeds={len(rows)}): "
        f"baseline_unsafe={baseline_unsafe} broke_on_expand={broke_on_expand}/{len(rows)} "
        f"still_broken_after_repair={still_broken_after_repair} hit_cap={hit_cap} "
        f"multi_pass_repairs={multi_pass} max_repair_passes={max_passes} -> "
        f"{'PASS' if ok else 'FAIL'}"
    )
    if not ok:
        for r in rows:
            if r["bad_before"] > 0 or r["bad_after_repair"] > 0:
                print(f"    seed={r['seed']} {r}")
    return ok


def test_expansion_breaks_something_realistic(n_mines=70, n_seeds=25) -> bool:
    """Sanity check on the test itself, not on the fix: confirms the no-
    repair case actually produces bad edges often enough for this sweep to
    mean anything -- if this were 0/N, repair_path_after_expansion would be
    fixing a problem this harness never exercises."""
    rng = random.Random(12345)
    seeds = [rng.randrange(1_000_000) for _ in range(n_seeds)]
    rows = [_run_one_seed(seed, n_mines) for seed in seeds]
    broke_on_expand = sum(1 for r in rows if r["bad_no_repair"] > 0)
    ok = broke_on_expand > 0
    print(
        f"test_expansion_breaks_something_realistic: broke_on_expand={broke_on_expand}/{len(rows)} "
        f"-> {'PASS' if ok else 'FAIL'}"
    )
    return ok


def main():
    results = []

    # Baseline density/count, matching droneWorkflowTest's own default.
    rng = random.Random(2024)
    results.append(sweep([rng.randrange(1_000_000) for _ in range(40)], n_mines=70))

    # Denser field -- more obstacles means more chances for expandField's
    # simultaneous growth to create new crossings, and more chances for a
    # single repair pass to not be enough (see the "Pathfinder bug-audit
    # process" precedent: denser configs are where real bugs surfaced).
    rng = random.Random(4242)
    results.append(sweep([rng.randrange(1_000_000) for _ in range(25)], n_mines=140))

    # Larger safety-margin expansion -- more likely to actually introduce
    # new crossings than the real 2.0ft value, useful as a stress case even
    # though it's not what the state machine actually runs.
    rng = random.Random(9090)
    results.append(sweep([rng.randrange(1_000_000) for _ in range(25)], n_mines=70, expansion_ft=6.0))

    results.append(test_expansion_breaks_something_realistic())

    print()
    print(f"{sum(results)}/{len(results)} passed")


if __name__ == "__main__":
    main()
