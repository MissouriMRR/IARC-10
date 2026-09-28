"""Multi-core verification runner: every test suite at once, on a frozen copy of the tree.

Takes:   [--snapshot LABEL | --tree PATH] [--only unit,nodefield,cellfield,sweep,flight,pytest,lint]
         [--flight-repeat N] [--jobs N] [--serial-flight] [--lint-baseline FILE]
Does:    1. freezes the working tree (like snapshot.sh) and prints the snapshot path;
         2. splits every suite into small jobs and runs them in one process pool, longest
            first (durations are cached in <repo>/.test_snapshots/verify_durations.json);
         3. prints a line per finished job, then a summary table and the failures.
Returns: exit 0 if every job passed (known pytest failures and lint warnings do not count),
         1 on any real failure or timeout, 130 on Ctrl-C.

Suites (--only names):
  unit       each test_* of flight/pathfinding/pathnode/test_pathnode.py, one process each
  nodefield  each test_* of flight/pathfinding/nodeField/test_nodefield.py (the slow
             completeness test is cut into 10 shards of layouts)
  cellfield  flight/pathfinding/tests/cellFieldTest.py as a whole (timing tests: retried once)
  sweep      the 90-seed set of sweep.py --standard, one process per seed
  flight     WSL flight harness: multi 2, 3, 4, 2 dense, 3 dense, 4 dense, one-drone;
             each its own process (4 dense x3 by default)
  pytest     tests/ in WSL; the 3 known test_emergency_land.py failures are tolerated
  lint       pyflakes in WSL; reported, never fails

Logs: <tree>/verify_logs/<job>.log, plus summary.txt and lint_warnings.txt.
Flight configs are timing sensitive (see docs/testing/tools.md): --serial-flight runs them
one at a time (still alongside the other suites).

Examples:
  python tools/testing/verify.py
  python tools/testing/verify.py --only unit,sweep --jobs 12
  python tools/testing/verify.py --tree . --only flight --serial-flight
"""
import argparse
import ast
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common  # noqa: E402

REPO = _common.REPO
SUITES = ("unit", "nodefield", "cellfield", "sweep", "flight", "pytest", "lint")
DISTRO = "Ubuntu-24.04"
UNIT_TIMEOUT, FLIGHT_TIMEOUT, SWEEP_TIMEOUT, MISC_TIMEOUT = 1800, 1200, 1200, 600
NODEFIELD_SHARDS = 10
SHARDED = "test_completeness_vs_bruteforce"
KNOWN_PYTEST = {
    "tests/test_emergency_land.py::TestMessageHandling::test_emergency_land_acts_once_on_duplicates",
    "tests/test_emergency_land.py::TestMessageHandling::test_land_starts_an_orderly_landing",
    "tests/test_emergency_land.py::TestMessageHandling::test_land_does_not_override_an_emergency_land",
}
PYTEST_EXPECT_PASSED = 44
LINT_DIRS = ["flight", "state_machine", "interdrone_communication", "integration_tests", "tools"]
FLIGHT_CONFIGS = [("multi", "2", False), ("multi", "3", False), ("multi", "4", False),
                  ("multi", "2", True), ("multi", "3", True), ("multi", "4", True),
                  ("one", "", False)]
DURATIONS = REPO / ".test_snapshots" / "verify_durations.json"
DEFAULT_EST = {"unit": 120, "nodefield": 60, "cellfield": 60, "sweep": 40, "flight": 300,
               "pytest": 30, "lint": 30}

_live = set()                 # running Popen objects, for Ctrl-C cleanup
_live_lock = threading.Lock()
_abort = threading.Event()
_print_lock = threading.Lock()


# -- snapshot ----------------------------------------------------------------------------


def take_snapshot(label):
    """Copy the code and small configs (same set as snapshot.sh) to .test_snapshots/LABEL."""
    dest = REPO / ".test_snapshots" / label
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    skip = shutil.ignore_patterns("__pycache__", "*.pyc")
    for d in ("flight", "state_machine", "interdrone_communication", "integration_tests",
              "tests", "tools"):
        if (REPO / d).is_dir():
            shutil.copytree(REPO / d, dest / d, ignore=skip)
    for f in (REPO / "vision").rglob("*") if (REPO / "vision").is_dir() else []:
        if f.suffix in (".py", ".json") and "__pycache__" not in f.parts:
            (dest / f.relative_to(REPO)).parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(f, dest / f.relative_to(REPO))
    for f in list(REPO.glob("*.py")) + list(REPO.glob("*.json")) + [REPO / "pyproject.toml",
                                                                      REPO / "uv.lock"]:
        if f.is_file():
            shutil.copy2(f, dest / f.name)
    try:
        stat = subprocess.run(["git", "diff", "--stat", "HEAD"], cwd=REPO, capture_output=True,
                              text=True, timeout=60).stdout
    except (OSError, subprocess.SubprocessError):
        stat = ""
    (dest / "SNAPSHOT_DIFFSTAT.txt").write_text(stat)
    (dest / "SNAPSHOT_TIME.txt").write_text(datetime.datetime.now().isoformat() + "\n")
    return dest


# -- job model ---------------------------------------------------------------------------


class Job:
    """One process to run. `check(returncode, output)` returns (status, note); status is
    'pass', 'fail' or 'known'."""

    def __init__(self, suite, name, argv, cwd, env=None, timeout=UNIT_TIMEOUT, check=None,
                 est=None, retry=0, cleanup=None, key=None):
        self.suite, self.name, self.argv, self.cwd = suite, name, argv, cwd
        self.env, self.timeout, self.check = env or {}, timeout, check
        self.key = key or f"{suite}:{name}"
        self.est = est
        self.retry, self.cleanup = retry, cleanup
        self.status = self.note = self.output = None
        self.start = self.end = None
        self.elapsed = 0.0

    def ok_default(self, code, out):
        return ("pass", "") if code == 0 else ("fail", f"exit {code}")


def _kill_tree(proc):
    """Kill a process and its children (Windows: taskkill /T)."""
    try:
        if os.name == "nt":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(proc.pid)],
                           capture_output=True, timeout=30)
        else:
            proc.kill()
    except (OSError, subprocess.SubprocessError):
        pass


def _run_once(job):
    env = dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUTF8="1", PYTHONUNBUFFERED="1",
               MSYS_NO_PATHCONV="1")
    env.update(job.env)
    proc = subprocess.Popen(job.argv, cwd=job.cwd, env=env, stdin=subprocess.DEVNULL,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    with _live_lock:
        _live.add(proc)
    try:
        out, _ = proc.communicate(timeout=job.timeout)
        timed_out = False
    except subprocess.TimeoutExpired:
        _kill_tree(proc)
        if job.cleanup:
            job.cleanup()
        out, _ = proc.communicate()
        timed_out = True
    finally:
        with _live_lock:
            _live.discard(proc)
    text = out.decode("utf-8", "replace") if out else ""
    return proc.returncode, text, timed_out


def run_job(job, on_done):
    """Run one job (with its retries), fill in status/note/output and report it."""
    if _abort.is_set():
        job.status, job.note, job.output = "fail", "aborted", ""
        return job
    job.start = time.time()
    check = job.check or job.ok_default
    for attempt in range(job.retry + 1):
        code, text, timed_out = _run_once(job)
        if timed_out:
            job.status, job.note = "fail", f"timed out after {job.timeout}s"
        elif _abort.is_set():
            job.status, job.note = "fail", "aborted"
        else:
            job.status, job.note = check(code, text)
        job.output = text
        if job.status != "fail" or timed_out or _abort.is_set():
            if attempt and job.status == "pass":
                job.note = (job.note + " (passed on retry)").strip()
            break
    job.end = time.time()
    job.elapsed = job.end - job.start
    on_done(job)
    return job


# -- checks ------------------------------------------------------------------------------


def check_flight(kind, drones):
    def check(code, out):
        problems = []
        if kind == "one":
            if "ERROR                 : none" not in out:
                problems.append("error line not 'none'")
            if "route check           : ok" not in out:
                problems.append("route check not ok")
        else:
            if "RESULT: PASS" not in out:
                problems.append("no 'RESULT: PASS'")
            if "full field, continuous, clear" not in out:
                problems.append("route not 'full field, continuous, clear'")
            if "final mine sets agree : True" not in out:
                problems.append("mine sets do not agree")
        if code != 0:
            problems.append(f"exit {code}")
        return ("fail", "; ".join(problems)) if problems else ("pass", "")
    return check


def check_pytest(code, out):
    failed = set(re.findall(r"^FAILED (\S+)", out, re.M))
    m = re.search(r"(\d+) passed", out)
    passed = int(m.group(1)) if m else 0
    unknown = {f.split(" - ")[0] for f in failed} - KNOWN_PYTEST
    if unknown:
        return "fail", "unexpected failures: " + ", ".join(sorted(unknown))
    if passed < PYTEST_EXPECT_PASSED:
        return "fail", f"only {passed} passed (expected {PYTEST_EXPECT_PASSED})"
    known = {f.split(" - ")[0] for f in failed} & KNOWN_PYTEST
    if known:
        return "known", f"{passed} passed, {len(known)} known failures"
    return "pass", f"{passed} passed"


def _lint_norm(out):
    """pyflakes lines without line numbers, so a baseline survives edits."""
    return sorted({re.sub(r":\d+(:\d+)?:? ", ": ", ln.strip()) for ln in out.splitlines()
                   if re.match(r".+\.py:\d+", ln.strip())})


# -- job builders ------------------------------------------------------------------------


def wsl_path(p):
    p = Path(p).resolve()
    drive = p.drive.rstrip(":").lower()
    return f"/mnt/{drive}" + "/".join([""] + list(p.parts[1:]))


def find_venv_python():
    """The repo .venv python as a WSL path (looks in the tree, then its parents)."""
    if os.environ.get("VERIFY_WSL_PY"):
        return os.environ["VERIFY_WSL_PY"]
    for base in (REPO, *REPO.parents):
        if (base / ".venv" / "pyvenv.cfg").exists():   # bin/python is a WSL symlink: unreadable here
            return wsl_path(base / ".venv") + "/bin/python"
    return wsl_path(REPO / ".venv") + "/bin/python"


def discover_tests(path):
    """Names of top-level test_* functions, in file order."""
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    return [n.name for n in tree.body if isinstance(n, ast.FunctionDef)
            and n.name.startswith("test_")]


def unit_code(module, name, shard=None):
    """python -c source that runs one test function of `module`."""
    src = ["import importlib", f"m = importlib.import_module({module!r})"]
    if shard:
        i, k = shard
        src += ["import builtins", "L = m.LAYOUTS", f"keep = set(builtins.range({i}, L, {k}))",
                "_r = builtins.range",
                "m.range = lambda *a: [x for x in _r(*a) if x in keep] if a == (L,) else _r(*a)"]
    src.append(f"m.{name}()")
    return "; ".join(src)


def build_jobs(suites, tree, opts):
    py, jobs = sys.executable, []
    env = {"PYTHONPATH": str(tree)}
    wsl_tree = wsl_path(tree)
    if "unit" in suites:
        mod = "flight.pathfinding.pathnode.test_pathnode"
        for name in discover_tests(tree / "flight/pathfinding/pathnode/test_pathnode.py"):
            jobs.append(Job("unit", name, [py, "-c", unit_code(mod, name)], tree, env))
    if "nodefield" in suites:
        mod = "flight.pathfinding.nodeField.test_nodefield"
        for name in discover_tests(tree / "flight/pathfinding/nodeField/test_nodefield.py"):
            if name == SHARDED:
                for i in range(NODEFIELD_SHARDS):
                    jobs.append(Job("nodefield", f"{name}[{i}/{NODEFIELD_SHARDS}]",
                                    [py, "-c", unit_code(mod, name, (i, NODEFIELD_SHARDS))],
                                    tree, env))
            else:
                jobs.append(Job("nodefield", name, [py, "-c", unit_code(mod, name)], tree, env))
    if "cellfield" in suites:
        def check_cell(code, out):
            m = re.search(r"(\d+)/(\d+) passed", out)
            good = code == 0 and m and m.group(1) == m.group(2)
            return ("pass", m.group(0)) if good else ("fail", f"exit {code}")
        jobs.append(Job("cellfield", "cellFieldTest",
                        [py, "-m", "flight.pathfinding.tests.cellFieldTest"], tree, env,
                        MISC_TIMEOUT, check_cell, retry=1))
    if "sweep" in suites:
        std = _standard_set(tree)
        for tag, (n, s0, s1, drop, delay) in std.items():
            for seed in range(s0, s1):
                code = (f"import sys; sys.path.insert(0, {str(tree / 'tools' / 'testing')!r}); "
                        f"import sweep; bad, line = sweep.run_seed({(tag, n, seed, drop, delay, 4)!r}); "
                        f"print(line); sys.exit(1 if bad else 0)")
                jobs.append(Job("sweep", f"{tag}-{seed}", [py, "-c", code], tree, env,
                                SWEEP_TIMEOUT, key=f"sweep:{tag}:{seed}"))
    if "flight" in suites:
        vpy = find_venv_python()
        for kind, drones, dense in FLIGHT_CONFIGS:
            label = "one-drone" if kind == "one" else f"multi-{drones}{'-dense' if dense else ''}"
            repeat = opts.flight_repeat or (3 if (drones == "4" and dense) else 1)
            module = ("integration_tests.one_drone_sim_test" if kind == "one"
                      else "integration_tests.multi_drone_sim_test")
            args = " ".join(x for x in (drones, "dense" if dense else "") if x)
            script = (f"cd '{wsl_tree}' && PYTHONPATH=\"$PWD\" "
                      f"timeout -k 5 {FLIGHT_TIMEOUT} '{vpy}' -m {module} {args}")
            for r in range(repeat):
                name = label + (f"#{r + 1}" if repeat > 1 else "")
                jobs.append(Job("flight", name, ["wsl", "-d", DISTRO, "--", "bash", "-c", script],
                                tree, None, FLIGHT_TIMEOUT + 60, check_flight(kind, drones),
                                cleanup=lambda: wsl_cleanup(wsl_tree), key=f"flight:{label}"))
    if "pytest" in suites:
        venv = wsl_path(_project_root(tree) / ".venv")
        script = (f"cd '{wsl_tree}' && UV_PROJECT_ENVIRONMENT='{venv}' ~/.local/bin/uv run "
                  f"--no-sync --with pytest python -m pytest -q tests")
        jobs.append(Job("pytest", "tests", ["wsl", "-d", DISTRO, "--", "bash", "-c", script],
                        tree, None, MISC_TIMEOUT, check_pytest,
                        cleanup=lambda: wsl_cleanup(wsl_tree)))
    if "lint" in suites:
        script = f"cd '{wsl_tree}' && ~/.local/bin/uvx pyflakes {' '.join(LINT_DIRS)}"
        baseline = None
        if opts.lint_baseline:
            baseline = set(Path(opts.lint_baseline).read_text().split("\n")) - {""}

        def check_lint(code, out):
            found = _lint_norm(out)
            new = [w for w in found if baseline is not None and w not in baseline]
            note = f"{len(found)} warnings"
            if baseline is not None:
                note += f", {len(new)} new" + (": " + "; ".join(new[:5]) if new else "")
            return "pass", note
        jobs.append(Job("lint", "pyflakes", ["wsl", "-d", DISTRO, "--", "bash", "-c", script],
                        tree, None, MISC_TIMEOUT, check_lint))
    return jobs


def _standard_set(tree):
    """sweep.STANDARD read from the tree's own sweep.py (no import side effects)."""
    src = (tree / "tools" / "testing" / "sweep.py").read_text(encoding="utf-8")
    for node in ast.parse(src).body:
        if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") == "STANDARD":
            return ast.literal_eval(node.value)
    raise SystemExit("STANDARD not found in sweep.py")


def _project_root(tree):
    """Folder holding pyproject.toml + the uv environment (the tree, else the main repo)."""
    for base in (tree, REPO, *REPO.parents):
        if (base / ".venv").exists() and (base / "pyproject.toml").exists():
            return base
    return REPO


def wsl_cleanup(wsl_tree):
    """Kill leftover harness processes whose cwd is this tree (a killed wsl.exe leaves them)."""
    script = ("for p in $(pgrep -f 'integration_tests\\.|pytest'); do "
              f"[ \"$(readlink /proc/$p/cwd)\" = '{wsl_tree}' ] && kill -9 $p; done; true")
    try:
        subprocess.run(["wsl", "-d", DISTRO, "--", "bash", "-c", script],
                       capture_output=True, timeout=60, stdin=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        pass


# -- running -----------------------------------------------------------------------------


def load_durations():
    try:
        return json.loads(DURATIONS.read_text())
    except (OSError, ValueError):
        return {}


def save_durations(jobs, old):
    new = dict(old)
    for j in jobs:
        if j.status in ("pass", "known") and j.elapsed:
            new[j.key] = max(round(j.elapsed, 1), new.get(j.key, 0) * 0.5) if j.key in new \
                else round(j.elapsed, 1)
    try:
        DURATIONS.parent.mkdir(parents=True, exist_ok=True)
        DURATIONS.write_text(json.dumps(new, indent=1, sort_keys=True))
    except OSError:
        pass


def execute(jobs, args, logdir):
    """Run all jobs in a pool (flight jobs in their own 1-worker pool with --serial-flight)."""
    total, done = len(jobs), [0]
    t0 = time.time()

    def on_done(job):
        (logdir / (re.sub(r"[^\w.-]+", "_", job.suite + "_" + job.name) + ".log")).write_text(
            f"# {job.suite} {job.name}: {job.status} {job.note}\n# {' '.join(map(str, job.argv))[:300]}\n\n"
            + (job.output or ""), encoding="utf-8")
        with _print_lock:
            done[0] += 1
            tag = {"pass": "PASS ", "fail": "FAIL ", "known": "KNOWN"}[job.status]
            note = f"  {job.note}" if job.note else ""
            print(f"[{done[0]:3d}/{total}] {tag} {job.suite:<9} {job.name:<40} "
                  f"{job.elapsed:6.1f}s  (t+{time.time() - t0:.0f}s){note}", flush=True)

    ordered = sorted(jobs, key=lambda j: -(j.est or 0))
    serial = [j for j in ordered if j.suite == "flight"] if args.serial_flight else []
    rest = [j for j in ordered if j not in serial]
    pools, futures = [], []
    try:
        main_pool = ThreadPoolExecutor(max_workers=args.jobs)
        pools.append(main_pool)
        futures += [main_pool.submit(run_job, j, on_done) for j in rest]
        if serial:
            fp = ThreadPoolExecutor(max_workers=1)
            pools.append(fp)
            futures += [fp.submit(run_job, j, on_done) for j in serial]
        for f in futures:
            f.result()
        return True
    except KeyboardInterrupt:
        _abort.set()
        print("\nCtrl-C: killing running jobs ...", flush=True)
        with _live_lock:
            for p in list(_live):
                _kill_tree(p)
        for p in pools:
            p.shutdown(wait=False, cancel_futures=True)
        return False
    finally:
        for p in pools:
            p.shutdown(wait=True, cancel_futures=True)


def summarize(jobs, wall, logdir, tree):
    lines = [f"{'suite':<10}{'jobs':>5}{'passed':>8}{'failed':>8}{'known':>7}{'wall':>9}{'cpu-sum':>9}"]
    for s in SUITES:
        js = [j for j in jobs if j.suite == s and j.status]
        if not js:
            continue
        starts = [j.start for j in js if j.start]
        wall_s = (max(j.end for j in js if j.end) - min(starts)) if starts else 0
        lines.append(f"{s:<10}{len(js):>5}{sum(j.status == 'pass' for j in js):>8}"
                     f"{sum(j.status == 'fail' for j in js):>8}{sum(j.status == 'known' for j in js):>7}"
                     f"{wall_s:>8.0f}s{sum(j.elapsed for j in js):>8.0f}s")
    lines.append(f"{'TOTAL':<10}{len(jobs):>5}{'':>8}{'':>8}{'':>7}{wall:>8.0f}s")
    failed = [j for j in jobs if j.status == "fail"]
    lines.append("")
    if failed:
        lines.append(f"FAILURES ({len(failed)}):")
        for j in failed:
            lines.append(f"--- {j.suite} {j.name}: {j.note}")
            tail = (j.output or "").rstrip().splitlines()[-15:]
            lines += ["    " + t for t in tail]
    else:
        lines.append("ALL PASSED")
    for j in jobs:
        if j.suite == "lint" and j.status:
            lines.append(f"lint: {j.note}")
            (logdir / "lint_warnings.txt").write_text("\n".join(_lint_norm(j.output or "")) + "\n")
    lines.append(f"logs: {logdir}")
    text = "\n".join(lines)
    (logdir / "summary.txt").write_text(text + "\n", encoding="utf-8")
    return text


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("\n\n", 1)[1])
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--snapshot", metavar="LABEL",
                     help="freeze the tree into .test_snapshots/LABEL (default: verify-<time>)")
    src.add_argument("--tree", metavar="PATH", help="test this folder as it is (no snapshot)")
    ap.add_argument("--only", default=",".join(SUITES), help="comma list of suites (default: all)")
    ap.add_argument("--flight-repeat", type=int, default=0, metavar="N",
                    help="run every flight config N times (default: 4 dense x3, others x1)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) - 2),
                    help="parallel processes (default: CPU count - 2)")
    ap.add_argument("--serial-flight", action="store_true",
                    help="run the flight configs one at a time (other suites stay parallel)")
    ap.add_argument("--lint-baseline", metavar="FILE",
                    help="lint_warnings.txt of an earlier run; new warnings are listed")
    args = ap.parse_args()

    suites = [s.strip() for s in args.only.split(",") if s.strip()]
    bad = [s for s in suites if s not in SUITES]
    if bad:
        ap.error(f"unknown suite(s) {bad}; choose from {','.join(SUITES)}")

    if args.tree:
        tree = Path(args.tree).resolve()
    else:
        label = args.snapshot or "verify-" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        tree = take_snapshot(label)
    print(f"tree: {tree}", flush=True)
    logdir = tree / "verify_logs"
    logdir.mkdir(exist_ok=True)

    jobs = build_jobs(suites, tree, args)
    old = load_durations()
    for j in jobs:
        j.est = old.get(j.key, DEFAULT_EST[j.suite])
    mode = "serial" if args.serial_flight else "parallel"
    print(f"{len(jobs)} jobs, {args.jobs} workers, flight {mode}", flush=True)

    t0 = time.time()
    finished = execute(jobs, args, logdir)
    wall = time.time() - t0
    ran = [j for j in jobs if j.status]
    print("\n" + summarize(ran, wall, logdir, tree), flush=True)
    if not finished:
        return 130
    save_durations(jobs, old)
    return 1 if any(j.status == "fail" for j in jobs) else 0


if __name__ == "__main__":
    sys.exit(main())
