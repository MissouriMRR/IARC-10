#!/usr/bin/env bash
# Pull flight logs off the four Pis into one local run directory.
#
#   ./tools/pull_pi_logs.sh                # newest run on each Pi
#   ./tools/pull_pi_logs.sh -n 3           # newest 3 runs on each Pi
#   ./tools/pull_pi_logs.sh -o Logs/badflight
#   ./tools/pull_pi_logs.sh -d 2 -d 4      # only drones 2 and 4
#   ./tools/pull_pi_logs.sh -k ~/.ssh/other_key
#
# Set SSH_KEY (or pass -k) if your key is not ~/.ssh/id_iarc. A non-default key
# name is not offered by ssh unless it is named explicitly, and BatchMode blocks
# the password fallback, so the whole run fails with "Permission denied".
#
# Each Pi invents its own run id (the systemd unit does not set FLIGHT_LOG_RUN),
# so the newest run directory on each Pi is a different name. This flattens the
# per-Pi drone_<id>.{jsonl,log} files into one local directory that
# tools/analyze_flight.py can read directly, and grabs the matching journald
# output for the unit alongside it.
set -uo pipefail

HOSTS=(
    "1 mrrdt-1@10.106.95.111"
    "2 mrrdt-2@10.106.88.135"
    "3 mrrdt-3@10.106.97.2"
    "4 mrrdt-4@10.106.94.190"
)

RUNS=1
OUT=""
JOURNAL_SINCE="-2 hours"
WANTED=()
SSH_KEY="${SSH_KEY:-$HOME/.ssh/id_iarc}"

while getopts "n:o:s:d:k:h" opt; do
    case "$opt" in
        n) RUNS="$OPTARG" ;;
        o) OUT="$OPTARG" ;;
        s) JOURNAL_SINCE="$OPTARG" ;;
        d) WANTED+=("$OPTARG") ;;
        k) SSH_KEY="$OPTARG" ;;
        h) sed -n '2,17p' "$0"; exit 0 ;;
        *) exit 2 ;;
    esac
done

SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=10)
if [ -f "$SSH_KEY" ]; then
    # IdentitiesOnly stops ssh from burning attempts on every agent key first;
    # some sshd configs cut you off at MaxAuthTries before the right one is tried.
    SSH_OPTS+=(-i "$SSH_KEY" -o IdentitiesOnly=yes)
    echo "Using key $SSH_KEY"
else
    echo "No key at $SSH_KEY -- relying on ssh defaults (-k to point elsewhere)"
fi

OUT="${OUT:-Logs/pull_$(date -u +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT"

wanted() {
    [ ${#WANTED[@]} -eq 0 ] && return 0
    for d in "${WANTED[@]}"; do [ "$d" = "$1" ] && return 0; done
    return 1
}

failed=()

for entry in "${HOSTS[@]}"; do
    id="${entry%% *}"
    host="${entry#* }"
    user="${host%@*}"
    wanted "$id" || continue

    echo "=== drone $id ($host) ==="

    # Ask the Pi which run directories are newest. Sorting by name works because
    # run ids are run_<UTC yyyymmdd_HHMMSS>.
    remote_root="/home/$user/IARC-10/Logs"
    if ! ssh "${SSH_OPTS[@]}" "$host" true 2>"$OUT/drone_${id}.ssh.err"; then
        echo "  !! cannot ssh to $host:"
        sed 's/^/     /' "$OUT/drone_${id}.ssh.err"
        failed+=("$id")
        continue
    fi
    rm -f "$OUT/drone_${id}.ssh.err"

    mapfile -t runs < <(ssh "${SSH_OPTS[@]}" "$host" \
        "ls -1d $remote_root/run_* 2>/dev/null | sort | tail -n $RUNS")
    if [ ${#runs[@]} -eq 0 ]; then
        echo "  !! no run_* directories under $remote_root -- the flight code never wrote any"
        failed+=("$id")
    fi

    for rd in "${runs[@]}"; do
        name="$(basename "$rd")"
        dest="$OUT/$name"
        mkdir -p "$dest"
        echo "  $name -> $dest"
        # Only this drone's files; another Pi may have written into a same-named
        # directory if the run ids happen to collide.
        scp -q "${SSH_OPTS[@]}" "$host:$rd/drone_${id}.*" "$dest/" || {
            echo "  !! scp failed for $name"
            failed+=("$id")
        }
    done

    # journald has the stdout/stderr of the unit, including the traceback that a
    # crash produces before the file logger ever sees it.
    ssh "${SSH_OPTS[@]}" "$host" \
        "journalctl -u drone-flight@$id --since '$JOURNAL_SINCE' --no-pager" \
        > "$OUT/drone_${id}.journal" 2>"$OUT/drone_${id}.journal.err"
    if [ ! -s "$OUT/drone_${id}.journal" ]; then
        echo "  !! journal empty; see $OUT/drone_${id}.journal.err"
    else
        rm -f "$OUT/drone_${id}.journal.err"
        echo "  journal -> $OUT/drone_${id}.journal ($(wc -l < "$OUT/drone_${id}.journal") lines)"
    fi
done

# Flatten the newest run from each Pi into one directory so the analyzer sees a
# complete four-drone run.
merged="$OUT/merged"
mkdir -p "$merged"
for entry in "${HOSTS[@]}"; do
    id="${entry%% *}"
    wanted "$id" || continue
    newest="$(ls -1d "$OUT"/run_* 2>/dev/null | sort | tac | while read -r d; do
        [ -e "$d/drone_${id}.jsonl" ] && { echo "$d"; break; }
    done)"
    [ -n "$newest" ] && cp "$newest"/drone_"${id}".* "$merged/" 2>/dev/null
done

echo
echo "Logs in $OUT"
ls -1 "$merged" 2>/dev/null | sed 's/^/  merged\//'
echo
echo "Analyze with:  python tools/analyze_flight.py $merged"
[ ${#failed[@]} -gt 0 ] && echo "Drones with problems: ${failed[*]}"
exit 0
