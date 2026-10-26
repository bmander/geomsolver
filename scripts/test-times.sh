#!/bin/bash
# Every test's time, into rust/test-times.tsv: `scripts/test-times.sh [cargo test arguments]`
# (default `-p gcs-core --test core`; `--features gcs-core/slow` times the slow tier too).
#
# The file is what `scripts/test-shard.sh` deals CI's shards by, and it is the place to look for
# what the suite spends: the slowest tests are printed when it is written.  A test timed again
# replaces its line and every other line is kept, so the fast suite and the slow tier can be timed
# on different days.  The times are libtest's own (`--report-time`, under `--format json`), which
# stable libtest gives when RUSTC_BOOTSTRAP is set where the test binary runs; it is set there
# and not for cargo, whose fingerprints it would change.  Time on a quiet machine (`uptime`).
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"
CARGO=${CARGO:-cargo}
[ $# -gt 0 ] || set -- -p gcs-core --test core
out=rust/test-times.tsv
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT

# the test targets only: an integration test's build also makes the package's binaries
$CARGO test --manifest-path rust/Cargo.toml "$@" --no-run --message-format=json \
    | grep '"test":true' \
    | sed -n 's/.*"manifest_path":"\([^"]*\)Cargo.toml".*"executable":"\([^"]*\)".*/\1 \2/p' \
    > "$work/binaries"
[ -s "$work/binaries" ] || { echo "nothing built by: cargo test $*" >&2; exit 1; }

status=0
while read -r dir bin; do
    echo "timing $bin" >&2
    # cargo runs a test binary in its package's directory; so does this
    (cd "$dir" && RUSTC_BOOTSTRAP=1 "$bin" -Z unstable-options --format json --report-time) \
        >> "$work/json" || status=$?
done < "$work/binaries"

sed -n 's/.*"name": "\([^"]*\)", "event": "ok", "exec_time": \([0-9.e-]*\).*/\2	\1/p' \
    "$work/json" | awk -F'\t' '{ printf "%.3f\t%s\n", $1, $2 }' > "$work/new"
touch "$out"
awk -F'\t' 'FNR == NR { new[$2] = 1; print; next } !($2 in new)' "$work/new" "$out" \
    | sort -t'	' -k1,1gr -k2,2 > "$work/merged"
mv "$work/merged" "$out"

awk -F'\t' '{ s += $1 } END { printf "%d tests timed, %.0f s of test time\n", NR, s }' "$work/new"
# awk and not `head`, which would close the pipe early and fail the script under pipefail
echo "slowest:"; sort -t'	' -k1,1gr "$work/new" | awk 'NR <= 25'
grep '"event": "failed"' "$work/json" | sed 's/.*"name": "\([^"]*\)".*/failed: \1/' || true
exit $status
