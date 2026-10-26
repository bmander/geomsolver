#!/bin/bash
# One shard of a test binary: `scripts/test-shard.sh K N <cargo test arguments>` runs the K-th of
# N parts of what those arguments build, e.g. `scripts/test-shard.sh 2 4 -p gcs-core --test core`.
# `scripts/test-shard.sh K N --binary <test binary> <its package's directory>` runs a part of a
# binary built already, without cargo: CI builds the core's suite once and hands the binary to
# each shard's machine (.github/workflows/test.yml, slow.yml).  It runs in its package's
# directory, as cargo runs it, and finds its files where it was built (`CARGO_MANIFEST_DIR` is
# compiled in), so it is run from a checkout at the same path.
#
# The tests are dealt by module, not one by one: a module's tests share fixtures built once per
# process (`OnceLock`s in `twist_drill`, `envelope::paired`), and splitting the module would
# build them on every machine it reached.  A test that takes 30 s or more is dealt on its own, so
# the slow tier's few long tests spread out.  The weights are rust/test-times.tsv (seconds, a
# tab, the test's name), which `scripts/test-times.sh` writes; a test not listed weighs
# DEFAULT_SECONDS (0.2; slow.yml gives 60, since a test missing from the file is most likely one
# of the slow tier's long ones).  The parts are filled longest first, each unit into the lightest part so far, so every machine
# computes the same split.  A test the build ignores weighs nothing.  `LIST=1` prints the part's
# tests and their total instead of running them.
set -euo pipefail

k=$1 n=$2; shift 2
root="$(cd "$(dirname "$0")/.." && pwd)"
CARGO=${CARGO:-cargo}
if [ "${1:-}" = "--binary" ]; then
    [ $# -eq 3 ] || { echo "--binary takes a test binary and its package's directory" >&2; exit 2; }
    [ -x "$2" ] || { echo "no test binary at $2" >&2; exit 2; }
    bin="$(cd "$(dirname "$2")" && pwd)/$(basename "$2")"; dir=$3
    cd "$root"
    libtest() { (cd "$dir" && "$bin" "$@"); }
else
    [ $# -gt 0 ] || { echo "say what to build: <cargo test arguments> or --binary" >&2; exit 2; }
    cd "$root"
    # built first and aloud: a build that failed would otherwise list no tests and pass
    $CARGO test --manifest-path rust/Cargo.toml "$@" --no-run
    built=("$@")
    libtest() { $CARGO test --manifest-path rust/Cargo.toml "${built[@]}" -- "$@"; }
fi

work=$(mktemp -d); trap 'rm -rf "$work"' EXIT
libtest --list --format terse 2>/dev/null | sed -n 's/: test$//p' | sort -u > "$work/names"
[ -s "$work/names" ] || { echo "no tests listed" >&2; exit 1; }
# an ignored test is dealt with its module but weighs nothing: it will not run in this build
libtest --list --ignored --format terse 2>/dev/null | sed -n 's/: test$//p' > "$work/ignored"

# the part's names into a file, its time and count on stdout
: > "$work/tests"
summary=$(awk -F'\t' -v k="$k" -v n="$n" -v d="${DEFAULT_SECONDS:-0.2}" \
    -v times=rust/test-times.tsv -v ignored="$work/ignored" -v out="$work/tests" '
    BEGIN {
        while ((getline line < times) > 0) if (split(line, f, "\t") == 2) t[f[2]] = f[1]
        while ((getline line < ignored) > 0) t[line] = 0
    }
    {
        w = ($0 in t) ? t[$0] : d
        u = (w >= 30 || index($0, "::") == 0) ? $0 : substr($0, 1, index($0, "::") - 1)
        if (!(u in weight)) units[++m] = u
        weight[u] += w; size[u]++; members[u] = size[u] > 1 ? members[u] "\n" $0 : $0
    }
    END {
        # longest first, ties by name, so the order does not depend on awk
        for (i = 2; i <= m; i++) for (j = i; j > 1; j--) {
            a = units[j - 1]; b = units[j]
            if (weight[a] < weight[b] || (weight[a] == weight[b] && a > b)) {
                units[j - 1] = b; units[j] = a
            } else break
        }
        for (i = 1; i <= m; i++) {
            best = 1
            for (p = 2; p <= n; p++) if (load[p] < load[best]) best = p
            load[best] += weight[units[i]]
            if (best == k) { print members[units[i]] > out; count += size[units[i]] }
        }
        printf "%.0f %d\n", load[k], count
    }' "$work/names")
read -r total count <<< "$summary"
echo "shard $k of $n: $count tests, about ${total} s by rust/test-times.tsv"
if [ -n "${LIST:-}" ]; then cat "$work/tests"; exit 0; fi
# with no names libtest would run everything
[ "$count" -eq 0 ] && exit 0
# shellcheck disable=SC2086  # the names are words
libtest --exact $(cat "$work/tests")
