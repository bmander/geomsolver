#!/bin/bash
# One shard of a test binary: `scripts/test-shard.sh K N <cargo test arguments>` runs the K-th of
# N parts of what those arguments build, e.g. `scripts/test-shard.sh 2 4 -p gcs-core --test core`.
# CI runs the parts on separate machines (.github/workflows/test.yml, slow.yml).
#
# The tests are dealt by module, not one by one: a module's tests share fixtures built once per
# process (`OnceLock`s in `twist_drill`, `envelope::paired`), and splitting the module would
# build them on every machine it reached.  A test that takes 30 s or more is dealt on its own, so
# the slow tier's few long tests spread out.  The weights are rust/test-times.tsv (seconds, a
# tab, the test's name), which `scripts/test-times.sh` writes; a test not listed weighs
# DEFAULT_SECONDS (0.2, or 60 when the slow tier is built, since a test missing from the file is
# most likely one of its long ones).  The parts are filled longest first, each unit into the
# lightest part so far, so every machine computes the same split.  A test the build ignores
# weighs nothing.  `LIST=1` prints the part's tests and their total instead of running them.
set -euo pipefail

k=$1 n=$2; shift 2
cd "$(git rev-parse --show-toplevel)"
CARGO=${CARGO:-cargo}
default=0.2
case " $* " in *slow*) default=60 ;; esac
default=${DEFAULT_SECONDS:-$default}

# Built first and aloud: a build that failed would otherwise list no tests and pass.
$CARGO test --manifest-path rust/Cargo.toml "$@" --no-run
names=$(mktemp); ignored=$(mktemp); part=$(mktemp); tests=$(mktemp)
trap 'rm -f "$names" "$ignored" "$part" "$tests"' EXIT
$CARGO test --manifest-path rust/Cargo.toml "$@" -- --list --format terse 2>/dev/null \
    | sed -n 's/: test$//p' | sort -u > "$names"
[ -s "$names" ] || { echo "no tests listed by: cargo test $*" >&2; exit 1; }
# an ignored test is dealt with its module but weighs nothing: it will not run in this build
$CARGO test --manifest-path rust/Cargo.toml "$@" -- --list --ignored --format terse 2>/dev/null \
    | sed -n 's/: test$//p' > "$ignored"

awk -F'\t' -v k="$k" -v n="$n" -v d="$default" -v times=rust/test-times.tsv \
    -v ignored="$ignored" '
    BEGIN {
        while ((getline line < times) > 0) if (split(line, f, "\t") == 2) t[f[2]] = f[1]
        while ((getline line < ignored) > 0) t[line] = 0
    }
    {
        w = ($0 in t) ? t[$0] : d
        u = (w >= 30 || index($0, "::") == 0) ? $0 : substr($0, 1, index($0, "::") - 1)
        if (!(u in weight)) units[++m] = u
        weight[u] += w; members[u] = members[u] "\n" $0
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
            if (best == k) out = out members[units[i]]
        }
        printf "%.0f%s\n", load[k], out
    }' "$names" > "$part"

# files and not pipes: `head` closing a pipe early fails the script under pipefail
total=$(sed -n 1p "$part")
sed -e 1d -e '/^$/d' "$part" > "$tests"
count=$(grep -c . "$tests" || true)
echo "shard $k of $n: $count tests, about ${total} s by rust/test-times.tsv"
if [ -n "${LIST:-}" ]; then cat "$tests"; exit 0; fi
# with no names libtest would run everything
[ "$count" -eq 0 ] && exit 0
# shellcheck disable=SC2086  # the names are words
$CARGO test --manifest-path rust/Cargo.toml "$@" -- --exact $(cat "$tests")
