#!/bin/bash
# The tests a change can reach, run before a push: `make test-changed` (or `BASE=<rev>`).
#
# CI runs the whole fast suite on every push, and the slow tier every night (see
# docs/testing.md), so this is the inner loop's selection and not the gate.  It reads the files
# changed since the merge base with `origin/main` (committed, staged, unstaged and untracked) and
# maps each to the modules of the core's suite that can see it:
#
#   rust/gcs-core/tests/<m>.rs, tests/<m>/…   that module
#   a source file, src/<a>/<b>.rs             the modules naming `a::b` (or the top module of a
#                                             top-level file), and those called `b` or `b_…`
#   a document, rust/examples/<d>.sv          the modules naming `<d>.sv` or `"<d>"`
#   any other file (a fixture, a doc)         the modules naming the file, if any
#   rust/gcs-cli/, rust/gcs-ffi/              that crate's own suite
#   web/                                      the web suite
#
# plus the smoke set, which is cheap and touches every layer, each module run by its tests' exact
# names.  A change to what everything is built on (a manifest, `lib.rs`, `std.sv`, the suite's
# shared helpers), or one that reaches more than half the modules anyway, runs the whole fast
# suite.  `LIST=1` prints the selection and runs nothing; `FILES="a b"` selects for those paths
# instead of the diff.  Written for the bash 3.2 macOS ships: no associative arrays, no `mapfile`.
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"
TESTS=rust/gcs-core/tests
# the smoke set, by first segment: two modules and main.rs's own test
SMOKE="smoke examples_sv every_file_is_a_module"
# a module path's bounds in a regular expression, BSD grep's as well as GNU's
w='(^|[^a-z_0-9])'; e='($|[^a-z_0-9])'
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

base=${BASE:-$(git merge-base HEAD origin/main 2>/dev/null || git merge-base HEAD main)}
if [ -n "${FILES:-}" ]; then printf '%s\n' $FILES
else git diff --name-only "$base"; git ls-files --others --exclude-standard
fi | sort -u > "$work/changed"

sed -n 's/^mod \([a-z_0-9]*\);$/\1/p' "$TESTS/main.rs" | grep -v '^common$' > "$work/modules"
: > "$work/selected"
whole=""; core=""; cli=""; ffi=""; web=""

# The modules whose sources match a pattern: `naming -E <regex>` or `naming -F <text>…`.
naming() {
    grep -rl "$@" "$TESTS" --include='*.rs' 2>/dev/null \
        | sed -e "s|^$TESTS/||" -e 's|/.*||' -e 's|\.rs$||' | grep -vxE 'main|common' || true
}

while IFS= read -r f; do
    case "$f" in
        rust/Cargo.toml|rust/Cargo.lock|rust/rust-toolchain.toml|rust/*/Cargo.toml|rust/lib/*|\
        rust/fixtures/*|rust/gcs-solve/src/lib.rs|rust/gcs-core/src/lib.rs|$TESTS/main.rs|\
        $TESTS/common/*)
            whole="$f" ;;
        $TESTS/*)
            core=1; m=${f#"$TESTS/"}; m=${m%%/*}; m=${m%.rs}
            if grep -qx "$m" "$work/modules"; then echo "$m" >> "$work/selected"
            else naming -F "${f##*/}" >> "$work/selected"; fi ;;
        rust/gcs-cli/*) cli=1 ;;
        rust/gcs-ffi/*) ffi=1 ;;
        rust/gcs-solve/src/bin/*|rust/gcs-core/src/bin/*|.github/*|.claude/*|scripts/*) ;;
        rust/gcs-solve/src/*.rs|rust/gcs-core/src/*.rs)
            core=1; p=${f#rust/gcs-*/src/}; p=${p%.rs}; p=${p%/mod}
            path=$(echo "$p" | sed 's|/|::|g'); stem=${p##*/}
            if [ "$path" = "$stem" ]; then pattern="$w$stem::|gcs_core::$stem$e"
            else pattern="$w$path$e"; fi
            naming -E "$pattern" >> "$work/selected"
            grep -xE "${stem}(_.*)?" "$work/modules" >> "$work/selected" || true ;;
        web/*) web=1 ;;
        rust/examples/*)
            core=1; name=${f##*/}; naming -F -e "$name" -e "\"${name%.*}\"" >> "$work/selected" ;;
        # docs/ too: the suite reads documents there (docs/lofts/) and the primer
        *)
            hits=$(naming -F "${f##*/}")
            [ -n "$hits" ] && { core=1; echo "$hits" >> "$work/selected"; } ;;
    esac
done < "$work/changed"

sort -u "$work/selected" -o "$work/selected"
total=$(($(wc -l < "$work/modules"))); picked=$(($(wc -l < "$work/selected")))
if [ -z "$whole" ] && [ $((picked * 2)) -gt "$total" ]; then
    whole="$picked of $total modules"
fi

echo "changed since ${base:0:12}: $(($(wc -l < "$work/changed"))) files"
if [ -n "$whole" ]; then
    core=1; echo "core suite: all of it (because of $whole)"
elif [ -n "$core" ]; then
    echo "core suite: $picked modules, and the smoke set: $SMOKE"
    sed 's/^/  /' "$work/selected"
fi
[ -n "$cli" ] && echo "gcs-cli suite"
[ -n "$ffi" ] && echo "gcs-ffi suite"
[ -n "$web" ] && echo "web suite"
[ -n "${LIST:-}" ] && exit 0

CARGO=${CARGO:-cargo}
core_suite() { $CARGO test --manifest-path rust/Cargo.toml -p gcs-core --test core "$@"; }
if [ -n "$whole" ]; then
    core_suite
elif [ -n "$core" ]; then
    # libtest's filters match anywhere in a name (`rounds::` would run `corner_rounds::` too), so
    # the selection is listed and named exactly; built first and aloud, since a build that failed
    # would list nothing, and no names at all runs everything
    core_suite --no-run
    { cat "$work/selected"; printf '%s\n' $SMOKE; } > "$work/wanted"
    core_suite -- --list --format terse 2>/dev/null | sed -n 's/: test$//p' \
        | awk -F'::' 'FNR == NR { want[$0]; next } $1 in want' "$work/wanted" - > "$work/tests"
    [ -s "$work/tests" ] || { echo "no tests listed" >&2; exit 1; }
    # shellcheck disable=SC2046  # the names are words
    core_suite -- --exact $(cat "$work/tests")
fi
# one job graph for both, so they build side by side
packages=""
[ -n "$cli" ] && packages="$packages -p gcs-cli"
[ -n "$ffi" ] && packages="$packages -p gcs-ffi"
# shellcheck disable=SC2086  # the packages are words
[ -n "$packages" ] && $CARGO test --manifest-path rust/Cargo.toml $packages
[ -n "$web" ] && make test-web
exit 0
