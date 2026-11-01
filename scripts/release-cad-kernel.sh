#!/bin/bash
# Release the B-rep kernel (rust/cad-kernel) to crates.io: docs/contract/brep.md says what a release
# is, .github/workflows/cad-kernel.yml makes one from a tag `cad-kernel-vX.Y.Z` on main.
#
#   prepare X.Y.Z   on a branch: the crate's version becomes X.Y.Z, Cargo.lock follows, and
#                   CHANGELOG.md's Unreleased section becomes X.Y.Z's; committed, for a pull request
#   tag             on main, after that pull request merges: tags HEAD `cad-kernel-v<version>` and
#                   says how to push it, which is what releases
#   check X.Y.Z     what the workflow asks of the tagged commit: the crate is X.Y.Z, and the
#                   changelog has X.Y.Z's section
#   notes X.Y.Z     X.Y.Z's section of the changelog, the release's notes
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
crate=rust/cad-kernel
manifest=$crate/Cargo.toml
changelog=$crate/CHANGELOG.md

die() { echo "release-cad-kernel: $*" >&2; exit 1; }
# the [package] version, the manifest's first `version =` line
version() { sed -n 's/^version = "\(.*\)"$/\1/p' "$manifest" | head -1; }
semver() { [[ "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?$ ]] || die "not a version: $1"; }
tagged() { git rev-parse -q --verify "refs/tags/cad-kernel-v$1" > /dev/null; }
clean() { [[ -z "$(git status --porcelain)" ]] || die "the working tree is not clean"; }
# the body of the changelog's section headed exactly `## $1`, blank lines at its ends dropped
section() {
    awk -v head="## $1" '
        $0 == head { inside = 1; next }
        inside && /^## / { exit }
        inside { lines[n++] = $0 }
        END {
            a = 0; while (a < n && lines[a] ~ /^[[:space:]]*$/) a++
            b = n - 1; while (b >= a && lines[b] ~ /^[[:space:]]*$/) b--
            for (i = a; i <= b; i++) print lines[i]
        }' "$changelog"
}
released() { grep -q "^## \[$1\] - " "$changelog"; }
notes() { awk -v v="$1" 'index($0, "## [" v "] - ") == 1 { print substr($0, 4); exit }' \
    "$changelog" | { read -r head; section "$head"; }; }

case "${1:-}" in
prepare)
    v=${2:-}; semver "$v"; clean
    now=$(version)
    tagged "$v" && die "cad-kernel-v$v is already tagged"
    released "$v" && die "the changelog already has a section for $v"
    # a newer version, or the manifest's own while it has never been released (the first)
    if [[ "$v" == "$now" ]]; then
        tagged "$now" && die "$now is released; name a newer version"
    elif [[ "$(printf '%s\n%s\n' "$now" "$v" | sort -V | tail -1)" != "$v" ]]; then
        die "$v is older than the crate's $now"
    fi
    [[ -n "$(section Unreleased)" ]] || die "the changelog's Unreleased section is empty"
    grep -q '^## Unreleased$' "$changelog" || die "the changelog has no '## Unreleased' heading"
    # awk, not sed -i: the same on macOS and Linux
    rewrite() { awk "$2" v="$v" d="$(date -u +%F)" "$1" > "$1.new" && mv "$1.new" "$1"; }
    rewrite "$manifest" '!done && /^version = "/ { print "version = \"" v "\""; done = 1; next } 1'
    rewrite "$changelog" '!done && $0 == "## Unreleased" {
        print; print ""; print "## [" v "] - " d; done = 1; next } 1'
    cargo update --manifest-path rust/Cargo.toml --workspace --offline --quiet
    git add "$manifest" "$changelog" rust/Cargo.lock
    git commit -q -m "cad-kernel $v"
    echo "Committed cad-kernel $v on $(git branch --show-current). Open a pull request; once it"
    echo "merges, run this from main:"
    echo "    scripts/release-cad-kernel.sh tag"
    ;;
tag)
    clean
    v=$(version)
    tagged "$v" && die "cad-kernel-v$v is already tagged"
    released "$v" || die "the changelog has no section for $v: run 'prepare $v' first"
    git fetch -q origin main
    git merge-base --is-ancestor HEAD FETCH_HEAD || die "HEAD is not on origin/main"
    git tag -a "cad-kernel-v$v" -m "cad-kernel $v"
    echo "Tagged $(git rev-parse --short HEAD) cad-kernel-v$v. Pushing the tag releases it:"
    echo "    git push origin cad-kernel-v$v"
    ;;
check)
    v=${2:-}; semver "$v"
    [[ "$(version)" == "$v" ]] || die "the crate is $(version), the tag $v"
    released "$v" || die "the changelog has no section for $v"
    [[ -n "$(notes "$v")" ]] || die "the changelog's section for $v is empty"
    ;;
notes)
    v=${2:-}; semver "$v"; notes "$v"
    ;;
*)
    sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//' >&2; exit 2
    ;;
esac
