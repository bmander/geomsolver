#!/bin/bash
# Release the B-rep kernel (rust/cad-kernel) to crates.io: docs/contract/brep.md says what a release
# is, .github/workflows/cad-kernel.yml makes one from a tag `cad-kernel-vX.Y.Z` on main.
#
#   prepare X.Y.Z   on a branch: the crate's version becomes X.Y.Z, Cargo.lock follows, and
#                   CHANGELOG.md's Unreleased section becomes X.Y.Z's; committed, for a pull request
#   tag             on main, after that pull request merges: tags HEAD `cad-kernel-v<version>` and
#                   says how to push it, which is what releases
#   check X.Y.Z     what a release asks of HEAD: the crate is X.Y.Z, the changelog has X.Y.Z's
#                   section, and HEAD is on origin/main
#   notes X.Y.Z     X.Y.Z's section of the changelog, the release's notes
#   version         the crate's version
set -euo pipefail
self="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
cd "$(git rev-parse --show-toplevel)"
crate=rust/cad-kernel
manifest=$crate/Cargo.toml
changelog=$crate/CHANGELOG.md

die() { echo "release-cad-kernel: $*" >&2; exit 1; }
# the [package] version, the manifest's first `version =` line
version() { sed -n 's/^version = "\(.*\)"$/\1/p' "$manifest" | head -1; }
tagged() { git rev-parse -q --verify "refs/tags/cad-kernel-v$1" > /dev/null; }
clean() { [[ -z "$(git status --porcelain)" ]] || die "the working tree is not clean"; }
# the body of the changelog's section whose heading starts `## $1`, blank lines at its ends
# dropped; fails if there is no such heading
section() {
    awk -v head="## $1" '
        index($0, head) == 1 { inside = found = 1; next }
        inside && /^## / { exit }
        inside { lines[n++] = $0 }
        END {
            a = 0; while (a < n && lines[a] ~ /^[[:space:]]*$/) a++
            b = n - 1; while (b >= a && lines[b] ~ /^[[:space:]]*$/) b--
            for (i = a; i <= b; i++) print lines[i]
            exit !found
        }' "$changelog"
}
notes() { section "[$1] - "; }
check() {
    [[ "$(version)" == "$1" ]] || die "the crate is $(version), not $1"
    [[ -n "$(notes "$1")" ]] ||
        die "the changelog's section for $1 is missing or empty: run 'prepare $1' first"
    git fetch -q origin main
    git merge-base --is-ancestor HEAD FETCH_HEAD || die "HEAD is not on origin/main"
}

case "${1:-}" in
prepare)
    v=${2:-}; clean
    [[ "$v" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?$ ]] || die "not a version: $v"
    now=$(version)
    tagged "$v" && die "cad-kernel-v$v is already tagged"
    notes "$v" > /dev/null && die "the changelog already has a section for $v"
    # a newer version, or the manifest's own while it has never been released (the first)
    [[ "$(printf '%s\n%s\n' "$now" "$v" | sort -V | tail -1)" == "$v" ]] ||
        die "$v is older than the crate's $now"
    [[ -n "$(section Unreleased)" ]] || die "the changelog's Unreleased section is missing or empty"
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
    check "$v"
    git tag -a "cad-kernel-v$v" -m "cad-kernel $v"
    echo "Tagged $(git rev-parse --short HEAD) cad-kernel-v$v. Pushing the tag releases it:"
    echo "    git push origin cad-kernel-v$v"
    ;;
check) check "${2:-}" ;;
notes) notes "${2:-}" ;;
version) version ;;
*)
    sed -n '2,/^set /s/^# \{0,1\}//p' "$self" >&2; exit 2
    ;;
esac
