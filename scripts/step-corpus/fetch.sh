#!/bin/sh
# Make $STEP_CORPUS's downloaded part from nothing: the 32 CADGenBench editing inputs
# (`cadgenbench/<id>/input.step` and `edit_description.txt`, from the public HuggingFace dataset
# HuggingAI4Engineering/cadgenbench-data, licence ODC-By: not to be committed to the repository).
# Idempotent: a file whose size and SHA-256 match the committed list is not fetched again, and
# every file is checked against that list, so the reference tables are known to describe the same
# bytes.  Needs curl and sha256sum (or shasum).
#
#   STEP_CORPUS=/path/to/corpus scripts/step-corpus/fetch.sh
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
LIST="$HERE/../../rust/gcs-core/tests/step_corpus/cadgenbench.tsv"
ROOT=${STEP_CORPUS:-/home/user/step-corpus}
URL=https://huggingface.co/datasets/HuggingAI4Engineering/cadgenbench-data/resolve/main
if command -v sha256sum >/dev/null; then SHA="sha256sum"; else SHA="shasum -a 256"; fi
size() { wc -c < "$1" | tr -d ' '; }
while IFS="$(printf '\t')" read -r id bytes sha; do
    [ "$id" = id ] && continue
    dir="$ROOT/cadgenbench/$id"
    mkdir -p "$dir"
    f="$dir/input.step"
    if [ -f "$f" ] && [ "$(size "$f")" = "$bytes" ] && [ "$($SHA "$f" | cut -d' ' -f1)" = "$sha" ]; then
        :
    else
        echo "fetching $id"
        curl -fsSL --retry 3 -o "$f.part" "$URL/$id/input.step"
        mv "$f.part" "$f"
    fi
    [ -f "$dir/edit_description.txt" ] || curl -fsSL --retry 3 -o "$dir/edit_description.txt" \
        "$URL/$id/edit_description.txt" || echo "no edit_description.txt for $id"
    got=$($SHA "$f" | cut -d' ' -f1)
    if [ "$got" != "$sha" ] || [ "$(size "$f")" != "$bytes" ]; then
        echo "MISMATCH $id: $(size "$f") bytes, sha256 $got; the list says $bytes, $sha" >&2
        exit 1
    fi
done < "$LIST"
echo "cadgenbench: $(tail -n +2 "$LIST" | wc -l | tr -d ' ') files, each as listed, in $ROOT/cadgenbench"
