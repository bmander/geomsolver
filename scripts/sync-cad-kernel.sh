#!/bin/bash
# Publish rust/cad-kernel's history to its own repository (https://github.com/bmander/cad-kernel),
# until that repository becomes the kernel's home and this one depends on it.  `git subtree split`
# makes one commit there for each commit here that touched the directory, the same commits on every
# run, so a sync after more work fast-forwards.  `REF=<rev>` syncs from another revision than HEAD.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
split=$(git subtree split --prefix=rust/cad-kernel "${REF:-HEAD}")
git push "${REMOTE:-https://github.com/bmander/cad-kernel}" "$split:refs/heads/main"
