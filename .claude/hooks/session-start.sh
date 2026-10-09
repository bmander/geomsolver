#!/bin/bash
# Readies a Claude Code cloud session (claude.ai/code) to build and test: the toolchain the
# environment's setup script installed, the wasm target `rust/rust-toolchain.toml` names, the
# crates and the web suite's packages.  Nothing is built here: a cold build takes longer than a
# hook should, and `make` / `make test` do it.  Locally it does nothing.
if [ "$CLAUDE_CODE_REMOTE" != "true" ]; then exit 0; fi

cd "$CLAUDE_PROJECT_DIR" || exit 0
log=/tmp/session-start.log
: > "$log"

# rustup's cargo, when the setup script put it in $HOME rather than on the image's PATH
if [ -f "$HOME/.cargo/env" ]; then
  . "$HOME/.cargo/env"
  [ -n "$CLAUDE_ENV_FILE" ] && echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> "$CLAUDE_ENV_FILE"
fi

failed=()
if command -v rustup >/dev/null; then
  (cd rust && rustup target add wasm32-unknown-unknown) >> "$log" 2>&1 || failed+=("wasm target")
else
  failed+=("rustup (not installed: make wasm needs the wasm32 target)")
fi
cargo fetch --manifest-path rust/Cargo.toml >> "$log" 2>&1 || failed+=("cargo fetch")
(cd web && npm ci --no-audit --no-fund) >> "$log" 2>&1 || failed+=("npm ci")

echo "Session setup: $(rustc --version 2>/dev/null || echo 'no rustc'), node $(node --version 2>/dev/null || echo 'missing')."
if [ ${#failed[@]} -gt 0 ]; then
  echo "Setup steps that failed: ${failed[*]} (see $log)."
fi
echo "Nothing is built yet: run make, make wasm or make test (slow; run it once, in the background)."
exit 0
