# gcs — one Rust core, one thin binding.
#
#   make            native shared library in build/ (the released C ABI artefact)
#   make solventc   build/solventc — check a Solvent document from a terminal
#   make wasm       web/src/wasm/gcs.wasm (what the web app instantiates)
#   make test       everything: both released artefacts, cargo and the web suite
#   make test-changed  the tests a change can reach, and a smoke set (before a push; CI runs all)
#   make bench      the native benchmark and the wasm one, meant to be read side by side

UNAME := $(shell uname -s)
EXT := $(if $(filter Darwin,$(UNAME)),.dylib,$(if $(filter Windows_NT,$(OS)),.dll,.so))
CARGO := cargo
# Inherit a parallel Make jobserver when present; otherwise overlap the two suites.
# Set TEST_JOBS=1 to run them serially (GNU Make 3.81 does not retain -j1 in MAKEFLAGS).
TEST_JOBS ?= $(if $(filter -j%,$(MAKEFLAGS)),,2)
OCCT ?= 0
comma := ,
# `.sv` too, and `rust/examples/` with them: every case in the library is a Solvent document
# compiled in with `include_str!`, so a document is source.  Left out of this list, editing one
# rebuilt nothing — the tests read the file from disk and passed while the browser went on
# running the case that was compiled in weeks ago.
RUST_SRC := $(shell find rust/gcs-brep/src rust/gcs-solve/src rust/gcs-core/src rust/gcs-ffi/src rust/gcs-cli/src \
                        rust/examples rust/lib -name '*.rs' -o -name '*.sv')
RUST_CONFIG := rust/Cargo.toml $(wildcard rust/*/Cargo.toml) rust/Cargo.lock rust/rust-toolchain.toml
WASM_TARGET := wasm32-unknown-unknown
# The native triple, named so the release artefacts can be asked for together (see `release`)
# and so they then come out of one directory whether asked for together or alone.
HOST := $(shell rustc -vV | sed -n 's/^host: //p')
RELEASE := $(CARGO) build --manifest-path rust/Cargo.toml --release -p gcs-ffi

.PHONY: all wasm solventc release test test-rust test-web test-changed test-times test-slow test-ccx \
        bench fmt clippy clean

all: build/libgcs$(EXT)

build/libgcs$(EXT): $(RUST_SRC) $(RUST_CONFIG)
	@mkdir -p build
	$(RELEASE) --target $(HOST)
	cp rust/target/$(HOST)/release/libgcs$(EXT) $@

solventc: build/solventc

.PHONY: solventc-config
# Cargo tracks the native feature as well as sources; ask it even when only OCCT changed.
build/solventc: $(RUST_SRC) $(RUST_CONFIG) solventc-config
	@mkdir -p build
	$(CARGO) build --manifest-path rust/Cargo.toml --release -p gcs-cli --target $(HOST) $(if $(filter 1,$(OCCT)),--features occt,)
	cp rust/target/$(HOST)/release/solventc $@

wasm: web/src/wasm/gcs.wasm

web/src/wasm/gcs.wasm: $(RUST_SRC) $(RUST_CONFIG)
	@mkdir -p web/src/wasm
	rustup target add $(WASM_TARGET)
	$(RELEASE) --target $(WASM_TARGET)
	cp rust/target/$(WASM_TARGET)/release/gcs.wasm $@

# Both released artefacts in one Cargo job graph, sharing the cache used by the solo rules.
# Separate concurrent Cargo invocations would just wait on the build-directory lock. The
# release profile uses incremental ThinLTO; a small edit can reuse unchanged codegen units.
release:
	@mkdir -p build web/src/wasm
	rustup target add $(WASM_TARGET)
	$(RELEASE) --target $(HOST) --target $(WASM_TARGET)
	cp rust/target/$(HOST)/release/libgcs$(EXT) build/libgcs$(EXT)
	cp rust/target/$(WASM_TARGET)/release/gcs.wasm web/src/wasm/gcs.wasm

# Finish both release artefacts before starting either suite, including with `make -j`.
# Then overlap the independent suites even for a plain `make test`.
test: release
	$(MAKE) $(if $(TEST_JOBS),-j$(TEST_JOBS)) test-rust test-web

# A workspace `cargo test` already runs every member's `tests/`, so `gcs-ffi/tests/` and
# `gcs-cli/tests/` come along: the panic boundary is the one thing only the native target can
# check, since `wasm32-unknown-unknown` aborts whatever the profile says, and `solventc` is run
# over the case library.  `all` is a prerequisite so the released cdylib has to link before the
# suite can pass — nothing else builds it any more.  Not `--release`: the suite runs under
# `[profile.test]` (optimised, no LTO, no debuginfo — see rust/Cargo.toml for what each costs),
# which is also what a bare `cargo test` gets.
test-rust: all
	$(CARGO) test --manifest-path rust/Cargo.toml

test-web: wasm
	cd web && npm test

# The inner loop's selection: the modules of the core's suite a change since the merge base with
# origin/main can reach, the CLI's, the ABI's or the web suite when those changed, and a smoke set
# (scripts/test-changed.sh says how it maps a file).  `LIST=1` prints the selection only.  It is
# not the gate: CI runs the whole fast suite on every push (.github/workflows/test.yml).
test-changed:
	CARGO="$(CARGO)" scripts/test-changed.sh

# Every test of the core's suite timed, into rust/test-times.tsv, which CI deals its shards by;
# the slowest are printed.  The slow tier, or another binary, is the script's own arguments
# (`scripts/test-times.sh -p gcs-core --test core --features gcs-core/slow`).
test-times:
	CARGO="$(CARGO)" scripts/test-times.sh

# The slow tier as well (tests of twenty seconds to a few minutes, `feature = "slow"`): the gear
# pair's whole-member verifications and, with `OCCT=1`, a native tooth space. CI runs it every
# night (.github/workflows/slow.yml); run it, or that workflow by hand, before a change to the
# gear example or the native export lands, not on every edit.
test-slow: all
	$(CARGO) test --manifest-path rust/Cargo.toml --features gcs-core/slow,gcs-cli/slow$(if $(filter 1,$(OCCT)),$(comma)gcs-cli/occt,)

# #510's verification corpus judged by CalculiX (`feature = "ccx"`): each case of
# rust/examples/analysis meshed by gmsh and solved by ccx, both on the path, against the claims its
# document states.
test-ccx:
	$(CARGO) test --manifest-path rust/Cargo.toml -p gcs-core --features gcs-core/ccx analysis_corpus::

bench: wasm
	$(CARGO) run --manifest-path rust/Cargo.toml --release -p gcs-core --bin bench
	cd web && npm run bench

fmt:
	$(CARGO) fmt --manifest-path rust/Cargo.toml

clippy:
	$(CARGO) clippy --manifest-path rust/Cargo.toml --all-targets -- -D warnings

clean:
	$(CARGO) clean --manifest-path rust/Cargo.toml
	rm -rf build web/src/wasm/gcs.wasm web/dist
