# gcs — one Rust core, one thin binding.
#
#   make            native shared library in build/ (the released C ABI artefact)
#   make solventc   build/solventc — check a Solvent document from a terminal
#   make wasm       web/src/wasm/gcs.wasm (what the web app instantiates)
#   make test       everything: both released artefacts, cargo and the web suite
#   make bench      the native benchmark and the wasm one, meant to be read side by side

UNAME := $(shell uname -s)
EXT := $(if $(filter Darwin,$(UNAME)),.dylib,$(if $(filter Windows_NT,$(OS)),.dll,.so))
CARGO := cargo
# `.sv` too, and `rust/examples/` with them: every case in the library is a Solvent document
# compiled in with `include_str!`, so a document is source.  Left out of this list, editing one
# rebuilt nothing — the tests read the file from disk and passed while the browser went on
# running the case that was compiled in weeks ago.
RUST_SRC := $(shell find rust/gcs-core/src rust/gcs-ffi/src rust/gcs-cli/src rust/examples rust/lib \
                        -name '*.rs' -o -name '*.sv')
WASM_TARGET := wasm32-unknown-unknown
# The native triple, named so the release artefacts can be asked for together (see `release`)
# and so they then come out of one directory whether asked for together or alone.
HOST := $(shell rustc -vV | sed -n 's/^host: //p')
RELEASE := $(CARGO) build --manifest-path rust/Cargo.toml --release -p gcs-ffi

.PHONY: all wasm solventc release test test-rust test-web bench fmt clippy clean

all: build/libgcs$(EXT)

build/libgcs$(EXT): $(RUST_SRC) rust/gcs-core/Cargo.toml rust/gcs-ffi/Cargo.toml
	@mkdir -p build
	$(RELEASE) --target $(HOST)
	cp rust/target/$(HOST)/release/libgcs$(EXT) $@

solventc: build/solventc

build/solventc: $(RUST_SRC) rust/gcs-cli/Cargo.toml
	@mkdir -p build
	$(CARGO) build --manifest-path rust/Cargo.toml --release -p gcs-cli --target $(HOST)
	cp rust/target/$(HOST)/release/solventc $@

wasm: web/src/wasm/gcs.wasm

web/src/wasm/gcs.wasm: $(RUST_SRC) rust/gcs-core/Cargo.toml rust/gcs-ffi/Cargo.toml
	@mkdir -p web/src/wasm
	rustup target add $(WASM_TARGET)
	$(RELEASE) --target $(WASM_TARGET)
	cp rust/target/$(WASM_TARGET)/release/gcs.wasm $@

# Both released artefacts from ONE cargo invocation.  Each is a fat-LTO link of the whole engine
# with `codegen-units = 1`, which is one thread working for most of two minutes, and two
# invocations cannot overlap — the second blocks on the build-directory lock — so `all` then
# `wasm` is 116 s + 91 s where one job graph holding both is 106 s.  `test` asks for this first;
# `all` and `wasm` then find their copies newer than every source and do nothing, and cargo's
# own cache is shared either way, since the solo rules build into the same `--target` directory.
release:
	@mkdir -p build web/src/wasm
	rustup target add $(WASM_TARGET)
	$(RELEASE) --target $(HOST) --target $(WASM_TARGET)
	cp rust/target/$(HOST)/release/libgcs$(EXT) build/libgcs$(EXT)
	cp rust/target/$(WASM_TARGET)/release/gcs.wasm web/src/wasm/gcs.wasm

test: release test-rust test-web

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
