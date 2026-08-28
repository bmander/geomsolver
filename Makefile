# gcs — one Rust core, one thin binding.
#
#   make            native shared library in build/ (the released C ABI artefact)
#   make wasm       web/src/wasm/gcs.wasm (what the web app instantiates)
#   make test       everything: cargo and the web suite
#   make bench      the native benchmark and the wasm one, meant to be read side by side

UNAME := $(shell uname -s)
EXT := $(if $(filter Darwin,$(UNAME)),.dylib,$(if $(filter Windows_NT,$(OS)),.dll,.so))
CARGO := cargo
# `.sv` too, and `rust/examples/` with them: every case in the library is a Solvent document
# compiled in with `include_str!`, so a document is source.  Left out of this list, editing one
# rebuilt nothing — the tests read the file from disk and passed while the browser went on
# running the case that was compiled in weeks ago.
RUST_SRC := $(shell find rust/gcs-core/src rust/gcs-ffi/src rust/examples \
                        -name '*.rs' -o -name '*.sv')
WASM_TARGET := wasm32-unknown-unknown

.PHONY: all wasm test test-rust test-web bench fmt clippy clean

all: build/libgcs$(EXT)

build/libgcs$(EXT): $(RUST_SRC) rust/gcs-core/Cargo.toml rust/gcs-ffi/Cargo.toml
	@mkdir -p build
	$(CARGO) build --manifest-path rust/Cargo.toml --release -p gcs-ffi
	cp rust/target/release/libgcs$(EXT) $@

wasm: web/src/wasm/gcs.wasm

web/src/wasm/gcs.wasm: $(RUST_SRC) rust/gcs-core/Cargo.toml rust/gcs-ffi/Cargo.toml
	@mkdir -p web/src/wasm
	rustup target add $(WASM_TARGET)
	$(CARGO) build --manifest-path rust/Cargo.toml --release -p gcs-ffi --target $(WASM_TARGET)
	cp rust/target/$(WASM_TARGET)/release/gcs.wasm $@

test: test-rust test-web

# A workspace `cargo test` already runs every member's `tests/`, so `gcs-ffi/tests/` comes along:
# the panic boundary is the one thing only the native target can check, since
# `wasm32-unknown-unknown` aborts whatever the profile says.  `all` is a prerequisite so the
# released cdylib has to link before the suite can pass — nothing else builds it any more.
test-rust: all
	$(CARGO) test --manifest-path rust/Cargo.toml --release

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
