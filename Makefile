# gcs — one Rust core, two thin bindings.
#
#   make            native shared library in build/ (what the Python package loads)
#   make wasm       web/src/wasm/gcs.wasm (what the web app instantiates)
#   make test       everything: cargo, pytest, and the web suite
#   make bench      the Rust-side benchmarks through both bindings

UNAME := $(shell uname -s)
EXT := $(if $(filter Darwin,$(UNAME)),.dylib,$(if $(filter Windows_NT,$(OS)),.dll,.so))
CARGO := cargo
RUST_SRC := $(shell find rust/gcs-core/src rust/gcs-ffi/src -name '*.rs')
WASM_TARGET := wasm32-unknown-unknown

.PHONY: all wasm test test-rust test-py test-web bench fmt clippy clean

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

test: test-rust test-py test-web

test-rust:
	$(CARGO) test --manifest-path rust/Cargo.toml --release

test-py: all
	.venv/bin/pytest -q
	.venv/bin/mypy

test-web: wasm
	cd web && npm test

bench: all wasm
	.venv/bin/python -m gcs.bench
	cd web && npm run bench

fmt:
	$(CARGO) fmt --manifest-path rust/Cargo.toml

clippy:
	$(CARGO) clippy --manifest-path rust/Cargo.toml --all-targets -- -D warnings

clean:
	$(CARGO) clean --manifest-path rust/Cargo.toml
	rm -rf build web/src/wasm/gcs.wasm web/dist
