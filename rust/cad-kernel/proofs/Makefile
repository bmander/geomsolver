# Checks every statement and proof here: `make` (Rocq 8.18 or later, with Flocq 4).
# `make gaps` runs the predicates outside the range they are stated over.
FILES = Float.v Rounding.v Interval.v Predicates.v Isolate.v

check: $(FILES)
	@for f in $(FILES); do coqc -Q . Kernel $$f || exit 1; done
	@echo "checked: $(FILES)"

gaps:
	cargo run --release --manifest-path gaps/Cargo.toml

clean:
	rm -f *.vo *.vok *.vos *.glob .*.aux .nra.cache .lia.cache

.PHONY: check gaps clean
