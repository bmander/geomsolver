# Checks every statement and proof here: `make` (Rocq 8.18 or later, with Flocq 4).
# `make gaps` runs the kernel on the inputs that depart from the statements marked so.
FILES = Float.v Interval.v Predicates.v Isolate.v

check: $(FILES)
	@for f in $(FILES); do coqc -Q . Kernel $$f || exit 1; done
	@echo "checked: $(FILES)"

gaps:
	cargo run --release --manifest-path gaps/Cargo.toml

clean:
	rm -f *.vo *.vok *.vos *.glob .*.aux

.PHONY: check gaps clean
