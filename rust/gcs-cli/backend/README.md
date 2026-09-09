# Native OCCT bridge

`solventc --step FILE` calls OCCT directly through a small C ABI around its C++ API.
C++ owns all kernel objects and catches exceptions. Rust owns the construction DAG,
reads solved core geometry, and replaces the destination only after native construction
and STEP reimport succeed. There is no Python process or gear-specific native operation.
Lengths cross the interface in millimetres and angles in radians. Native validity and
volume agreement do not certify export accuracy.

Install native Open CASCADE development files (for example, `brew install opencascade`)
and build the optional CLI feature:

```sh
make solventc OCCT=1
build/solventc rust/examples/spiral_bevel/blank.sv --solid blank.body --step blank.step
```

For another installation prefix, set `OCCT_ROOT` to the directory containing
`include/opencascade/` and `lib/`. Cargo also accepts `-p gcs-cli --features occt`.
The bridge currently builds on Unix native hosts. The default Cargo build, the core,
and WebAssembly have no OCCT dependency. A CLI built without the feature explains
how to enable it when asked for STEP.

Supported operations are extrusion, revolution and Boolean bodies with line/arc/circle
profiles, holes and through cutters. Along-guide lofts and generating-motion sweeps
remain unsupported. Models require explicit units and a successful solve, including
when `--allow-unsolved` is specified.

The optional independent tests use Python's OCCT bindings only to read resulting STEP
files and check them against analytic volumes. Install `requirements.txt` in a test
Python environment, then run:

```sh
SOLVENTC="$PWD/build/solventc" python -m unittest discover \
  -s rust/gcs-cli/backend -p test_occt.py
```

The tests point the former Python-host variable at a nonexistent interpreter to catch
any accidental subprocess fallback.

Validated on Intel macOS with native OCCT 7.9.3: seven export/failure tests and the
ten CLI tests pass. The FFI bevel-blank export also matches the former host's blank
under both directed native Boolean differences (neither leaves a solid).
