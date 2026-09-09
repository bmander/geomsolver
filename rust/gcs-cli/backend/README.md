# Optional native CAD host

`solventc --step FILE` embeds `occt.py` and sends the solved core recipe over stdin.
The adapter performs general solid operations; it has no gear names or gear equations.
The core owns source interpretation, units and spatial placement. Lengths cross the
interface in millimetres, angles in radians. The native host retains analytic circles
and lines, constructs profiles and solids, and validates STEP reimport before replacing
the destination. Native validity and volume agreement do not certify export accuracy.

Install with the Python interpreter you want to use:

```sh
python3 -m venv .venv-cad
.venv-cad/bin/python -m pip install -r rust/gcs-cli/backend/requirements.txt
export SOLVENT_CAD_PYTHON="$PWD/.venv-cad/bin/python"
make solventc
build/solventc rust/examples/spiral_bevel/blank.sv --solid blank.body --step blank.step
```

No Python dependency is required for normal compilation, the browser, or the existing
faceted STL path. STEP currently supports extrusion, revolution and Boolean bodies with
line/arc/circle profiles, profile holes and through cutters. It refuses unsupported
profiles and along-guide lofts. Generating-motion sweeps are the next gear integration task.
Models need explicit length units and must solve successfully even with `--allow-unsolved`.

Run the optional end-to-end native tests after building the CLI:

```sh
SOLVENTC="$PWD/build/solventc" "$SOLVENT_CAD_PYTHON" -m unittest discover \
  -s rust/gcs-cli/backend -p test_occt.py
```
