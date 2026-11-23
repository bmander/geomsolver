STEP files written by Open CASCADE 7.9.3 (the Python bindings, cadquery-ocp 7.9.3) with
`make.py` beside them, for `tests/step_file.rs`: OCCT is a source of fixtures and an oracle in
tests only; nothing of it enters the kernel.

| file | what | written as |
|---|---|---|
| `drilled_ap203.step` | a 40 × 30 × 20 block drilled through (r 6) | AP203 (`CONFIG_CONTROL_DESIGN`), mm |
| `drilled_ap214.step` | the same | AP214 (`AUTOMOTIVE_DESIGN`), mm |
| `drilled_ap242.step` | the same | AP242, mm |
| `drilled_inch.step` | the same | AP214, inches (a conversion-based unit, 25.4 mm) |
| `drilled_metre.step` | the same | AP203, metres |
| `box_ap242_foot.step` | a 10-cube | AP242, feet (304.8 mm) |
| `named_ap214.step` | the drilled block through XCAF, named `bracket` | AP214, mm |
| `void_ap214.step` | a 20-cube with a sphere's void (r 4): `BREP_WITH_VOIDS` | AP214, mm |
| `shell_ap214.step` | a 10-cube's shell as a `SHELL_BASED_SURFACE_MODEL` | AP214, mm |
| `assembly_ap214.step` | two cubes placed by `NEXT_ASSEMBLY_USAGE_OCCURRENCE`s (XCAF) | AP214, mm |
| `placed_ap214.step` | one cube translated (5, 6, 7), as XCAF writes a located shape | AP214, mm |
| `two_solids_ap214.step` | two solids in one product's shape: `drilled_ap214.step` with `void_ap214.step`'s outer shell (renumbered by 1000) as a second `MANIFOLD_SOLID_BREP` — written by hand, not by `make.py` | AP214, mm |

Each run of `make.py` rewrites them (their time stamps change); the tests assert what does not.
