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

Validated on Intel macOS with native OCCT 7.9.3. The FFI bevel-blank export matches the former host's blank
under both directed native Boolean differences (neither leaves a solid).

The internal bridge also interpolates regular rectangular contact grids into native
B-spline faces and evaluates their supporting surfaces. Grid rows/columns use uniform
parameters; callers supply coordinates in mm. Surface queries return position and the
unit `du cross dv` normal, without claiming material orientation or trim membership.
Invalid dimensions, nonfinite points and kernel failures return diagnostics. Candidate
faces cannot pass the final solid validator.

Run the source-driven fitting checks with:

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --features occt --test native_surfaces -- --nocapture
```

These tests read `gears.sv` through ordinary parsing, module resolution and solving,
then pass contact points from the declared cutter sweeps directly to C++. No imported
tooth grids or Python construction are involved. Sixteen flank/fillet candidate charts
are checked at withheld points, with observed maximum position error about 0.00013 mm.
The charts cover meridian parameters [0.05,0.95] and roll [-0.3,-0.2] radians only.
A former chart crossed a ring with no contacts; that missing ring remains a regression
check. A separate orbiting-sphere fixture checks refinement and the independent torus
equation. These sampled checks establish neither whole-domain coverage nor an error bound.

The source evaluator also supplies a temporal chart for a rotation viewed from another
fixed-axis rotation: hold cutter `(u,v)` and enumerate contact times. A native regression
fits across the join of the earlier pinion chart's two branches, checking both positions
and tangent planes. This is local chart continuation, not automatic domain partitioning
or global self-intersection trimming. See the [roadmap](../../../docs/spiral-bevel-roadmap.md).

This is an internal fitting primitive, exercised by integration tests; the CLI does not
yet assemble continuous sweep solids. Chart boundaries, source trimming, sharp-edge sweeps,
endpoint caps and global trimming remain necessary before enabling their STEP/STL export.

`boundary.cpp` reads the actual topology of a constructed native solid, including edges
made by Booleans. Its C interface first counts edges, then returns rows containing the edge
handle, both incident face handles and explicit seam/pole flags. Edge queries return a
position, unit curve tangent, two outward material normals and their measured face/curve
incidence discrepancies in mm, followed by the signed dihedral in radians (15 doubles).
Negative dihedral is convex, positive is concave and zero is smooth. Its sign uses the
oriented face boundary, not just the angle between two normals. Seams retain their two parameter curves; collapsed edges
remain in the inventory and explicitly refuse tangent queries. Queries check face incidence
and consistent curve parameters instead of projecting onto unrelated supporting surfaces.
The shared session ownership and exception boundary live in `occt.hpp`.

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --features occt --test native_boundary -- --nocapture
```

These tests use the CLI's ordinary recipe builder directly. Cube/hole checks cover edges
created at the stock's ends by an overshooting cylindrical cutter and the hole's inward
normals. Sphere/torus checks distinguish poles and periodic seams from creases. The actual
pinion and gear cutters have 10 and 18 native edges; 84 sampled positions and material-side
checks agree with the separate source field. This supplies source topology for sharp-edge
sweeps; it does not yet select the exposed swept regions or assemble a swept solid.

`envelope::edge_contact` now supplies the local sharp-edge candidate test. It checks the
convex outward normal cone against the motion velocity and returns the candidate's outward
normal. Smooth/concave edges contribute no regular sharp-edge face; tangent motion and
collapsed cones remain explicit degeneracies. A blind-hole regression checks the dihedral
sign, and a rotating cube edge generates a native fitted cylindrical patch compared against
the known cylinder. The actual gear cutters produce 17/37 sampled sharp contacts, with 106
nearby-time source-material checks on strict interior cone contacts. These local checks
do not establish visibility over the whole motion interval or a complete swept solid.

The native boundary tests also connect candidate positions/normals to
`MaterialEvaluator::probe`, which checks the complete declared swept material. It returns
strict interior/exterior ball margins or a bracket between opposite material signs at
outward-rounded offsets. Uncertain results remain explicit. At one edge station and three
roll times, the actual pinion/gear cutters yield 14 outward brackets and four candidates
buried by another pose, in about two seconds including solve/construction. Covering witnesses
are rechecked through the static source evaluator. Generic tests cover swallowed finite
caps, cut orientation, input boxes, phantom zeros and exhausted budgets. These are local
material checks at a 0.01 mm offset; they do not yet trace native trim curves or establish
unique crossings, complete surface coverage or the final export error.

`surfaces.cpp` now owns fitting, supporting-surface queries and native face splitting.
`solvent_cad_split_face` uses OCCT's
[Splitter](https://dev.opencascade.org/doc/occt-7.7.0/refman/html/class_b_rep_algo_a_p_i___splitter.html)
to construct intersection curves and return every fragment of one source face cut by
other faces. It preserves inputs, excludes tool fragments and refuses kernel errors,
warnings or invalid results. It adds no fuzzy tolerance or material selection.
`solvent_cad_faces` enumerates the result; `solvent_cad_face_contains` distinguishes
inside, outside and on-trim at the same normalized supporting-surface parameters used
by `surface_point`. A fragment's UV bounding rectangle does not redefine the chart.

Tests check a parabolic intersection against its equation, a spherical closed trim
that creates both a disk and a face with a hole, and a swept-sphere chart cut by a plane
whose fragments are probed against the complete resulting material. Both source gear
members now have a local temporal chart split by the declared native toe sphere, with
169 withheld partition samples each checked against the separate sphere field. Their
observed local interpolation errors are about 0.0000023 mm; this does not bound the
entire trim curve or the finished solid. All eight native surface tests take about
0.42 s on the current host, excluding compilation. The bridge constructs trims for
supplied intersecting faces; automatic candidate coverage, global sweep trimming and
closed-solid assembly are still required by the public export path.

`SweepContacts::cover` now supplies automatic time/angle chart candidates to the same
fitting bridge. Interval source geometry and the relative-rotation contact equation
partition every source face's complete parameter/time domain. Excluded cells have a
strict nonzero equation bound; a chart has a nonzero source normal, opposite endpoint
signs over its entire free-parameter box and a dependent derivative separated from zero.
This proves one contact root per free pair, not mapped-surface regularity or exposure.
Poles, folds, domain transitions and budget exhaustion stay explicitly unresolved.
Charts can overlap across internal partition boundaries. Full revolutions also permit
local continuation through the angular seam, limited to `[-0.25,1.25]` so interval
trigonometry stays in its supported domain. Partial revolutions, restricted spans and
motion intervals retain their physical boundaries. The source-domain partition remains
unchanged. `at_chart` evaluates unwrapped angular coordinates and refuses missing or
ambiguous roots; it does not assume exact periodic equality for binary64 TAU. When an
extended chart loses its monotonicity bound, subdivision can refine the dependent
coordinate as well. Overlaps and coincident seams must still be reconciled before
constructing a final face arrangement.
At 30,000 evaluated cells per member, the pinion has 845 time/82 angular charts and the gear
222 time/186 angular charts, including 18/15 seam charts. Search/audits take about 5.2/2.1 s;
all source faces receive work and unfinished domains remain explicit. Sixteen selected
gear charts, including four seam charts per member, are fitted and checked at withheld
points. The sphere coverage/fitting test takes about 0.1 s, including eight angular fits
compared with the independent torus equation. Sampled regular contacts at both angular
endpoints have charts, while poles remain unresolved. Timings exclude compilation and
do not describe complete swept-solid export.

The angular derivative uses a shared interval motion-coefficient implementation for
both terms of the position/normal product rule. Trigonometric boxes now use a midpoint
Taylor evaluation and angle-addition displacement bounds, retaining outward rounding
and the existing supported angular domain. The independent rational checker passes
1,841 arithmetic/trigonometric records, including sampled checks of whole-box bounds.

`native/sweep.rs` constructs finite sweep endpoint candidates from the declared static
cutter recipe. One native source is copied under the two exact declared endpoint poses;
both copies keep all native faces and trims. The placement matrix shares the ordinary
placed-solid path, including conversion of translation into millimetres. These are
candidate caps; retaining a complete cutter at an endpoint does not make every face
part of the swept boundary.

`solvent_cad_face_point` queries normalized coordinates in the native face's finite UV
trim box. It reports outside (leaving the output untouched), inside or on-trim, with
position and the face's oriented normal for retained points. It classifies holes rather
than treating the UV box as a filled rectangle. This is a separate coordinate contract
from `surface_point` and `face_contains`, which continue using the supporting surface's
bounds for stable chart coordinates after splitting. Position/normal evaluation and trim
classification now share internal implementations.

Endpoint tests verify a drilled box, the generating sphere in mm/in, and both default
gear cutters. The independent circular-arc distance check distinguishes covered from
exposed sphere cap samples in about 4 ms per unit configuration. On the gear cutters,
184 native/source position and normal checks pass; 26 whole-motion probes report
17 covered and 9 outward brackets with no unresolved samples. This checks selected
points, not entire endpoint faces. Endpoint trimming and final exposed-face assembly
are not connected to public sweep export yet.

`trims.cpp` adds contact edges directly on native faces:

- `solvent_cad_face_parameters` returns normalized finite-face coordinates and a
  measured incidence distance in mm. It projects onto the supporting surface, handles
  periodic representatives, then checks actual trims. Clamping a rounded endpoint
  always remeasures the spatial distance. Projection failure remains an error;
  this is not a globally continuous seam/pole parameterization.
- `solvent_cad_pcurve` interpolates supplied face coordinates and constructs an attached
  spatial edge. Open endpoints must reach trims; closed input omits the duplicate final
  point. The spatial construction tolerance does not bound the original contact fit.
- `solvent_cad_split_pcurves` uses OCCT's `BRepFeat_SplitShape` (linked through `TKFeat`)
  to retain all face fragments on copied topology. Face/edge inputs remain reusable.
- `solvent_cad_curve_point` evaluates the actual spatial edge for independent checks.

Tests split an open parabola and a closed loop, checking face membership, preserved
support geometry and withheld edge residuals. Source-computed contact branches split
six pinion and four gear starting-endpoint faces, including clipping at the gear's
existing Boolean trims. Across 1,816 sampled sign/membership checks, each tested fragment
stays on one side of the normal-velocity contact equation. Withheld edge/source-contact
errors are about 0.000157/0.000000101 mm. The fixture trace locator can miss narrow runs
and does not continue branches through meridian turning points; neither ending endpoint
is completed by this check. The primitive supplies native trimming for provided curves,
not whole endpoint coverage, global visibility or a closed swept solid.
