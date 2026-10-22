# The verification corpus

The analytic cases every structural analysis of #510 (design intent as structural performance)
is held to, item P8. Each is a Solvent document: a solid, or a frame of beams, made of a
material, and a `scenario` whose claims bracket the case's answer. The frames are analysed by the
core since S0, which judges their claims (`rust/gcs-core/tests/beams.rs`); the solids' claims
are judged by the gate and by an external solver until S2–S3 analyse them.

- **The gate** (`rust/gcs-core/tests/analysis_corpus.rs`, every `make test`) holds each document
  to what it says it is. It solves to no freedom, its solid is the size its numbers say, and every
  bracket it claims is the closed form of its own drawing. That closed form is read off the solved
  solid, the material and the scenario, not off the text. The closed forms are held in turn to the
  equations that make them the answer: equilibrium, boundary conditions and limits
  (`analysis_corpus/closed.rs`).
- **The frames' analysis** (`rust/gcs-core/tests/beams.rs`, every `make test`) is the core's
  own: each frame case's answers are its references, exact where a cubic is exact (a load at a
  joint or spread evenly along a member) and within 1e-5 where it approximates (a mode, a
  buckling load), and every claim is kept.
- **The oracle** (`analysis_corpus/ccx.rs`, `make test-ccx`, with `ccx` and `gmsh` on the path)
  is CalculiX, kept in the tests alone as OCCT is for the B-rep kernel. It reads the solids; a
  frame is no oracle's. Each solid is built by this kernel and written as STEP. gmsh meshes it
  in quadratic tetrahedra, or in quadratic triangles for a case read in the plane. The supports and loads go on the facets that lie on the faces the
  scenario names. CalculiX's answer must fall inside every bracket.

Read them in this order:

| File | Case | Reference | Read as | Stage |
|---|---|---|---|---|
| `cantilever.sv` | A bar built in at its root, loaded at its tip; its first mode | `P L³ / 3EI`, Timoshenko's shear inside the bracket; `1.8751² / 2πL² · √(EI / ρA)`; its mass exactly | solid | S3 |
| `column.sv` | Euler's strut, built in at its foot, pressed on its head | `π² E I / 4L²`, a repeated pair (the section is square) | solid | S3 |
| `lame.sv` | Lamé's thick cylinder under internal pressure, ends open | the bore's `u(a)`, and von Mises `A √(1 + 3c⁴/a⁴)` at the bore; exact in three dimensions | solid | S3 |
| `kirsch.sv` | Kirsch's plate with a hole, pulled | `Kt = 3`, Heywood's 3.008 for a plate 20 holes wide | plane stress | S2 |
| `plate_modes.sv` | A simply supported square plate's first six modes | the flexural Rayleigh–Lamb root at `π √(m² + n²) / a`, exact in three dimensions under diaphragm supports; two repeated pairs | solid | S3 |
| `nafems_le1.sv` | NAFEMS LE1, the elliptic membrane | σyy = 92.7 MPa at D, stated as a factor of safety | plane stress | S2 |
| `nafems_fv32.sv` | NAFEMS FV32, the cantilevered tapered membrane | 44.623, 130.03, 162.70, 246.05, 379.90, 391.44 Hz | plane stress | S2 |
| `cantilever_frame.sv` | The cantilever as a beam on its neutral axis | `P L³ / 3EI` and the root's `P L / S`, exact; its first mode and mass as the solid's | frame | S0 |
| `column_frame.sv` | Euler's column as a beam | `π² E I / 4L²`, a repeated pair | frame | S0 |
| `clamped_frame.sv` | A beam built in at both ends, loaded at its middle, then by its weight | `P L³ / 192EI` and `P L / 8`; `q L⁴ / 384EI` and `q L² / 12`, exact | frame | S0 |

Each document states its tolerance in its own claims. A displacement, a frequency or a buckling
factor is bracketed within 1%. A factor of safety is bracketed within 3%, since stress is read at
nodes and is the least forgiving output.

What the documents write:

- **Supports are symmetry where the case is a quarter.** `pinned F along: d` holds a face square
  to itself, so a cut face is a plane of symmetry. One point held along the thickness takes out
  what the cuts leave free.
- **Every case is drawn in the front** (`std.front`, x along and z up) and swept across y. A case
  read in the plane is read in the face the solid sweeps, its depth the thickness, and the oracle
  refuses one swept any other way. A point held across the thickness is the solid reading's
  alone.
- **A tension is a negative pressure** (`pressure plate.pulled -sigma`), since a pressure pushes
  into the face.

- **A frame is drawn on its axis lines** in the front: each line a beam's member, its section's
  z the front's normal, so `Iz` bends it in the front and `Iy` across it.  A point drawn on a
  member where a load stands cuts it there.

What is not here yet, and why (NAFEMS FV2's frame, how a document asks to be read in the plane,
NAFEMS LE10 and FV52), is recorded in the contract: `docs/contract/analysis.md`, "What P8
leaves".
