# The verification corpus

The analytic cases every structural analysis of #510 (design intent as structural performance)
is held to, item P8. Each is a Solvent document: a solid made of a material, and a `scenario`
whose claims bracket the case's answer. Nothing in the language analyses a scenario yet, so for
now the claims are judged by the gate and by an external solver, and S0–S3 will be held to them as
each lands.

- **The gate** (`rust/gcs-core/tests/analysis_corpus.rs`, every `make test`) holds each document
  to what it says it is. It solves to no freedom, its solid is the size its numbers say, and every
  bracket it claims is the closed form of its own drawing. That closed form is read off the solved
  solid, the material and the scenario, not off the text. The closed forms are held in turn to the
  equations that make them the answer: equilibrium, boundary conditions and limits
  (`analysis_corpus/closed.rs`).
- **The oracle** (`analysis_corpus/ccx.rs`, `make test-ccx`, with `ccx` and `gmsh` on the path)
  is CalculiX, kept in the tests alone as OCCT is for the B-rep kernel. Each solid is built by this
  kernel and written as STEP. gmsh meshes it in quadratic tetrahedra, or in quadratic triangles
  for a case read in the plane. The supports and loads go on the facets that lie on the faces the
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

Each document states its tolerance in its own claims. A displacement, a frequency or a buckling
factor is bracketed within 1%. A factor of safety is bracketed within 3%, since stress is read at
nodes and is the least forgiving output.

What the documents write:

- **Supports are symmetry where the case is a quarter.** `pinned F along: d` holds a face square
  to itself, so a cut face is a plane of symmetry. One point held along the thickness takes out
  what the cuts leave free.
- **Every case is drawn in the front** (`std.front`, x along and z up) and swept across y. A case
  read in the plane is read in the face the solid sweeps, its depth the thickness. A hold across
  the thickness is the solid reading's alone.
- **A tension is a negative pressure** (`pressure plate.pulled -sigma`), since a pressure pushes
  into the face.

What is not here yet:

- **Frames** (S0): a cantilever, Euler's column and NAFEMS FV2 (the pin-ended double cross) as
  frames. They wait on how a frame's section is written, which is S0's to decide. The
  cantilever's and the column's closed forms are already `closed.rs`'s.
- **How a document asks to be read in the plane** (S2): the corpus's plane cases are thin prisms,
  and the table above says which. Read as solids, `kirsch.sv` and `nafems_le1.sv` come out near
  their plane-stress answers, but `nafems_fv32.sv` bends out of its plane far below its in-plane
  modes.
- **NAFEMS LE10** (the thick plate under pressure) has a target of σyy at a point. No output word
  reads a component of stress at a point yet; that is an open decision of the requirement words.
- **NAFEMS FV52** (the simply supported solid plate). Its supports are published two ways (the
  lower edges, or the middle plane's), and the corpus waits for the publication in hand.
  `plate_modes.sv` is the same physics with supports under which the answer is exact.
