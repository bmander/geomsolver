# Generating sweeps: execution plan

This plan delivers the [supported class](generating-sweeps.md) through to a hypoid pair
export. Each phase ends with an executable result and an exit gate. Unit tests use small
closed-form fixtures that run in seconds. The gear cases are acceptance runs, not the lens
the components are designed through. Measure at the point a decision is made before
changing the construction.

## Phase 1 — Measure where the hypoid stands

Before any new construction, find out which offsets are in the class.

- For `offset_angle` at 0°, 6°, 15°, 30° and 45°, and for each member, evaluate E1–E4 by
  sampling on the actual `gears.sv` source:
  - the clearance at both roll limits;
  - the number of contact times per source point that reaches the blank;
  - the minimum generated area factor;
  - self-crossings of one placement's sheet inside the blank.
- Run the existing native OCCT path (`generic_sheet_reproduces_the_pinion_tooth_space`, with
  the offset rewritten through `read_gears_with`) at the same offsets. Apply the field probe
  to its faces as well as to the mesh path's triangles. The recorded failure is the mesh
  path's. Whether the native path fails the same way has not been measured.
- Record a table of offset, member, each predicate's worst value and witness, native
  field-probe disagreement, and time.

**Exit:** the table is in this document, with a statement of which offsets are inside the
class. **Decision gate with the user:** if the offsets the design needs fail E2–E4, choose
between restricting supported offsets and widening the class (a Phase 5b below). Do not
start construction changes before this gate.

## Phase 2 — Admission predicates in the core

Add one core entry point that takes a solved sweep graph and returns either `Admitted`,
carrying its evidence, or `Refused { condition, witness }`. The name is decided at
implementation time; the entry point lives beside `SweepContacts` in `solid/`.

- T1, T2 and M1 are structural reads of the solid and motion DAGs. M2 and E1 reuse the
  existing refusals. E2–E4 reuse `SweepContacts::at_source_over` and the ring roots.
- Every check is sampled and reports its sample spacing. The result type distinguishes
  "sampled, none found" from "certified", so a later interval check can strengthen the
  answer without changing its callers.
- Fixtures, each with a closed form:
  - sphere and cylinder under a rotation about a parallel axis (M2 refused);
  - cone under a rotation about an intersecting axis (admitted, one branch);
  - torus under a relative rotation about skew axes (admitted, known contact);
  - a profile with a concave corner (T2 refused);
  - a union tool (T1 refused);
  - a roll that leaves the tool in the blank (E1 refused);
  - a small-radius round driven into a fold (E3 refused).
- The general-sweep fixtures (tumbling cylinder, thin plate, slid and tilted cylinders,
  dumbbell) each produce the expected refusal.

**Exit:** every fixture's test passes in under a few seconds. `solventc` reports the
condition row and witness for a refused document.

## Phase 3 — Admission gates export

- `solventc --step/--stl` runs admission before any construction. A refused sweep writes
  no output and preserves old files, as today.
- The native path is the construction for admitted sweeps. Remove from it the per-case
  checks that admission now owns, so there is one statement of each rule.
- Add the field probe to the native export's acceptance, over a bounded sample of faces
  with its coverage reported. Export fails on any disagreement.

**Exit:** the common-apex pair still exports and reproduces the recorded tooth-space
volumes (120.7088 / 117.1373 mm³ within the recorded tolerance) with zero probe
disagreement. The mesh path's hypoid tests remain diagnostics.

## Phase 4 — The hypoid inside the class

Using Phase 1's supported offsets, export both hypoid members through the ordinary path.

- One tooth space per offset first (a `repeat 1` rewrite), then the indexed members.
- Gate on field agreement, encoded shell checks and a STEP round trip. Compare volumes
  against the Ju et al. reference run on the same tool and motion as an independent check.
- Offsets outside the class must refuse with E2–E4 witnesses. That refusal is a passing
  test.

**Exit:** both hypoid members export at every supported offset with zero field
disagreement. Every unsupported offset refuses with its reason.

## Phase 5 — Widening only where the design needs it

Each item needs a user decision and its own gate. None is started by default.

- **5a Undercut:** a declared intersection between the tip-round envelope and the flank
  envelope of one sweep, built on `BoundarySeam`/`EnvelopeSeam`. It admits E4 crossings
  only between those declared pairs.
- **5b Crossing branches at larger offsets:** if Phase 1 shows the needed offsets cross, the
  same mechanism generalizes to declared branch pairs.
- **5c Certified admission:** interval enclosures of E2–E4 over whole regions, extending
  the circular-crown verifier.

## Phase 6 — Pair acceptance and the WASM path

- The pair acceptance work in the [roadmap](spiral-bevel-roadmap.md) resumes: engagement
  across a tooth period, the 0.0254 mm end-to-end budget, and multiple configurations.
- The kernel-free path for the one-core WASM target is decided here. It reuses the core's
  admission and contact machinery, and it must pass the same field gate the native path
  does. The Manifold mesh arrangement is one candidate if it can pass that gate.

## What does not change

Solvent syntax is unchanged. `solid removal(tool, under:, from:, to:)` is still the
declaration, and the class is a property checked of it. Material evaluation
(`MaterialField`, `SweptField`) still answers membership for every sweep, admitted or not.
Only boundary construction and export are restricted.
