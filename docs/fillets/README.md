# Fillets and chamfers written in Solvent

The record of #183, the epic to write chamfers and fillets in Solvent rather than in Rust. Each
experiment adds its section here as it reports.

## X0: the target programs (2026-10-11)

Thirteen programs in the proposed syntax, in [`x0/`](x0/). **None of them parses yet**, and nothing
in the repository reads them: they are outside `rust/examples/`, the only directory the suites
sweep. They are what the language steps (L1–L8) must make true, and they become those steps'
examples as each lands.

| program | from | what it exercises |
|---|---|---|
| [`pad.sv`](x0/pad.sv) | `solid_fillet.sv` (#79) | a concave ring, two convex straight rounds ending flush, a later bore |
| [`turned.sv`](x0/turned.sv) | `solid_fillet_turned.sv` (#79) | a torus root and rim; the same part as a region, which fails (finding 11) |
| [`knob.sv`](x0/knob.sv) | `solid_fillet_knob.sv` (#83) | plane–cylinder and cylinder–sphere roots |
| [`rail.sv`](x0/rail.sv) | `solid_fillet_rail.sv` (#83) | a section with a curved side, flush ends |
| [`tee.sv`](x0/tee.sv) | `solid_fillet_tee.sv` (#85) | a traced crotch |
| [`bore.sv`](x0/bore.sv) | `solid_fillet_bore.sv` (#85) | a convex saddle made by a cut |
| [`slot.sv`](x0/slot.sv) | `solid_fillet_slot.sv` (#90) | chains of tangent edges, concave and convex |
| [`block.sv`](x0/block.sv) | `solid_fillet_block.sv` (#93) | every edge; spherical corners, convex and concave |
| [`lid.sv`](x0/lid.sv) | `solid_fillet_lid.sv` (#99) | mitres: a rim rounded with its third edges sharp |
| [`runout.sv`](x0/runout.sv) | `solid_fillet_runout.sv` (#110) | traced runs cut off by the face they run onto |
| [`hypoid_blank.sv`](x0/hypoid_blank.sv) | `spiral_bevel/blank/member.sv`, `ends.sv` | chamfers in a region's body (#145 F5) |
| [`hypoid_tooth.sv`](x0/hypoid_tooth.sv) | `spiral_bevel/crown/section.sv`, `rounding.sv` | rounds in a region's body (#145 F5) |
| [`conrod.sv`](x0/conrod.sv) | #66's closing case; `engine/conrod.sv` | everything at once, on a real part |

### The vocabulary the programs assume

- **`A round(r) B`**, `A` and `B` face paths of one body (`pad.boss`, `pad.plate.near`): every edge
  where a face of `A` meets a face of `B` at an angle is rounded by a ball of radius `r`.
  - Concave edges are filled and convex ones taken away, **read off the geometry, never said**.
  - A seam where two faces meet tangent is no edge, so a chain of tangent edges is one run.
  - The body is the paths' common prefix: a round is a statement about a body, like `cut`.
- **`round(r) X`**: every edge among `X`'s faces, the prefix form beside the infix, as
  `horizontal l` is beside `a horizontal b`. On a swept solid it makes the solid a body over its
  own sweep, as `union` and `cut` do (§6.9, [0.31]).
- **`A chamfer(d) B`**, and `chamfer(d) X`: the same, a flat `d` each way in place of the ball.
  Two legs (`chamfer(d1, d2)`) or a leg and an angle (`chamfer(d, angle: θ)`) were not needed by
  any program here; see finding 7 on what a leg measures.
- **In a region's body**, `A` and `B` are two of the region's own bounding sets (#145 F5).
- **A size of zero is the identity.**
- **A round may be named:** `root := pad.boss round(r) pad.plate` makes its faces `pad.root.round`,
  `.a` and `.b`. Today a relation defines no name; see finding 2.
- **Order** (X4's candidate): a round comes after every term that makes a face it names, and
  before every other term.

### What the programs come to

Solid statements (`solid(…)`, body words, rounds), the drawing aside:

| | stack | X0 |
|---|---|---|
| pad | 12 | 9 |
| turned | 8 | 6 |
| knob | 10 | 8 |
| rail | 6 | 5 |
| tee | 6 | 5 |
| bore | 7 | 5 |
| slot | 11 | 8 |
| block | 11 | 7 |
| lid | 15 | 9 |
| runout | 13 | 10 |
| **total** | **99** | **72** |

- **Most of the saving:** each fillet was a noun and a `union` or `cut`; it is now one statement.
- **The rest:** three named intermediates the stack needed (bore, slot, runout) go under the
  ordering rule, and the block and lid lose their bodies over the prism.
- **The hypoid's end relief:** `EndChamfer` (44 statement lines), `EndCut`, and two `repeat
  design.ends_relieved` switches become two lines in the blank's region.
- **The hypoid's crown section:** `RackSection` (30) and `TipRounding` (9) become an eight-line
  region.
- **The con-rod:** `engine/conrod.sv`'s 42 lines of 2D fillet geometry become three statements: a
  face group and two rounds.

### Findings

1. **One word, no side, works everywhere.** Every round in the thirteen programs is `round`; none
   says `union`, `cut`, convex or concave. The stack's E085 for the wrong word has nothing to
   refuse. A pair of face groups meeting at both kinds of edge (the stack refused it) is no
   special case.
2. **A round is a statement, but its faces need names.** Drawings and dimensions reach a round's
   faces (`pad.root.round` in #79). The programs name one with `:=`, which today defines values,
   declarations, instances, groups and chains, never a relation. Decide:
   - allow `name := A round(r) B`; or
   - derive the name from the operands (`pad.round(boss, plate)`); or
   - name only by the statement's path.
3. **The prefix form is needed:** a block rounded all over, a pocket rounded inside, and the
   con-rod's "every other edge".
4. **Face groups need a list.** Two face paths with a common prefix covered every program but the
   con-rod. There, the shank's two side faces must be rounded into each eye and the shank's flat
   faces must not be (the pockets leave no room for the ball). The program writes the group as a
   group of aliases, `sides := {left: rod.shank.left, right: rod.shank.right}`, the form `dims`
   bundles already take. Using a group as a round's operand is new.
5. **"Every other edge" is the open question of order.** The con-rod rounds the shank into the
   eyes, chamfers the bores, then says `round(re) rod`. The ordering rule settles a round against
   cuts and unions, but not a round against a round over the same edges:
   - **(i) The narrowest statement owns an edge**, and the catch-all reads the body before any
     round. The edges the shank fillets end on stay sharp.
   - **(ii) A round reads every face it names, other rounds' included**, so it comes after them.
     The fillets' ends are rounded, but so are the chamfers' edges, which a 1 mm round on a 1 mm
     chamfer eats.

   Neither is right for the con-rod as a draughtsman means it ("round what is left sharp, but not
   the chamfers"). X4 should settle this with the con-rod as its case; a word to exclude
   (`except:`) is one way out.
6. **Edges are named by faces, always.** No program needed a single-edge path (§17's reserved
   `body.block.side_l.near`) or an edge constructor. Every round is "where these faces meet
   those". L1 can leave the edge's own spelling for later.
7. **What a chamfer's leg measures is not settled, and the hypoid pins one.**
   - Today's end relief is `size` along the tip cone from the corner, and on the sphere the point
     `size` below the tip cone, not `size` from the corner.
   - A leg measured as the distance from the edge differs on the sphere by `1 / sin` of the angle
     between the faces there, so `gear_cells.rs`'s recorded volumes would move.
   - Decide a leg's one measure (from the edge, in the face) and re-record, or give the word a key
     for it.
8. **A round names the sets the body bounds by, as stated.**
   - #145's `Tooth` bounds by `outside(grow) a` and rounds `a round(r) hi`. But `a` is not a bound
     of the region; `a` shrunk by `grow` is.
   - The program names the offset sets and rounds those. That is correct, and costs two lines.
   - Alternatives: let a round name `a` and mean the bound it is stated through (one bound per
     set), or let a bound be named where it is written.
9. **Rounds that consume a face need a ruling.**
   - The stack refused a ball larger than a face holds (E085).
   - The hypoid today keeps its tip a land by a bound (`distance(>= 0.01 * module) tip`), which the
     region form has no tip line to state.
   - The con-rod's eye fillets would eat the shank's flat faces beside the pockets, if finding 4's
     group did not exclude them.

   Under the definition every such round has an answer: the faces between meeting rounds go, and
   the tooth comes to a full round. Decide whether that is a warning (a W code naming the face) or
   a refusal; #66's acceptance asked for a refusal by name.
10. **An offset of zero must be the identity.** The tooth's backlash is 0 in
    `fixtures::gear::design`, and 0.64 refuses a distance of zero from a set
    (`distance(0, side: inside) a`). A small spec change, needed before L5 can carry the hypoid's
    fixtures.
11. **A region cannot say a join** (`turned.sv`). Regions are conjunctions; a frustum standing on
    a disc is a union of two, which no one region states. So rounds in a region's body (L5) cover
    one region's own edges: convex between two bounds, concave where `outside` makes a difference.
    A cast part's roots are joins and need rounds on solids (L6). L5 alone carries the hypoid; L6
    is the step every cast part needs.
12. **A face path reads two ways.** The con-rod's pockets are bounded by distance from
    `shank.left`, read as its carrier (the whole plane); a round reads a face as its patch (the
    trimmed face). L1 must say which reading each use takes; a carrier in a region's body and a
    patch in a round is what these programs assume.
13. **A solid is a set in a region's body.** The con-rod's pockets keep clear of the eyes by
    distance from the solids `big_end` and `small_end` (the signed distance from a solid,
    L3). Reading a body's own faces there (`rod.big_end`) would make the pocket read the body it
    cuts, so the pockets read the operands. That is how X4's cycle stays out of region bodies.
14. **Sketch fillets become solid rounds.** The con-rod's outline in its plane is sharp, and the
    rounding is said once in space, where `engine/conrod.sv` draws four tangent arcs. The derived
    views (§6.11) show the rounds, read from the exact solid; the sketch no longer does. That is
    the point, but `engine/conrod.sv`'s sheet would change.
15. **A round's size is a dimension.** `round(3mm)` states a Length. `dimensions(body)` (§6.12)
    should report `R3` on the round's face, and a sheet should be able to call it out; the body
    words have no `CKind`, so this is new.

### What X0 hands on

- **X1** (rounds and chamfers of a coaxial region, exactly in the meridian):
  - `hypoid_tooth.sv` and `hypoid_blank.sv` are its targets;
  - findings 7, 8 and 10 must be settled first, or X1 measures against a moving fixture.
- **X2** (the definition, by brute force):
  - `lid.sv` (mitres), `block.sv` (spherical corners) and `runout.sv` (cut off by a face) are its
    three hard cases;
  - finding 9 is its question too.
- **X3** (faces as sets): findings 6, 12 and 13.
- **X4** (rounding a body's own edges, and order): findings 2, 4 and 5, with the con-rod as the
  case. The ordering rule removed every named intermediate the stack needed, which is evidence for
  it; finding 5 is what it does not yet decide.
- **Spec, before L1:** findings 1 (no side), 3 (prefix form) and 6 (edges by faces) are settled by
  the programs; 2, 4, 5, 7, 8, 9 and 10 are decisions.
