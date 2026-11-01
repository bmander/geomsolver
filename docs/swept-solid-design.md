# The swept solid, closed (#721 X0–X6): a design for review

`sweep(S, M)` is the set `∪_{t∈[a,b]} M(t)·S` and nothing more (design bible, principle 2). This
note designs **X0**, the one pipeline that builds that set as a closed B-rep in `cad-kernel`, so
that X1–X5 are its cases rather than rows of their own and X6 is its output. It is written against
the kernel as it stands after F2: the trajectory (`trajectory::Trajectory`), each face's contact
under a twist (`contact`), the tool read as faces (`swept::tool`), the classes' sheets
(`swept::{constant_twist, helical, trace, extruded, planar, region, canal}`) and the kernel's
split (`boolean::split`). Nothing below is built yet; it is the plan the boxes are built to.

## 1. What the boundary is made of

For a tool `S` with boundary faces `f(u, v)`, outward normal `n`, carried by `M(t)` over
`[a, b]`, every boundary point of the sweep is one of (Abdel-Malek & Yeh 1997; Blackmore, Leu &
Wang 1997 — the sweep-envelope differential equation; for free-form tools, Machchhar, Plakhotnik & Elber
2017, and Sellán, Aigerman & Jacobson 2021, as the epic's G10 cites):

1. **An envelope point**: `M(t)·f(u, v)` where the face is grazing, `n · v = 0` (`v` the velocity
   of the material point under `M` at `t`). The contact set `Z_f = {(u, v, t) : n·v = 0}` is a
   surface in the face's parameter–time space; its image is the face's **envelope sheet**.
2. **A fan point**: on a convex edge `e(w)` every normal between its two faces' is carried; the
   contact set is `{(w, θ, t) : n(θ)·v = 0}`, θ across the fan — the edge's **fan sheet** (the
   walks' `Fan`, a cap's rim, a drafted prism's crease). A vertex carries a cone of normals: its
   contact set is a curve family over the cone (G1), a **cone sheet**.
3. **An end point**: `M(a)·∂S` where the tool is entering (`n·v ≤ 0`) or `M(b)·∂S` where it is
   leaving (`n·v ≥ 0`): the **end faces** (X1). A whole turn has none (#684 N).

The sweep's boundary is a subset of the union of all these. Which subset is the global question,
and it is answered by membership, not by local rules (§4).

## 2. Sheets over explicit domains

Each candidate is a **sheet over a domain**: a map `Σ: D → ℝ³` with `D ⊂ ℝ²` bounded by known
curves. The kernel already builds every kind the classes need; X0 names them one type:

| source | domain `D` | the map | built today by |
|---|---|---|---|
| a face under a constant twist | (station along the characteristic, roll) | the characteristic carried by the screw | `constant_twist`, `helical` |
| a face under a relative rotation | (walk length, station) | each tool point at its contact time | `trace::traced_sheet` |
| a face under a slide or turn of its plane | (curve parameter, extrusion) | the planar cut extruded | `extruded::sheet` |
| a face under any other motion (K2) | (roll, ring height, ±) | the ring root `A cos θ + B sin θ + C = 0` at each instant | — (K2) |
| a fan | (edge parameter, fan angle) | the fan normal's contact time | the walks' fans |
| an end pose | the face's own `(u, v)` | `M(a)` or `M(b)` applied | `geom::Rigid` |

`D`'s boundary is made of: the face's own boundary (where it meets a neighbour, so where a fan
takes over), `t = a` and `t = b` (where an end face takes over), and the **singular loci** (§3).
A sheet whose domain is cut by a singular locus is split there into **regular patches**, each an
immersion of its piece of `D`.

## 3. Singular loci, computed in the domain

These are zero sets of functions the contact chart already has, so they are curves in `D` found
by the same root machinery, not detected afterwards in space:

- **The branch locus**: where two roots of the contact equation meet. For a ring it is
  `A² + B² = C²` (`contact::RingRoots::Double`); for a generating chart, the double root the
  admission counts (`near_double_roots`). Across it a sheet turns from one root's branch onto
  the other's (#684 lesson 4).
- **The fold locus**: where `Σ` is singular, its area factor zero (E3's `area_factor`, a
  constant twist's S3). Beyond it the sheet folds back, and the part beyond is interior.
- **The cusp points**: where the fold locus is tangent to the kernel of `dΣ`, on the fold locus.
- **The degenerate sets** the walk already names: a ring in contact all round (X), a whole ruling
  (AF), a flat kept in its plane (O), an apex or pole. Each is a curve or patch of `D` the chart
  reports by its root kind (`RingRoots::All`, `RulingRoot::All`), and each becomes a patch
  boundary with a named junction, as in the walk (F3).

Splitting at these loci gives patches that are each locally embedded and regular. That is the
precondition for the arrangement: two regular patches meet in curves the kernel's surface–surface
intersection traces (`ssi`), and a patch meets itself only across a fold or a crossing far along
it (X5), both of which §4 handles uniformly.

## 4. The arrangement, classified by membership

1. **A bounding solid** `B`: a box holding the sweep, from the tool's box carried by the
   trajectory (the speed bound `Trajectory::inverse_point_speed_bound_over` gives the reach), a
   margin past it.
2. **Every candidate patch fitted** (`sheet::settle`, held to the tolerance, `Fitted`), end faces
   moved exactly, fan and cone patches fitted the same way.
3. **`B` split by all of them** (`boolean::split`, the batching `swept::sector::split_by` already
   does): the cells of the arrangement. Each cell is a connected closed shell.
4. **Each cell classified** at deep interior samples (`query::interior`) by **membership**:
   `x ∈ sweep(S, M)` exactly when `M(t)⁻¹ x ∈ S` for some `t ∈ [a, b]`. The kernel owns this
   test (`swept::member`, to be built): the tool's own B-rep point classification (as the Booleans
   classify a piece; a public reader of it is part of this step) of the point carried back,
   minimised over the roll by branch and bound with the trajectory's speed bound — the inner
   envelope's `Keep::inside`, turned round. The solver's `MaterialField` stays the reference the
   gates hold the result to; the kernel's test is how the kernel decides without the drawing.
5. **The kept cells united**: the closed swept solid, **X6**. A face of the arrangement lies on the
   sweep's boundary exactly where the cells on its two sides classify differently. Faces with
   material on both sides (a buried sheet, the far side of a fold, a trimmed branch) drop out by
   construction.

Classification is sampled, so a cell read at more than one deep sample must agree. A cell that
reads both ways was not separated by a sheet, and the build is refused there with the two points
as witness: the sector's `judge` rule, as it is today.

## 5. X1–X5 as cases of the pipeline

- **X1 (end poses)**: end faces enter the arrangement as patches, so plunge cuts and cuts that stop
  partway need nothing else. Cutting such a solid from a body is #699 C, the ordinary Boolean.
- **X2 (several contacts per tool point)**: each root branch of `Z_f` is its own patch (the branch
  locus separates them), so a point with two contacts contributes two patches, each classified.
- **X3 (undercut, concave corners and creases)**: the two faces' sheets at a concave edge cross.
  The arrangement splits them where they cross, and membership drops the parts that are inside.
  This includes a prism drafted outward both ways.
- **X4 (a flat end whose corners dig past it)**: the corners' fan or cone sheets and the cap's
  envelope intersect. The cap's buried part is classified interior.
- **X5 (a tube meeting itself)**: a patch split at its fold locus is regular, and its pieces are
  intersected pairwise like any two patches. A ball's tube that comes back on itself (B2's refusal
  today) becomes a case the arrangement resolves.

## 6. Tolerance and robustness

- Fitted patches stand within the fit tolerance of the exact sheets (`Tolerance`, 0.01 mm
  default). The split works to the kernel's own tolerance (`SPLIT_TOL`). A cell thinner than the
  fit tolerance cannot be told from the fit's error, so it is refused naming where. This is F4's
  ground: the thin bands `SHALLOW_FAN` dodges today are such cells.
- Deep samples are taken farther from a cell's boundary than the fit error (the `judge` margin).
- Tangential contacts between patches (E4's near-tangent pairs) are where `ssi` is
  ill-conditioned. They are detected in the domain where two patches' normals agree, and refused
  until the arrangement handles a tangency as one edge.

## 7. The kernel's operation and its refusals

```
swept::sweep(tool: &Brep, path: &Trajectory, roll: [f64; 2], tolerance: Tolerance, say: &Say)
    -> Result<Brep, Refused>
```

It runs in stages, each with typed refusals carrying a witness:
1. **Read**: the tool's faces, edges and vertices (F1).
2. **Contacts**: each face's chart, chosen from the trajectory's structure.
3. **Patches**: the domains, singular loci and fits.
4. **Arrangement**: the split.
5. **Membership**: the classification.
6. **Assembly**: the union and `Brep::check`.

The solver reads the drawing, lowers it to these and says the refusals. The blank-scoped sheet of
today's classes (#699) is this operation with `B` the blank's box and only the patches that reach
it. It stays as the fast path where #699 D says so.

## 8. The order to build it in, each step gated

1. **`swept::member`**, the kernel's membership test, held to `MaterialField` at sampled points of
   every example sweep (agreement, zero disagreement away from the boundary). *Built*: the tool's
   mesh read by its generalised winding number and nearest triangle, the roll by branch and bound
   (`tests/swept_member.rs`, `a_swept_solids_membership_is_the_material_fields`).
2. **The patch type** `Patch { domain, map, loci }`, wrapping the sheets the kernel builds today:
   the constant twist's characteristic sheet, the traced sheet and the extruded sheet. No
   behaviour change; each class's sheet becomes one patch.
3. **End faces** with entry and exit trimming (X1).
4. **X6 for a constant twist**: a box slid (a known prism), turned (a known solid of revolution)
   and screwed. Each is held to its closed form's volume and to `member`. *Built for a slide*
   (`swept::closed::slid`): a polyhedral tool's candidate planes, extended to the whole plane so
   the kernel's split takes them, cut a box half-space by half-space (`build::kept`); each cell
   kept by membership; a box slid aslant is its zonotope to 1e-9, an L-shaped prism's sweep its
   blocks' united, its concave edges resolved by the arrangement as X3 says
   (`tests/swept_closed.rs`). A turn's candidates are surfaces of revolution (an edge's line, a
   face's contact line, turned whole), next.
5. **Fans and cones** (G1) as patches. X3 and X4 follow, held by field agreement and an
   independent volume.
6. **Fold and branch loci** in the domain, then X2 and X5.
7. **The generating and canal classes' sheets as patches**, then **K2's** chart over
   `(t, h, ±)`.
8. **The blank-scoped path** as the optimisation of X6 (#699 D).

Every step meets the epic's bar: rows in `docs/contract/generating.md` naming the gate test, field
agreement and an independent volume at the default tolerance, negative controls kept, an example
per motion family.
