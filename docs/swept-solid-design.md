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
   contact set is a curve family over the cone (G1), a **cone sheet** — in the normals; its image in
   space is the vertex's path alone, a curve (step 5).
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
| a leaned prism's side under a rack's roll (#826) | (curve parameter, height) | the transverse planar cut carried up its helix | `extruded::screwed` |
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
   (`tests/swept_closed.rs`). *Built for a turn* where the tool's edges run along the axis or
   square to it: its candidates are planes and coaxial cylinders (a cell split by a solid cylinder,
   in common and cut away). Two rules the turn taught, now the pipeline's: only candidates that
   carry some of the boundary are kept (membership differing on their two sides, sampled), since a
   superset candidate touching another along a line leaves cells that topology cannot hold; and a
   face along the axis touches its own contact cylinder at each end pose, so the plane through the
   axis and that line is added as a separator, parting the two sides of the touch into two cells.
   A bar turned a quarter turn beside the axis is its polar integral to 2e-15. Edges leaning to the
   axis (hyperboloids), screws, curved tools and K2 are the steps after (a curved tool under a
   slide is built, below).

   *Tried and set aside, for the next step:* a block leaning to the axis. Two findings. A face's
   contact line under a turn always meets the axis (it lies in the plane through the axis and the
   face's normal), so a leaning face turns into a double cone, and only a skew edge into a
   hyperboloid. A hyperboloid's solid is exact as a rational quadratic meridian turned, and a cone's
   two nappes as triangles turned. But a face at an end pose touches its own contact cone along a
   line, and the kernel's Booleans refuse a plane and a cone meeting at that angle ("nearly
   touching: their meeting is ill-conditioned there, not built yet"). The cells' unions near such
   touches come out not closed or not a manifold, even on a rounding-level change in how a
   separator is written. **So the arrangement past planes and cylinders waits on the Booleans
   meeting tangent and near-tangent surfaces robustly**: F4's kernel side, #699 E. It is that
   work, not more candidates, that the leaning tool, the screw and the curved tool all need next.
   *Begun*: a plane tangent to a cone along a generator is met in closed form, the tangent line
   (`ssi::intersect`; `brep::surfaces_meet_in_curves_on_both`), where it was traced and refused.
   With it a leaning block's whole turn builds past the refusal. Three rules then made its
   arrangement split and classify: candidates are read where they can carry boundary, at their
   source's points carried over the roll (a box-wide grid missed a narrow band near the tool's top
   and bottom), inside the source and the roll (a site at a source's own edge reads its neighbour's
   boundary); each surface of revolution is built over the heights its source spans, which the turn
   keeps (extended through a touch, a cylinder and a hyperboloid leave cusp-shaped cells between
   them); and the plane square to the axis at a hyperboloid's throat parts it from the cylinder of
   its throat's radius. What is left is the cells' union: a vertex lands off its curve where a
   hyperboloid's traced meeting with a leaning plane ends — the Booleans' robustness.
   *Built for a whole turn of a convex tool, by another road*: no arrangement at all. Turned
   whole, a tool sweeps the solid of revolution of the radii its slices hold; a convex tool's slice
   is convex, so those radii are one span `[ρ_min(z), ρ_max(z)]`. The farthest is reached at a
   slice's corner (an edge's meridian, a hyperbola), the nearest on the axis, on a side (a face's
   contact line, a line) or at a corner; which one reaches is fixed between the heights where two
   reach as far (their `ρ²`, quadratics in `z`, equal or touching — a corner passing the foot of a
   side's perpendicular is a touch) or the axis crosses a face. The profile is cut there, each
   stretch read at its middle, and turned once (`build::revolve`): exact, and with no Boolean, so
   none of the touches above arise. A leaning block, and a block straddling the axis tipped two
   ways, are each the integral of their slices' spans (`tests/swept_closed.rs`).
   *Built for a part turn of a leaning tool*, by the arrangement, once three of its touches were
   met. A face at an end pose holds the edge whose hyperboloid is a candidate, so it meets that
   hyperboloid in two rulings; traced, the meeting would not close. A plane meets a quadric of
   revolution in a conic, now in closed form (`ssi`), held to the meridian's heights. A cell lying
   inside a hyperboloid and touching it along its own edge leaves the Boolean a piece of the
   hyperboloid with no inside; such a piece bounds nothing and is dropped. And a cusp cell between
   two surfaces of revolution touching along a circle, united before its neighbour, makes a union
   that is no manifold for a moment; it waits a pass. One rule tried and set aside: extending each
   surface of revolution a little past its source's heights, so its disks stand clear of the
   vertices' heights, leaves a thin cusp around each vertex's circle that is no member, a few
   parts in a million of the volume, which only an independent volume (the slices' arcs carried
   and united, to 1e-8) found. X1–X5 then come as §5 says, cases and not rows: a bar turned
   through all but its width meets itself and is trimmed (X5, X1), an L-shaped prism's concave
   edge is trimmed under a turn as under a slide (X3), each held to its slices' arcs carried and
   united.

   *Tried and set aside, for the screw*: a bar beside the axis under a screw. Its edges along the
   axis and its side faces' contact lines still turn into cylinders, but its level edges screw into
   helicoidal sheets, which close into no solid as a line turned does. A level triangle on the line
   screwed is a valid tube (a level plane crosses a screw's velocity everywhere, by its advance, and
   a helix crosses each level once), and the kernel builds it exactly (`ruled`'s `Blend`s). But a
   small tube is a thin twisted layer, and inside or outside it is not the sheet's two sides: a cell
   reaches round its ends and is classified whole. A triangle and a roll reaching far past the box
   make the tube's boundary within the box the sheet alone, but a sheet of several turns meets
   each plane of the arrangement in a dozen pieces, and the Booleans lost one. Met in closed form
   along the way: two coaxial screwed lines' sheets meet along helices (`Curve::Iso`), a screwed
   line's sheet and a coaxial cylinder likewise, a level line's sheet and a level plane in one
   ruling, and any plane explicitly (`u(v)` along each ruling). And an independent volume: in
   helical coordinates `(ρ, ψ, ζ = z − kψ)` the screw is a shift in `ψ`, the volume element
   `ρ dρ dψ dζ`, so the sweep's volume is each helix's intervals of the tool, lengthened by the turn
   and united, integrated over `ρ` and one period of `ζ` (for that bar, `1 + Θ (3 + k)`). So the
   screw's next step is a closed piece of sheet the arrangement can cut by, as the turn's
   solids of revolution are. A box tight along the axis (a square prism as tall as the sweep) lets
   the far-reaching tube leave it within a turn, so each plane meets each sheet once; the cut then
   failed where a fitted plane–sheet meeting crosses an edge of a cell on the box's face. What the
   screw waits on is the Booleans meeting a fitted curve at a cell's edge exactly (F4).
   *The one arrangement (#735)*: `closed` is `boolean::arrangement_within` — the box split by every
   candidate at once, touches laid as edges — and `Complex::select` taking the members' boundary:
   no separators, no cut order, no per-cell Booleans, no deferred unions. The leaning block's part
   turn builds so, its slices' volume to 1e-6 and agreeing with membership, once A3 held its
   touches: crossings read to a shallow meeting's own reach; a triple point where two of its three
   surfaces touch placed by the third's root along their curve (where all three touch, at the
   vertex already there); an edge touching a surface its own face touches, at their touch curve;
   two rulings of one meeting cut where they cross; and the faces round an edge ordered away from
   a touch. It is slower than the separators' road (about 85 s against 26), all 17 candidate solids
   met pairwise in one arrangement. *And at a tolerance of 1e-7*, once a face holding a
   hyperboloid's ruling was met in its two rulings at any tolerance: the closed form had read the
   plane's constant at the hyperbola's centre (rounding, some 1e-14) as a hyperbola whose semi-axis
   (its square root, some 1e-7) parted it from the lines at any tolerance finer, so the meeting came
   out as branches running along the rulings the other candidates hold, never recognised as them.
   The hyperboloid itself was never off: every edge lies on its own to 1e-15.
   *Built for the screw* (#735's gate): a bar beside the axis under a screw, on the one arrangement,
   its volume `1 + Θ (3 + k)` to 1e-9 at 1e-6 and at 1e-7 and agreeing with membership. What it
   took, against the attempt set aside above. Each level edge's sheet is entered as an **open
   sheet**, not a tube: a level triangle's face on the line screwed past a box square to the axis
   and as tall as the sweep, its rims and ends outside it, so no cell reaches round it and no inside
   is asked of it. Every meeting a sheet has with what the screw keeps is in closed form: a coaxial
   cylinder in helices (the sheet touching the cylinder of its foot's radius along the foot's
   helix), another sheet of the screw in a helix, a level plane in a ruling, any other plane in its
   cut `S(−α/β, v)`, a new curve kind. The configuration that defeated the traced meetings is a
   triple touch: the bar's inner face at the end pose holds a ruling of its inner level edge's
   sheet and is tangent to the sheet at the edge's foot, where the cylinder of the foot's radius
   touches both. Traced, the plane's meeting with the sheet drifted some 8e-4 near the touch and
   the foot's helix was cut at two vertices; with the cut read about the held ruling by
   differences, all three curves pass through one vertex exactly. Then speed: crossings of two
   curves on one sheet are roots of their rules' difference in `v`, the two planes' cuts of two
   sheets on one plane cross on the sheets' helix, and a sheet's foot starts from the ruling level
   with the point (no grid search): the build went from minutes to some 20 s a tolerance.
   *Cut from a blank, the same way* (#699 C on #735's arrangement, `swept::closed::cut_from`): the
   blank is the domain the candidates split, and its cells are kept where they are no member — the
   classifier inverted, nothing else changed. The candidates that bound nothing are healed out of
   the result's faces (`boolean::selection`), so a groove slid into a block leaves the block's faces
   whole. Whether this or the blank-scoped sheet is the road for a given sweep is #699 D's question.
   *Dispatched from `admit_body`* (#721 F2c5, the closed class, rows C1–C3 in
   docs/generating-sweeps.md): a sweep the other classes refuse whose tool and motion `closed::reads`
   takes — every face a plane and every edge a line, one slide, turn or screw, under a screw nothing
   leaning — is admitted by that structure alone and cut from its blank on one arrangement, the
   tool and its screw placed where each placement stands. So the X1 cases (a bar plunged aslant partway into a
   plate, turned partway into a ring sector, screwed partway up a rod: `solid_sweeps/plunge.sv`,
   `stopped_turn.sv`, `stopped_screw.sv`) and the vertex and ruling refusals (a bar pushed aslant
   wholly through, a leaning bar turned wholly within a ring) are built, each to 1e-9 of a volume
   made apart and its STL agreeing with the field. Cut from a ring whose walls are surfaces of
   revolution, a leaning bar's hyperboloids met each other and the walls in traced curves of
   thousands of points, and crossing them took 300 s; two spline meridians about one axis now
   cross in closed form (`ssi::spline_spline`), and the cut takes 12 s.
   *Built for a curved tool under a slide* (`closed`'s slide, admitted as C1 allows): a tool whose
   faces are planes, cylinders, cones and balls and whose edges are lines and circles. Under a slide every
   candidate is a quadric the kernel meets in closed form, so nothing new is traced: a round face at
   each end pose is its own surface (the tool's cylinder or frustum over the stretch the face spans);
   its rulings whose normal is square to the slide (two on a cylinder across the slide; on a cone the
   roots of the ring equation `A cos u + B sin u = C`, the one `contact` writes) sweep the planes
   tangent along them, and a cylinder along the slide sweeps itself; a ball's great circle square
   to the slide sweeps the cylinder of its radius along its centre's path; a circle edge's fan
   (where its faces meet at an angle: a smooth one fans one normal, the faces' own) sweeps an
   oblique cylinder, entered as its disc's right cylinder sheared along the slide (`Brep::mapped`), so
   a plane, a coaxial cylinder or another such shear meets it in the preimage's closed forms. A drum
   slid aslant, a frustum slid aslant, a capsule slid aslant and a drum slid along its axis are each
   `V + |D| A` to 1e-9,
   `A` the shadow along the slide in closed form (for the frustum, two ellipses' hull, a stretch of two
   circles'), and agree with membership; cut from a blank, `punch.sv`'s drum pushed aslant wholly
   through its plate and partway in (`stopped_punch.sv`) are each their sections' swept hexagons
   clipped to the plate, to 1e-9. Where the drum stops in the plate, the plate keeps its end pose's
   cylinder, which its side's tangent planes touch along lines and its rims' oblique cylinders at the
   rims' silhouette points, the arrangement's touches as the turn's were. A candidate whose solid
   stands clear of the domain's box is left out of the arrangement: an end pose clear of the blank
   is never met. A torus slid along its axis or square to it has whole circles of contact (its
   equators; or its top and bottom rings and its meridians square to the slide), swept into
   cylinders and level planes, each meridian's cylinder touching the torus round it at the end poses
   (`ssi`'s `tube_along_torus`): a bull-nose mill slid to a stop inside a block builds to 1e-9. A
   torus slid at a lean to its axis touches along a curve no plane holds. The constant-twist class's
   curved refusals under a slide that this covers are its
   cases now: a hollow mill's ring bottom (G6), an engraving bit's apex (G5) and a drill beside a
   post. A torus's face (its contact under a slide a curve no plane holds, its sweep no quadric) and
   any curved face under a turn or a screw are the steps after.
   *Built for a curved tool under a turn* where its round faces stand about axes along the turn's,
   or are balls: a cylinder's or a cone's normal passes through the turn's axis along its two
   generators in the plane through both axes, so they turn into a cylinder or a cone about it; a
   ball's along its great circle in the plane through the axis and its centre, which turns into a
   torus; a circle square to the axis turns within its own plane. Each touches the tool at its end
   poses along a whole line or circle: parallel cylinders and cones whose sections' circles touch
   at every height meet along that line, and a ball centred on a torus's middle circle with its
   tube's radius touches it round that meridian (`ssi`, in closed form). Where those touches meet a
   plane square to the axis, two of its arcs leave one point tangent, and a face's pieces were
   ordered by their ways a ten-thousandth along each half's own length, where an arc of five times
   another's radius and length reads alike; ways tied within a thousandth of a radian are now read
   again at one length along each, by their curvature (`boolean`'s `pieces_of_face`). A drum, a
   frustum and a ball turned 100° are each their sections' Pappus volume to 1e-9; the drum turned
   whole bores its annulus, and turned to a stop partway into a ring sector the covered angles of
   each circle, each to 1e-9. A cylinder or a cone about an axis leaning to the turn's (its contact
   a curve on it, turned into a surface of revolution with no closed-form meridian) and a ball
   reaching across the axis (a spindle torus) are the steps after.
   *Tried and set aside, for a leaning edge under a screw* (#721 F2c5): three ways to enter the
   ruled helicoid a line leaning to the axis screws into, none of which the arrangement holds yet.
   (1) **A sheet across the box**, as a level edge's is: a leaning line's two points either side of
   its foot stand at one radius and climb apart, so one helix holds both wherever their heights
   differ by the pitch times the angle between them, a whole number of turns over — the whole sheet
   meets itself, and for a steep edge (0.35 rad off the axis, two a turn) within the box's reach,
   about a lead out along the line. (2) **A patch** over the line's own stretch and the sweep's
   roll, a little past each (where no point of it can come round onto another's helix: their
   heights differ by more than the roll climbs), its rim just past the candidates that seal it:
   where a rim stands inside another candidate's face, their meeting stops partway across that
   face, a slit no face's pieces close round (`boolean`'s "pieces do not close"), and the level bar
   built so failed where its sheets across the box had built. (3) **The triangle on the segment
   screwed whole**, a closed solid, so every meeting ends on an edge: its sheet's face then holds
   pairs of helices a few hundredths of its run apart (its own and its neighbours' vertices'), and
   its pieces would not close either. Met along the way, and set aside with them: two sheets of one
   screw meet in the helices of the points of the one's line lying on the other's sheet, found as
   roots of `exp(i Δz/k) e^{−iΔψ} − 1` (the angles' wrap no jump); a leaning face's contact line
   under a screw is moved off the plane through the axis by `n·v(c)`; and an independent volume for
   a tool less tall than a lead, its own and `θ ∫ ρ L(ρ) dρ`, `L` the range of `z − k ψ` over its
   slice by each cylinder (within 2e-5 of the bar's closed form) — a membership Monte Carlo of the
   leaning block (7.73 ± 0.07) agreed with it to two standard deviations, and the patches' build
   (6.08) with neither. So the leaning screw waits on the arrangement holding a face with a slit,
   or a sheet bounded inside the domain (F5's A1, non-manifold topology), and was refused
   (`Unbuilt::Leaning`, since removed).
   *Built (#777 M5)*: way (2) is the road. Under a screw a leaning edge, and a leaning face's
   contact line (its points where `n · v(x) = 0`, a line of its plane), each screw into a patch of
   the ruled helicoid over the segment and the roll a twentieth past each (`closed::Cutter::Patch`,
   `closed::patch`), its rim inside the domain; `reads` no longer refuses them. The arrangement
   holds the patches — no piece fails to close — its cells classified across the cores and the
   sheet search pruned by Krawczyk's preconditioned exclusion. Two sheets of one screw (a patch and
   its neighbour's sheet, two patches) meet in helices, the rails of the points of the one's line
   the screw carries onto the other's sheet: three equations in the line's parameter, the other
   line's and the turn, isolated with a bound (`ssi::screwed_lines_meeting`, M1's `isolate::solve`),
   each meeting exact (`Curve::Iso`). Traced and fitted instead, the corner's helix left the start
   pose's corner a few degrees off, among curves leaving it nearly tangent (the start plane's cut,
   the level edge's ruling, the cylinder of radius 3's helix 0.0075 away); the face walk ordered
   them wrongly, a sheet's piece walked the helix both ways, and the judge refused the build at a
   point it held outside the sweep. The gate,
   `swept_closed::a_block_leaning_to_a_screws_axis_closes_into_its_helical_sweep` (slow tier, about
   two minutes), holds the build to 1e-6 of a volume found apart from it: in helical coordinates
   the sweep's slice by each cylinder is the block's stretched along `ψ`, its area read along lines
   of constant `ψ` — the union, over the window `[ψ − Θ, ψ]`, of where each vertical line meets the
   block, clipped exactly. The earlier `Θ ∫ ρ L(ρ) dρ` held only where every helix meets the tool
   once; this block's slice by a cylinder near its far face falls in two runs (a circle crossing a
   rectangle's corners), so a helix meets it twice and that form overstates the sweep by 0.8%.
   *Built for a path of pieces* (K1): a sweep distributes over its path's pieces, so the arrangement
   takes every piece's candidates at once and membership reads the pieces one by one. A piece after
   another does not start at the identity: it takes the tool up where the one before left it, a
   constant twist after a placement, `twist(t) · at` (`Trajectory::Placed`; its speed bound its
   twist's), read on the tool placed so. A cube slid then turned about the edge it stops beside is
   its pieces' sweeps united, `5 + π/2`, to 1e-9. Screws among the pieces wait on the screw's box,
   which is a lone screw's.
5. **Fans and cones** (G1) as patches. X3 and X4 follow, held by field agreement and an
   independent volume. *Designed (#780 L2).* §1's fan and cone, read where their contact lands:
   - **A fan's sheet lies on the surface its edge sweeps.** At a point `e(w)` of a sharp edge, tangent
     `T`, the fan is the arc of unit normals from one face's `n₁` to the other's `n₂`, all square to
     `T`. The normals square to both `T` and the velocity `v` are `±T × v`, so `(w, θ, t)` is in the
     contact set exactly where one of them is in the arc, and then `θ` is fixed by `(w, t)`: the
     contact set is the set of `(w, t)` where `n₁ · v` and `n₂ · v` differ in sign (the slide's
     silhouette-edge rule, read under any motion), and its image is the edge's own points carried.
     So the fan's patch is a patch of the edge's sweep `∪ₜ M(t) e`, over the domain `(w, t)` that
     rule bounds. Its rim inside the domain is where `n₁ · v = 0` or `n₂ · v = 0`: where a face's own
     contact reaches the edge, and there the fan's sheet and that face's envelope sheet share their
     normal — they are **tangent along the curve that point sweeps**.
     The arrangement must hold that touch as one edge, as it holds a face at an end pose touching its
     own contact's surface (C3's `own_contact`). Under a constant twist the domain is fixed on the
     tool (each point's velocity is), so the patch is the edge's sweep over arcs of the edge, and
     the candidate is entered whole or a little past those arcs, as M5's patches are, with
     membership dropping what bounds nothing: a line's sweep (a plane under a slide; a cylinder, a
     cone or a hyperboloid under a turn; a helicoid's sheet or patch under a screw, all built), and
     a circle's (an oblique cylinder under a slide, a plane under a turn where it is level, a screwed
     disc's side under a screw where it is level, all built; **a ring under a turn where it leans**,
     built here).
   - **A vertex's cone sweeps no sheet.** Its contact set `{(n, t) : n in the cone, n · v(p, t) = 0}`
     is two-dimensional in the normals, but every point of it maps to `M(t) p`: its image is the
     vertex's path, one curve. That curve is where the sweeps of the vertex's edges (and of its faces'
     contacts) meet, and the arrangement already lays it there, each of those candidates reaching a
     little past the vertex. So nothing is entered for a vertex, and §1's "cone sheet" is a sheet in
     the normals, not in space.
   - **X3, a concave edge.** Its faces' material wraps round it more than half a turn, so its fan is
     empty: it carries no boundary, and `reads` enters nothing for it (a stem entering a ball, C3).
     What bounds the sweep near it is its two faces' sheets, which cross; the arrangement splits them
     where they cross and membership drops the buried parts (an L-shaped prism under a slide and a
     turn, built at step 4).
   - **X4, a flat end whose corners dig past it.** A flat end's own contact (a line of its plane)
     misses the face, or reaches only part of it, and its rim's fan carries the boundary beyond.
     *Built under a turn* (`tests/swept_turned.rs`): a sharp circle leaning to the turn's axis
     sweeps a ring. Measured from a point `p` of the axis, `ρ² + z² = |x − p|²` and `z` are affine in
     the circle's own coordinates `q` about its centre `c` (`|q| = r`): `(|x − p|², z) = m + L q`
     on it, `L = [[2 (c − p)·e₁, 2 (c − p)·e₂], [e₁_z, e₂_z]]`, so in `(ρ² + z², z)` its meridian is
     the ellipse `m + L (circle)`, and in `(ρ, z)` a simple closed curve wherever `L` is invertible
     (the circle's normal off the plane through the axis and its centre, where its two halves would
     turn into one surface) and the circle stands clear of the axis (`turned::Rim`, C3's `Turned`
     with a circle for its source, its jets through `sin_cos` in floats and intervals). A flat-ended
     drum leaning 25° and turned whole closes into its solid of revolution, to 1e-9 of its slices'
     radial spans read in closed form (it agrees to about 3e-12). **Turned part of a turn it is
     built too**, once every meeting at its end poses was given in closed form. At each end pose the
     end's plane and the side both hold the circle the ring holds. On the end's plane, at `c + q`,
     `(|x − p|², z) = m + L q + (D, 0)`, `D = |q|² − r²` the power of the circle, so the ring is where
     `|q + D k|² = r²`, `k = L⁻¹ (1, 0)`: `D (1 + 2 q·k + D |k|²) = 0`, the circle and
     `|q + k/|k|²|² = r²` — **a second circle of the same radius** (`ssi`'s `plane_holding_rim`). On
     the side's cylinder at the pose, at `c + h a + r (cos φ e₁ + sin φ e₂)`, `(|x − p|², z) = m +
     r L (cos φ, sin φ) + (δ(h), h a_z)`, `δ` quadratic in `h`; with `u(h) = h v(h)`, `v = M₀ + h k`,
     the ring is where `|r (cos φ, sin φ) + u|² = r²`: the circle (`h = 0`), and `cos(φ − arg v(h))
     = −σ(h)`, `σ = h |v| / 2r` — **one closed loop**, read round an angle `τ` with `σ(h) = −cos τ`
     and `ρ = cos τ v̂ + sin τ v̂⊥` wherever `σ` climbs from −1 to 1 (its rate's sign that of the
     quadratic `2|M₀|² + 6h M₀·k + 4h²|k|²`, held above nought over the stretch), crossing the circle
     at `τ = π/2, 3π/2` where the side touches the ring, where the side's contact reaches the rim
     (`turned::Crossing`, `ssi`'s `side_holding_rim`). And at the point the side's contact reaches the
     rim three surfaces meet — the end's plane, the side's contact turned (tangent there to the ring)
     and the ring — so the plane's cut of the contact's surface is given exactly too: at the
     meridian's height `z(v)` the plane meets the circle of radius `ρ(v)` where `cos(ψ − ψₙ) = C(v) =
     (d − n_z z) / (|n_h| ρ)`, read round `τ` with `C(v) = −cos τ` wherever `C` runs one way over the
     span (`turned::Section`). Two readings the part turn needed beside them: a circle and a turned
     curve in its plane cross where the curve meets the circle's cylinder, so their tangency is one
     vertex (`query::curve_curve`); and a closed loop and a closed meridian are periodic, one rule
     for both (`Turned::period`: a whole period of its source's angle, read round it in Newton) — an
     edge from a loop's
     start round to its end had been dropped as no edge, and a trace across a ring's meridian's start
     stalled where its slant was held to the span. The side's surface at an end pose reaches past
     each rim beyond its loop, so its ends stand clear of where the side and the ring all but touch.
     A rim beside a cone or a ball is refused by name part of a turn, that face's meeting with the
     ring the next closed form to give.
     The flat-ended drum turned 100° is its own volume and `Θ/2π` of its whole turn's, to 1e-9; drawn
     (`solid_sweeps/leaning_mill.sv`), swung inside a ring, its STL agrees with the field.
   - **An arc of a circle**, under a turn: its points sweep part of its circle's ring, so the whole
     ring is its candidate, read at the arc's own points; membership keeps what bounds the sweep, and
     two arcs of one circle (a rim a Boolean split) enter one ring. A half drum (a drum cut along its
     axis) turned whole closes into its solid of revolution, to its slices' spans worked in closed form
     — an ellipse cut by half-planes, its extremes at its arcs' critical points, its corners, its
     lines' crossings and feet, the integrand cut wherever the kind of point holding an end changes.
     Turned part of a turn it is refused by name (`Unsettled`): at an end pose, where its arcs end at
     its cut's edges, the end's plane, the cut's, a ring and the cut's lines' surfaces all pass, and
     a cell there is thinner than the mesh's sag can settle — the vertex's path, which the cone above
     says enters nothing, laid where five candidates meet.
   - **A leaning circle under a screw** is refused by name (`Curved`, at the circle): its sweep is a
     screwed circle's sheet, its disc screwed (the pipe's candidate, for any lean, where `v · n` keeps
     one sign over the disc: `|v(c) · n| > r |ω| |n × axis|`, `v · n` being affine). Tried on a ball
     with a flat, two meetings were traced and did not close, and want closed forms first: the ball
     at an end pose holds the circle and meets the sheet in it and a curve read over the roll — at
     each turn `s`, `A(s) cos w + B(s) sin w = C(s)` round the circle; and the ball's pipe meets the
     sheet in helices, the points of one circle the screw carries onto the other, isolated in
     `(θ₁, θ₂, t)` as two screwed lines' are (`screwed_lines_meeting`).
   - Left after it: a free-form edge; and under K2, where each instant's velocity moves on the tool,
     the fan's domain `(w, t)` bounded by the two signs' zero sets, L3's chart read at the edge.
6. **Fold and branch loci** in the domain, then X2 and X5. *Designed (#780 L1).* A patch's contact
   is the zero set of `g = n · v` over its domain: the face's own `(θ, h)` (a ring's angle and its
   height) under a constant twist, which fixes the contact on the tool, or `(θ, h, t)` at each
   instant under K2. Both loci are zero sets of what that chart already computes, so they are
   found in the domain, by the same root machinery, and never detected afterwards in space:
   - **The branch locus**, `g = 0 = ∂g/∂θ` (on a ring `A² + B² = C²`, its two roots one) is a fold
     of the *chart* over its base, not of the sheet: the `+` and `−` roots are one curve through it,
     which the height chart reads with a square root's infinite slope. So the patch is read there
     in the chart transverse to it — `θ` the parameter, the height solved from `g = 0` — and is not
     cut. Wherever `g` is affine in the height (a cylinder's or a cone's rings under any twist,
     `g = α(θ) + h β(θ)` with `α, β` sinusoids in `θ`; a torus's latitudes under a slide, where
     `tan v` is the ratio) that chart is a closed form, `h(θ) = −α/β`, smooth through the locus and
     singular only where `β = 0` (a ruling in contact all along, AF). Written rationally,
     `ρ̂ = ((1 − s²) x + 2s y)/(1 + s²)` with `s = tan(θ/2)` about the branch's middle, its jets stay
     in `turned`'s `Num` (floats and outward intervals), so a meridian read across the locus keeps
     C3's exact reading. Where no such chart closes, the two roots' patches are cut at the locus
     and share it as their rim.
   - **The fold locus**, where `Σ` is singular: the kernel of `dΣ` (the tool's body velocity `w`
     against the roll) tangent to the contact, `g = 0 = ∂g/∂t − ∇g · w`. Under a constant twist
     `∂g/∂t = 0`, so it is where the contact curve runs along the motion's flow — a helix's tangent
     under a screw (a ball reaching its centre's helix's radius of curvature, `(ρ² + κ²)/ρ`), the
     circle about the axis under a turn (a cusp of the meridian), the slide's direction under a
     slide (a cusp of the apparent contour). Beyond it the sheet doubles back. The patch is cut
     there, each side a regular immersion, both entered, and membership drops the side that is
     buried; a cusp-thin cell between them is `Unsettled`, never kept by guess.
   - **A sheet meeting itself far along it** (X5's other half) is read in the motion's orbit
     space, where the patch is its source curve's orbits. Under a screw, in helical coordinates
     `(ρ, ψ, ζ = z − κψ)`, the screw is a shift in `ψ` and an orbit is `(ρ, ζ mod 2πκ)`: the sheet
     a curve `c(θ)` screws into meets itself exactly where `c` holds two points at one `ρ` whose
     `ζ` differ by a whole number of turns' rise, `t = ψ₁ − ψ₂ − 2πm` of the roll apart — each such
     pair a helix, isolated as the roots of `c(θ₁) = S(t) c(θ₂)` with a bound (#777 M1). So the
     patch's roll is cut into stretches each shorter than the least such return (`2π − 2w`, `w`
     the curve's reach round the axis), each embedded, and the stretches are met pairwise like any
     two candidates: along those helices, and round the curve where two abut. Where the stretches
     are joined is the build's choice, not the sweep's, and is placed so that no two joints, nor a
     joint and an end, stand a return's turn `t` apart (their curves would cross one return's
     helix at one point, a cell too thin to settle); where no spacing does, refused. Under a turn every candidate is a whole surface
     of revolution, so a return is its meridian crossing itself; under a slide nothing returns.
   - **X2, several contacts per tool point.** A constant twist keeps each tool point in contact for
     the whole roll or never, so a point has several contacts only in an instant's chart (K2),
     where the branch locus is a curve in `(t, h)` (`swept::instant::branch_locus`) and the two
     roots' patches over `(t, h, ±)` are read across it, or cut at it, as above: L3's patches. Under
     a constant twist what X2 leaves is a ring's two roots meeting on the face. C3's cones do not
     reach it: in a scan over leans, half-angles and offsets, every cone that level planes can
     bound (`β + |α| < 90°`, every generator climbing) kept its branch locus off the face, its
     contact turning back only where a generator lies near level. The case is a torus slid at a lean
     to its axis (refused `Curved` today): its latitudes' roots meet at its silhouette's top and
     bottom, and in the angle chart its contact is two smooth closed curves, `tan v(u) = −(D_x cos u +
     D_y sin u)/D_z` and that turned by `π`, each swept along the slide into a cylinder over its
     apparent contour — a curve kind of its own, exact as C3's meridian is, whose own fold (the
     inner contour's cusps where the contact runs along the slide) is the fold locus above. Moved to
     its own issue with this design.

   *X2 under a constant twist built* (#805, `tests/swept_slid.rs`): a whole ring slid at a lean, its
   two contacts read in the angle chart (`slid::Slid`, rational in `tan(u/2)` but for one root, its
   jets in floats and outward intervals), each swept into the cylinder over its apparent contour,
   which the ring at each end pose touches along its contact in closed form. The fold comes first at
   `tan² β = (R − r)/r`, where the slide runs along an asymptotic direction of the tube's saddle at
   `u = ±π/2`; short of it the contour is regular and every line along the slide through a contact
   point meets the ring there alone — so no line along the slide meets the ring twice, and the sweep
   is `V + |D| A` exactly, `A` the annulus between the contours. A line meeting the ring twice (and a
   cylinder crossing the ring beside its touch) comes only past the fold, which is refused by name
   (`Unbuilt::Folded`) until the patch is cut there. A ring slid 6 and 2 at 35° and aslant at 20° are
   each `V + |D| A` to 1e-9; pushed into a block, it removes the centre's depth times `A` and half the
   ring (`solid_sweeps/leaning_ring.sv`, its STL agreeing with the field).

   *Touches given by construction, the first gate a bull-nose slid at a lean* (#810, #780 R5): a
   torus face with rims (a bull-nose's round) has its contact meet the neighbour's at a smooth seam,
   where the sheet over its contour and the plane the neighbour's ruling sweeps touch along the path
   of the point where the contact crosses the seam. The sweep knows that line before anything is
   met, and hands it to the arrangement as a given edge (`boolean::Given`), as it now hands every
   touch it knows by construction — the torus at each end pose against its contact's sheet (#805),
   a leaning round face against its contact turned (C3), a face holding a sharp circle against its
   ring (L2) — each of which `ssi` had recognised pair by pair, and can now be bypassed while the
   gates still hold. Three things it took besides. The contour's sheet is cut at the seams exactly
   and run along the slide past the domain, its rims within the domain lying on the given lines: run
   a little past the seams, as a patch's are, it left a strip inside the sweep whose meetings with
   the side at the end poses stopped partway across it, a slit the judge caught at most leans. A
   candidate's seam is turned away from the touches (`seam_away`): a slide across the tool's own
   seam meridian laid each touch along a candidate's seam. And membership reads the roll best
   first: a point on the tool's top at the middle pose had spent its reads halving the stretch
   before it. A bull-nose slid at three leans and a drum rounded at both corners are `V + |D| A` to
   1e-9, and pushed at a lean into a block until wholly in it, `solid_sweeps/leaning_bullnose.sv`,
   the block less `V − ∫ z n_z dS` over the mill's faces turned up, to 1e-9; a round meeting a
   neighbour in a sharp rim at a lean is refused by name until its fan is entered.

   *A ring under any rigid motion, built* (`swept::rings`,
   `tests/swept_rings.rs`): the tumbling torus (`swept_tumble.sv`) and every tool of revolution whose
   meridian is a loop clear of its axis. Such a tool has no pole, so its contact is read meridian by
   meridian in closed form — on an arc `n · v = A cos ψ + B sin ψ`, two roots half a turn apart,
   since the arc's radius is its normal; on a segment one, affine; a convex corner's fan between
   them — each instant's contact a set of closed curves, graphs over the angle about the axis. The
   fold is read on the tool before anything is fitted, as `s''`, how the tool's distance from the
   point it touches bends over the roll there: `κ_m (v · t_m)² + κ_p (v · e_θ)² + n · (ω × v) − n · v̇`.
   Where it is negative the tool holds the point at the instants beside, and the sheet there is
   buried; where it changes sign, the fold. Under the tumble the contact is two curves at every
   instant, the outer touching all round, the inner folding four times, and each contact is held or
   not beside its instant as its `s''` says. Three lessons of the build. A sheet cut open at its folds
   leaves rims inside the sweep that cross the torus where it starts in curves ending inside its
   face, which no arrangement parts space by: each curve is entered whole, a closed tube, its buried
   pieces too, and across each fold a bridge dipped into the tool (inside the sweep, and regular
   where the cuspidal sheet is not). And a search for the tube's meeting with the tool at an end
   pose starts beside the touch along the rim: it is read instead in the tube's own domain, the
   zero set of the tool's signed distance with the touch divided out. The third lesson closed it. A
   tube entered from its rim lay over its own rim beside each fold corner at the end poses — near a
   fold the contact slides along itself, and a fold piece's sheet just past its rim stood over the
   rim, inside the torus — a meeting of the tube with itself that no arrangement seeks: the
   torus's face was left a slit along the touching rim beside it, and the sweep's inside one cell
   with its outside. So a tube that folds is entered only where it stands outside the tool at both
   end poses (`rings::trimmed`): each face cut along those meetings, the pieces inside dropped (inside
   the sweep, they part nothing of it), every free edge of the tube on the tool where it starts or
   stops and given as its whole meeting. The tumbling torus is built closed and judged in some
   fifteen seconds (the slow tier): 25.27504, where its volume made apart is 25.2750727653 (#880
   E2, `tests/swept_tumble_apart.rs`; cadrs's agrees to 1.1e-8). *The build is 3.3e-5 low
   (1.3e-6) and its area 5.6e-4 high (9.5e-6), and that is no fit's error* — 0.6.3's solid is the
   same at every fit tolerance from 1e-5 to 1e-7 and at either membership bar. Its fitted tubes
   stand on the sweep's boundary to 8e-9, but four small faces of the inner tube's dipped
   stretches (`dip`), 0.0105 of area together, two beside each end pose where a stretch meets the
   torus there, stand wholly inside the sweep, up to 3.6e-3 deep, with patches of the torus at the
   end poses beside them inside it too: they bound thin pockets of the sweep the solid leaves out.
   The dipped stretch is inside the sweep, as `dip` says, but near an end pose nothing kept
   buries it: which meeting the arrangement lacks there, or which cell it misreads, is not yet
   read. R1's to mend; `the_tumbling_torus_is_built_closed` holds the build within 5e-5 until then. (The Monte
   Carlo of the definition this paragraph cited, 25.260 ± 0.005, was three of its deviations low;
   the 100 000 points read one by one met no pocket, each too thin.) The road is not yet a class the dispatch admits;
   `closed::ring` builds it alone, and `closed::ring_cut_from` cut from a blank (a ring pushed into a
   block at a lean, a torus turned across one, each within the Monte Carlo of its definition); a
   ring of several faces whose tube crosses the tool at an end pose across two of them is refused
   until that crossing is cut where it passes between them.

   *X5 built* (`tests/swept_loci.rs`): a ball screwed through more than a turn at a lead short of
   its diameter, whose pipe, entered whole, crossed itself and was refused by the judge. Its pipe is
   entered in stretches of at most half a turn (`closed::pipe_stretches`); two stretches of one pipe
   meet round the circle they share and along the returns' helices (`ssi`'s `pipes_meeting`), and a
   plane holding a circle of the pipe — a joint's — meets it in that circle and its level-by-level
   cut (`pipe_plane`, as a level circle's sheet meets any plane). A turn and a half at 0.2 a radian
   and two and a half turns at 0.12 (a point of the circle on the helices of two others) are each
   their volume worked apart in helical coordinates to 1e-11 — at each `(ρ, z)` the angles swept are
   one interval, the image of the ball's convex slice by `σ = ψ − z/κ` over a strip of heights, and
   round the circle they cover `min(2π, σ_max − σ_min)` — and agree with a Monte Carlo of the
   definition and with membership. One reading had to change: a point past a patch's rim, on its
   surface's extension, had been read as in the face where the inverse clamps it onto the rim, so a
   ray crossing beside the circle where two stretches abut was counted on both; it is in none of the
   face (`query::Located::face_place`, the C4 rule read where the face is read).
7. **The generating and canal classes' sheets as patches**, then **K2's** chart over
   `(t, h, ±)`. *K2's chart built* (`swept::instant`): each ring's roots against the instant's
   twist, carried by the pose, told apart by the contact's rate round the ring, and the branch
   locus `A² + B² = C²` found in `(t, h)`; a ball's contacts under a tumble bound its sweep by
   membership (`tests/swept_instant.rs`). Its patches in the arrangement follow the patch type.
   *K2's patches built* (#780 L3, `swept::patches`): under any rigid motion no constant twist takes
   (a frame along a path, a motion seen from another), a convex tool of revolution's contact is read
   **per tool, at each instant, in a pencil of half-planes** about the line through its centre along
   its velocity. Each half-plane cuts the convex tool in an arc from back to front along which the
   arc's normal turns monotonely and the surface's leans out of the half-plane by a positive factor,
   so `n · v` changes sign once there: its one root is the contact, landing on a sharp edge where
   the edge's fan holds it (step 5's rule, read under any motion). The instant's contact is then one
   closed curve `x(t, φ)` over the pencil's angle, smooth through the branch locus, which this chart
   is transverse to — the ring chart's `(t, h, ±)` turns back there, the pencil's does not (step
   6's rule: read the patch in the chart transverse to the locus). It runs in pieces, each on one
   face or one edge, joined where it crosses a seam or reaches an edge; a face's piece is read at
   even fractions of its stretch of `φ`, an edge's along the edge's own circle (the pencil's angle
   runs as a square root along an edge where the edge runs along the velocity, where a face's
   contact reaches it). The sheet `M(t) · x` is a quintic net over (the fraction round the loop,
   the roll), fitted within a tenth of the arrangement's tolerance, its rims the contact at the end
   rolls. With the tool at each end pose, whole, it is all the arrangement takes, and every meeting
   of the two is given (R5): each piece's iso-curve at that roll on the faces holding it, nothing
   on the others — the sheet stands inside neither end pose, every point of it on the boundary of
   the sweep over a roll holding that pose. Carried past its rims (as M5's patches are), its flap's
   section on a blank's face the tool stood across was tangent to the tool's section there and the
   cell between misread, so the sheet stops at its rims. A sheet that folds or comes back onto
   itself is refused by name (`Folded`, `Returns`); one whose pieces change over the roll
   (`Curved`), as is a tool no convex solid of revolution. A ball along `path.sv`'s spline frame is
   the canal class's fitted tube to 1.2e-9, a bull-nose slid at a lean read instant by instant the
   constant twist's build to 1e-8, and a bull-nose screwed (the constant twist's road refuses a
   torus screwed) and one along a spline frame are each the volume their faces advance through,
   `V + ∫ ∫ (n · v)⁺ dA dt`, to 1e-6 (`tests/swept_patches.rs`); drawn, `solid_sweeps/spline_mill.sv`.
8. **The blank-scoped path** as the optimisation of X6 (#699 D). *Decided*: the closed solid cut
   from a blank is itself blank-scoped — `cut_from` takes the blank as the arrangement's domain and
   keeps its cells that are no member, no closed solid built and subtracted — so X6 has two roads
   to a body and one definition. Where a class admits a sweep, its blank-scoped sheet (the walk,
   the fitted or closed-form sheet, the sector split) stays the road: measured on a sweep both
   build, `punch.sv`'s drum, it is the faster by about an order (0.09 s against 0.8 s) and the two
   agree to 1e-9 (`closed_class::the_fast_path_and_the_one_arrangement_agree_where_both_build`, two
   builds made apart cross-checking each other). Where every class refuses, the one arrangement is
   the road (`solid::closed_class`, asked after the class the sweep was first asked). `closed`, over
   a box, is the kernel's `sweep(S, M)` proper, for a sweep with no blank.

**Where it stands (F2c5).** Built on the one arrangement: polyhedral tools under a slide, a turn,
or a screw with nothing leaning to its axis; under a slide, tools of planes, cylinders, cones, balls
and tori (a torus's axis along the slide or square to it, or a whole ring at any lean short of its
fold, #805, or a band of one between smooth seams — a bull-nose's round — its seams' touches given,
#810); under a turn, round faces about axes
along the turn's (cylinders, cones, tori) and balls clear of its axis; under a screw, cylinders and cones about axes along it and
level circles (a drum, helical milling; a frustum, a chamfer mill, its contact rulings screwed into
patches, #780 C2); paths of slides and turns (K1); a body mixing the
closed class with another; the blank-scoped decision. What is left meets one of three gaps, each
recorded above with what was tried:
- **a candidate bounded inside the domain** — a leaning edge under a screw (whose whole sheet
  crosses itself) — waits on the arrangement holding a face with a slit, or a sheet whose rim
  stands inside a cell (F5's A1);
- **a meeting with no closed form near a touch** — K2's patches (each instant's contact swept,
  fitted), *since #780 L3 built for a convex tool of revolution, its touches with the tool at the
  end poses given whole* — is refused as nearly touching: each needs its closed form (as the parallel cones', the
  ball's in a torus, the meridian's cylinder were), or the tracer holding a near-tangent meeting
  (F4, #699 E). *A fitted sheet lying in a candidate's plane is built* (#780 C4): it is the plane
  (`ssi::same`, every pole within the tolerance of it, the sheet in its poles' hull), a coincidence
  in A2's classification and not a touch to trace, and the plane is the sheet over its patch alone
  (`query::Located::face_holds`) — met where `plunge.sv`'s bar, cut beside `punch.sv`'s drum,
  stands its end face at its start pose in the plane of the slot's end wall, which had been two
  faces coinciding round the slot's corners, and then, the plane's piece beside the slot read on
  the wall where the sheet's foot clamps to its edge, one cell where the plane splits two. *A round face about an axis leaning to a
  turn's is built* (#780 C3): its contact, on each ring `A cos u + B sin u = C`, is two curves along
  the face, each turned into the surface of revolution of its meridian read exactly — a closed form
  in the face's slant with its derivatives, read in floats and in outward-rounded intervals
  (`turned::Turned`), not fitted — which a line meets certified (`query::line_turned`), coaxial
  surfaces meet in circles where the meridians cross, and the face at each end pose touches along
  its contact turned there in closed form (`ssi`'s `own_contact`). A whole turn takes no face at its
  end poses (its sweep is the tool's solid of revolution, which they only touch). A leaning drum, a
  leaning frustum and a rounded key are built turned whole and 100°, and a ball on a leaning stem
  turned whole, each to a volume worked apart to 1e-9 (`tests/swept_turned.rs`); the ball on its
  stem turned 100° is refused by name at its concave seam (`Unsettled`). *A circle screwed is built* (a round tool's
  rim under a screw: helical milling): its sheet's
  meetings with a level plane, a cylinder about the axis, a round tool's own side at an end pose and
  a plane through the axis are closed forms (`ssi::screwed_circle_meeting`), a line meets it exactly
  (`query::line_screwed_circle`) and its inverse starts level with the point. A drum screwed beside
  the axis (helical milling: its side's generators screwed into coaxial cylinders, each rim's disc
  screwed into a solid) is its helical volume `π + Θ (6 + k π)` to 1e-15, a drum screwed half a
  millimetre a turn through 280° `π + Θ (20 + k π)` to 1e-15, and the same drum through a ring
  sector the sector less `π/3 (20 + k π)` to 1e-9, each held to membership. What had left a cell
  holding both sides of the sweep's boundary was not a missing separation but four readings: a
  branch's end on its coincidence circle laid on neither (`boolean`'s `ends_on_one_another`), that
  end read one-sidedly so that its first chord bent and was projected onto the circle, two stretches
  of one circle's sheet two leads apart read as one surface and each other's edges laid on the
  other, and a sampled search for a ray's crossings missing one of the two a ramp a hundredth of a
  millimetre thick makes — a solid's point location wrong at seven of 400 points about it, and a
  cell kept or dropped so. Where the rim's farthest and nearest points run along the screw the ramp
  folds against the wall its side sweeps over a radius of about a tenth of a micrometre, and a chord
  of the wall's helix a mesh takes at the bar sags across the fold, so the triangles beside it
  faced against the sheet: the mesher now halves the chords such triangles stand on and meshes
  again, and the drawn drum's STL agrees with the field (`solid_sweeps/helical_groove.sv`). A cone
  about an axis along the screw's touches where `ω cos β (n·(k × d)) = τ sin β` on each ring — the
  same angle on every ring, so along two of its rulings, turned off the plane through both axes by
  `asin(τ tan β / (ω |d|))`: lines leaning to the screw's axis, whose sheets are the leaning edge's
  (the first gap). *A ball screwed is built*: it touches round its great circle square to its
  centre's velocity, which leans to the axis wherever the screw climbs; that circle screwed is the
  pipe about its centre's helix (`Cutter::Pipe`), touching the ball at either end pose round that
  circle (`ssi`'s `pipe_end`), and a ball screwed 1.5 rad is Weyl's tube and a ball to 1e-15 — once
  an edge's flux settles (`props`'s `settled`: the great circle passing beside the ball's pole had
  left its piece 0.2% off). A ball-end mill screwed (a ball under a cylinder) is the step after: its
  pipe touches the cylinders the side's lines screw into along the helices of the ball's equator,
  in closed form (`screwed_circle_round`), but each such helix starts on the rim of the cylinder's
  candidate — a crossing at 0.06 rad a sampled edge–sheet search placed 0.01 off — and a candidate
  run past the rim leaves a cusp-thin cell between pipe and cylinder whose reading went wrong
  through a ring sector (14.4 removed against 12.0, though a box built it to 1e-14). It waits on
  an exact meeting of an edge with the pipe, and on cells along a touch read right. *Since #777*
  cells along a touch are read right (M4) and touches found generally (M3), and the mill is
  admitted; screwed through the sector it is refused by its budget, naming the inverse stage at
  the pipe's sheet (whose inverse map is a general search on every ray and outline), in about two
  minutes where it had stalled for hours — the case #721 resumes with. *Built since #780 C1*: the
  pipe is inverted off its spine (each foot of a point on it the circle about a foot on the centre's
  helix, isolated with a bound, or an end circle), but the profile showed the inverse map's calls,
  not their cost, running away — a short arc beside a traced meeting polished from each of 26,000
  pairs of near chords to the same place, by global inverses through the pipe (`meeting::crossings`
  now polishes from the pairs no neighbour stands closer than). Two failures met on the way were
  reduced (`swept::dump`) and fixed where they lay: a traced curve's inverse stepping off along the
  whole curve from points crowded where its trace set out, and the circle's tangency to the side's
  cylinder read as four helices. Through the sector it removes its helical-coordinate volume to
  1e-11, in ten seconds; the mill screwed alone, its own to rounding
  (`swept_closed::the_ball_end_mill_screwed_closes_into_its_helical_sweep`). *Drawn since #791*
  (`solid_sweeps/ball_end_mill.sv`, 3 mm a turn, and at 2 and 4): where the pipe's meeting with the
  plane of the side's screwed foot, the foot's rim and the touch helix all touch, at the pipe's
  start, the foot's pieces did not close. The meeting set out a tolerance past the pipe's start and
  is now ended at the vertex, but that was not the defect: tangent there to the rim, it was read
  leaving the node 80 radians off, its way extrapolated from two probes of its parameter, which
  counts its points and so stood within a millionth of the node where they crowd; each tied way is
  now read at a length along its half. At 4 mm a turn a second rounding-sensitive point showed: the
  seam of the cylinder the side screws into touches the pipe on that helix, a double root the search
  found at one lead and missed at another, and the helix, crossing the seam uncut, split nothing, so
  the cusp-thin corner below it merged with the cell above and was cut away (refused by the judge,
  not returned wrong); a line along the screw's axis now meets the sheet along the same helices the
  cylinder's meeting reads, in closed form. *Built, a torus about an axis along a turn's* (a bull-nose's corner turned): it touches
  round its two meridians in the plane through both axes, turned into tori about the turn's, and
  round its top and bottom rings, kept in their planes. The torus at an end pose and a meridian's
  torus touch round that meridian and cross beside it — in each level both are two circles about
  their own axes, two pairs touching and one crossing where `A cos θ + B sin θ = C(h)`, a closed
  form as the rim's branches are. All three meet at one point: the meridian's top is where the
  ring's top circle touches the plane it turns in and the meridian's torus touches that plane too.
  The branches reach that point tangent to the ring's top circle, parting from it as the square of
  the way along, and a chord between their last two points sagged across that circle — read there,
  the branch left on the wrong side, the walk round the ring's face joined two pieces through a
  there-and-back along the circle, and the meridian's torus outside the ring came out empty. Read on
  a parabola through three points (`Traced::between`), the branch leaves on its own side: a ring
  turned 1.5 rad is its sweep by levels to 1e-11, a bull-nose mill to 1e-15, and the bull-nose turned
  partway into the ring sector (`solid_sweeps/stopped_bullnose.sv`) to 1e-9 through the drawing, its
  STL agreeing with the field;
- **fold and branch loci in a patch's domain (X2)**: designed (step 6, #780 L1). A constant twist's
  contacts are fixed on the tool, so several contacts per tool point are K2's (L3); what X2 leaves
  under a constant twist is a ring's two roots meeting on a face, a torus slid at a lean: *built for a
  whole ring* (#805), read across the branch locus in the angle chart, its inner contour's fold
  refused by name; a bull-nose's round, its seams' touches given by construction (#810). *A sheet meeting itself far along it (X5) is built*: a ball screwed past a turn at
  a lead short of its diameter, its pipe in stretches met along the helices where it returns
  (`tests/swept_loci.rs`). A pipe that folds (a ball reaching its helix's radius of curvature) is
  refused by name.

Every step meets the epic's bar: rows in `docs/contract/generating.md` naming the gate test, field
agreement and an independent volume at the default tolerance, negative controls kept, an example
per motion family.
