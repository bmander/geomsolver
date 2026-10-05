# Planes from rays, the page a plane: plan

**Status (2026-10-04): proposed**, for issue #76 items 1 and 2. Not started.

Today a point is either *on the page* or drawn *in a view*. A point on the page has no place in
space, and a view is two things at once: a plane in space (its basis) and a picture placed on
the paper (its datum's `origin`, `toward` and rotor). That second job is where most of the
complexity lives. It is the reason for the role rule (`program/reading.rs`), the page-placement
gauge (`Sketch::page_held`), the layout lines in the corpus (`fix(x == 0, y == 200) og`,
`std.ThreeViews(O, right: 150, up: 90)`) and the "on the page" refusals. Since 0.20, paper
layout is the `.svd`'s job.

This plan makes three changes:

* **every point is in a plane,** and the page is the plane `std.front`;
* **a plane is a plane in space and nothing else,** written as two rays;
* **a ray is a new kind,** a directed line in space, placed by relations like any other entity;
* **`on` is retired:** `coincident` is the one word for incidence.

Nothing has been deployed, so there is no compatibility: the old spellings stop parsing, the
corpus is rewritten in the same change, and no diagnostic names an old spelling. The spec goes
to Draft 0.42.

## Decisions

These were made in conversation on 2026-10-04.

1. **The page is `std.front`.** Every document has it, whether or not it says `use std`. A point
   with no `in` belongs to it, so it has a place in space: x right, z up, at the world origin.
   It is a real plane in the model (B, "the implicit plane"), not a `None` that readers
   special-case. Writing the name `std.front` still takes `use std` (nothing is imported bare),
   but drawing on the page never does.
2. **Views carry no place on paper.** A point's `(x, y)` are its own plane's coordinates. The
   `.svd` places each view on the sheet.
3. **A plane's attitude is two rays:** `plane(u: r1, v: r2)`.
   * right is `u`;
   * out of the plane is `u × v`;
   * up is `out × u`.

   So `v` need not be square to `u`: it only says which plane, and which side is up. The rays
   give **only the attitude**. Where they lie does not matter to the plane, so two
   planes may share their rays.
4. **Every plane has members `P.u`, `P.v` and `P.origin`.** `P.u` and `P.v` are its axes, as
   rays from its origin. `P.origin` is a point drawn at `(0, 0)` in `P`, which a component may
   name and a relation may place. **The origin floats unless something places it.** The
   relations on a plane:
   * `P coincident p`: the plane passes through point `p` (incidence);
   * `P.origin coincident p`: the plane's origin *is* `p` (point to point);
   * `P distance(d) Q`: parallel planes, `d` apart.

   A parallel stack, which is what `offset:` said:

   ```
   plane_a := plane(u: r, v: s)
   plane_b := plane(u: r, v: s)
   plane_a distance(d) plane_b
   n := ray
   n perpendicular plane_a
   n coincident plane_a.origin
   n coincident plane_b.origin
   plane_a.origin coincident p
   ```

   The first three lines leave both planes floating, and `plane_b`'s origin would still slide
   within `plane_b`. The ray `n`, square to both and through both origins, stops the slide. The
   last line, with `p` fixed, fixes both planes. The language adds no sugar for this: a library
   component (`std.Stacked(P, by: d)`) says it in one line. A special case in the solver may be
   added later if one is ever warranted.
5. **A ray is a directed line**: a line in space with a sense, and no start (four freedoms: two
   of direction, two of position). It is constrained by the relations that already exist.

   ```
   t := ray
   t perpendicular std.y
   t angle(tilt) std.x
   aux := plane(u: t, v: std.y)
   aux.origin coincident std.origin
   ```

   There is no vector arithmetic. A vector expression (`rotate(u, about: v, by: θ)`) would be a
   second, algebraic way to say where something is, beside relations, and an entry to imperative
   drawing. A direction is a thing you constrain, not a value you compute.
6. **Two planes lying on one another are permitted, and are a lint** (a warning, not an error).
7. **`on` is retired, and `coincident` is incidence everywhere.** `p coincident l`, `p coincident
   c`, `p coincident P`, `l coincident P`, `t coincident p`, as well as two points. What it
   means is the kinds of its operands, as `on` was. One word fewer, and no idea spelled twice.
8. **An unsigned angle of 0° or 180° is refused** (E040, by value, as a negative magnitude is).
   Between rays, or across planes, the angle is unsigned, and it has no gradient at either end:
   the row is rank-deficient there. The regular spellings:
   * `t parallel s`, with the seed picking the sense, as the seed picks a side for a magnitude;
   * a `fix` on the direction, where the library holds a ray outright (`std.up`'s reversed x).

   Within one plane the directed angle is regular at 0° and 180°, and stays: the spiral bevel's
   ten `angle(0deg)`/`angle(180deg)` between drawn lines are unchanged.
9. **Retired:** `fold:` (an angle, an unknown and `along`), `offset:`, `from:`, `through:`,
   `attitude: free`, `u:`/`v:`/`o:` vectors, a plane's `origin:`/`toward:` arguments (the
   origin is now a member), the role rule, the page-placement gauge, `std.ThreeViews`, and the
   "has no place in space" refusals (E062's page case, the extruded envelope's, `through:`'s,
   `project`'s and the spatial words').
10. **Points in space are a later issue.** A plane's origin is a position in space, but the only
    point anyone draws or names is `P.origin`, which is drawn in `P`.

## The model

### Rays

`EntKind::Ray` owns six parameters, a point `p` on it and a direction `d`, with two intrinsic
rows:
* `|d|² − 1` (degree 0, like `frame_unit`);
* `p·d`, a gauge that takes `p` to the foot of the perpendicular from the world origin, so it
  cannot slide along the ray.

Its net freedom is four. Its parameters' `Param::scale` is the drawing's extent, as a plane's
rotor is scaled by its chord today.

What it can be told, all through `constraints::infix_op`'s existing words:

| statement | meaning | kernel |
|---|---|---|
| `t coincident p` | `p` is on the ray (a view point, by its lift) | `PointOnLine3` |
| `t parallel s`, `t perpendicular s`, `t angle(θ) s` | between two rays, or a ray and a line | `Parallel3`, `Perpendicular3`, `Angle3` |
| `t coincident P` | the ray lies in plane `P` | `LineOnPlane` |
| `t perpendicular P` | the ray is square to `P` | `Parallel3` against `P`'s normal |

The spatial kernels read two points in space. A ray hands them `p` and `p + d`, so most rows are
the existing kernels over new columns. Each one is checked in `tests/jacobians.rs`.

**A line drawn in a view is a ray** wherever a ray is expected, from `p1` toward `p2`, by its
points' lifts. So `plane(u: hinge, …)` takes a drawn line directly, which is what
`fold: along hinge` meant.

**A plane's axes are rays.** `P.u` and `P.v` pass through `P.origin`. A ray square to `P` is
stated with `t perpendicular P`, so there is no `P.n` member. `std.x`, `std.y` and `std.z` are
the world's axes, fixed rays through the world origin: `std.x` is `std.front.u`, `std.z` is
`std.front.v`, and `std.y` points away from the page's viewer.

**A seed** is the one `hint(…)` clause: `t := ray hint(x: 0, y: 0, z: 1)` seeds the direction.
Its position is seeded through the origin, and moved by its relations.

**A ray's position nothing reads is no freedom.** A ray used only as a direction, like most of a
plane's rays, has a position that no equation mentions. Those two freedoms are retired to
`fixed` at elaboration, as an orphaned `Param` and an unread free variable are already, so the
diagnosis does not call the drawing under-constrained because of them. A position that a row
does read, for example where the ray is `coincident` a point or lies in a plane, counts as an
ordinary freedom.

### Planes

`PlaneE` becomes `{ u: Ray, v: Ray, o: [Param; 3], origin: Point, basis, fixed }`:
* `o` is where the plane's origin stands in space, three unknowns;
* `origin` is the member point `P.origin`, drawn in `P` with its `(x, y)` held at `(0, 0)`, so
  its lift is `o`.

`FrameE`, the rotor `(c, s)`, the `frame_unit`/`frame_align` intrinsics, `Att` and its
quaternion, and the `Hinge*` kernels all go.

* **Where a view point stands:** a point `(x, y)` in plane `P` is `o + x·û + y·v̂`, with
  `û = u/|u|` and `v̂ = normalize((u × v) × u)` over the rays' directions.
* **`Lift`** (a solved plane) reads the point's `(x, y)`, `o`, and the two rays' directions.
* **`LiftFixed`** reads the point and holds `(o, û, v̂)` as constants. It drops the datum
  columns: 5 columns instead of 9.
* **`world_in`** becomes `basis(P).lift(x, y)` and **`on_view_sheet`** becomes
  `basis(P).view_coords(w)`. Neither has a `None` arm.

The relations on a plane:

| statement | meaning | rows |
|---|---|---|
| `P coincident p` | the plane passes through `p` | `PointOnPlane`: 1 |
| `P.origin coincident p` | `P`'s origin is `p` | `Coincident3` of `o` and `p`'s lift: 3 |
| `P distance(d) Q` | parallel planes `d` apart, `Q` on `P`'s out side | `(o_Q − o_P)·n_P − d`: 1, plus `P parallel Q` if their rays are not shared |
| `P parallel Q`, `P perpendicular Q`, `P angle(θ) Q` | between their normals | the ray kernels over `u × v` |

A plane's freedoms are 3 for `o` plus whatever its rays leave free. A floating plane is free to
slide in itself (2) and along its normal (1). Placing it by `P.origin coincident p` takes all
3. Placing it by `P coincident p` or `P distance(d) Q` takes only the normal one, and the
document places the rest as it places anything else.

**A plane is fixed when its rays and origin are**, and then its basis is settled at elaboration:
no unknown, and no lift row beyond `LiftFixed`, as today's fixed views. A ray or an origin is
fixed when it is tied only to fixed things: standard rays, fixed points and constants.
`views::solve_planes` already solves the planes before seeds settle a second time. It becomes
the pass that solves the planes' subsystem first, meaning rays, origins and the relations among
them, fixed points and numbers, and freezes every plane that subsystem determines (zero DOF,
full rank, by `System::conditioned`). Anything else stays solved in the main system. This
matters for speed: without it, every tilted view in the corpus would pay a solve per frame.

### The page

`std.front` is plane 0 of every `Sketch`, built with the sketch. Its rays are fixed at the
standard axes, so its basis is `Basis::page()`. `PointE.plane` becomes `u32`, and every
`plane_of` caller loses its `None` arm (about 35 sites; the agent survey lists them). JSON writes
`"plane"` only when it is not 0, so a 2D document's export does not grow.

`use std` brings in the other standard planes (`std.side`, `std.top`, `std.up`) and the rays
`std.x`, `std.y` and `std.z`, as ordinary library statements. `std.front` is not in
`StandardDatums`, but the flattener binds the name to plane 0 when the document says `use std`.

### Reading a relation across planes

The rule stays: one plane, the 2D relation; different planes, the relation in space. The role
rule is deleted, because there are no layout points left to read by role.

**Two planes lying on one another** (`std.front` and `std.up`, or a part's plane turned in the
page) are permitted. The document gets a lint (a new W-code, at the second plane's declaration,
judged on the solved pose) saying they are one plane in space and suggesting the geometry be
drawn in one.

The relations across them still need care. The 3D twin's out-of-plane row is identically zero
there, and the diagnosis would read the drawing as redundant. Two ways to handle it:
* where both planes are fixed, read the relation as the 2D one through the constant in-plane map
  between them;
* otherwise, let the screen that already sets aside double roots (`witness::screen`) recognise
  the zero row.

The first is cheap and covers the corpus (the vtwin parts drawn `in std.up`). The second is
needed only for floating planes that happen to lie on one another, and can come later.

**`p distance(d, along: u) P`** is measured in space: `(X_p − o_P)·û_P`. Within `P` it is
`p.x − d`, which is the same number as today. Across planes it is what an engineer means by a
coordinate along that axis. The 2D `CoordinateU`/`CoordinateV` kernels lose their datum columns.
The cross-plane case is a new spatial twin.

### `project`

`project` stays and becomes simpler: `d_A·p − d_B·q + d·(o_A − o_B) = 0`, linear, over the two
points and, where a plane floats, its origin's columns. Where both origins are fixed, the last
term is a constant (zero where they coincide), and the row has 4 columns. `plane::fold_line`
reads bases only and is unchanged.

"A view slides freely along its projectors" becomes a statement about origins: a floating
view's origin slides, and with it everything drawn in the view.

### What else loses its sheet pose

* `FacePoly.pose`, `solid::PageFrame`'s pose and `kernels::EXTRUSION_FRAME`'s first four
  constants (13 → 9) become identity, and are removed.
* `overview::placement` and `workspace::Placement` become the basis.
* `overview::corners_in` stops pairing every datum origin as one shared point.
* `renderer::document::view_frame` takes the basis only.

Recorded fixtures whose bytes carry these constants are re-recorded, and their numbers are
checked unchanged.

## The language

```
use std

// the page: nothing to write
a := point hint(x: 0, y: 0)

// a view looking from +x, y right, z up, standing at the world origin
right := plane(u: std.y, v: std.z)
right.origin coincident std.origin

// a view parallel to the top, 50 above it (what `offset:` said)
top := plane(u: std.x, v: std.y)
top.origin coincident std.origin
deck := std.Stacked(top, by: 50mm)   // the stack above, as a library component

// a tilted auxiliary view, its angle a number or a declared unknown
t := ray
t perpendicular std.y
t angle(tilt) std.x
aux := plane(u: t, v: std.y)
aux.origin coincident std.origin

// a view through a drawn hinge, square to its parent
w := ray
w perpendicular hinge
w perpendicular parent
section := plane(u: hinge, v: w)
section.origin coincident hinge.p1
```

`t` and `w` are used only as directions, so nothing reads their positions, and they count no
freedom. `hinge` is a line drawn in `parent`, used as a ray.

`std.sv` gains a few small components, closed over their arguments, so the common cases stay
one line:

* `std.Turned(axis, about: r, by: θ)` declares a ray and its relations.
* `std.Square(to: P, through: l)` declares the plane through a line and square to `P`.
* `std.Stacked(P, by: d)` declares the parallel plane `d` along `P`'s normal, its origin over
  `P`'s.

The names are placeholders. They are ordinary components, and the grammar does not change.

`std.ThreeViews` is deleted. `std.side := plane(u: std.y, v: std.z)` and
`std.top := plane(u: std.x, v: std.y)` are the views it laid out. `std.up` is
`plane(u: std.z, v: w)` with `w` the x axis reversed: the page turned a quarter. The library
states `w` with a `fix` on its direction, which holds it outright. A document saying the same
writes `w parallel std.x` seeded the other way (decision 8).

**Membership** is unchanged: `a := point in right`, `in right { … }`, and an instance joins a
plane whole. A point with no `in` is on `std.front`. A component's `in f { … }` over a plane
formal is unchanged.

## The `.svd`

`sketch m at (X, Y)` draws plane 0. A new clause draws any plane of the model:

```
sketch m.right at (150, 0)
sketch m.top at (0, 90)
```

Each sketch view draws its plane's own coordinates, scaled like a solid view. Callouts are laid
out per plane. A callout across two planes is drawn in neither, as now. `bracket.svd`,
`engine.svd` and `vtwin/assembly.svd` are the three sheets that need it. The spacing that
`ThreeViews` and bracket's eight `fix`es stated moves here.

## The corpus

The [corpus survey](#appendix-the-corpus) found two kinds of rewrite.

**Mechanical**, by a throwaway migration tool built on the *old* elaborator:

* For every point in a view, its new `(x, y)` is `in_view(c, s, o, p)` of its solved sheet
  position. Every literal `hint` and `fix` number on such a point is rewritten by span. A seed
  that is an expression is reported for hand rewriting, not rewritten.
* Every plane declaration is rewritten to `plane(u:, v:)` with standard rays where its basis is
  a standard one, and flagged otherwise.
* Layout statements are deleted: relations and fixes naming only datum points that place a view.

The tool lives in the scratchpad and is not committed.

**By hand:**

* `engine.sv` and its modules: about 190 ordinates measured from `O` and `views.top_origin`,
  and 77 seeds reading sheet coordinates (`o.x`, `_origin.x`). The parts' `o_s`/`o_t` formals
  become their plane's origin.
* `vtwin/assembly.sv`, `components/side_view.sv` and `parts.sv`: 188 page-axis relations in the
  turned right view, `so`, and the seed reads.
* `bracket.sv`: the `aux` view's tilt as a ray, and the 8 placement fixes.
* `sphere_cone_cylinder.sv`, `hypoid_pitch_cones.sv`, `skew_axes.sv`, `gauged_gear.sv`: hints in
  sheet coordinates in moved views, and the solved folds as rays.
* `spiral_bevel/views.sv` (`PitchView`, `FoldedView`) and `twist_drill/wheel.sv`: `fold: along`
  as a plane through a drawn hinge.
* `vtwin/components/throttle.sv`: its `longitudinal` view is turned differently from its fold.
* The vtwin part frames and `std.up` users (`Cylinder(std.up, …)` reads `f.origin`/`f.toward`):
  a component reads `f.origin`, `f.u` and `f.v`, and draws `in f`. A part frame at a solved
  angle becomes a plane whose `u` is the part's drawn reference line.

**The gate** is stronger than the usual DOF-and-values check:

* Before the switch, the old `solventc --json` records each example's **world position of every
  point** (`Sketch::world_point`).
* After it, every point that still exists stands within 1e-9 of the extent of where it stood.
* DOF is unchanged except where the gauge and layout points leave the ledger. Each such change
  is listed in the PR.
* Volumes and exports of solids are byte-compared where no sheet pose reached them, and
  compared by measurement where one did.

## The app and the FFI

* `Point.plane` is never null. `gcs_point_plane` returns 0 for the page, and `-1` leaves the ABI.
* The workspace tables drop the page slot: plane 0 *is* the page. `PAGE`, `slot()` and
  `view_at()` go, and "places" compare bases. `std.front` and `std.up` stay one place.
* The plane chooser lists `std.front` always. `pendingPlane` stays for `std.side`/`std.top`
  until `use std` is written.
* The plane tool's two clicks placed a datum on the sheet. It becomes "pick two lines or rays",
  which writes `plane(u: a, v: b)`. The three-views tool is deleted.
* A ray is drawn in the workspace as a line across the scene's bounds with an arrowhead for
  its sense, and picked like a line. There is no 2D picture of a ray.
* The tests that pin page-ness (`app.test.ts` "the front is the page", "drawing on the page
  stays on the page"; `core.test.ts` `plane` null) are rewritten to say plane 0.

## Order of work

One branch and one switch. The commits are staged so each one builds and its own tests pass.
The corpus only goes green at step 8.

0. **Baseline.** Rebuild `build/solventc` (it predates #77). Record each example's world
   positions, DOF, report and SVG with the old binary. Write the migration tool.
1. **The page plane.** Plane 0 in every `Sketch`, `PointE.plane: u32`, no `None` arms, the page
   refusals removed, JSON. Every 2D test stays green. This step is behaviour-preserving apart
   from the refusals.
2. **`on` retired.** `coincident` takes every kind `on` took (`constraints::infix_op`'s one
   table, `CKind::operator()`), the 143 corpus statements and the tests' inline documents are
   rewritten by span, and `on(t == …)` pins become `coincident(t == …)`. A face's `on:` key is a
   slot, not the word, and stays. Behaviour-preserving: every report is unchanged.
3. **Rays.** `EntKind::Ray` in every exhaustive arm. Intrinsics, kernels and relations (the
   table above) with Jacobian rows. `ray` in the parser, seeds, `io`, `graft`, FFI `ent`/`kind_id`.
   Tests: a ray placed by two relations against closed forms, its DOF, a free ray's rank, an
   unread position retired, and an unsigned 0°/180° refused.
4. **Planes from rays.** `PlaneE` over rays, `Lift`/`LiftFixed` without datum columns, the
   ray-subsystem settle in `solve_planes`, `project`'s new constants. Remove `FrameE`, the
   quaternion, the hinges, `page_held` and the role rule. Port `tests/spatial.rs`,
   `spatial_lang.rs`, `plane.rs` and `plane_lang.rs` to the new spellings, asserting the same
   numbers in space.
5. **Readers.** The coplanar lint and the 2D-through-a-map reading between fixed planes lying
   on one another, `along: u/v` in space, the overview, the workspace, the renderer, solids'
   poses, extrusion frames.
6. **`std.sv` and the `.svd`.** The standard planes and rays, the helper components,
   `ThreeViews` gone, `sketch m.P at (…)`.
7. **The app and FFI.**
8. **The corpus.** The tool's rewrite, then the hand rewrites, then the gate above.
   `tests/examples_sv.rs` and `cross_view_audit.rs` are updated.
9. **Docs.** The primer (§1.13 and 2.10 shrink), the spec (§6.7, §9.2, the E062 row, Draft
   0.42), CLAUDE.md (the plane, page, role-rule and workspace paragraphs), and a status line on
   `spatial-constraints-plan.md` saying which of its decisions this plan replaces.

`make test` runs once at the end of each step that touches the corpus, and once before the PR.

## Settled in conversation

All of these were settled on 2026-10-04. No question is open.

* Every plane has `P.u`, `P.v` and `P.origin`.
* Planes lying on one another are a lint.
* A plane's origin floats until placed.
* `P distance(d) Q` stacks parallel planes, and a stack's origins are tied by a ray. That is a
  library component, with no sugar.
* `coincident` is incidence everywhere, and `on` is retired.
* `P.origin coincident p` places an origin.
* An unsigned 0° or 180° is refused in favour of `parallel`.
* A ray is a directed line with no start.
* **A part's own plane, turned within the page** (a frame like the vtwin crank's) is written with
  existing words:

  ```
  w := ray
  w perpendicular ab
  w coincident std.front
  frame := plane(u: ab, v: w)
  ```

  The seed picks which side faces out. No new form is needed.
* **`along: u/v` is measured in space**, across planes too: `p distance(d, along: u) P` is how
  far `p` stands along `P.u` from `P.origin`. Within `P` it is `p`'s own x, the number it has
  always been.

## Appendix: the corpus

The figures come from the 2026-10-04 survey:

* **Layout.**
  * 8 layout relations through `ThreeViews` (4 each in `engine.sv` and `vtwin/assembly.sv`).
  * About 30 placement statements in total:
    * `std.sv`: 4 fixes and 10 ThreeViews relations;
    * `bracket.sv`: 8 fixes;
    * `sphere_cone_cylinder.sv`: 4;
    * `hypoid_pitch_cones.sv`: 4;
    * `gauged_gear.sv`: 2;
    * `spiral_bevel/views.sv`: 3;
    * `swept_torus.sv` and `swept_tumble.sv`: 2 each.
* **Ordinates measured from a view's own datum point:** about 213 (engine 189).
* **Relations read in space:** 108. The audit gate asserts 209 cross-membership relations
  (`cross_view_audit.rs:117`). The comment in `reading.rs` saying 212 is stale.
* **Solved planes:**
  * `fold:` with an unknown: 2 (`skew_axes`, `bracket` `aux`);
  * `fold: along`: 3 (`spiral_bevel/views.sv`, `twist_drill/wheel.sv`);
  * `attitude: free`: 2 (`hypoid_pitch_cones.sv`).
* **Constant planes:** `fold:` with a number 14, `offset:` 19, `u:` 6.
* **Other uses:**
  * `project`: about 70, in bracket, the engine family, vtwin assembly and throttle, and the
    spiral bevel.
  * `along: u`/`v`: 77.
  * `against`: none (tests and fixtures only).
* **Sheets that place views:** `bracket.svd`, `engine.svd` and `vtwin/assembly.svd` (multiview
  via `sketch m at`). Every solid view already carries its own `at`.
