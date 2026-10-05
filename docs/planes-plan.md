# Points in space, planes from rays, views off the paper: plan

**Status (2026-10-05): implemented** on branch `planes-from-rays-76`, tracked by #81, for issue
#76 items 1 and 2. Follow-up spellings are #80. Where the build departed from this plan, or
settled what it left open, is [As built](#as-built) at the end.

Today a point is either *on the page* or drawn *in a view*. A point on the page has no place in
space, and a view is two things at once: a plane in space (its basis) and a picture placed on
the paper (its datum's `origin`, `toward` and rotor). That second job is where most of the
complexity lives. It is the reason for the role rule (`program/reading.rs`), the page-placement
gauge (`Sketch::page_held`), the layout lines in the corpus (`fix(x == 0, y == 200) og`,
`std.ThreeViews(O, right: 150, up: 90)`) and the "on the page" refusals. Since 0.20, paper
layout is the `.svd`'s job.

This plan makes these changes:

* **a point not drawn in a plane stands in space,** with three coordinates; a 2D sketch is
  drawn `in` a plane, `std.front` for what was the page;
* **a plane is a plane in space and nothing else,** written as two rays;
* **a ray is a new kind,** a directed line in space, placed by relations like any other entity;
* **`on` is retired:** `coincident` is the one word for incidence.

Nothing has been deployed, so there is no compatibility: the old spellings stop parsing, the
corpus is rewritten in the same change, and no diagnostic names an old spelling. The spec goes
to Draft 0.42.

## Decisions

These were made in conversation on 2026-10-04.

1. **A point not in a plane stands in space.** A point declared outside every `in` has three
   coordinates, `(x, y, z)`, and is placed by the same relations as any point. A 2D sketch is
   drawn `in` a plane: `use std` and `in std.front { … }` for what was the page. That is
   boilerplate, and accepted as such. The page as a concept is gone: `std.front` is the plane
   at the world origin with x right and z up. The model keeps it as plane 0 of every `Sketch`
   (see [Points in space and `std.front`](#points-in-space-and-stdfront)).
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
   rays through its origin. `P.origin` is a point drawn in `P`, held at `(0, 0)`, which a
   component may name and a relation may place; the plane owns where it stands in space (three
   unknowns). So `c coincident std.origin`, with `c` in `std.front`, stays a relation in one
   plane. **The origin floats unless something places it.** The
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
9. **`against` is deleted**, with placed planes, E083 and `CKind::Mate`: it placed a `from:`
   plane, and the corpus never uses it (only tests and fixtures do). Mating can return as a
   relation or a library component over floating planes.
10. **Retired:** `fold:` (an angle, an unknown and `along`), `offset:`, `from:`, `through:`,
   `attitude: free`, `u:`/`v:`/`o:` vectors, a plane's `origin:`/`toward:` arguments (the
   origin is now a member), the role rule, the page-placement gauge, `std.ThreeViews`, and the
   "has no place in space" refusals (E062's page case, the extruded envelope's, `through:`'s,
   `project`'s and the spatial words').
11. **Points in space are shown and selectable in the workspace, not dragged** (dragging in
    3D is later work).

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
`std.front.v`, and `std.y` points away from `std.front`'s viewer.

**A seed** is the one `hint(…)` clause: `t := ray hint(x: 0, y: 0, z: 1)` seeds the direction.
Its position is seeded through the origin, and moved by its relations.

**A ray's position nothing reads is no freedom.** A ray used only as a direction, like most of a
plane's rays, has a position that no equation mentions. Those two freedoms are retired to
`fixed` at elaboration, as an orphaned `Param` and an unread free variable are already, so the
diagnosis does not call the drawing under-constrained because of them. A position that a row
does read, for example where the ray is `coincident` a point or lies in a plane, counts as an
ordinary freedom.

### Planes

`PlaneE` becomes `{ u: Ray, v: Ray, o: [Param; 3], origin: Point, basis, fixed }`: `o` is where
the plane stands in space, and `origin` is the member point `P.origin`, drawn in `P` with its
`(x, y)` held at `(0, 0)`, so its lift is `o`. A plane written over a drawn line
(`plane(u: hinge, …)`) mints a hidden ray parallel to it, so `u` and `v` are always rays.

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
| `P.origin coincident p` | `P`'s origin is `p` | as two points: 2D within `P`, else `Coincident3` over lifts |
| `P distance(d) Q` | parallel planes `d` apart, `Q` on `P`'s out side | `(o_Q − o_P)·n_P − d`: 1, plus `P parallel Q` if their rays are not shared |
| `P parallel Q`, `P perpendicular Q`, `P angle(θ) Q` | between their normals | the ray kernels over `u × v` |

A plane's freedoms are `o`'s 3 plus whatever its rays leave free. A floating plane is free to
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

### Points in space and `std.front`

**A point in space** is a `PointE` with a third coordinate and no plane:
* `PointE` gains `z: Option<u32>`;
* `plane: Option<u32>` keeps its type, but `None` now means *in space* (three parameters) where
  it meant *on the page*;
* a language point's `z` is minted **after `memberships`**: a point no `in` reached is put in
  space then (memberships can reach a point after it is built, through a later declaration's
  `in`);
* `Sketch::world_point` of a point in space is its coordinates, and it is **its own lift** (a
  `LiftE` over its own three parameters, with no row), so every spatial kernel reads it
  unchanged.

What a point in space may make:
* lines between points in space are lines in space, and a line is a ray wherever one is
  expected;
* a sphere's centre may be one;
* a circle, an arc, a spline, a traced curve and a face need a plane. Built over points in space
  they are refused (E060, naming the point), unless every point is in one plane.

Its seed is `hint(x:, y:, z:)` (`z:` on a point in a plane is refused at the key) or a place
(`hint(at: …)`) read in space. Its gauge is `fix(x == …, y == …, z == …) p`.

**`std.front` is plane 0 of every `Sketch`,** built with the sketch as a fixed plane with the
constant basis `Basis::page()` and no member entities, so no point index moves. `use std` binds
its members (`std.origin`, `std.x`, `std.z`) to it. The *sketch-level* API keeps meaning a point on
it: `Sketch::point(x, y)`, a JSON document with no `"plane"` or `"z"`, and the bindings' 2D tools.
So the Rust tests' hand-built sketches and the JSON format are unchanged. Only the *language*'s
default is space. JSON writes `"plane"` when it is not 0, and `"z"` for a point in space.

`use std` brings in the other standard planes (`std.side`, `std.top`, `std.up`), the rays
`std.x`, `std.y` and `std.z`, and `std.origin`, which is `std.front.origin`, as ordinary library
statements. `std.front` is not in `StandardDatums`, but the flattener binds the
name to plane 0 when the document says `use std`.

### Reading a relation across planes

The rule stays: one plane, the 2D relation; otherwise, the relation in space. A point in space
is in no plane, so any relation naming one is in space, and a word with no meaning there
(`horizontal`, a run or rise, `tangent` between drawn figures) is E062. The role rule is
deleted, because there are no layout points left to read by role.

**Two planes lying on one another** (`std.front` and `std.up`, or a part's plane turned in
`std.front`) are permitted. The document gets a lint (a new W-code, at the second plane's
declaration, judged on the solved pose) saying they are one plane in space and suggesting the
geometry be drawn in one.

The relations across them still need care. The 3D twin's out-of-plane row is identically zero
there, and the diagnosis would read the drawing as redundant. Two ways to handle it:
* where both planes are fixed, read the relation as the 2D one through the constant in-plane map
  between them;
* otherwise, let the screen that already sets aside double roots (`witness::screen`) recognise
  the zero row.

Both are deferred unless the corpus rewrite needs them: components read a plane they are
handed (`f.u`, `f.v`) as directions and draw in their caller's plane, rather than drawing
`in std.up`, so no relation crosses two planes lying on one another. The lint lands now.

**`p distance(d, along: u) P`** is measured in space: `(X_p − o_P)·û_P`. Within `P` it is
`p.x − d`, which is the same number as today. Across planes it is what an engineer means by a
coordinate along that axis. The 2D `CoordinateU`/`CoordinateV` kernels lose their datum columns.
The cross-plane case is a new spatial twin.

### `project`

`project` stays and becomes simpler: `d_A·p − d_B·q + d·(o_A − o_B) = 0`, linear, over the
two points and, where a plane floats, its origin's columns. Where both origins are fixed, the
last term is a constant (zero where they coincide), and the row has 4 columns.
`plane::fold_line` reads bases only and is unchanged.

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

// a 2D sketch on the front plane
in std.front {
  a := point hint(x: 0, y: 0)
}

// a point in space
c := point hint(x: 10, y: 0, z: 25)
c distance(30) a

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
`plane(u: std.z, v: w)` with `w` the x axis reversed: `std.front` turned a quarter. The library
states `w` with a `fix` on its direction, which holds it outright. A document saying the same
writes `w parallel std.x` seeded the other way (decision 8).

**Membership** is spelled as before: `a := point in right`, `in right { … }`, and an instance
joins a plane whole. A point with no `in` stands in space. A component's `in f { … }` over a
plane formal is unchanged. A 2D component need not name a plane at all: its caller writes
`t := Tooth(…) in std.front`, or calls it inside an `in` block, and the membership is stamped on
every point it makes, as it is today. So the library's 2D components (`std.CenteredRectangle`,
the gear teeth) keep their signatures.

## The `.svd`

A sketch view keeps the existing grammar, `sketch NAME(TARGET) at (X, Y)`, with the target a
path filter as before. `from P` on a sketch, refused today, selects the plane drawn, in that
plane's own coordinates:

```
sketch right(m) from m.right at (150, 0)
sketch top(m) from m.top at (0, 90)
```

Each sketch view draws its plane's own coordinates, scaled like a solid view. Callouts are laid
out per plane. A callout across two planes is drawn in neither, as now. `bracket.svd`,
`engine.svd` and `vtwin/assembly.svd` are the three sheets that need it. The spacing that
`ThreeViews` and bracket's eight `fix`es stated moves here.

## The corpus

The [corpus survey](#appendix-the-corpus) found two kinds of rewrite, and points in space add a
third, which is the largest but the simplest.

**Every document drawing on the page** gains `use std` and `in std.front { … }` around its root
drawing (a `preview { … }` block included). Statements that are not geometry (values, `style`,
solids, claims) may stay outside it. Module components are untouched (their callers stamp the
plane). The tests' inline documents are wrapped the same way, through a test helper where many
share a shape.

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

* `Point.plane` is null for a point in space, and `gcs_point_plane` returns -1 for it (it
  meant the page). A point's `z` crosses the ABI beside `x`, `y`.
* The workspace tables drop the page slot: plane 0 is `std.front`. `PAGE`, `slot()` and
  `view_at()` go, and "places" compare bases. `std.front` and `std.up` stay one place.
* The plane chooser lists `std.front` always and opens on it. Drawing on it writes `use std`
  and the `in std.front` clause, as drawing on `std.side` does today (`ensurePlane`).
* A point in space is drawn as a dot in the scene and is selectable, but not dragged.
* The plane tool's two clicks placed a datum on the sheet. It becomes "pick two lines or rays",
  which writes `plane(u: a, v: b)`. The three-views tool is deleted.
* A ray is drawn in the workspace as a line across the scene's bounds with an arrowhead for
  its sense, and picked like a line. There is no 2D picture of a ray.
* The tests that pin page-ness (`app.test.ts` "the front is the page", "drawing on the page
  stays on the page"; `core.test.ts` `plane` null) are rewritten to say `std.front`.

## Order of work

One branch. Each step lands green (`make test`) and is committed, so the corpus is rewritten in
the step that changes its meaning; the gate below runs after steps 1, 3 and 4.

0. **Baseline.** Rebuild `build/solventc` (it predated #77). A scratch tool records each
   example's DOF and the world position of every named point with the old code.
1. **`on` retired.** `coincident` takes every kind `on` took (`constraints::infix_op`'s one
   table, `CKind::operator()`), the corpus statements and the tests' inline documents are
   rewritten by span, and `on(t == …)` pins become `coincident(t == …)`. A face's `on:` key is a
   slot, not the word, and stays. Behaviour-preserving: every report is unchanged.
2. **Rays.** `EntKind::Ray` in every exhaustive arm. Intrinsics, kernels and relations with
   Jacobian rows. `ray` in the parser, seeds, `io`, `graft`, FFI. Additive.
3. **Planes from rays, views off the paper.** `PlaneE` over rays and `o`; `Lift`/`LiftFixed`
   without datum columns; the freezing pass; `project`'s new constants; every reader of the
   sheet pose; `FrameE`, the quaternion, the hinges, `page_held`, the role rule and `against`
   removed; `std.sv`; the `.svd`'s `from P` on a sketch; the corpus's planes rewritten by hand
   and by a migration tool; the plane tests ported.
4. **Points in space and plane 0.** `z`, plane 0, the language's default flipped to space, and
   every document and test wrapped in `in std.front` by span in the same commit.
5. **The app remainder and the docs** (primer, spec Draft 0.42, CLAUDE.md).

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
* **A part's own plane, turned within `std.front`** (a frame like the vtwin crank's) is written
  with existing words:

  ```
  w := ray
  w perpendicular ab
  w coincident std.front
  frame := plane(u: ab, v: w)
  ```

  The seed picks which side faces out. No new form is needed.
* **A point not in a plane stands in space**, in this switch: a 2D sketch is drawn
  `in std.front`. Points in space are shown and selectable in the workspace, not dragged.
* **`along: u/v` is measured in space**, across planes too: `p distance(d, along: u) P` is how
  far `p` stands along `P.u` from `P.origin`. Within `P` it is `p`'s own x, the number it has
  always been.

## As built

Decided while building, 2026-10-04 to 2026-10-05:

* **No plane 0.** `Sketch::new()` mints no plane. A point with neither a plane nor a `z` is a
  *2D sketch's* point (a JSON document, a hand-built test sketch, the bindings' 2D tools),
  pictured on the front plane but with no lift, so a relation in space refuses it; the language
  never makes one. `use std` is what gives a document
  `std.front`, as an ordinary library plane.
* **The standard datums flatten after the document**, so a document's own points keep their
  indices; `use std` adds five points (four plane origins and `std.origin`), four planes and four
  rays (`std.x`, `std.y`, `std.z`, and `std.back`, the x axis reversed, for `std.up`'s v). A plane's
  origin is minted in the point pass. `std.origin` is a point of its own in `std.front`, fixed at
  `(0, 0)`, not an alias of `std.front.origin`.
* **Settling, not freezing.** `views::place` solves the rays, plane origins and the relations
  among them before the main solve, round by round, alternating with seed settlement while a seed
  reads across planes. It moves nothing that the main solve then holds: planes and rays stay
  unknowns of the solve. Rows over parameters all held (`use std`'s axes) are left out of
  `cgraph`, so plans and drags are unchanged.
* **A frame turned within a plane is `std.Turned(o, t)`**: the plane through `o` whose u runs
  toward `t`, drawn in the plane `o` and `t` are in (`axes := std.Turned(o, t) in std.front`).
  `hint(at: P, x: …, y: …)` seeds a point in plane `P`'s own coordinates, which is how a part
  turned within its plane is seeded. `std.Square` and `std.Stacked` were not needed by the corpus
  and are not written.
* **Across planes `along: u` is `Ordinate3U`/`V`**, in space, and draws no callout.
* **E060** for a circle, arc or spline over a point in space ("an arc is drawn in a plane, and
  `x` stands in space: draw it `in` one"), **E080** for a face over one; a traced component
  refuses a relation in space.
* **The app**: a new document is `use std`; every view opens drawing on `std.front`, so a press
  never lands a point in space by accident. The plane tool picks two drawn lines and writes
  `plane(u: …, v: …)`. A paste reuses the destination's held plane of the same basis and origin,
  so it draws in the plane the document already has; a drag part brings the planes its points
  are drawn in, as walls.
* **The sheet pose is gone** with the identity it had become: `plane::in_view`, `on_page`,
  `FacePoly::pose` and `PageFrame`'s pose.

**The gate** (every example's DOF and the world position of every named point, against the
pre-switch baseline) held but for: `square` (its DOF-1 scatter seed shifts), the V-twin's free
crank angle (stays at its 180° seed, was 195.6°; DOF 1 either way) and the bracket's auxiliary
view (now unrotated on paper). The hypoid layout decomposes into 115 blocks (was 119), so two
rescue tests pick new jitter seeds.

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
