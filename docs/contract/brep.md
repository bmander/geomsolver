# The Rust B-rep kernel

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `brep/`. An
issue number (#n) or a spec version ([0.n], see the [language changelog](../solvent-changelog.md))
says where a rule came from.

**The Rust B-rep kernel** ([plan](../rust-kernel-plan.md)); clean-room — OCCT is LGPL, this repo
MIT: its code is never translated, only its behaviour and conventions studied. `gcs_core::brep`:
`geom` (surfaces parameterised as OCCT/STEP do, closed-form inverses, signed distances, `Traced`
intersections), `topo` (`Brep`: oriented uses with pcurves, degenerate poles; `check`, `pinches`),
`build` (prisms, drafted prisms, revolutions and canals of line/arc/circle/spline profiles),
`props` (volume by Green's theorem), `query`, `ssi` (closed forms, else traced), `boolean` (split, arrange, classify,
assemble), `mesh` (constrained Delaunay refined to a measured sag), `step` (AP214 with pcurves and
seam curves), `recipe`. A traced pair is seeded where edges cross faces; between analytic
surfaces `ssi::unseen` then searches one face for curves no edge reaches (cells proved clear by
the signed distance's Lipschitz and second-order bounds, else beside a known curve, else seeded),
refusing what it cannot resolve: an untraced meeting is never read as none (issue #58).
`Brep::bounds` encloses (issue #59; `through:` cutters, tool culls): `Curve::bounds` exact for
lines and conics, the poles reaching a B-spline's range, a trace by its measured sagitta; a face bulges
past its edges only off a `ruled` surface (`Surface::bounds_over`: meridians at each axis's
extreme angle). Never a sample grid: extremes between samples were missed.
**The recipe is read once** (`recipe::Recipe::read`): every reader — this kernel's build and
meridian, `section`, the swept cutter and sector, `solid::exact`/`canal`, the native host's
`Session` — matches a typed `recipe::Node`, never the JSON; a profile stays as written
(`recipe::Section`, the numbers the native host hands on) until `Profile::of` reads it. A new node
or field is one arm of `Node::read` and of each match. Every writer writes its points through one
`recipe::Writer` (millimetres, about the world's origin or `cad::recipe_about`'s). **A kernel error
has a kind** (`brep::KernelError`, `Kind`): `Brep::check`'s edge not used once each way is
`NotManifold` (what `boolean::combined` and the body's stratum act on), a recipe of another kind
than a route builds (`Recipe::meridian`, `Sectioned`) is `Refused` (its caller builds it
another way), an unreadable recipe `Recipe`; its message is all it says, and it becomes a `String`
where a caller outside `brep` wants one. Never match an error's text.
**This kernel is every export's default** (phase 5): OCCT answers only `--kernel occt` /
`--stl-backend occt` / `SOLVENT_KERNEL=occt` (the oracle; refused in a build without OCCT), its
STEP checked by `step_check` against `Solid::of`. `gcs-cli tests/brep_oracle.rs` builds every
corpus recipe node both ways (where OCCT is off the bar a closed form or OCCT's own declaration
decides: a whole revolution of lines and arcs is held to Pappus, `pappus`; a Boolean whose edges
OCCT declares far coarser than its operands' — two tori touching in the hypoid's tooth space — to
that tolerance times its area, ours rebuilt at tolerances ten times either way within 1e-9,
`rebuilt`; a general surface OCCT builds for an analytic one of ours must lie on ours within its
declared face tolerance); `tests/brep_census.rs` holds a census and `brep_body_debug`;
`SOLVENT_BREP_DEBUG` narrates, `SOLVENT_BREP_TIME` times slow Boolean steps. OCCT's volume of a
STEP it reads (~1e-5) is no gate on our accuracy. Rung 1 is done: every corpus solid with a recipe
agrees with OCCT (volume 1e-9, faces by kind) but for named refusals of self-touching designs
(surfaces meeting under 1°, pinches).
**Phase 1 (OCCT's shape, our files):** `SOLVENT_WRITER=rust` reads OCCT's solid into `brep`
(`backend/dump.cpp` → `Session::brep_json` → `brep::json::read`; uses ordered by vertex and
parameters since OCCT's wire explorer misreads a closed edge) and writes STEP/STL by ours.
`Surface::BSpline` is a `nurbs::Net`, **rational where it carries weights** (`BSpline`/`Net`
`weights: Option<…>`, finite and positive; `None` divides nothing, so polynomial exports keep their
bytes; `nurbs::arc` is a circle exactly; STEP's `RATIONAL_B_SPLINE_*` complex entities;
`tests/rational.rs`, `brep_oracle.rs`'s OCCT round trip); `Pcurve::Curve` written exactly; `Edge::tol` is
measured and `check` honours it, so it no longer proves pcurves meet edges: a reader holds
tolerances to a bar itself. `props::fluxes` tables a polynomial B-spline face's `G` per span (a
rational one's Gauss stretch by stretch) and **closes every
loop in the face's parameters** across the kernel's gaps (unclosed, the volume moved with the
origin). The mesher is a neighbour-array CDT, refined for sag **and** for a facet turning from the
surface's outward normal (signed); slivers get a point off the longest side; points nearer than
1e-9 are welded. An edge is sampled from first pieces no midpoint test can be blind to: a
B-spline's knot spans, a family rail's pieces, a screwed rail a quarter turn at most
(`edge_params`; a coil of four turns has its ends, middle and quarter points in one line, and was
one chord). `Mesh::turned` counts facets facing against their surface (thicker than bar/100);
the export refuses any.
**Phase 2 (the pattern ours):** `brep::pattern` turns a one-sector body into the whole (the
kernel's union is never made): sides matched by a pitch's turn, revolution faces keep the sector's
surface with pcurves shifted along `u`, pieces of one surface across a side become one face, a ring
keeps its last junction as seam. `Net::segment` cuts sheets exactly. `Built::mesh` meshes the
sector once (`mesh::mesh_with`) and shares seam points by index. Full verification reads our STEP
back by OCCT and measures it by `props` (1e-5; OCCT's volume held only to 1e-4).
**Phases 3–5 (the gear without OCCT):** `brep::sweep` builds an admitted swept body: blank from its
meridian region (`brep::planar`, `recipe::meridian`), cutters sectioned exactly (`brep::section`,
`planar::Chart`), sheets traced and fitted (`sweep::sheet`, `nurbs::interpolate_net`; a fitted
edge still in the blank between its nodes — a cutter's corner grazing it at stations where it is
elsewhere a fan the rows stop before, `contact_trace::charted` — is carried on past it by its own
surface, `Net::extended`, only where what that puts in the blank is within the fit's bar, else left
as it is: an edge in the blank divides nothing; `brep_export.rs`'s 20-tooth gear), blank split
(`boolean::split`), cells judged by the field (`sweep::sector`; else `sector::whole`), sector
patterned. `brep::export::{exact,step,stl}` is shared by `solventc` and `app/export-worker.ts`
(`gcs_solid_exact`), so native and wasm files are byte-identical: the core's trigonometry, `exp`,
`ln`, `pow`, `hypot` are its own (`fmath`, `x.dsin()` via `Det`; no platform libm). With OCCT and
`--verify-step full` the CLI reads our STEP back (`cad::read_back`). A face met twice is told apart
by `cutter::OCCURRENCE`; a trace ends off a sheet's patch (`Surface::off_patch`, a revolution's or
an extrusion's of a curve too); a revolution about the axis is kept by `pattern`'s turns; interpolations
past `nurbs::BANDED` points solve banded; a gross sheet falls back to chord length.
**Rung 3 (the app reads the exact solid):** a static solid's `EvaluatedSolid` is the B-rep's
(`solid::Exact`, `Sketch::exact_solid`, cached against `solid::reads`; `from_brep`), built about its
own origin (`cad::recipe_about`, every point written through `recipe::Writer`), faces named by document path (caps leading as the facet term's, so
edges are named alike), meshed no coarser than `BREP_RELATIVE_SAG` or 64 steps a turn, one
polyhedral `Prim`. The facet term answers what the exact path refuses (touching tori, empty
solids); `SOLVENT_SOLIDS=facets` throughout. Silhouettes on curved faces are traced on the surfaces
(`EvaluatedSolid::silhouettes`), never the mesh's seams, which zigzag.
**A face family's walls** (#243 L2, `brep::family`): `Surface::Family` over a `Sheet` — an
edge's kind (`Run`: line, round, spline), the pieces of the interval with six nodes each
(`Stations`, the Vandermonde inverses kept), and the edge's numbers at every node — `S(u, s)` the
edge's own form in `u` (never differenced) of numbers that are, between nodes, the degree-five
polynomial through them; `S_s` the polynomial's own derivative, a rail's `S_ss` a central
difference of it (as a loft's rail's).  `u ∈ [0, 1]` (periodic for a lone circle), `s` the running
number; `breaks` the pieces' joints, so `props` integrates each piece exactly; `bounds` by each
number's polynomial range (constant term ± the rest), never a sample; `inverse` per piece nearest
box first, warm from the thread's last foot.  Rails are `Curve::Rail` (the wall at `u = 0`,
parameter `s`), rims at the ends `Curve::Line`/`Circle` where they are one, else `Curve::Rim`; an
edge vanished at an end is a degenerate edge, its rails meeting in one vertex, and a section
vanished whole has no cap.  **The fast path** (`lower`) puts a wall on a plane (a line edge in one
plane), a cylinder or cone (a line turned about one axis, or coaxial arcs square to their axis,
radius constant or affine in the height) or a torus (an arc turned about an axis in its plane) —
read off the numbers, never a word — kept only when the sheet's points at every node lie on it
and the face's area there (`props::area`) agrees with the sheet's, both within `AGREE` (1e-10).
**A section carried rigidly** (`carried`, #243 L6): where every station is the first one moved
by one screw (a turn about one axis and a step along it in proportion, a turn alone included) or
by one step along a line — read off the section's points at every node and both ends, the turn's
axis off the station turned farthest, its centre off the one nearest a half turn, each turn
unwound from the last, every point then held to the motion within `AGREE` of the size and the turn
running one way — the solid is built **exact throughout**: each rail the corner's path
(`Curve::Iso` of the swept wall, parameter `[0, 1]`), each wall the analytic surface where it is
one, else the surface its first rim sweeps (`swept`: `Surface::Blend` of the rim with itself under
the `Carry`), each held as above against the solid on its sheets (`general`, edge for edge);
where any wall will not lower, the general solid is kept, lowered wall by wall.  The sheet is
exact only at its nodes (between them within its piece check), so the rails must be the motion's
too: on the sheet's rails a lowered face's area missed by 2e-10.  `Blend::inverse` starts warm
from the thread's last foot on it where that settles near the point (as a family's wall does: the
evaluated solid asks at every triangle's corners, and searched afresh the spring meshed 6× slower),
else seeds each `u` where the screw turns it to face the point (`turned_to`; a grid along `v` alone
starts on a coil's next turn); `inverse_near` runs Newton from its hint first; `step::blend_net`
fits quintic along a screw a cubic cannot follow at the fit's cap.  A twisted duct, a thread, an auger's
flight and a coil lie on blends; a section that changes as it runs stays on its sheets
(`tests/helix.rs`, `a_rigid_screw_lies_on_its_swept_rim`).
**Creases** (#243 L7, `creases`): a joint of the pieces at which some wall's `S_s` turns, read
from either piece (`Sheet::rate_in`) at five places along it, by more than `CREASE` (`1 − cos`,
1e-6) is a section the walls meet at an angle — a ruled loft's, where a family runs chains in
step.  The solid is built in **bands** between them (`Band`, `Stations::band`/`Sheet::band`: each
wall's pieces of the band alone), each wall a face a band under its one name, each band's rims
edges shared with the next, rails a band at a time; the caps and the vanished ends as before, and
each band's walls lowered on their own (a ruled bottle's two cones).  A joint within `CREASE_GAP`
of another or of an end is none: the pieces closing in on a law's kink (`max`) stay in their face.
A family in bands is never carried rigidly.
STEP writes a sheet as the net fitted within `step::FIT` (`family_net`, cubic along the edge and
quintic along `s`: a cubic through 512 rows misses a 3½-turn auger by 1e-6), rails and rims as
cubics within it.
Rung 2 is done: `Curve::BSpline` (`brep::nurbs`), `Surface::Extrusion`/`Revolution`, a face's
stretch of a traced or formula curve (`k from p to q`, `CurveE::trim`, `cad::FIT_MM`), and a
section carried to its own image (`build::ruled`: a drafted prism's offset, a canal's screwed disc;
`Surface::Blend`, `Curve::Iso` rails, `step::FIT`), each edge joined to the edge it became at equal
parameter fractions.  Two sections drawn apart are no input to it (#243 L3): what a section becomes
along a path is the face family's (`brep::family`).
`curve_surface` drops roots off the surface. The meter reads a static body by `accuracy::Exact` and
a field taking splines as chords within `CHORD_SLACK`. Edge sampling and `curve::tessellate` test
quarter points (a midpoint misses a cubic's S-bend).
Phase 0: `SOLVENT_ABI_TRACE=FILE` logs native entry points, Boolean face pairs
(`backend/probe.hpp`, `BOPDS_InterfFF`) and smallest pieces; `rust/gcs-cli/tools/abi_trace.py`.
