# Developable panels

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `path.rs`,
`developable.rs`, `model/developed.rs`, `solid/sheet.rs`, `flatten/developable.rs`,
`program/developed.rs`, a `RegionShape::Sheet`, a `CurveBody::Developed`, the plane–path
tangency kernels, `line_fraction`, the rate rows that compare rates, or `std.Developable` and
`std.Development`. An issue number (#n) or a spec version ([0.n], see the
[language changelog](../solvent-changelog.md)) says where a rule came from. The gates are in
`rust/gcs-core/tests/developable.rs` unless named otherwise.

**The panel and its pattern are definitions** (#645, [0.192]). `std.Developable(a, b)` is a set
whose body says what a developable panel is — its point on the ruling between `m` on `a` and `n`
on `b` where one plane touches both, `b` running through it the way `a` does — and
`std.Development(S, s, d)` a component whose body states an isometry along rulings. Neither word
carries behaviour the core adds by name: every rule a reader needs is a statement in `std.sv`.

**Recognised by what the body says** (`flatten::developable::sheet_body`, `development_body`).
A set's body is a sheet where it states two contacts `m` on a path `a`, `n` on a path `b`, two
tangencies of one plane at them — the second's `sense:` `same` or absent — and the set's point
over an interval of `line(m, n)`, whatever the set or its formals are called; a component is a
development where its body reads a set's `a` and `b` and states the four rows. The rails are
resolved where the set is used: a named chain (recorded as it is walked, `RailsSeen`), a path that
comes round (`closed`), a line, an arc, a circle, a spline (in space a traced curve), a curve
(`std.Ellipse` drawn in a plane, read in space through its plane). A hand-written copy builds
the same panel to the bit (`a_hand_written_panel_is_recognised`). An instance of a development is
walked by `walk_development`, not by its body: it draws `f` and `g` in the instance's plane, the
private ends `a0`, `b0`, `a1`, `b1` on the laid rails at `t == 0` and `t == 1`, the end rulings
`start` and `end`, and — a band — `seam` and `seam_end`; the parts check (E108) passes it by.

**One path reader** (`crate::path::Path`). The guide of a frame along a path, the plane–path
tangencies and the recogniser read a path through it: a line, an arc, a circle, a spline (rational
too), a curve traced in space, a curve of a plane lifted through its basis (a tape), a chain of
them and a path turned round. `jet` reads the point and three derivatives; `jet_side` a kink's
one-sided tangents; `jet_near(u, at)` the smooth piece `at` stands on, carried past its ends — a
spline's knots are no pieces unless one repeats to break its tangent, so between kinks the span of
`u` is read, never the span of `at` extrapolated; `kinks` where a tangent breaks; `joins` where
pieces meet, kinked or not. `encode`/`decode` carry a path in a kernel's constants, bounds-checked
(a document reaches them through wasm). Its jets against central differences, a chain's kinks
one-sided: `the_path_reader_differentiates_what_it_reads`; the guide's gates did not move.

**Correspondence and branch** (`developable.rs`). A ruling joins `A = a(u)` and `B = b(w)` where
`F(u, w) = det(a′, b′, B − A) = 0`; the rulings are the level set of `F` marched as its flow, `(u′,
w′) = (F_w, −F_u)/(|a′|F_w − |b′|F_u)`, in the panel's running number `σ` — the two rails' arc
lengths together — by the core's integrator (`ode::step`), events located by secant on a step. The
home is the root of `F(u₀, ·)` of `sense: same` (`(a′ × r)·(b′ × r) > 0`) nearest `a(u₀)`, roots
found by sign changes on samples refined by bisection; the march carries that branch. Each node
the march lands on is set back on `F = 0` by a Newton step along `∇F` (`Flow::settle`); no step
is longer than a quarter of the ruling; where several events fall in one step, the march stops at
the first; and a ruling is read one step on from the node the march came by, never back from where
it stopped. The tilted
adapter's home ruling is the outer root an independent root-find agrees with, and every ruling
keeps the sense (`a_tilted_adapter_takes_the_outer_root`).

**Supports, fans and ends.** At a kink of `a` the march stands at the corner while `b` runs — a
**fan** (`Piece::FanA`), a piece of a cone — from where the plane holds the tangent arriving to
where it holds the one leaving (`det(t_out, b′, B − A_k) = 0`); a kink of `b` likewise (`FanB`). A
rail's end is a corner whose other side is open: where the regular rulings reach one rail's end
first, that end fans over the rest of the other. The square-to-round is four planar triangles —
each piece's rulings one plane, meeting at one point of the round — and four fans standing at the
corners (`a_square_to_round_is_triangles_and_corner_fans`); the dory's chine fans at both ends
(`a_dory_panel_fans_at_its_ends_and_lies_on_its_sheet`). **Where the rails' ends meet** the
ruling shrinks to nothing and the panel comes to a point: `F` has a saddle there, its level set
crossing itself, and a march's drift off it is no longer small against the ruling, so the march
stops where the ruling is `MEET` (a hundred-thousandth) of the panel's size — that ruling its end,
laid like any other. The dory's bottom is pointed at its forefoot, and its two sides share the
stem line (`a_dory_is_three_panels_cut_from_two_sheets`); rails meeting short of either's end are
refused below (`rails_meeting_at_their_ends_come_to_a_point_and_elsewhere_are_refused`). `PlaneSupportsCorner` is the corner's row
in kernel form, held by `tests/jacobians.rs`; the recogniser builds a fan exactly.

**Closure.** Two closed rails march once round `a` from the home; the march must end on the home
ruling, `u₀ + period` and `w₀ + period` to `CLOSE` of a period, else the band is refused: "the
rulings do not come round" (`a_band_comes_round_or_is_refused`, a rail wound twice).

**Refusals**, a `Refusal` naming the place on `a`, said **E080** once at each use — the band's
solid (`solid::sheet::band`, read by `solid::refusal`) and the development (`solid_diagnostics`, at
its laid `a`): rulings that fold (one rail would run backward past `FOLD`, as where the panel turns
flat along a ruling and the two families of planes touching both rails cross, so the march would
switch family), a ruling along a rail's tangent (checked at the home to `ALONG`, the root of a
double zero of `F` being found only to its square root, and on the march), no root of `sense:
same`, a band that does not come round, one rail closed and one open, rails that meet short of
their ends (the panel would pinch to a point and go on). A rail that is no path is
**E040** at the use. Gates: `folding_rulings_are_refused` (two, the band's and the flat's),
`a_ruling_tangent_to_a_rail_is_refused`, `no_root_of_the_same_sense_is_refused`,
`one_rail_closed_and_one_open_is_refused_once`, `a_rail_that_is_no_path_is_refused`.

**Developability**: the panel's normal along a ruling is one — read at `m` from `a′`, at `n` from
`b′` — to rounding where the rails are read exactly (`the_tangent_plane_is_one_along_each_ruling`).
The normal is `a′ × r` normalised, `b′ × r` on a fan about a corner of `a`: the panel's outside is
the rails' order, and given the other way round the pattern is turned over
(`the_rails_the_other_way_round_turn_the_pattern_over`).

**The sheet's solid** (`solid::sheet`). A band term (`RegionShape::Sheet { rails, side }`, `lo` the
thickness) is pushed into its region solid's probe by the elaborator (`program::developed::sheets`)
and read first by `region::form` (`Form::Sheet`, `Route::Sheet`): the panel built from the middle of
`a` (a band from where `a` starts), its rulings at `stations(flatness(unit))` — halved where a
ruling's feet or middle stand off the chord between its neighbours' — capped at `MAX_STATIONS`, each
the quad between the ruling and the ruling `d` along the normal, faceted closed with faces
`inside`, `outside`, `a`, `b`, `start` and `end`, oriented to a positive volume. Remembered in
`Caches::sheet` against `solid::reads`. The frustum's band has the thin wall's closed form; its
STL is a closed shell, its glTF a binary glTF, and its STEP refused once in the core's words
(`a_band_is_a_solid`). The field refuses a band (`Form::Sheet`): it is never swept.

**The development, exact and generic** (`Developable::develop`, `develop_generic`). The layer `d`
out is `m + dN`; laid outside up from its home ruling at `(0, 0)` along `+x`, `φ′ = N·(ĝ × ĝ′)`,
`f′ = (m′_d·ĝ)e(φ) + (m′_d·β)e⊥(φ)` (`β = N × ĝ`), `g = f + |n − m|e(φ)`, marched over each piece
from the home both ways. The **check** marches the body's own four rows — each ruling as long as its
image, each rail as fast as its image, `a` meeting each ruling at its image's angle — solved by
Newton for `(f′, g′)` at every evaluation, the foot standing still on a fan at `d = 0` held by
rows that say so. On the four smooth panels the two agree to 1e-7
(`the_exact_development_agrees_with_the_bodys_rows`). **The isometry is the property**: each laid
ruling as long as its ruling, each laid rail as long as its rail to a relative 1e-9 (`Path::length`
cut at a spline's knots against the pattern's `∫ |f′|`; `Development::measures` reads both laid
lengths and the area in one pass). **No step crosses a knot**: a rail's spline knots break its
pieces as its kinks do (`Rail::joins`, fanning only at a kink), since there the flow turns a
corner in a higher order and the two marches — the panel's and the laid one — would each take an
error of their own across it, a dory's mirrored sides coming out 2e-8 apart. The pattern's area
is the panel's and an independent shoelace's
(`every_pattern_is_an_isometry_of_its_panel`). Closed forms: the frustum's sector and its layer
`5d` out (`a_frustum_lays_flat_on_its_annular_sector`), the offset adapter's equal angles through
its apex, the cut cylinder's cosine; the panel follows its rims (`a_panel_follows_its_rims`); two
builds agree to the bit (`a_panel_and_its_pattern_are_built_the_same_twice`).

**The laid rails are curves of the drawing** (`CurveBody::Developed`, `kernels::DEVELOPED`). Each
of `flat.a`, `flat.b` carries the rails encoded in its constants (`developable::consts`, `HEAD`),
built once and remembered by content (`developed`, thirty-two); its columns `[f.x, f.y, g.x, g.y]` place
the canonical laying rigidly, turned to the bearing from `f` to `g` and moved to `f`, over `t ∈ [0,
1]` evenly in `σ`; a contact reads only the orders it asks (`canonical`: `C″` and `C‴` each two
more readings of the exact rate), and a frame is one placing. Until the panel builds the curve is the point `f` or `g`. The intrinsic
`DevelopedLength` row holds `|g − f|` to the home ruling's length (the current `|g − f|` until the
panel builds; `reads_pose`, re-read where the solve ends): the instance is a figure of three
freedoms, placed by relations, and a relation reads `flat.a` as any curve of its plane
(`the_flat_is_a_figure_placed_by_relations`).

**The report** (`report::positions_where`). Of each panel the document names (`Sketch::panels`,
carried by the source alone): `{panel}.rulings` (its stations at the report's flatness),
`.fans`, `.closed` and `.area`; of each development: `{flat}.a.length`, `.b.length`, `.area` and
`.bounds.{x,y}{0,1}` in its plane (`the_report_reads_the_panel_and_its_pattern`).

**The four relations** ([0.192]). `T tangent(at: m) k`, `T` a plane: `PlaneTangentLine` of a line,
`PlaneTangentRound` of a circle or an arc, and of a curve traced in space a rate of its contact held
by the plane (`contact_rate::Word::Holds`) beside `m` in the plane (`PointOnPlane`, one statement
two rows); a curve drawn in a view, and `sense:` outside a sheet's body, are **E040**
(`a_plane_touches_a_circle_and_a_curve_in_space`; `plane_tangent.rs` keeps a plane and a point
refused). `p coincident(t in (a, b)) l` outside a set's body is the incidence and a `LineFraction`
bound (`flatten::expand::within`): a start past an end is steered onto the segment, and a distance
only past it breaks the bound (`a_contact_over_an_interval_holds_the_segment`); in a set's body the
interval is the recogniser's, never a region. `rate(x over s) equal rate(y over s)` (`EqualLength`
of the rates' scratch lines) and `rate(x over s) angle(rate(y over s), l) k` (`EqualAngle`) are
admitted rate rows (`program::curves::admissible`, `rated_ref` for a rate in the parentheses); not
linear in the rate, they are solved by Newton from the rate before and, where that fails, damped
from a rate in every coordinate (`rates::Body::rate_newton`) — the root reached the branch the
march carries (`a_rate_equal_to_a_rate_keeps_pace`); a rate of another formal is refused
(`a_rate_against_a_rate_of_another_formal_is_refused`). A body mixing points in a plane with points
in space is read where the core recognises it (`std.Development`), not yet by the generic march.
