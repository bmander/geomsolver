# Curves

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `curve.rs`,
`locus.rs`, `tape.rs`, `variational.rs`, `extremal.rs`, `ode.rs`, `rates.rs`, `lines.rs`. An issue number (#n) or a spec version
([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule came from.

**Variational curves (§9.10, §6.1, 0.53–0.59, #121, #144, #149, `variational.rs`, `extremal.rs`):**
`k minimizes E` / `k maximizes E` (`StmtKind::Minimize`, `parser/minimize.rs`; indicative, read by
the word past its ref, as the body words are) with `E` a sum of `c * integral(EXPR over p)` (or
`over (p, t)`, `t` the unit tangent) along `k`, ds-weighted; `flatten::values::settle_integrand`
writes the scope's numbers in and renames the binders `p`/`t`. Each term is a `CKind::Stationary`
(`[curve, weight, integrand, degree, maximize]`), only over a **free curve** `rope := curve(a, b)`
(`program/entities.rs::build_free_curve`: `EntKind::Curve`, `CurveBody::Extremal`, `CurveE::length`
its one own param, `CurveE::pegs`). **The curve is the solution of its Euler–Lagrange equation**,
solved inside it (`extremal.rs`): optimal control in arc length — state `z = (x, y, λx, λy)`, the
direction algebraic (`H_θ = 0` by Newton, `H_θθ > 0` Pontryagin, so a branch flip is refused, not
followed), `λ' = −f_p` — from `Lagrangian`'s three tapes over `(θ, x, y)` (`t.x → cos θ`; each
first in one variable, so second derivatives only). `Flow` is an `ode::Rate` (its branch the
direction), integrated by the core's Gragg–Bulirsch–Stoer (`ode.rs`, below) with the variational
equations through the same substeps; the
BVP (`extremal/shoot.rs`) is multiple shooting, four pieces an arc, a **peg** (a held point the
curve passes: `peg coincident rope`, absorbed — `rows_in` 0, its place held) splitting arcs with
the costate free to jump and `H` unbroken (a corner); dense Newton, a long curve started as a
gentle sag (`EASY`) and walked out, a pegged one from the unpegged shape with each peg where it
passes, warm starts walked along `dq` (`walk`). The drawing sees a curve of columns `[a, b, L]`
(`entity_params`): contacts are the ordinary curve kinds (`kernels::EXTREMAL` body,
`extremal::kernel_eval`/`kernel_frame`, gradients by the implicit function theorem; `∇C''` and
`C'''` by differences), the shape memoised by its constants' content (`extremal::shape_for`,
shared with `Sketch::curve_shape` and drawing). `length(L) rope` is `CKind::CurveLength` (the
radius kernel on `L`); with none holding it, the length is the drawing's when the drawing
determines it (`Sketch::lengths_held`, a rank question at the pose, contacts seeded first: hold the
columns the rows reading no free curve determine and the curve's ends, and ask whether a null
vector still moves `L` — a structural count cannot tell a deck's slide from the length's
freedom), and only otherwise does the energy's first statement carry the transversality row
`H = 0` (`variational::rows`/`kernel`, `KernelKey::Stationary`) — when `stationary_length` finds
one (`Energy::stationary`); else the length is `unsettled`, stated no row (one sent the solve after
ever longer ropes) and left a freedom, W114. A held row pressing a curve of held length that is not a peg is E040 (a corner,
never a tangency); with the length free it is what sets it. **The corner on a held line is
`rope touches rod`** (#149, `CKind::CurveTouchesLine`, `built`, 0 rows, refused a claim): a
**slide** (`shoot::Stop::Slide(k)`, a peg is `Stop::Peg`; `CurveE::slides`, `Energy::slides`), its
place on the line `σ` one more unknown of the BVP and `(λ⁺ − λ⁻)·d = 0` (no force along it) one
more row. The line's two points are the curve's **columns** after its length (`entity_params`,
`Ends::lines`, `extremal_def`'s `line{k}.p1.x`…; the constants carry only the count), the BVP's
outer Jacobian carried through them (`turn`: `∂d/∂p₂ = (I − ddᵀ)/ℓ`), so a rod held or not is
dragged and the rope follows; at most `variational::MAX_SLIDES` (2, `tape::MAX_VARS`) a curve, E040
past it or where the curve is not free. Never walked in from the corner-free shape touching a moved line (a fork:
sliding along the line and the curve are one motion there): `pressed` pins it as a peg at the
foot of the free curve's nearest point, then `release` eases the force along the line it bore to
none (`Problem::pull`). The verdict's Hessian takes `σ` (gradient the costate's jump along `d`);
a free curve's polyline samples each stop's place, so a corner draws sharp. `tests/slide.rs`,
`ring.sv`. `Sketch::
settle_variational` (at `add`/`remove`, the end of `graft`, `from_json`, elaboration) compiles each
curve's definition (keyed by its terms and peg and slide counts), its stops and free length into
`Sketch::variational`; `seed_extremals` then seeds lengths (a row's number, else `H = 0` by
bracketing above the chord) and contacts left at 0. The verdict (`extremal::verdict`: Legendre,
conjugate points as sign changes of `det ∂p/∂λ₀`, the Hessian in pegs' places and a free length)
is `Diagnosis::extrema` (`unsolved` where no shape is found), reported by the CLI, the JSON
(`[id, verdict, asked]`) and the app's marks. `length(L) s` on a spline stays `SplineLength`
(`integral.rs`). `std.hangs(L) k` is a word. `tests/{extremal,catenary,slide,minimize,spline_length}.rs`
are the gates. **A free curve is an edge** [0.96]: it runs between the two points it is written
with, so `Sketch::curve_ends` (a stretch's trim, else a free curve's `args`) is its ends for
`edge_ends`, the face walk, the profile, material fields and the CAD fit, and it stands in a face
whole (`face(rope, deck)`); a stretch of it from its own end is E080 naming that form. **A
stretch of a free curve is that curve, cut**: `Sketch::curve_shape` reads it through `trim.of`,
and `settle_variational` settles no stretch as a free curve of its own (one would be a second
curve with no energy, so no shape). A curve with no shape sweeps nothing (`curve_polyline_within`
empty) and its face is refused, never walked
(`catenary::a_hanging_rope_and_its_chord_are_a_face`).

**The integrator (#339 D1, `ode.rs`):** one Gragg–Bulirsch–Stoer over a state of any width, for
everything the core integrates along a formal. The caller supplies the rate and its Jacobian
(`ode::Rate`: `rate(u, y, f, jac)`; nothing is differenced) and, for a rate that solves a condition
by Newton from where it last was, its **branch** — taken where a step starts, given back before
each column, kept with each node as the step's last evaluation left it (a start, not the root at
the node). The explicit midpoint rule over `ode::STEPS` (even) substeps, extrapolated by
Aitken–Neville, `Settings::columns` (7) columns, order twice that; the error is the last two
columns' difference over the state, each component a fraction of the caller's `scale`, held to
`ode::TOL` (1e-13, well below "solved"); the step adapts to it alone, so a march is the same on
every target. The **variational equations** `Φ' = (∂f/∂y)Φ` ride through the same substeps (`Φ`
`n × m`, any seed, the identity for the initial value), the exact derivative of the step taken.
**Dense output** is one more step from the node before (`Path::at`), not an interpolant, so its
sensitivities are the flow's. A march runs either way along the formal; it is **refused, never
followed** (`ode::Refusal`): a step whose rate cannot be had is tried shorter to the floor (`1e-12`
of the span) and the march stops at the last place it reached (`Undefined`), and one past
`Settings::steps_max` (4000) steps is `Steps` (stiff, or nowhere near its answer). Free curves
(`extremal/shoot.rs::integrate`, `within`) run on it bit for bit as they ran on their own copy.
`tests/ode.rs` is the gate: linear flows against `exp(At)` (an independent series) and their `Φ`
against it, the formal threaded through the substeps, the order `2k + 1` of one step with `k`
columns measured by halving on an entire flow, `Φ` against differences of one step and of a
march, a seed of fewer columns, dense output forwards and back, a carried branch, and refusals.
For a wide state (#339 D4): the branch is any `Clone` value (a method of lines' is one per node);
`Rate::along` gives `f` and `(∂f/∂y) Φ` read along `Φ`'s columns, never forming `∂f/∂y` (`None`,
the default, forms it, and only then is `Work::jac` sized); `march_until` keeps the road a refused
march took (`march` is it, refusing); `Path::at_within` is a dense output's step stopping at the
first column past the second within the tolerance — the march's own steps keep every column.

**Rates along a running formal (#339 D2, [0.119], `rates.rs`):** a traced body may state
relations that read a rate (`rate(p over s)`, `syntax::RateOf` on a `Written`; language.md says
how it is written and flattened).  Such a body is **integrated from its home**, never solved again
at a place.  Its rows are three sets over one variable table `[outer] ++ q ++ w ++ q′` — the
algebraic rows, the **rate rows** (each rate lowered as a scratch line from an origin whose slots
read a zero tape to a tip whose slots are the rates of the point's coordinates, `RATE_BASE` until
`place_rates` puts them past `w`; the one-operand form `rate(p) distance(k, along: l)` the
ordinate between the line's two points) and the **initial rows** (`initially`, the home's
alone) — encoded after the old ones (`locus::Rates`, header ten wide; `traced::Rates`, eleven,
the interval in it) and read by one integrator, `rates::Body`, in the plane (`dim` 2) and in
space (3).  `admissible` refuses a rate anywhere but `parallel`, `perpendicular`, `angle` and the
ordinate; `rate_operand` refuses two rates in a row, a rate in `initially`, one over another
formal or an `Angle` (degrees: its unit is D3's to settle); a point the body does not move has
no rate.  **The rates at a place** are the algebraic rows differentiated along the formal (their
first Taylor order with `q` standing) and the rate rows, solved by Newton from the flow's
`Branch` (the rates before: one step for the rows admitted, all linear in the rate); `M = [J_q;
∂R/∂q′]` is factorised with each row scaled to **unit length** (smooth in the state: where rows
past the count disagree off the body's rows, at a substep, their least squares weighs them by it,
and a weight with a kink cost the extrapolation its order — a geodesic read 2e-7 between nodes
with rows scaled by their largest entry), LU where square, Householder QR where it has more rows,
a pivot under `1e-10` a **rank loss**: the index check, at every evaluation, refused there
(`ode::Refusal::Undefined`).  In the plane the system must be square by count (`curves::short`);
in space, where `parallel` is two rows and the body's own rows may imply one, at least square, and
rows past the count must agree (`strict`) at the home and at a reading, not at a substep.  The
flow's Jacobian is **exact**: `∂q′/∂z = −M⁻¹ ∂G/∂z`, the algebraic rows' part a mixed second
derivative by polarisation over `taylor::Jet`s (`r` along `u + t, q + t q′ ± t e_z` differs by
`2 D²r[ẋ, e_z]` in its second order), the rate rows' part a first; a column no row or tape reads
is skipped.  The state is `[q, θ free]`, the free columns constant states, `Φ` seeded by the
home's sensitivity to every `θ` column.  **The home**: a drawn instance's pose is the initial
value and its coordinates are **columns** of the curve (`CurveDef::home`, keyed `HOME`; `vars`
gain `#home.k` before the constants; `entity_params` reads `CurveE::pose`), projected onto the
body's algebraic rows (`rates::Body::project`: `q₀ − pose = Jᵀλ`, `r(q₀) = 0`, Newton on the
whole condition, `W = Σ λᵢ∇²rᵢ` by polarisation, so its sensitivity is exact off the rows; at a
solution it is the pose) — without it a solve from a rough start integrated frames the body
could never be, a clothoid's contacts took 58 s; with it 0.5 s.  Written in place the home is the
algebraic and initial rows solved from the seeds (restarts as `cold_start`'s), its sensitivity by
the implicit function theorem.  A drawn home must be a number (`Home::At`).  **The curve**: a
march from the home to the end of the interval on the place's side (`rates::reach`; past it the
distance doubled), so every place inside is read off one march, a function of the columns alone;
marches are remembered by content (`rates::remembered`: encoding, columns, home, reach), a refused
one kept as far as it reached (`March::refused`, re-marched to the refusal).  A reading is
`ode::Path::at` (dense output, `Φ` with it): `C` and `∂C/∂θ` from `Φ`, `C′` the rate,
`∂C′/∂θ = (∂f/∂y)Φ` exact, `C″`, `C‴` the solution's Taylor coefficients (`higher`: along `q(t) =
Σ q_k t^k` the algebraic rows' `t^k` coefficient is `J_q q_k + R_k`, the rate rows' `t^{k−1}` is
`k ∂R/∂q′ q_k + S_{k−1}`, one more solve with `M` each), and `∂C″/∂θ` (a curvature's Jacobian)
by difference of that local map along `Φ`'s column — no march.  The planar contacts read it
through `locus::kernel_eval_to`/`kernel_frame` (the interval rides where a pose would in the
constants), the drawing through `eval_flat`/`sweep`, the spatial ones through `traced::read`.
`report::rates_text` and `traced_text` say how far each such curve is integrated, or where it is
refused.  `tests/traced_rates.rs` is the gate: the tractrix (drawn, and written in place from
`initially` both ways), the clothoid against the Fresnel series, contacts against closed forms
and the assembled Jacobian against differences before the solve, the hanging chain shot to its
far end (`c sinh(50/c) = 75`), the geodesic on a cone in space (rows past the count) with its
contacts' Jacobians, a body one rate row short naming its freedom, the refusals, a march refused
where its rates lose rank, and the spelling printed back.  **The rates series**
(`rust/examples/rates/`, its README the reading order): one document per idiom — a tractrix drawn
and written in place, a clothoid transition into its osculating bend, a chain shot to its far end
beside the free curve, a pursuit refused at its capture, a geodesic on a cone in space — each curve
at the closed form its comments give and each claim a theorem, held by `tests/rate_examples.rs`;
the app lists the folder as one project (`example-catalog.ts`, *Rates*).

**The rate of a contact (#365, [0.122], `CKind::Rate`, `CKind::RateDistance`):** `rate(p over t)`
runs over **the slot of the contact holding `p`** wherever `t` names no formal in scope and the
statement is outside a trace (`Walk::rate_over` returns the empty name, `RateOf::slot`): a
spline's or a curve's `t`, a traced curve's `u`, a meshing's phase `t`, a carried point's roll `t`
(`RateSource::slot`).  The five questions the issue left open are settled so:
1. **Partial, never total.**  The rate is the derivative of the point the contact holds in its own
   slot with **every other column of the drawing held** — the contact's own other unknowns (a
   meshing's lag and places) following the contact's rows by the implicit function theorem, as
   its tangent always read them — never the implicit derivative of the whole solution.  So the
   slot is one contact's: `over` names a slot label, not a number; a name in scope (a `param`, a
   formal given a number, a face family's running number) is read as it always was, and a
   declared unknown is E040 ("an unknown of the drawing"); a point no contact holds with that
   slot, or two contacts holding it with one, is E061 at `io::seed_omitted`, which infers the
   carrier (`infer::rate_carriers`) — `g tangent(at: p) l` names the curve where two could.
2. **Closed forms kept, behind one surface.**  The kind is one; its kernel is the source's
   (`Constraint::rate_key`): a spline's `spline_tangent_at`, a curve's `FamilyKernel::TangentAt`,
   a traced curve's `traced::Contact::{Tangent, TangentSquare}`, a meshing's
   `meshing::Contact::{Path, PathSquare}`, a carried point's `screw::Reading::{Rate, RateSquare,
   RateAlong}` — each the rows its tangent was read by, so folding the per-source kinds changed no
   solved value.  A circle's or an arc's tangent at a point (`TangentCircleAt`, the radius square
   to the line, a degree-2 polynomial) stays its own kind: a point on a round owns no slot, and the
   closed form is cheaper and better scaled than a velocity.  A curve's rate in its view is held
   along a line only (its kernel reads no other); a rate's distance is read of a carried point only.
3. **Second order is not a rate yet.**  A curvature at a point (`curvature(at:)`) and an eased
   normal stay their source's kinds; `rate(rate(p over t) over t)` is a syntax error (a rate is
   of a reference, E001), and a second rate, when one is wanted, will be that operand rather than
   a separate word.
4. **Scaling.**  A rate is a length a unit of its slot — a radian of an angle's (`held::carried`
   reads the roll in radians, `meshing::along_line` turns a degree's into a radian's) — and its
   rows carry the degree of what they compare (`solver.md`): direction rows of a source that
   always moves are cosines (degree 0); a rate's components, where the point may stand still, a
   velocity (degree 1); a curve's in its view `(C′ × (B − A))/‖B − A‖` (degree 1).
5. **Diagnosis reads the statement written.**  `syntax::operator_text` prints a `Rate` as
   `rate(p over t) parallel l` — or, `word: tangent`, as `g tangent(at: p) l` — and a
   `RateDistance` as `rate(p over t) distance(d, along: l)`; a rate a defined word states reads as
   the word (`L reciprocal mesh`, `Constraint::word`).  A claimed word makes what it declares all
   the same (`flatten::apply`: the `coincident` placing a point the body declares is stated, never
   claimed) and judges the rest.
The retired kinds: `SplineTangentAt`, `CurveTangentAt`, `TracedTangentAt`, `MeshingTangentAt`
(each `g tangent(at: m) l`, now a `Rate` with `word: tangent`), and `Reciprocal` and `ScrewAxis`
(now `std.sv`'s `reciprocal` and `Screw`, rates of carried points).  `tests/rate_of_contact.rs` is
the gate: a carried point's rate against the epicycloid's derivative (along the world's axes, along
a line and square to one), a spline's tangent written three ways one kind over the contact's
place, a meshing's path written two ways one statement, and the refusals.

**The method of lines (#339 D4, [0.121], `lines.rs`):** a body traced over two formals (§6.5's
surface, `traced.rs`) may state a rate along either.  The first is **marched**, the second
**sampled**: its interval cut into `Body::pieces` equal pieces (`lines::PIECES`, 32, encoded in
the body's header) of the six Gauss–Lobatto nodes of #243 L2's walls (`brep::family::NODES`), a
joint node shared by the two pieces it ends, a whole turn of an `Angle` whose two ends' Cauchy
data meet **closed** (`Grid`, its last node its first); a rate along it is the derivative of the
quintic through a piece's nodes, at a joint the mean of the two pieces' (`Grid::derive`), and
between nodes the surface is that quintic (`Grid::at`: a place past an open end reads the end
piece's, a closed one wraps).  A rate along an `Angle` is per radian (`Body::per`, `180/π` on the
formal's degrees; over one formal still refused).  The variable table at a node is `[d, u, θ,
values] ++ q ++ w ++ q_d ++ q_u` (`traced.rs`'s header sixteen wide: the second interval, the
two `per`s, the pieces; the `q_u` slots lowered at `curves::SAMPLED_BASE`, placed past `q_d` by
`place_rates`).  **The split** (`Body::plan`, D0's finding 3): the coordinates whose rate along
the marched formal a rate row reads are **moved** (`I`), the state, integrated; the rest are
**placed** (`A`), solved at each node at each evaluation by Newton from the branch (the rows that
read an `A` coordinate, square in `A`, judged by their residual), given `I` and its rates along
the sampled formal — so a row reading `q_u` must read an `A` coordinate or be a rate row (one
over `I` alone, differentiated along the marched formal, would tie a piece's nodes), a `q_u` is of
a moved point, `initially` reads `I` alone, and the traced point is moved; `program::traced`'s
`Lowering::marched` refuses each naming the point, and `curves::short` over `I` names a missing
gauge's freedom.  **The rates at a node** are `rates.rs`'s system over `I` — the rows over `I`
differentiated (`per·∂r/∂d`) and the rate rows, by Newton from the branch, stopping where a step
is within `1e-8` of the rates (quadratic: the next is at rounding), `factor`'s unit rows and rank
— and a rate row's space `parallel` reads its two directions **across the line it holds the rate
to, as it stands at that node** (`Body::across`; stated once across a line of no length in the
scratch sketch they lose rank wherever the line turns square to them, and at a solution the
frame's own derivative multiplies a cross product that vanishes).  **The home** is
`initially` and the rows over `I`, Gauss–Newton at each node (least squares where they are more),
their sensitivity by the implicit function theorem; then `A` from its seeds.  The state is `[Q_I
node-major, θ]`; the flow (`ode::Rate`, branch every node's `A` and rates) is read **along `Φ`**
(`Rate::along`): each column a directional derivative at every node over the node's first-order
picture (`Lin`, made once a node where a derivative is asked: the placed rows' and the rate rows'
Jacobians over the whole table, the tapes' derivatives, `∂r/∂A` and `M` factorised at the root),
`δA` by the implicit function theorem, the rows over `I` by polarisation over jets — exact, and
banded by piece without a matrix formed.  **Regularity** at every node at every evaluation:
where `|x_d × x_u|` of the traced point falls below `FOLD` (1e-2) of the node's at the home the
march is refused there (`Why::Fold`, the node's place; `Why::Rank` where a node's rows will not
hold or its rates lose rank).  A march (`Body::march`, `ode::march_until`) keeps its road when
refused; marches are remembered by content (`lines::remembered`, sensitivities or not), the state
at each place read memoised in the march (`March::states`, `Path::at_within`), so every place
along the sampled formal at one marched place is one step.  **A reading** (`Body::read`, through
`traced::read` → `lines_read`) is the frame — `x`, `x_d`, `x_u` and, for a curvature, `x_dd` (the
rate's derivative along the flow), `x_du`, `x_uu` — and its derivatives in the place and every
`θ` column (`x_θ` from `Φ`, `x_dθ` the variational rate, `x_uθ` the quintic's of `Φ`); a
curvature's Jacobian (a third order) is not read, so `relations::traced_contact` refuses a
curvature contact on a marched surface.  `traced::nearest` remembers its last answer (each place
slot of a contact seeds alike).  **A march is held to the march on twice the pieces**
(`lines::check`, `traced::lines_check`: D0's decision 4 — the discretisation's spurious modes
grow faster than the surface, and there is no solve between nodes to check against).
`report::traced_text` says how far a marched surface is integrated, or where it is refused, why
and at which node.  `tests/lines.rs` is the gate: the offset front of an ellipse
(`rust/examples/rates/front.sv`) against `E(u) + d·n(u)` and both derivatives at every place read,
its convergence in the pieces (fifth order) and the check against twice them bounding the error,
its fold refused short of `b²/a` and nearer it on twice the pieces, a contact's place found and its
Jacobian (the variational equations) against central differences, a whole turn closed, and every
refusal of the split.

- A parametric curve (`curve.rs`) is *linear in its control points*, `C(t) = Σ Bᵢ(t) Pᵢ` (every
  B-spline).  A contact carries its own curve parameter as a `Param` slot and says
  `p − C(t) = 0`: two residuals, one new unknown.  A contact kernel needs only the basis values
  and two t-derivatives — a second curve family is a second basis, not a second constraint
  family.  A line against a curve is a tangency; a *circle* against one is `SplineCurvature`, the
  osculating circle ("the centre is the centre of curvature", no `side` to infer).  It divides by
  the turning rather than multiplying: multiplied, every row vanishes with `C'` and the solver
  collapses the parameterisation.  Control points are ordinary `Point`s.  Only `DEGREE + 1`
  control points are non-zero at any t, so a contact addresses one *span* (fixed-width blocks);
  the span is derived from t, not stored, and `Sketch::topology_key` carries it — a contact
  walking past a knot is a recompile.
- **A spline may be rational** (`spline(…) weights [1, w, w, 1]`, `SplineE::weights`, `None` for
  all 1): document data like the knots, so the curve stays linear in its control points over the
  rational basis `Rᵢ = Bᵢwᵢ / ΣBⱼwⱼ` (`curve::weigh`, applied after `basis` in `eval_on` and the
  kernels' `span_frame`; all 1 divides nothing, so polynomial splines keep their bits). A spline
  contact's consts are `SPAN_C` = knot window then the span's weights; `topology_key` carries
  them. Knot insertion is homogeneous; a deleted control point takes its weight (`graft`). The
  trailer's entries are `syntax::Weight`s, settled over the scope by expansion (E103 if not a
  positive plain number). The recipe writes `"weights"`, so the profile is `BSpline::rational` in
  our kernel and in OCCT's (`solvent_cad_bspline` takes a weights pointer). `nurbs.sv`,
  `tests/weighted_spline.rs`.
- **A spline through a chain's corners** (§6.1 [0.111], #243 L7): `spline(through: rims, start:
  l, end: m)` (`Decl::through`, parsed in `decl_list`, printed by `decl_args`) is expanded in
  `flatten::through` once the walk is done (`Aside::fits`, so the chain may be written after it):
  `n` corners make `n + 2` clamped uniform control points, the corners' own points first and last
  and `rail.k1` … `rail.kn` minted between (seeded at the corners, the end ones a third of the way
  on), each inner corner `coincident(t == k)`, and at each end the control point beside the corner
  at a third (one link: a half) of the line toward the next control point — natural, `C'' = 0` —
  or, given a tangent, toward that point's foot on the tangent through the corner (`#sfoot`, the
  lines `#srun ∥ l`, `#sdrop ⟂ l`): `C'' · T = 0`, the least bending the tangent leaves.  Each
  statement is the declaration's (its id and span), the helpers private construction.  An axis
  given as the tangent of a spline in a plane states one row more than it needs (a line in space
  parallel to it), consistent and harmless.  `tests/sections.rs` holds the control points to those
  the conditions themselves give (`C'' = 0`; `C' ∥ T`, `C'' · T = 0`), solved apart.
- A control polygon is edited three ways.  *Inserting* is `curve::insert_control` (Boehm's knot
  insertion: C(t) unchanged, every contact keeps parameter and place; `DEGREE - 1` neighbours
  move, keeping identity).  *Deleting* shortens the curve: `Sketch::min_children` is the general
  rule (an entity survives while enough children do), and `curve::knots_without` drops one
  interior knot per lost control point.  *Interpolating* is `Sketch::spline_through_held`
  (chord-length parameters, averaged knots, one collocation solve).  A place from empty space
  leaves nothing behind; a place from a Point is *held* by a `PointOnSpline` whose parameter is
  **pinned** at the fit's value (unpinned, the curve keeps m DOF sliding along itself).  A pin
  travels in the seed: `Arg::Seed { value, pinned }`, consumed by `Sketch::add` at the one seam
  turning a number into a Param.  `clamp_contacts` leaves a pinned parameter alone.
- A curve parameter is bounded (`t0 <= t <= t1`), which least squares cannot say.
  `System::solve` does — clamp, compare `curve::contact_spans` against the compiled spans,
  rebuild when one moved — so *every* caller gets it.  A clamped parameter is *pinned* for the
  retry.  A contact *seeded* off the end is clamped **before** the first solve and left free (a
  seed says only where the search begins, P3; #45.7).  `SolveOpts::rehome` turns it off for
  `PullPolish`, which owns a pair of systems and re-homes both together.  An empty span map
  costs nothing.
- A block's columns and its constants are ONE compile-time choice: which span of a spline a
  contact sits on.  `System::new` makes it once and passes it to both `params_on` and
  `consts_on`, and remembers it — so `refresh_consts` skips curve contacts outright (their knots
  are document data no solve moves) rather than re-deriving a span that may since have walked.

- **The ellipse is a library component** (issue #47, item 4): `std.Ellipse(c: point, a, b,
  tilt, u)` in `rust/lib/std.sv`, a computed point at eccentric angle `u` about `c`, its major axis
  turned `tilt`, traced as a curve — so
  `p coincident e`, `e tangent l` and `e curvature k` are the curve contacts, exact to third order;
  the entity kind, its kernels and `CKind`s are gone from every arm, the FFI, the binding and the
  app (an ellipse *tool* is a follow-up).  The parser keeps the word only to refuse it, naming the
  spelling; `io::from_json` refuses a document carrying the old `"ellipses"` table.
  `tests/ellipse.rs` holds the rim, the tangent and the osculating circle against closed forms,
  and the rim turning with its tilt.  An axis is a value the curve takes — stated or a `param` —
  since a component of one computed point cannot be drawn as an instance whose formal is left
  free.
- **The sphere, the cone and the cylinder are library components too** (see "Spheres, cones and cylinders are
  library sets" in [sets.md](sets.md)): FFI kind ids stay contiguous, the axis now 16.
- **A curve is a point of a component, as one of its numeric formals runs** (Solvent §6.5).
  No curve family: `path := leg.toe over theta in (0, 360)` asks a *drawn* instance, and
  `e := Involute(base, phase: a0).p over u in (u0, u1)` one written in place and never drawn.
  `syntax::CurveSpec` is the statement (`CurveTarget` `Drawn`/`Anon`, swept formal, interval);
  the flattener records every instance it binds (`flatten::InstanceInfo` — prefix, component,
  absolute actuals, numbers) and resolves a drawn target onto the instance owning the longest
  prefix, so `build_curve` never re-derives it.  `program::compile_curve` picks the body: a
  **computed** point, `p := point(x: xexpr, y: yexpr)` (`Decl::computed`), compiles to two
  `tape.rs` tapes, and a component with one is refused on the sheet; any other point is a
  **locus**, lowered by `compile_trace`.  Either way the tapes are differentiated forward in the
  swept formal *and* every coordinate they read (`∂C/∂θ`), or a point falls off when the circle
  is dragged.
  **The body is expanded by the real flattener, symbolically** (`flatten::expand_component`,
  `Sym`): every numeric formal is a *free value named after itself* (the `Aff` an unbound
  formal is on the sheet): `substitute` writes it back by name (or `(m * name + c)`), `settle`
  keeps a dimension with no number, and `Walk::keep_text` keeps a text nothing can work out
  (`sin(u)`) as text where it is read (`Sym::texts`, keyed by absolute name, looked up through the
  scope's prefixes).  A nested
  instance's *own* unbound formal is its own unknown (`#c12.i.u`), no column of the curve, never
  captured by an outer formal of the same name (`tests/curve_of.rs`).  One expansion per
  `(component, point, formal)` — `CurveDef::key` — shared by every instance.  `Walk::owner_of`:
  the innermost drawn instance owning the prefix *whose component has the swept formal*, handed
  on as `CurveSpec::of`.
  The variable table is the swept formal, then the entity formals' scalars **in `entity_params`
  order** (`EntKind::scalar_names`), then the numeric formals a drawn instance left unbound
  (`CurveDef::columns`, keyed; `CurveE::unknowns` names them in `free_vars`), then the other
  numeric formals (constants) — `params_on`'s column order, so a tape's gradient *is* a Jacobian
  row.  A whole turn of an `Angle` formal that comes back is closed (`Sketch::curve_closed`): a
  contact wraps across the seam (`clamp_contacts`).  A dimension reads geometry (`c.r`) only in a
  trace body; elsewhere E103 (`Checks::dim_reads`).  `EntKind::Curve` is the one kind whose children
  need not be points, built and grafted **last**.
  A curve's kernel belongs to its **definition**, not its type (definitions differ in width).
  `CKind::kernel()` panics for the three curve kinds, `kernel_id_in(sk)` returns
  `N_KERNELS + 3·def + slot`, and `System` owns the static kernels plus **three per definition**
  — `PointOnCurve`, `CurveTangentLine` (`inv tangent l`) and `CurveCurvature`
  (`inv curvature k`); the registry publishes `kernel: -1` for all three, which the bindings'
  tests key on.  The tapes ride in `consts`, so `KERNELS` stays `'static`.  Tangency and
  curvature need the **frame** (`kernels::CurveFrame`): `C` to `C'''` and the gradient of the
  first three orders in `[u, θ…]`.  A formula gives it exactly (`tape::eval_series_flat`,
  `tape::Series`, checked in `tests/tape.rs`).  A trace gives `C`, `C'` exactly and, asked
  (`need`, `Val::orders`), `C''`, `C'''` exactly: **Taylor orders of the implicit function**, one solve each
  with `finish`'s factorisation (`locus::higher_orders`; Wagner–Walther–Schaefer), the rows read
  over `taylor::Jet`s — each kernel's **Taylor form** (`taylor::form`; affine kernels by their
  `J`), held to its kernel by `tests/taylor.rs`.  Gradients along θ are a **forward difference**
  from the memoised centre (`locus::kernel_frame`: one warm block solve per column from the
  remembered pose, so the branch cannot change); along `u` the exact orders.  **A residual never
  builds the frame**: `curve_value` gives derivatives alone, only a Jacobian pays for the
  gradient (the `EllFrame` bargain).  A trace with a row lacking a form
  (`Locus::without_form`) gives no `C''`: `constraints::validate` refuses curvature naming the
  kernel and its slot is the `refused` kernel (rows NaN, not converged).  In the plane a
  generated profile is such a trace (normal through the instant centre; `tests/generation.rs`).  `CKind::family_kernel` (`FamilyKernel`: discriminant = slot, knows its row count)
  is read by `kernel_id_in`, `n_residuals`, the registry and `kernel_table`, so a fourth kind is
  one arm.  `Sketch::curve_polyline` is memoised against everything it reads (picks walk every
  curve per pointer move).  `tests/curve_contact.rs` holds the contacts against the
  involute's closed forms; `tests/common` is the shared finite-difference Jacobian check.
  **An unbound numeric formal is an unknown of the drawing.**  `leg := Leg(axle, pivot)` without
  `theta` binds a *free* `Aff` named `leg.theta`, one per instance; `value_aff` passes it
  (refusing a name nothing binds), a `param` over it is affine in it, a nested instance shares
  it, and `substitute` writes the name into dimensions (`expr::Free`).  The curve's anchor
  follows it: `CurveE::home` is `Home::Free(name)`, read by `Sketch::curve_home`, since the
  unknown is allocated after the curve is built.
- A **locus** (`locus.rs`) is what a traced point is: the curve is wherever the body's
  constraints put `p`.  The body is lowered once (`program::compile_trace`) through a scratch
  sketch and `Constraint::params_on` — never a second column mapping — into static-kernel rows
  over `[u, θ, values, q, w]`: `q` the body's coordinates, `w` a dimension over `u` computed by a
  tape and read by its **free twin** kernel (`(m, c)` the unit conversion), so no new derivative
  code exists.  `C(u)` is a small damped Newton solve, its derivatives the implicit function
  theorem from one factorisation.  The body encodes to flat `f64` in the contact's consts
  (`locus::eval_at` the one evaluator, `eval_flat` its cold entry); `System` picks
  `point_on_body_kernel` over `curve_kernel` per definition, bindings untouched.  A trace contact's
  constants are `[anchor, n_values, values…, has_pose, flat…, pose…]` (`kernel_eval` reads,
  `consts_on` writes, `kernel_table` sizes; `view` ignores what trails): for a **drawn**
  instance the **pose on the sheet**, read at every compile and refresh (`CurveDef::pose_of`,
  `CurveE::pose`, `Sketch::curve_pose` the one reader, `model::whole`: a pose with a hole is no
  pose), where the anchor solve starts (`locus::Anchor`).  The polyline sweep walks **outward
  from the anchor**, down to `u0` then up to `u1`.  Chirality is a *branch*, which no regular
  residual can state.  Three instruments, by strength (spec §6.5): a **signed constraint**
  (`point_line_distance`); an **orientation predicate** — `ccw(a, b, x)`, no residual, enforced
  only at the **anchor** by reflect-and-resolve, with deterministic restarts (fixed-seed
  `rng::Rng`, scaled by the entity formals' coordinates and *nothing else*) when there is no
  pose; and a **seed**.  Continuity carries the branch elsewhere — one warm-started march, and a
  body with predicates never trusts a direct solve *from the seeds* at the target.  A carried
  branch is **kept**: each contact's pose is remembered and the next evaluation *resumes*,
  trusted only as far as `locus::continues` checks it against the tangent `∂C/∂(u, θ)`
  predicted; a failure falls back to the anchor and full march.  A pose is addressed by *where
  its contact's constants live* (`refresh_consts` rewrites in place; values it may change ride
  in `outer` and miss rather than read stale), so `System::new` calls `locus::forget`.
  A seed is a *place*: `hint(at: c, bearing: u + phase)`, `hint(at: t)` (`program::at_seed`
  lowers both to tapes), and `hint((xexpr, yexpr))` only in a component that is only ever
  traced (`build` refuses a drawn one, since on the sheet a seed is a number a solve writes).
  A traced body must be square — as many rows as inner coordinates — or elaboration refuses it.
  `tests/trace.rs` holds the taut-string involute against its closed form (seeds 3× wrong), a
  gear on traced flanks, and the same parameters asked in three orders (guarding the resume);
  `tests/jansen.rs` holds the drawn form against a circle-intersection model at 24 angles.
- A curve is *geometry*, laid out in the core and only stroked by the front end:
  `curve::tessellate` refines to `FLATNESS_PX` through `unit`, and `curve::closest` is the pick
  test and a fresh contact's seed.  No binding evaluates a basis function or writes the degree:
  `report::registry_json` publishes `curve.minCtrl`, matching `Sketch::spline_with`.

- **A body traced in space** (#326, [0.107], [0.108]; `traced.rs`, `program/traced.rs`) is
  `EntKind::Traced` (`TracedE`: the body, the entities its component was given — points, lines,
  motions — its constant numbers, the formals' domain and home), built where a curve declaration
  runs over two formals (`CurveSpec::second`, `over (a in (…), b in (…))`, its ends settled by the
  flattener as the first's) or is given a motion — a placeholder in the geometry phase, compiled by
  `traced_bodies` once the motions are, before the envelopes and meshings — or where its planar
  compile fails and the spatial one does not (`curves::build_curve`; the name's resolver entry
  moved off the curves, `elaborate::geometry`).  Written in place only; numbers given as numbers.
  The lowering draws every point in space in a scratch sketch (its own lift), reads a word between
  them as the relation in space (`reading::in_space`), an ordinate at `OrdinateForm::Space`, a
  dimension as a `Tape::compile_wide` tape read by the free twin (`K::OrdinateSpaceFree` for an
  ordinate), a pinned place as a tape, and a carried point as a row of no kernel id
  (`screw::Reading::Carried`, the motion's steps its constants, its axes `θ` columns).
  `traced::Body` encodes `[n_formals, n_outer, n_theta, n_q, n_w, n_rows, traced]`, the tapes,
  the rows, the seeds; every static row must have a Taylor form.  `View::solve` is damped Newton
  from the seeds, else a march from the home — no pose remembered, a reading a function of its
  place.  `View::series` is the implicit function's Taylor series along any direction of the outer
  variables (rows over `Jet`s, `taylor::residual` / `screw::rows`; tapes over them by
  `tape::eval_num`, a `TapeNum` walker exact to the third order), and `Series::mixed` the
  symmetric multilinear derivative by polarisation over every subset's series.  `read` gives the
  `Frame` (point, `d[a]`, `dd[a][b]`) and, for a Jacobian, its derivative in every place and `θ`
  column — exact.  The contacts (`traced::Contact`: `PointOnTraced`, a curve's rate — `Rate`, `Tangent` held along a
  line, `TangentSquare` its part along one (#365) —
  `TracedSurfaceTangentAt`, `TracedNormalAt`, `TracedPlaneAt`, `TracedCurvatureAt` of a surface;
  `KernelKey::Traced`) own a place `[u, s]`, a curve's `s` pinned at the add
  (`relations::traced_contact`, which also reads `tangent` against a surface as its tangent plane
  and refuses a surface's words of a curve); the four with `at:` share the contact's place
  (`tangent_at`, `holds_on`).  Columns `[lead…, u, s, θ…]` (`traced::columns`: lifts of points and
  line ends, then motions' axes); constants `[n_values, values…, home×2, body…]`.  Seeds:
  `traced::nearest`.  `report::traced_text` says regular or where it folds
  (`traced::regularity` over a 13×13 grid).  `tests/traced.rs` is the gate: a helix by its
  ordinates and by a screw, a point and a tangent held to it; a torus's point, normal and the
  meridian's osculating circle against closed forms; every contact against finite differences;
  a fold reported.
  **Two design constraints it is built to** (#326): a traced surface is held to and read, not a
  face of a solid — exporting one is making it a wall of #243 L2's `Surface::Family` (whose
  `d1`/`newton` it would answer through `traced::read` and `traced::nearest`), not a second
  surface type; and the variable table puts the running formals first and reads the body along
  any direction of them, so a body that reads a rate along one (#339 D2, D4: the drawn instance an
  initial value, the march by the method of lines over the other) is one more column of the same
  table and one more order of the same series.  Over two formals such a body is `lines.rs`'s
  (above), read through the same `traced::read`, its contacts the same kernels.
