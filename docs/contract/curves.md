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

**A frame that moves is a frame read at a roll** (#366, [0.138]; the rule is
[frames.md](frames.md)'s): a station word's frame (`std.Helix`, `std.OnHelix`, `std.Twisted`) is a
motion's pose at a roll written over the running number, `X coincident x at(under: m) τ` and `n :=
base at(under: m) τ` — never `cos`/`sin` ordinates of the running number.  The roll,
stated, is read as a dimension is: a station reads it again at every place (`expr::evaluate` over
the held number), and one that states a rate leans on it as on any number written over its running
number (`solid::rated`'s `Levers`: its free twin over the running number's column, the built
kernel's last column), so a carried frame moves along the flow exactly.  `t == τ` is the contact
form, its roll the relation's own unknown, which a rate runs over (below); a stated roll owns no
unknown, and a rate of a point carried to one runs along the formal its roll is written over.  A
family whose station is carried so is its motion's sweep (`generating.md`).

**The rate of a contact (#365, [0.130], `contact_rate.rs`, `CKind::Rate`,
`CKind::RateDistance`):** a contact holds a point at its place, and every statement at that point
reads the same thing — the contact's **frame** there, the derivative of its place in its slots —
so one kind states them and **one kernel** reads them.  `rate(p over t)` runs over **the slot of
the contact holding `p`** wherever `t` names no formal in scope and the statement is outside a
trace (`Walk::rate_over` returns the empty name, `RateOf::slot`): a spline's or a curve's `t`, a
traced curve's `u`, a traced surface's `u` or `s`, a meshing's phase `t`, a carried point's roll
`t` (`RateSource::slot`).  `Rate`'s `word` says what of the frame is held against what
(`contact_rate::Word`): `parallel` (and `tangent`, `g tangent(at: m) l`, the same) the rate across
a line, `perpendicular` along it, `RateDistance` along it less a distance, `normal` a line square
to every rate of a surface (`S perpendicular(at: p) l`), `plane` a plane along that normal
(`S tangent(at: p) P`), `in_plane` a line square to it (`S tangent(at: p) l`), `curvature` the
circle osculating the section by its plane (`S curvature(at: p) k`).
**The kernel** (`contact_rate::kernel`, `KernelKey::Rate { word, form, … }`): columns the lead
(a line drawn in the view, four; a line's lifted ends or an axis's direction, six; a plane's axes,
six; a circle's centre, radius and plane, ten; a centre of curvature's lift, three), the source's
own, a free distance last; constants
a header (source, word, slot, free, `(m, c)`) and the source's own.  Each **source** says only its
frame (`contact_rate::Frame`: the point, the rate in the slot named, a direction a normal is read
across, the unit normal, two tangent directions and the second fundamental form along them, the
second rate in the slot) and
its derivative in each of its own columns, worked out once a block (`Read`): a spline's
`kernels::spline_rate` (`C′`, `C″` and the basis' derivatives), a curve's `curve_rate` (its
body's frame), a traced body's `traced::frame` (`read`'s exact gradient, a surface's normal and
second order derived over `Dual`s), a meshing's `meshing::frame` (the path of contact by the
implicit function theorem over `Fwd`, a radian of the phase, and the gear's flank's normal), a
carried point's `held::carried` and an envelope's `held::frame` (its normal, and its second order
from the normal's derivatives along the envelope).  The Jacobian linearises that frame over a
pass of `Dual`s (`Read::linear`); the words are written once over `kernels::Num`.  The per-source
tangent, normal, plane and curvature kernels are gone with their kinds.
The five questions the issue left open are settled so:
1. **Partial, never total.**  The frame is the derivative in the contact's own slots with **every
   other column of the drawing held** — the contact's own other unknowns following its rows where
   the source says so (a meshing's lag and places, by the implicit function theorem; an envelope's
   roll, along the envelope) — never the implicit derivative of the whole solution.  So the slot is
   one contact's: `over` names a slot label, not a number; a name in scope (a `param`, a formal
   given a number, a face family's running number) is read as it always was, and a declared
   unknown is E040 ("an unknown of the drawing"); a point no contact holds with that slot, or two
   contacts holding it with one, is E061 at `constraints::seed_omitted`, which infers the carrier
   (`infer::rate_carriers`) — `g tangent(at: p) l` names the curve where two could.
2. **Closed forms behind one surface.**  The words' rows are the source kernels' forms, kept, the
   source choosing (`contact_rate::Form`): a curve's in its view `(C′ × (B − A))/‖B − A‖`
   (`Planar`); a traced curve's tangent the line's direction against two unit directions square to
   `X′` (`Direction`), a meshing's against two square to the path across the world axis it runs
   least along (`Path`), cosines both; a carried point's the rate itself across the line
   (`Velocity`, regular where it vanishes, which `reciprocal` and `Screw` need); a normal's and a
   plane's the direction against two directions square to the normal, a line's in the tangent
   plane against the normal, a curvature by Meusnier, a curve's centre of curvature from `X′` and
   `X″`.  The same rows take the same Newton path, which matters where a solve settles something
   its rows leave free: `paths/frenet.sv`'s plane holds `u` only parallel to the line to the
   centre, so its sense is the path's (a meshing's frame reads its path a degree of the phase, as
   its kernel did, a distance along it turned a radian's in the row).  **"No document changes
   its solved values" means agreement to rounding**, what `tests/examples_sv.rs` holds a document
   to — not bit for bit, which a word of `std.sv` standing for a retired kind cannot promise: its
   rows read the same unknowns in a different order of arithmetic.  Held so against the base,
   the 170 documents of `examples/` (#365): 167 solve bit for bit as they did; `rates/tractrix.sv`
   differs by 4e-22 on values that are zero; `phillips.sv` in the last bit of its screw axis
   `isa` (`isa.py` 57.6 against 57.600000000000001), which `std.Screw`'s rates now place where
   `ScrewAxis` did; and `meshing_problem.sv` by 8e-9 mm.  That last is `reciprocal`'s, not the
   rate's: with `normal reciprocal mesh` read as the distance it solves to, the document solves
   bit for bit as it did, through the 100 iterations its solve stops at (residual 8e-13);
   `reciprocal` as `std.sv`'s word is a carried point and its rate, four rows and three unknowns
   where the old kind was one row, so the stalled solve stops elsewhere — within its own reach,
   the base's own answer moving 3e-7 mm between 100 iterations and 160.  A circle's or an arc's
   tangent at a point (`TangentCircleAt`, `TangentLineCircleAt`, `TangentCircleAt3` in space, the
   radius square to the line, a degree-2 polynomial) stays its own kind: a point on a round owns
   no slot, and the closed form is cheaper and better scaled than a velocity.  What a source has
   no frame for is refused (`validate_rate`): a curve's normal, an envelope's rate.
3. **Second order is the frame's, not a second rate.**  A curve's centre of curvature reads the
   frame's second rate in its slot (`Frame::accel`, a traced curve's `X″`); a section's curvature
   reads the frame's second fundamental form, which each surface source gives (a traced surface's
   `dd·n`, an envelope's from `∂n`); `rate(rate(p over t) over t)` is a syntax error (a rate is of
   a reference, E001), and a second rate as an operand, when one is wanted, will read the same
   frame.  A marched surface's curvature is refused (#339 D4).
4. **Scaling.**  A rate is a length a unit of its slot — a radian of an angle's (`held::carried`
   reads the roll in radians, `meshing::frame` turns a degree's into a radian's) — and its rows
   carry the degree of what they compare (`solver.md`), by the source's form: a carried point's
   rate across or along a line a velocity (degree 1, so a point that stands still, on the axis of
   a screw of no pitch, holds them regularly); a traced curve's tangent and a path of contact, and
   a direction against the normal, cosines (degree 0); a distance along a line, a circle's offsets
   and a centre lengths (degree 1).
5. **Diagnosis reads the statement written.**  `syntax::operator_text` prints a `Rate` as
   `rate(p over t) parallel l`, or by its word at the point (`g tangent(at: p) l`, `S
   perpendicular(at: p) l`, `S tangent(at: p) P`, `S curvature(at: p) k`), and a `RateDistance` as
   `rate(p over t) distance(d, along: l)`; a rate a defined word states reads as the word (`L
   reciprocal mesh`, `Constraint::word`).  A claimed word makes what it declares all the same
   (`flatten::apply`: the `coincident` placing a point the body declares is stated, never claimed)
   and judges the rest.
The retired kinds, fifteen: `SplineTangentAt`, `CurveTangentAt`, `TracedTangentAt`,
`MeshingTangentAt` (`g tangent(at: m) l`, now `word: tangent`), `TracedSurfaceTangentAt`,
`EnvelopeTangentAt` (`in_plane`), `TracedNormalAt`, `EnvelopeNormalAt`, `MeshingNormalAt`
(`normal`), `TracedPlaneAt`, `EnvelopePlaneAt` (`plane`), `TracedCurvatureAt`,
`EnvelopeCurvatureAt` (`curvature`), with #363's `TracedCentreAt` (`centre`, folded as it
landed), and `Reciprocal` and `ScrewAxis` (now `std.sv`'s `reciprocal`
and `Screw`, rates of carried points).  `tests/rate_of_contact.rs` is the gate — a carried point's
rate against the epicycloid's derivative, a spline's tangent written three ways one kind over the
contact's place, a spline's rate held square and measured, a meshing's path written two ways one
statement and measured, the refusals — with `jacobians` (every source and word against finite
differences), `held_envelopes`, `traced` and `meshing`.

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
Jacobian (the variational equations) against central differences, a whole turn closed, the
document's own point found at its place with its claim a theorem and `folded` refused where
`solventc` says, and every refusal of the split.  **The rates series' fronts** (#370,
`tests/rate_examples.rs`): `geodesic_circles.sv`, a front held to a sphere and to a cone by a row
over the moved point alone, its normal placed in the tangent plane — the sphere's circles of
latitude, refused short of the antipode where they focus, the cone's circles about the apex,
unrolled — and `housing.sv`, a Wankel bore as the epitrochoid's parallel curve at the seal's
radius, converging in the pieces, and past the waist's radius of curvature refused at the waist on
the pieces and on twice them.  A fold is refused *early*, by the march's own error, and twice the
pieces do not always bring it nearer: where a front closes on a point or a concave waist the place
is not monotone in the pieces (the antipode 78.06 / 78.25 / 78.17 on 16 / 32 / 64, the waist
14.52 / 13.72 / 13.92), so these gates bound it from below rather than hold it to converge.

**A number the body places (#378, [0.132], `traced::RowKind::Number`):** a dimension of a body
traced in space that reads a coordinate of a point the body declares (`p.x`) is not a tape of `w`
— `w` is read over the outer variables alone, everywhere — but **one more coordinate of `q`**, held
to the expression by a row of the body's algebraic rows: `v − f(x…)`, its columns `[v, x…]` (`x`
the variable-table slots the text reads, outer or `q`, at most `NUMBER_COLS − 1`), its constants
`f`'s tape compiled over those names (`program::traced::Lowering::dimension`; the row the
dimension stands in reads `v` through its free twin where it would read `w`).  One text is one
number.  Its seed is the text over the seeds of what it reads (`expr::Ast::map_vars`).  Numbers are
lowered as relations are, their slots placeholders past the table (`NUMBER_BASE`), and put past
the declared coordinates once the body is lowered (`settle_numbers`, every `w` slot moving on by
as many), so `owner` names each by its text.  Nothing else learns of it: it is an unknown of the
body like any coordinate, so **every derivative the integrators read carries it** — a plain body's
Newton and its Taylor series (`number_jets`, the tape over jets exact to the third order, as `w`'s
is), over one formal a coordinate of the state integrated through its row differentiated (one
more row of `M`, so a speed `x/L` along a line is `x₀e^{s/L}`), over two formals a **placed**
coordinate (`A`: no rate row reads its rate), solved at every node with the rest of `A` and
differentiated by the implicit function theorem in the variational equations.  `rates::Row`
carries it as its `RowKind` (#377, below): `RowKind::at_widths` its kernel at the row's widths,
`RowKind::jets` its row over jets, `Row::checked` its shape (`number_shape`), judged once as the
body is read.  Over two formals `initially` cannot read one (it is the Cauchy data `A` is placed
after) and a relation reading one must place another placed point or be a rate row
(`Lowering::marched`: a row over moved points alone with a number would be placed and
differentiated at once), each **E103**.
`tests/placed_numbers.rs` is the gate: a speed along a line growing as its distance (the
exponential, a held point at `L ln 3`, its contact's Jacobian), a saddle curve with no rate (point,
tangent and a tangent line exact), the fish-eye's front holding a point at the closed form's
optical length with its contact's Jacobian through the variational equations, and the two
refusals.  **The eikonal in a graded medium** (`rates/fisheye_fronts.sv`, `tests/rate_examples.rs`):
a front from a point source in Maxwell's fish-eye (`n = 2/(1 + r²/R²)`, R 50) moving at `1/n` along
its normal, from the closed form's circle at `s = 2` — every front the circle `cos(s/R)(x² + z²) +
2Rx + R²cos(s/R) = 0` (the lens's sphere's geodesic circles, projected) to 1e-7 through `s = 150`,
past the line `x = 0` where the circle turns inside out; refused at 155.2, short of the source's
conjugate at `πR` = 157.08 where the fronts refocus; and `fisheye.sv`'s free-curve ray crossing
every front square to it (1e-6), at the optical length the closed form gives its point.

**A point a motion carries, in a body that states a rate (#377, [0.131]):** `X coincident(under:
m, t == τ) x` is a row like any of a rate body's, over one formal or two: `program::traced`'s
`Lowering::carried` pushes it where its statement is (the body's rows or `initially`'s), and
`rates::Row` carries a `traced::RowKind` rather than a kernel id — `RowKind::at_widths` its
kernel (`screw::kernel` at the row's widths), `RowKind::jets` its rows over jets (`screw::rows`
written once over `Num`, so the carried point's Taylor form is the row itself, exact to every
order), `Row::checked` the shape both integrators check (the motion's steps read back, `7 +
steps_width` columns).  So every derivative the integrators read — the rows differentiated along
the formal, the flow's Jacobian by polarisation, the higher orders, and the home's sensitivity to
every `θ` column by the implicit function theorem (`lines::Body::cauchy`, `rates::Body::home`) —
reads the carried row as it reads a kernel's, the motion's axes and the carried point among the
columns.  A point on a spline is still refused in a rate body (`Row::checked`) but as a marched body's
Cauchy data (#376, below).  Gates:
`tests/lines.rs` holds `geodesic_circles.sv` (each starting circle a drawn point turned about the
axis) and `housing.sv` (the apex the rotor's motion carries) to the same documents with their
Cauchy data written as ordinates — under 1e-11 at the home, 1e-8 at d = 70 on the sphere, a
hundredth of the error against the closed form — and a point held on the bore exact in the apex
and the motion's centres against central differences; `tests/traced_rates.rs` a helix, a point
running up an axis carried about it in the body's rows and turned 30° in `initially`, against its
closed form, its contact's Jacobian against differences.

**Cauchy data on a drawn curve** (#376, [0.133], `program::traced::Drawn`, `Lowering::on_drawn`):
a marched body's `initially` may hold a moved point on a curve the drawing has, `p coincident(t
== u) k`, `k` an entity formal — `curve` takes a spline, a curve of a plane (`EntKind::Curve`: a
formula, a trace, an envelope, a free curve) or a curve traced in space; `spline` a spline alone
— and `t` pinned over the formals (`pinned`, a tape in `w`).  It lowers to one row of three
residuals: a spline's `RowKind::Spline` over its control points' lifts; a curve of a plane's
`RowKind::Curve` (`point_on_drawn`: the definition's own contact kernel at `[0, 0, t, θ…]` gives
`−C` and its gradient, lifted through the plane's basis, which the constants carry and so must be
held — `Sketch::plane_fixed`, E103 otherwise; a curve standing for a surface or a ring's turned
copy is refused); a curve traced in space's `RowKind::Traced` (the `point_on_traced` contact, its
second place a zero in `w`, its constants the curve's `consts_of`, read once it is compiled — a
surface is refused).  The body's `θ` takes the curve's columns (`traced::columns`, `theta`,
`lifted`: the control points' lifts, `Sketch::entity_params`, the traced curve's own), so a
contact on the surface moves with the curve's geometry.  **These rows have no Taylor form**: they
are Cauchy data alone (`RowKind::drawn`; `Body::new` refuses one anywhere but a two-formal body's
`initially`, and `rates::Row::checked` where it is not told `cauchy`, as `lines::Body::plan` tells
the initial rows alone), as wide as their curve (the integrators' row buffers give way to a
vector past `COLS`), and the home's implicit-function step reads them **to the first order**
(`RowKind::jets` → `first_order`: `J·v`, every higher order NaN so nothing can take one for a
number) — all `cauchy` reads.
A curve's constants (a trace's home and pose, an envelope's roll) are read when the body is
compiled; its columns are live.  Any other relation reading `k`, or a point on it outside a
marched body's `initially`, is E103 naming it.  `tests/drawn_cauchy.rs` is the gate: `rates/bore.sv`
(the apex's envelope) marches to `housing.sv`'s (the apex the motion carries, #377) to 5e-12 and
folds at its waist at the same place on 16 and 32 pieces; `rates/pitch.sv`'s pitch curve marched from the heart
cam's profile is the envelope of the roller's centre to 8e-10, and a drawn three-lobed cam's
roller of 22 is refused at its hollow (u = 60°, radius 19.2) at 17.7 / 18.3 on 16 / 32 pieces;
a spline's front is its parallel curve, a traced circle's its concentric; a contact on the pitch
curve has its Jacobian in the profile's columns against central differences; and the refusals.

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
- **A spline over points in space is a curve traced in space** (#363 S3, [0.126]): route (b) of
  the issue, a representation and never a meaning.  `entities::splines_in_space` (in `holds`,
  once every membership is in, before E060 is asked) makes a spline with a control point no plane
  reaches a `TracedE` over the control points (`args`), its body one row of
  `traced::RowKind::Spline` — `X − Σ Rᵢ(u)·Pᵢ`, columns `[u, P₀ … Pₙ₋₁, X]`, constants the knots
  then the weights (`spline_shape` checks them as any encoding is read), its Newton rows
  (`spline_res`/`_jac`) and its jets (`spline_jets`: each basis function the cubic of the jet's
  offset from `curve::basis`'s value and three derivatives, exact to the series' order) — over
  the spline's domain, home at its middle.  The name moves to it (`res.of`, `SourceMap::moved`)
  and the spline stays, no index moving, but is no primitive (`Sketch::spline_traced`): neither
  drawn nor grafted.  So its contacts are a traced curve's (`t == u` pins its place, S1), it is a
  `path`, and `spline(through: …)` over corners in space lowers as it does in a plane, its
  relations the ones in space.  A face over it is E080 ("a curve in space, drawn in no plane").
  Drawn as an axis is, in space and in no view: `overview::scene3d_of` carries every curve traced
  in space in its datums layer as `overview::traced_polyline` — its point read over the domain,
  sixteen spans each halved (to depth 8) where the curve strays from its chord past
  `curve::flatness` — and the app strokes it in 3D (`box3d.ts`'s `IN_SPACE`).
  `a_spline_in_space_is_drawn_in_the_box`.  `tests/space_paths.rs`: a Bézier
  over four points in space — point, `C′`, `C″` and length against Bernstein's to 1e-12, a point
  pinned and a tangent held, every contact against finite differences — a tube along it at 9π ×
  its length to 1e-9, and `spline(through: …)` over corners in space (natural; along lines in
  space) control point for control point against the cubic solved apart (L7's method), over a
  spread of end tangents.  **An end held along a tangent** is built over a line from the corner
  to a point as far off as the next control point (`#run`, `equal` to `#span`, `parallel` to the
  tangent), the foot on it and the next control point nothing along it from the foot: a line
  from the corner to the foot, as through 0.127, lost its direction as the foot crossed the
  corner and the solve stalled there.
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
  No curve family: `path := trace(leg.toe over theta in (0, 360))` asks a *drawn* instance, and
  `e := trace(Involute(base, phase: a0).p over u in (u0, u1))` one written in place and never drawn.
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
  gear on traced flanks, and the same parameters asked in three orders (guarding the resume).
  A point of the drawing followed along a declared parameter (`trace(m over t)`) a relation holds
  to is lowered to a locus too, its block the drawing's moving part (`followed.rs`,
  [parameters.md](parameters.md), #398 A); `tests/jansen.rs` holds Jansen's leg, so followed,
  against a circle-intersection model at 24 angles.
- A curve is *geometry*, laid out in the core and only stroked by the front end:
  `curve::tessellate` refines to `FLATNESS_PX` through `unit`, and `curve::closest` is the pick
  test and a fresh contact's seed.  No binding evaluates a basis function or writes the degree:
  `report::registry_json` publishes `curve.minCtrl`, matching `Sketch::spline_with`.

- **A body traced in space** (#326, [0.107], [0.108]; `traced.rs`, `program/traced.rs`) is
  `EntKind::Traced` (`TracedE`: the body, the entities its component was given — points, lines,
  motions — its constant numbers, the formals that run (`TracedE::over`, one `model::Running`
  each, [parameters.md](parameters.md); `TracedE::domain` packs their ends) and its home), built
  where a curve declaration
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
  column — exact.  A point on it (`PointOnTraced`, `traced::Contact::Point`, `KernelKey::Traced`) owns a place
  `[u, s]`, a curve's `s` pinned at the add; what is read at a point it holds — a curve's tangent,
  a surface's normal, a plane or a line touching it, a section's circle — is its frame, a `Rate`
  of it (#365, `traced::frame`, above), sharing its place (`tangent_at`, `holds_on`);
  `relations::traced_contact` reads `tangent` against a surface as its tangent plane and refuses a
  surface's words of a curve.  Columns `[lead…, u, s, θ…]` (`traced::columns`: lifts of points and
  line ends, then motions' axes); constants `[n_values, values…, home×2, body…]`.  Seeds:
  `traced::nearest`.  `report::traced_text` says regular or where it folds
  (`traced::regularity` over a 13×13 grid).  `tests/traced.rs` is the gate: a helix by its
  ordinates and by a screw, a point and a tangent held to it; a torus's point, normal and the
  meridian's osculating circle against closed forms; every contact against finite differences;
  a fold reported.
  **A curve in space is a path** (#363 S1, [0.122]): a `path` formal takes a body over one
  formal (`relations::kinds`, `traced_curve`; a surface is E103, "not a surface traced in
  space"); `p coincident(t == s) k` pins the contact's first place, `t` read as the slot `u`
  (`Written::slot_of`), to a held running number as a spline's is (`contact_carrier` names the
  traced curve, so `shared_on_one_curve` and the held place allow it); `hint(at: p, along: k)`
  steps along `d[0]` at `traced::place_of` (`entities::rate_along`); and a family's part keeps
  what a traced body was given though the body owns no parameter (`Part::around_all`), else the
  part dropped its contacts.  `tests/space_paths.rs` is the gate.
  **A curve's centre of curvature** (#363 S5, [0.125]): `k curvature(at: p) c` of a point is a
  rate of the contact holding `p` (#365, [0.130]: `CKind::Rate`, word `centre`, three rows, its
  lead the point's lift; `traced::frame` gives the frame's second rate `X″` under
  `Need::Curvature`): `c − (X + |X′|²·(b × X′)/|b|²)`, `b = X′ × X″` floored at
  `contact_rate::FLATTEST²`, so where the curve has no curvature the rows stay finite and lose
  their rank.  Of a surface it is E040
  (`relations::traced_contact`), as a surface's normal, plane or section of a curve is.
  `std.Frenet(g, s:)` is the word over it (`solids.md`); `tests/traced.rs` holds the contact to
  finite differences, `tests/space_paths.rs` to the helix's closed form.
  **Two design constraints it is built to** (#326): a traced surface is held to and read, not a
  face of a solid — exporting one is making it a wall of #243 L2's `Surface::Family` (whose
  `d1`/`newton` it would answer through `traced::read` and `traced::nearest`), not a second
  surface type; and the variable table puts the running formals first and reads the body along
  any direction of them, so a body that reads a rate along one (#339 D2, D4: the drawn instance an
  initial value, the march by the method of lines over the other) is one more column of the same
  table and one more order of the same series.  Over two formals such a body is `lines.rs`'s
  (above), read through the same `traced::read`, its contacts the same kernels.
