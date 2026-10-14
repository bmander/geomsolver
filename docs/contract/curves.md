# Curves

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `curve.rs`,
`locus.rs`, `tape.rs`, `variational.rs`, `extremal.rs`. An issue number (#n) or a spec version
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
first in one variable, so second derivatives only). Integrated by Gragg–Bulirsch–Stoer
(`extremal/flow.rs`, seven columns) with the variational equations through the same substeps; the
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

- **A body traced in space** (#326, [0.105], [0.106]; `traced.rs`, `program/traced.rs`) is
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
  column — exact.  The contacts (`traced::Contact`: `PointOnTraced`, `TracedTangentAt` of a curve,
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
  table and one more order of the same series.
