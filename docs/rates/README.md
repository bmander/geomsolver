# Rates along a running formal

The record of #339, the epic that gives Solvent a way to state a rate along the parameter a body
runs over: ODEs, and first-order PDEs by the method of lines. Each item adds its section here as
it reports.

**The rule every step is held to** (#243's, restated for rates): what a rate *means* is written in
Solvent. Rust grows only the integrator, the rate's semantics, and fast paths held to the general
evaluation.

## D0: the target programs (2026-10-14)

**Verdict: the bet holds.** Nine programs state their curves, frames, fronts and contact lines
as a body's ordinary relations, plus **rate rows**: relations that read a rate. The home is the
initial value. Each program needs only a few new pieces of language:
- `rate(x over s)`;
- `initially { … }`;
- a body over two formals, the first one marched;
- for the gear contacts, #326's R2, R4 and R5 and a `relative_to:` on a rate.

**Every closed form checks out against its formulation.** The rate rows were written out
exactly as the programs state them. At every place, the differentiated algebraic rows and the
rate rows were solved as one linear system and integrated from the home
([`d0/experiments.py`](d0/experiments.py), numpy only). The ODEs and the two gear contact lines
land on their closed forms to 1e-12–1e-13 (table below), and the offset front converges as
decision 4 says. At the places probed, each also lands where the probe in today's spelling
stands.

The programs are in [`d0/`](d0/). Nothing in the repository reads them yet (`rust/examples/` is the
only directory the suites sweep), and each stops at its first new construct:
- five stop at their first rate row ("more on this line than the statement wanted");
- `offset_front`, `helical_contact` and `spherical_contact` stop at `initially` ("a statement
  relates two things with a word between them");
- `formate_contact` stops at R4's `under`.

The probes are in [`d0/places/`](d0/places/), one per program except `formate_contact`. Each
draws the body without its rate rows, in today's spelling, at two places:
- **the home**, with what holds there alone. Deleting those statements leaves exactly as many
  freedoms as the program has rate rows.
- **one place away from it**, at the closed form. In the plane, the closed form is drawn as
  today's computed curve, and the rate rows are *claimed* there through today's exact contacts
  (`tangent(at:)`, `curvature`). Those claims are theorems.

They run: `solventc docs/rates/d0/places/tractrix.sv`.

| program | what it states | rate rows | closed form | probe (today's spelling) | integrated (E#) |
|---|---|---|---|---|---|
| [`tractrix`](d0/tractrix.sv) | a point dragged by a string `a` long, its end running along a line | 1 | x = a(u − tanh u), y = a sech u | home DOF 0 (1 without it); at u = 1.2, string length and tangency claimed: theorems | E1: 3e-13 at u = 1.2 and 3 |
| [`clothoid`](d0/clothoid.sv) | κ = s/A², A = 50 | 3 | Fresnel integrals | home DOF 0 (3 without); at s = 80, the Fresnel series drawn as a curve, its tangent and its osculating circle (R A²/s on `nl`) claimed: theorems | E2: 9e-13 at s = 80 and 100 |
| [`rmf_helix`](d0/rmf_helix.sv) | a frame along a helix (a 10, pitch 5) with U′ ∥ T | 1 | turns −2πb/√(a² + b²) a turn against Frenet's | home DOF 0 (1 without); at 270° the frame at the closed-form turn, DOF 0, f.u = (−0.0290, −0.9309, 0.3640) | E4: same f.u; turn −0.373818 at 270° (6e-14), −0.498424 rad (−28.558°) a turn |
| [`geodesic_cone`](d0/geodesic_cone.sv) | a curve on a cone (half 30°) whose curvature lies along the normal | 4 (rank 3 on p′, 1 on q′) | a straight line unrolled | home DOF 0 (3 without); at s = 30, p = (7, 24, 43.30), tangent at acos(s/ρ) to the generator, DOF 0 | E5: p at 2e-13; T = (−0.684, 0.512, 0.5196), the probe's |
| [`catenary_flow`](d0/catenary_flow.sv) | the chain's force balance, the far end on `b` | 2 | today's free curve | DOF 2: `c` and `v`, the far end's to settle; the closed form claimed through the flow's tangent and through today's free curve at two places: theorems | E3: shot to c = 30.823647, v = −75 (2e-12) |
| [`offset_front`](d0/offset_front.sv) | an ellipse (40, 25) moving inward at unit speed square to itself | 2 (law and gauge) | E(u) + d·n(u), fold at b²/a = 15.625 | at d = 12, u = 50°: the path d long and square to both curves: theorems | E6: below |
| [`helical_contact`](d0/helical_contact.sv) | a contact line in the plane of action of parallel axes (rb 20, 40; αt 25°; βb 15°) | 3 a node (law, gauge, in P) | translates rb1 a radian; the involute helicoid | the plane of action from the pinion, tangent to the gear's base cylinder claimed: theorem; the line at 20°, DOF 0 | E7: 6.981317 at 20° (rb1·φ), plane and straightness 5e-13 |
| [`spherical_contact`](d0/spherical_contact.sv) | the same on intersecting axes (20:40, α 20°), through the apex | 3 a node | turns sin δb1 a radian; the spherical involute | the plane of action from the pinion's base cone, tangent to the gear's claimed: theorem; the line at 30°, DOF 0 | E8: 0.220039 at 30° (φ sin δb1), straight through the apex 6e-14 |
| [`formate_contact`](d0/formate_contact.sv) | #289's pair (90°, 25 mm): the surface of action read off the formed gear, one contact line marched | 3 a node | `formate.sv`'s generated pinion flank | not probed: no statement of a surface of action exists today (needs #326 R2, R4, R5) | not integrated (D5's gate) |

### The vocabulary the programs assume

```
rate(p over s) parallel tl                       // p moves along tl: one row in the plane, two in space
rate(p over s) distance(1, along: tl)            // its component along tl is 1 (unit speed)
rate(f.u over s) perpendicular f.v               // an axis's rate: U′ has no part along V
rate(q over s) parallel O                        // in space, parallel to a plane: one row
rate(p over u) angle(90deg) nl                   // directed, in a view: nl is the inward normal
rate(x over phi) perpendicular rate(x over sigma)   // two rates of a body over two formals
rate(x over phi, relative_to: pinion) perpendicular nl   // x's rate as the pinion's turn sees it
initially { o distance(a, along: up) p }         // holds at the home alone
Front(c, a: 40mm, b: 25mm).p over (d in (0mm, 15mm), u in (0deg, 360deg))   // the first is marched
```

### The five decisions

1. **How a rate is spelled: `rate(x over s)`, which always names its formal.**
   - **Why not the others.** `x'` cannot be used: `'` is the foot mark (`1' 6 3/16"`; the lexer
     takes a quote as a unit), and `p''` reads as inches. `d(x)` would take the name `d`, which
     is a formal throughout `std` (`on_u(d)`, `HalfSpace(…, d:)`, `Offset(S, d:)`), and a name
     after a built-in is W112. `over` already means "as this formal runs" (`… over u in (…)`,
     `integral(… over p)`).
   - **Why the formal is always named, even in a body over one formal.** A component does not
     know which of its formals runs; the call says so. So `rate(x)` alone would mean nothing
     inside a component.
   - **Two formals** use the same spelling: `rate(x over phi)`, `rate(x over sigma)`.
   - **What a rate is.**
     - Of a point, its velocity: a vector of dimension [x]/[s]. That is dimensionless along an
       arc length and a length per unit of a `Scalar` formal.
     - Of an axis (a plane's `u`, `v` or `normal`), the rate of its direction.
     - Its dimension is checked like any other: `rate(q over s) distance(10mm * s / A^2, along:
       nl)` is dimensionless on both sides.
   - **Where a rate stands:** wherever a direction is read. That is `parallel`, `perpendicular`
     (with a line, an axis, a plane or another rate), a directed `angle` in a view, and
     `distance(k, along: l)` for its component along a drawn direction.
   - **Where it does not stand:**
     - never as a place (`coincident`);
     - never as a magnitude (`length(1) rate(p over s)`): that row is quadratic in the rate, so
       it would leave a root to choose at every step. The speed is said as a component along a
       drawn direction instead (decision 3).
2. **How initial data is said: the home is the drawn instance, and `initially { … }` says the
   rest.**
   - **What is said of a drawn instance outside its body holds at the home alone.** That is the
     natural place, and four programs need nothing else:
     - the tractrix's string starting square to the line;
     - the clothoid's start;
     - the cone geodesic's start;
     - the catenary leaving `a`.
   - **Today's traces already accept a drawn instance whose running formal is bound** (`Drag(o,
     a: 30mm, u: 0)` traced `over u`, tested), anchored at the drawn pose. The probes show the
     count: without the home statements, each body is left with exactly as many freedoms as it
     has rate rows.
   - **`initially { … }` in a body holds where there is no sheet to say it:**
     - a word's own initial data (`Minimal`'s `f.u perpendicular up`, D3's std word);
     - an instance written in place (`Dragged`), whose home is the call's value or the
       interval's start;
     - **Cauchy data over a second formal** (`offset_front`'s ellipse, and the contact lines). A
       drawn instance stands at one `u`, so nothing outside the body can say "for every `u`,
       at `d = 0`".
   - **Why not `when`.** `when` is decided while elaborating, and the home is a place in the
     solve. Rows in `initially` are rows of the home's solve and nowhere else.
   - **Which formal is marched.** In `over (…, …)` the first formal is marched and the second is
     sampled. `initially` holds where the first starts.
3. **Whether a rate relation is ever explicit: no.**
   - Every rate row is an ordinary relation that reads a rate, and the index check treats them
     alike. Solvent has no equations between numbers.
   - The explicit form `x′ = f(x)` is just the case where the rows name the rate whole:
     `parallel` together with `distance(1, along:)` *is* p′ = T.
   - **A rate of a number is not written.**
     - A law in the formal that integrates in closed form is written integrated. The clothoid's
       angle law, `datum angle(s^2 / (2 * A^2) * 1rad) tl`, is a quadrature; this is #243 L0's
       finding 10.
     - A number that truly integrates is carried by geometry the body draws (the clothoid's
       turning tangent).
   - **Every rate row the nine programs need is linear in the rates.** Each is an incidence, or
     a component along a drawn direction. So the epic's "a linear system in the rates" holds, and
     no step chooses a root. Refusing magnitudes (decision 1) keeps it so.
4. **Method of lines or characteristics for D4: the method of lines, on L2's pieces, checked by
   characteristics.** Measured on `offset_front` (E6, an arc of the ellipse over u in (−60°,
   60°), marched to d = 12, 77 % of the way to its fold):

   | σ representation | error at d = 6 | at d = 12 |
   |---|---|---|
   | quintic pieces (six Gauss–Lobatto nodes), 8 / 16 / 32 / 64 pieces | 5e-5 / 1.5e-6 / 5e-8 / 1.6e-9 | 1.6e-3 / 9e-5 / 2e-6 / 6e-8 |
   | one polynomial, 16 / 32 / 48 / 64 Chebyshev–Lobatto nodes | 1.4e-3 / 5e-7 / 3e-10 / 8e-10 | 0.19 / 0.012 / 1.3e-3 / **0.46** |
   | characteristics (each node carries its normal) | — | 1.7e-13 |

   - **One polynomial over σ is not stable.**
     - The semi-discrete flow's fastest mode grows at a rate proportional to the number of
       nodes:
       - 0.9 and 1.9 per mm at the home, for 32 and 64 Chebyshev nodes;
       - 0.5, 1.0 and 1.5 per mm for 32, 64 and 96 Fourier nodes on the whole, closed ellipse.
     - The front's own rate is its greatest curvature: 1/(b²/a − d) = 0.064 per mm at the home
       and 0.28 at d = 12.
     - So adding nodes eventually makes the march worse. The growth comes from the
       discretisation, not from the ends: the periodic case has none.
   - **L2's pieces keep the spurious growth bounded and converge.**
     - The growth is 0.24–0.31 per mm at the home and 0.8–1.3 at d = 12, the same at 4, 16 or 64
       pieces.
     - The error converges at about fifth order in the piece length.
     - That growth is still about five times the front's own, so a representation error is
       amplified by orders of magnitude: on 16 pieces, a rate 9e-10 off at the home leaves the
       front 9e-5 off at d = 12.
     - **D4's march must therefore be checked against the same march on twice the pieces**, as
       L2 checks a piece against one more solve. A march has no solve between nodes to check
       against.
   - **Joints.** A joint node shared by two pieces has two derivatives. Their mean serves (it is
     what E6 uses).
   - **The gauge is also the law that places the nodes.** "Square to itself" lets nodes bunch
     where the front focuses.
   - **Characteristics.** For the offset front they are exact and need no σ-rate at all
     (x_d = n, n_d = 0). That makes them D4's check on the scalar case, as the epic proposes.
   - **The fold.** Refused where a node's |x_u| falls below 1e-2 of its home value: at d = 15.12
     on 32 pieces, against 15.47 for the closed form. The march's own error grows as the front
     focuses, so a regularity read off the nodes errs early. D4 should state its threshold
     against the refined march too.
5. **Whether any target is stiff: no.** Measured as the largest |λ| of the flow's Jacobian times
   the interval:

   | target | stiffness |
   |---|---|
   | tractrix | 2.3 |
   | clothoid | 2.0 |
   | catenary | 0 (its rates read no state) |
   | rmf_helix (a turn) | 6.3 |
   | geodesic_cone | 2.5 |
   | offset_front on 16 pieces (15 mm) | 4.5 |
   | spherical_contact | 0.6 |

   The method of lines' spurious modes have positive real parts. That is growth, not stiffness:
   an explicit extrapolation integrates them faithfully, and bounding them is the
   representation's job (decision 4). A stiff flow stays refused.

### Findings

1. **The index check is a rank, and it holds at every place the targets reach.**
   - `geodesic_cone`'s `p′` is held by four rows of rank three: three rate rows, plus `p` on the
     cone differentiated, which the tangency already implies.
   - `rmf_helix` says its parallel as the one row a frame's unit length does not imply.
   - The tractrix's home is a cusp, where the speed is zero, yet its four rows have rank four
     there.
   - Every experiment solves its rate system at every step, without a rank loss.
   - A body short of rate rows is visible already: without its gauge, `offset_front`'s rate rows
     at a node are one row of rank one in two rates. The report should name the freedom:
     "`rate(p over d)` is free along the front: state a gauge".
2. **Today's traced body refuses a body one row short:** "a trace block must determine its
   points: 4 unknowns against 3 equations" (E103, `Drag` traced as it stands). D2's squareness
   counts rate rows; that is the one place where the existing check changes.
3. **Solve what the algebraic rows fix; integrate only what the rate rows reach.**
   - Some rows read a rate along the sampled formal: `offset_front`'s `nl`, the contacts'
     normals. They are algebraic at each step. Differentiated along the marched formal, though,
     they read x_σφ = D·x_φ, which couples a piece's nodes into one linear system.
   - Two routes:
     - integrate every coordinate, through the coupled system of each piece;
     - or re-solve the coordinates the algebraic rows fix at each step, from where the last step
       left them (as a trace solves its instance today), with their sensitivities by the
       implicit function theorem. Only the coordinates a rate row reaches are integrated.
   - **The second is recommended.** It also removes drift. Integrated through its
     differentiated rows, the tractrix's |q − p| = a drifts by the integration error (3e-5
     after 30 RK4 steps, 3e-9 after 300; E1). At the 1e-13 budget that is harmless, but it is
     not zero.
4. **A word is handed the running formal.**
   - Scope is closed, so `Minimal` (D3's word) cannot see `phi`: it is called `Minimal(…, s:
     phi)` and rates over its own `s`, as `std`'s station words take `s` today.
   - `rate(x over s)` with `s` bound to an expression in the running formal is the rate along
     that formal divided by ds/dφ.
   - A rate over a formal bound to a number is refused, except in a drawn instance at its home,
     where no rate is a row.
5. **Condition (1) of #326 vanishes where the relative motion has no pitch.**
   - With parallel or intersecting axes, the relative motion turns about the pitch line, where
     v₁₂ = 0 (E7, E8: 4e-15). Every helical contact line crosses it, and the spherical home line
     *is* it. So a normal stated by (1) and (2) together (`nl reciprocal mesh`, `nl
     perpendicular rate(x over sigma)`) loses rank once a line, or along a whole line.
   - Both programs therefore state the normal by the surface of action (in the plane of action)
     and *claim* (1). It holds to 7e-13 and 7e-14.
   - On skew axes (Formate) the relative twist has a pitch, so v₁₂ vanishes nowhere and (1) can
     state the normal. D5 must accept both statements of the normal.
6. **`rate(x over phi, relative_to: m)` states condition (3) without carrying a normal.**
   - It is #326 R4's carried point, differentiated: x_φ − vₘ(x), in the fixed frame.
   - Stated so, together with the in-surface normal and the square gauge, the march moves the
     helical line rb1 a radian (6.981317 at 20°) and turns the spherical one sin δb1 a radian
     (0.220039 at 30°). Straight and in the plane, to 5e-13.
   - R4's `under(…, at:)` is still needed, to read the flank as the member sees it.
7. **A two-point problem is a drawing.** The catenary flow's `c` and `v` are unbound formals; its
   far end on `b` settles them by shooting, to 2e-12 in E3 (its Jacobian is differenced there;
   D2's comes from the variational equations). That is §9.10's free curve stated as a flow, and
   D2's contact gradients are what make it solvable.
8. **Side finding: `std.OnHelix`, drawn by itself, stands square to the mirror helix.**
   - Its lean is an unsigned angle in space (`n.v angle(lead) sc.n.v`), so the seed picks the
     hand. `std.OnHelix(std.z, std.top, r: 10mm, pitch: 5mm, turns: 4, s: 0.5)` gives normal
     (0, −0.9968, 0.0793) at (10, 0, 10), where the helix's tangent is ±(0, 0.9968, 0.0793):
     9.1° off.
   - The face family's spring carries the right root: `solventc` builds 1782.144915 mm³, the
     closed form A·L, where the mirror lean would give A·L·cos 2λ = 1759.716 (checked by
     quadrature). So nothing that is gated is wrong.
   - But a station drawn alone is, and a bound, or a lean stated with a sign, would fix the word.
     The probe and the target write the helix out instead.
9. **Formate's tolerance (D5): 1e-7 mm on the marched flank's points against the held envelope,
   and 1e-7 rad on its normals, with `meshing` reporting line contact.**
   - The held envelope is solved to about 1e-12.
   - The march's budget is the integrator's 1e-13 plus the σ representation. On L2's pieces E6's
     front is at 2e-6 / 6e-8 at 32 / 64 pieces, but that is 77 % of the way to a fold; the
     admitted Formate flank has none (least area factor > 0.1).
   - 1e-7 mm is about 4e-9 of the pinion's pitch radius: well below "solved" (1e-6 relative)
     and the export's 0.01 mm, and above what doubled pieces reach away from a fold.

### What D0 hands on

- **D1** (the integrator): no change of scope.
  - The variational equations through the same substeps, and dense output.
  - Finding 3's algebraic solve between steps is D2's, but its sensitivities enter the
    variational equations by the IFT.
- **D2** (rates in a traced body):
  - decisions 1–3: `rate(x over s)`, its operands, refusing magnitudes, `initially`;
  - finding 1's report of a missing row, and finding 2's squareness;
  - findings 3 and 4;
  - `tractrix`, `clothoid` and `geodesic_cone` as gates, `catenary_flow` as the two-point one.
- **D3** (rates in a face family):
  - `rmf_helix`'s `Minimal` as the std word;
  - finding 8's `OnHelix` root.
- **D4** (two formals):
  - decision 4: L2's pieces, joints by the mean, each march checked against twice the pieces,
    characteristics as the scalar check, the first formal marched;
  - the fold refused on the refined march;
  - `offset_front` as the gate.
- **D5** (line contact on skew axes):
  - findings 5, 6 and 9;
  - `helical_contact`, `spherical_contact` and `formate_contact` as gates.

## D1: the integrator (#346, 2026-10-15)

Gragg–Bulirsch–Stoer over a state of any width, with the variational equations through the same
substeps and dense output, is `ode.rs`; the free curves run on it bit for bit. The rule is
`docs/contract/curves.md` ("The integrator"), the report the issue's D1 comment.

## D2: rates in a traced body (2026-10-15)

**Landed:** `rate(x over s)` and `initially { … }` in a traced body (0.119, §6.5; a face
family's are D3's, 0.116, §6.9), a traced body that states a rate integrated from its home in the
plane and in space (`rates.rs`), and the gate `tests/traced_rates.rs`.  The rule is
`docs/contract/curves.md` ("Rates along a running formal") and `docs/contract/language.md`
("`rate(x over s)` and `initially { … }`").

### What landed

- **The spelling is D0's.** `rate(p over s)` is an operand (`syntax::RateOf` on the written
  relation); `rate(p over s) distance(k, along: l)` stands alone before its word; `initially {
  … }` is a block. A component handed the running formal as itself rates over the caller's. Of
  D0's programs, `tractrix`, `clothoid` and `catenary_flow` run exactly as written (`solventc
  docs/rates/d0/clothoid.sv`); `tractrix`'s `written` starts at the interval's start, `-3`, since
  its call gives `u` nothing. `geodesic_cone` does not: a body traced in space is written in place
  over points, lines, motions and numbers (#326 R3), so the gate states its axis as a line and its
  start in `initially`.
- **One integrator, plane and space.** A body is three sets of rows over one variable table: its
  algebraic rows, its rate rows (a rate is a scratch line from an origin to the point's rate, so
  every rate row is an ordinary kernel's row, linear in the rate) and its initial rows. At each
  place the algebraic rows differentiated (their first Taylor order) and the rate rows are one
  system in the rates, `M = [J_q; ∂R/∂q′]`; the flow `q′ = f(u, q)` is integrated by `ode::march`,
  the free columns as constant states, `Φ` seeded with the home's sensitivity to every column.
- **The index check is a rank, at every evaluation** (D0 finding 1). In the plane the system must
  be square by count; in space a `parallel` is two rows and the body's own rows may imply one, so
  more rows than rates is allowed, read by Householder QR, and they must agree at the home and at
  a reading. A place where `M` loses rank is refused there, and `solventc` says how far each curve
  is integrated: a point whose speed along `right` is 1 on a circle of 30 is refused at ±30.
- **A body short of a rate row** (D0 finding 2) is refused naming the freedom: the under-determined
  block of the rate system's structure, so the clothoid without its curvature row says *"its rates
  leave 1 freedom among those of `q`, `k`"*.
- **Exact derivatives, nothing differenced in the flow.** The flow's Jacobian is `−M⁻¹ ∂G/∂z`, the
  algebraic rows' part a mixed second derivative read by polarisation over Taylor jets. `C′` is the
  rate, `C″` and `C‴` the solution's Taylor coefficients (each one more solve with `M`), `∂C/∂θ` and
  `∂C′/∂θ` the flow's sensitivities. Only `∂C″/∂θ`, which a curvature contact's Jacobian alone
  reads, is a difference — of the local map along `Φ`'s column, no march — as a trace's gradients
  along θ are today.
- **A two-point problem is a drawing** (D0 finding 7): the hanging chain's pulls, unbound formals,
  are solved by its far end landing on `b`, the contact's gradient in them the flow's sensitivity.

### The gate, `tests/traced_rates.rs`

| item | result |
|---|---|
| tractrix, drawn (home at the cusp) and written in place from `initially`, both ways | 1.6e-12 / 1.2e-12 over 301 places |
| clothoid against the Fresnel series, both ways over 200 | 9.5e-12 over 401 places |
| contacts on the clothoid: a point at 60, a line along the tangent there, the circle osculating at 80 | the point 6e-13, the tangent's turn `s²/2A²`, the radius `A²/s` 1e-12; the assembled Jacobian against differences before the solve |
| the hanging chain shot to `b` | `c` 30.8236469736 (the root of `c sinh(50/c) = 75`) to 2.4e-13, `v` −75 to 1e-12, the curve 1.4e-11 |
| geodesic on a cone in space (13 rows for 12 rates, rank 12) | 6.3e-12 over 241 places; a point held at a height and the line along the tangent there are the closed form's; both contacts' Jacobians against differences |
| a body one rate row short | E103 naming `q`, `k` |
| a rate as a place, a magnitude, along another formal or an angle, of a point that does not move, two in one row, in `initially`, outside a component | each E103, saying why |
| rank lost on a circle | refused at ±30, to within one substep past it (30.0016); the curve short of it the circle |

**The series.** [`rust/examples/rates/`](../../rust/examples/rates/) holds one document per idiom
— the tractrix drawn and written in place, a clothoid transition eased into its bend, the hanging
chain shot to its far end beside the free curve, a pursuit refused at its capture, a geodesic on a
cone — each with its closed form, and the app's examples list it as one project (*Rates*).
`rust/gcs-core/tests/rate_examples.rs` holds each to it. The D0 programs stay above as the record
of how each was first written.

### Findings, for D3–D5

1. **A drawn home must be on the body's rows, or the solve crawls.** Its pose is the initial value
   and its coordinates columns of the curve, so while the drawing is unsolved the flow ran from a
   frame the body could never be (a clothoid's tangent 1 long turning ten times as fast): 150–400
   steps a march, and a solve of three contacts took 58 s. The flow now runs from the pose's
   nearest point on the body's algebraic rows (`q₀ − pose = Jᵀλ`, `r(q₀) = 0`), its sensitivity
   exact off the rows too (`W = Σ λᵢ ∇²rᵢ` by polarisation); at a solution it is the pose. Seven
   steps a march, 0.5 s. D3's station drawn mid-interval will need the same.
2. **Weights must be smooth where rows past the count disagree.** Off the body's rows, at a
   substep, a `parallel`'s second row and the cone's own disagree, and their least squares weighs
   them by the row scaling. Scaled to their largest entry (a kink), the geodesic read 2e-7 between
   nodes that were right to 1e-11; scaled to unit length, 7e-13 everywhere.
3. **Integrate every coordinate; D0 finding 3's partition is not structural.** The clothoid's `q`
   and `k` are fixed by `p` and the turning, which only the rates reach, so no partition of the
   coordinates into "solved" and "integrated" exists in general. Integrated whole, the algebraic
   rows drift only by the integrator's error, which the closed forms above bound near 1e-12.
4. **A refused march keeps its road.** A march to the end of the interval refused part way is made
   again to where it was refused, so a curve that stops is drawn and read as far as it reaches.
5. **One march serves a curve.** Contacts read a march to the end of the interval on their side,
   remembered by content (every contact carries a copy of the encoding), so a residual and its
   Jacobian, and every contact on one curve, share it; a place past the end doubles the reach, so a
   solve's trial places share marches too.
6. **Not yet read:** a rate along an `Angle` formal (the formal's number is degrees; D3 decides the
   unit), along a second formal (D4), of a direction (D3's `rate(f.u over s)`), and `relative_to:`
   (D5).

## D4: two formals and the method of lines (2026-10-15)

**Landed:** a body traced over two formals that states a rate along either, marched along the
first by the method of lines (0.121, §6.5; `lines.rs`), and the gate `tests/lines.rs` on an
ellipse's offset front, [`rust/examples/rates/front.sv`](../../rust/examples/rates/front.sv).
The rule is `docs/contract/curves.md` ("The method of lines").

### What landed

- **The spelling is D0's**, in today's spatial body: `rate(p over d)` and `rate(p over u)` in
  one body, `initially { … }` as the Cauchy data, `over (d in (…), u in (…))`, the first marched.
  D0's `offset_front.sv` itself does not run: it reads `right` and `up` and a directed `angle` as a
  plane does, and a body over two formals is traced in space, so `front.sv` gives the ellipse its
  axes as lines and draws the normal in its plane and square to `rate(p over u)`.
- **The representation is decision 4's**: 32 pieces of six Gauss–Lobatto nodes, joints by the
  mean, a whole turn whose ends meet closed. A rate along an `Angle` formal is per radian.
- **Solve what the rows place; integrate what the rates move** — D0's finding 3, which D2 set
  aside for one formal. Over two it is structural: a row reading a rate along the sampled formal,
  differentiated along the marched one, ties every node of a piece, so such a row must place a
  point the rates do not move (the normal's end `k`), solved at each node from where it was. The
  state is the moved points alone, and nothing couples two nodes but the derivative along `u`:
  the flow's Jacobian is banded by piece and never formed (`ode::Rate::along`).
- **Exact derivatives.** Each column of the variational equations is a directional derivative at
  every node over the node's first-order picture: `δA` by the implicit function theorem, the rows
  over the moved points by polarisation, the rate rows' first order. A contact's point, tangent
  plane and normal are exact in the place and the geometry; its curvature (a third order) is
  refused.
- **The fold is refused where it happens**: a node's area `|x_d × x_u|` under a hundredth of its
  home's.

### The gate, `tests/lines.rs`

| item | result |
|---|---|
| the front against `E(u) + d·n(u)`, 32 pieces, 241 places at each d | point 3e-12 / 7.7e-9 / 5.1e-8 / 2.6e-7 / 2.0e-6 at d = 0 / 3 / 6 / 9 / 12; `x_d` 1.3e-10 … 1.7e-6; `x_u` 2.8e-11 … 1.8e-5 a degree |
| convergence in the pieces, the worst of 121 places | 8 / 16 / 32 / 64 pieces: 5.1e-5 / 1.3e-6 / 4.4e-8 / 1.7e-9 at d = 6, 1.6e-3 / 6.2e-5 / 1.6e-6 / 6.1e-8 at d = 12: fifth order, as D0's (the gate holds 8 to 32) |
| held to twice the pieces (`lines::check`) | 16 against 32: 8.8e-5, its error 6.2e-5; 32 against 64: 2.0e-6, its error 1.6e-6 |
| the fold (b²/a = 15.625; a hundredth of the area at 15.47) | refused at u = 0 at 14.66 / 15.11 / 15.35 on 16 / 32 / 64 pieces; read at 15, not at 15.5 |
| a point held at the closed form's (9, −25°) | its place found to 1e-7; the contact's and a normal's Jacobians, through the variational equations, against central differences |
| the whole ellipse, u in (0°, 360°) | closed; the same a turn on, to rounding; 1.5e-5 at d = 6 on its 32 pieces, three times the arc's length |
| refusals | a front without its gauge names `p`'s freedom; a rate along `u` of a placed point; one read by a row over moved points alone; `initially` placing `k`; two rates along `d`; a rate along a third formal; one along `u` with none along `d`; a curvature contact |

### Findings, for D5

1. **A space `parallel` in a rate row is read across its line at every node.** Its two rows carry
   directions across one line, stated where the relation is — in a traced body's scratch sketch,
   across a line of no length — and they lose rank wherever the line turns square to them: the
   front's normal does, in its plane. Read across the held line as it stands at each node they
   vanish exactly where the cross product does, so their own derivative, which multiplies it, is
   nothing at a solution. D2's one-formal integrator still states them once; its geodesic is
   generic enough not to care, and a body whose held line turns through the frame would.
2. **Cost.** A march of the front to 15 on 32 pieces (161 nodes, 483 states) is 11 steps and
   about 800 evaluations, 0.75 s; with its sensitivities to 12 columns 2.5 s.
   A node evaluation is about 6 µs, half of it the placed points' Newton (about three iterations:
   a substep starts them 0.1–0.7 mm off). A dense output is order-adaptive (`Path::at_within`),
   and each marched place read is one step, memoised in the march: seeding two contacts by their
   nearest place took 10 s before both, 1.3 s after.
3. **Resolution is per length, not per count.** 32 pieces over the whole ellipse are three times
   as long as over the arc, and the error at d = 6 is 1.5e-5 against 4.4e-8. D5's contact lines are
   short and regular (no fold within the blank), but a count chosen by the interval's features,
   or doubled until the check holds, may be worth its cost there.
4. **The traced point must be moved.** D5's pinion flank is `X`, carried by the pinion's turn from
   the contact `x` — a placed point. Reading a placed point's derivatives along the march (its rate
   by the implicit function theorem along the flow, its sensitivities the same) is D5's to add.
5. **Not yet read:** `relative_to:` on a rate (D5), and a curvature contact on a marched surface.


## Examples for the method of lines (#370, 2026-10-15)

**Landed:** two documents in the rates series, each with its closed form and its fold gated in
`tests/rate_examples.rs`, and `front.sv` given a contact, a claim and a front run past its fold.

| document | what it shows | result (32 pieces) |
|---|---|---|
| [`geodesic_circles.sv`](../../rust/examples/rates/geodesic_circles.sv) | a front on a sphere (R 30) and on a cone (half angle 30°), each from a circle of radius 15 about the axis, held to the surface by a row over the moved point alone, its normal placed in the tangent plane; both a whole turn | the sphere's circles of latitude: 7.7e-10 at the home, under 8e-8 to d = 70; refused at 78.25 (a hundredth of the area at 78.39, the antipode at 78.54). The cone's circles about the apex: 7.7e-10 at the home, under 3e-8 to d = 30 |
| [`housing.sv`](../../rust/examples/rates/housing.sv) | a Wankel bore, the epitrochoid (R 75, e 15) offset outward by the seal's 3; half the bore, the other its mirror | 9.2e-9 at the home, 3.5e-6 at d = 3; on 16 / 32 / 64 pieces 7.6e-5 / 3.5e-6 / 3.0e-7. Run on to 20, refused at the waist (u = 90°) at 13.72 / 13.92 on 32 / 64 pieces, against 14.85 for the closed form (the waist's radius of curvature 15) |
| [`front.sv`](../../rust/examples/rates/front.sv) | `held`, on the major axis 35 from the centre, found on the front; the surface's normal there claimed the plane's; `folded` run to 16 | the place (5, 0°) to 1e-10; the claim a theorem; `folded` refused at 15.11, as `solventc` says |

D0's [`offset_front.sv`](d0/offset_front.sv) is kept as D0 proposed it, its header pointing at
`front.sv` as its runnable form.

### Findings

1. **A contact on a marched surface costs a march with sensitivities to every `θ` column, fixed
   or not.** The kernel is handed every column and cannot tell which the solve holds. On the front
   (15 mm, a 120° arc) that is 1.5 s; on a cone (a whole turn of a circle about a point up a
   generator, two placed points a node) a point held on it took 7–9 s to solve, and on the
   sphere, whose march runs to its fold, 14 s. So `geodesic_circles.sv` holds no contact and its test reads the surfaces. A
   march that carries sensitivities only along the columns the solve moves would make a contact on
   such a surface as cheap as its march.
2. **A long march is not improved by more pieces.** With `wankel.sv`'s own proportions (R 105, e
   15) the waist's radius of curvature is 120, and a march out to it over the whole bore is worse on
   64 pieces than on 32 past d = 90 (2.3e-3 against 1.7e-3 at d = 100): the discretisation's growth
   compounds over the length. The fold, a hundredth of the area at 118.8, was refused at 118.2 /
   115.3 / 109.6 on 16 / 32 / 64 pieces. `housing.sv` takes a short rotor, R/e = 5, whose waist
   folds at 15.
3. **A fold's place is not monotone in the pieces where a front closes on a point or a concave
   waist** — the antipode 78.06 / 78.25 / 78.17, the waist 14.52 / 13.72 / 13.92 on 16 / 32 / 64
   pieces — where `front.sv`'s ellipse (14.66 / 15.11 / 15.35) comes nearer each time. The gates
   bound the place from below, short of the closed form's, rather than hold it to converge.
4. **Resolution per length matters at the seal's radius already.** The whole bore on 32 pieces is
   7.6e-5 off at d = 3, half of it 3.5e-6: pieces twice as long, twenty times the error. A pieces
   count per document would let the bore be drawn whole.
5. **A starting curve is written by its ordinates, not drawn.** A body that states a rate reads
   no point a motion carries (`traced::Body::new`), so the starting circles cannot be a drawn
   point turned about the axis, nor the trochoid the apex carried by the rotor's motion as
   `wankel.sv` carries it: each is written as its own `r·cos u`, `r·sin u` (the epitrochoid's two
   terms). The seeds are plain numbers that pick each root (`nl`'s end at the centre, the south
   pole, up the axis), not the closed form. A point a motion carries in `initially` needs the
   home's sensitivities of its row (a Taylor form of the carried point, which the home's
   implicit-function step reads by jets), and is the nearest of the follow-ups below. *Landed
   in #377, below.*

### What waits

- `spiral_bevel/phillips_pure.sv` as D0's `helical_contact`, each flank generated by the march:
  D5 (`relative_to:` on a rate, a placed traced point).
- ~~`generation/heart_cam.sv` and `conjugate_cams.sv`'s pitch curves as fronts from their
  profiles, undercut a fold, and the Wankel bore read off `wankel.sv`'s envelope: Cauchy data on a
  drawn curve (`p coincident(t == u) k` in `initially`).~~ Landed in #376, below.
- `fisheye.sv`'s and `projectile.sv`'s wavefronts, the eikonal marched: a speed that reads the
  front's own point. The fish-eye's landed with #378 (below).
- The bore drawn whole, and any front over a whole turn of a long curve: a pieces count per
  document, or pieces doubled until `lines::check` holds.

## A point a motion carries, in a rate body (#377, 2026-10-16)

**Landed:** `X coincident(under: m, t == τ) x` in a body that states a rate, in its rows and in its
`initially` (0.131), over one formal (`rates.rs`) and two (`lines.rs`). The rule is
`docs/contract/curves.md` ("A point a motion carries, in a body that states a rate").

- **No new derivative was needed.** A carried point's rows are written once over `kernels::Num`
  (`screw::rows`), so over `taylor::Jet`s they *are* its Taylor form, to every order. What
  refused it was the integrators' row: a static kernel's id. It now carries what the row states
  (`traced::RowKind`), and every read — the rows differentiated along the formal, the flow's
  Jacobian by polarisation, the home's implicit-function step over every column — goes through
  the row's kind.
- **The documents draw their Cauchy data.** `geodesic_circles.sv`'s starting circle is `s`, a
  drawn point 15 from the axis on the sphere (and so on the cone), turned about the axis by `u`:
  `initially { p coincident(under: spin, t == u) s }`, three rows, the surface's a fourth that
  agrees. `housing.sv` draws the motion as `wankel.sv` does — the rotor's centre `e` from the
  shaft's, the apex `R` beyond, the rotor a third as fast against the shaft — and its Cauchy data
  is the apex that motion carries at `t == 3 * u`, the shaft's turn three times the eccentric's
  angle of the epitrochoid's terms.

| item (32 pieces) | the carried data against the written, every 2.5° |
|---|---|
| the sphere's fronts, point / `x_d` / `x_u` | 6e-13 at the home; 8e-12 at d = 20, 2.3e-11 at 40, 2.8e-9 at 70 (the march's error against the closed form there 8e-8); refused at 78.2456 against 78.2522 |
| the cone's | 1e-12 at the home; 1e-10 at d = 30 |
| the bore | 2e-13 at the home; 1e-11 at d = 3; `past` refused at the waist at 13.7196 against 13.7198 |
| a point held on the bore, off the solution | its Jacobian in the place, the apex and both turns' centres against central differences to 1e-6; with every one of its 27 columns, 23 s: a march with sensitivities to every column, #370's finding 1 |
| a helix, one formal: `x` up the axis at unit speed, `X` it carried 3° a millimetre, `x` from `x0` turned 30° in `initially` | 1e-10 over 25 places against `20 (cos(3s + 30)°, sin(3s + 30)°, s)`; a point held at a height there, its Jacobian against differences |

**Not yet:** a point on a spline in a rate body (refused, `rates::Row::checked`), and the traced
point carried in a marched surface: the split still asks that it be moved (D4's finding 4, D5's).

## A speed that reads its own point (#378, 2026-10-16)

**Landed:** a dimension of a body traced in space may read a coordinate of a point the body
declares (0.132, §6.5), so a rate row's speed may read where the point is; the eikonal equation in
a graded medium, [`fisheye_fronts.sv`](../../rust/examples/rates/fisheye_fronts.sv), gated in
`tests/rate_examples.rs`; and the gate `tests/placed_numbers.rs`.  The rule is
`docs/contract/curves.md` ("A number the body places").

### What landed

- **The spelling is today's:** `p.x` in a dimension's text, as `c.x` of a point the component is
  written over already was. `rate(p over s) distance((1 + ((p.x - c.x)^2 + (p.y - c.y)^2 + (p.z -
  c.z)^2) / R^2) / 2, along: nl)` moves the front at 1/n, n read at `p`.
- **A placed number, not a new derivative.** The text cannot be a tape of `w`, which every
  integrator reads over the outer variables alone. It is one more coordinate of `q`, held to the
  text by a row of the body's own, `v − f(x…)` (`traced::RowKind::Number`), the row the dimension
  stands in reading `v`. So nothing downstream learns of it. A plain body solves it with its points.
  Over one formal it is a coordinate of the state, integrated through its row differentiated.
  Over two it is placed at every node with the points the rates do not move, its sensitivities by
  the implicit function theorem in the variational equations. The rates' system and the
  polarised second derivatives carry it as they carry any row; the row over jets is the tape over
  jets, exact to the third order as `w`'s is.
- **Two refusals over two formals:** a number in `initially`, which is the Cauchy data the placed
  points are solved after; and one in a row that places only the points the rates move, which would
  be placed at each step and differentiated along the march at once.

### The gate

| item | result |
|---|---|
| the fish-eye's fronts from `a` = (−R, 0), R 50, against `cos(s/R)(x² + z²) + 2Rx + R² cos(s/R) = 0`, the arc u in (−120°, 120°), 32 pieces | 3e-12 at s = 2, 9e-10 at 40, 1.6e-8 at 100, 5e-8 at 150, through `x = 0` at s = πR/2 where the circle turns inside out; 2e-5 at 155 at the arc's ends, 1e-7 inside |
| the refocus at a's conjugate (R, 0), s = πR = 157.08 | refused at 155.2, a node's area under a hundredth of its home's (the closed form's at 157.06), every node within the front's radius 2.1 of the conjugate at 155 |
| `fisheye.sv`'s free-curve ray, 15 of its points | each on the fronts to 1e-9 by the march's own reading, at the closed form's optical length to 1e-6, the front square to the ray to 1e-6 |
| a speed `x/L` along a line (one formal) | `x₀e^{s/L}` to 1e-10, its rate too; a point held 30 along at `L ln 3`; the contact's Jacobian against differences |
| a saddle curve, `z = x·y/20` on a circle of 10, no rate | point and tangent exact to 1e-10 / 1e-12; a line along its tangent; both contacts' Jacobians |
| a point held on the fish-eye's front (two formals) | at the closed form's optical length to 1e-7, on the axis' ray; its Jacobian through the variational equations against central differences, the centre the index reads among the columns |
| refusals | a number in `initially` over two formals; one in a row placing only moved points |

### Findings

1. **A perfect focus is not resolved by more pieces.** On 16 / 32 / 64 pieces the fronts are 8e-7 /
   1.6e-8 / 2.5e-10 off the circles at s = 100, but 1.3e-7 / 5e-8 / 4.7e-8 at 150, 7e-6 / 3e-6 /
   4e-6 at 154 and 5e-5 / 2e-5 / 3e-5 at 155: near the conjugate the march's own error, amplified
   by the focusing, outgrows the discretisation's. The check against twice the pieces says so,
   1e-3 at 154 and 1e-2 at 155. Only 32 pieces are refused where the closed form folds: on 16 and
   64 the open arc's end pieces spread instead of closing, `x_u` there turns and grows rather than
   vanishing, and no node's area falls below the floor — the march runs on past πR, unrefused and
   wrong (0.1–1 off at 156). A regularity read at each evaluation cannot see a focus the march does
   not reach. (A sign of `x_d × x_u` kept along the branch was tried and does not help: the normal,
   placed square to a vanishing `x_u`, takes its other root at the same step, so both turn.) The
   document is held at its own 32 pieces, as `geodesic_circles.sv` and `housing.sv` bound their
   folds from below.
2. **A node's place errs more than the front's shape.** The check against twice the pieces at s =
   100 is 2e-6 where the fronts are 1.6e-8 off their circles: a node slides along the front more
   than the front leaves its circle.
3. **A space `parallel` held along a world axis loses rank over one formal**, rates or no — D4's
   finding 1, still open in the one-formal integrator: the line of the exponential gate runs off
   the axes.

### What waits

- `projectile.sv`'s fronts: the same law over Jacobi's metric, `n = √(h − z)`, can now be written;
  it has no closed form for its fronts to be held to but its rays.
- A front from a point source written from the point: the Cauchy data is a circle by the closed
  form a short way out, since a front at its source has no normal.

## Cauchy data on a drawn curve (#376, 2026-10-16)

**Landed:** a marched body's `initially` may hold its moved point on a curve the drawing has,
`p coincident(t == u) k` (0.133, §6.5), and two documents that use it,
[`rates/bore.sv`](../../rust/examples/rates/bore.sv) and
[`rates/pitch.sv`](../../rust/examples/rates/pitch.sv), gated in `tests/drawn_cauchy.rs`. The rule
is `docs/contract/curves.md` ("Cauchy data on a drawn curve").

### What landed

- **The spelling is the issue's.** `k` is an entity formal of the body, `k: curve`, given a
  spline, a curve of a plane (a formula, a trace, an envelope, a free curve) or a curve traced in
  space; `t` is pinned over the formals as any place is (`t == 3 * u` reads the apex's envelope,
  whose parameter is the shaft's roll, at three times the eccentric angle).
- **Three representations, one statement.** A spline is read over its control points' lifts by
  #363 S3's row; a curve of a plane by its definition's own contact kernel, lifted through its
  plane's basis (constants, so the plane must be held); a curve traced in space by its contact.
  The body's columns are the curve's, so a contact on the surface moves with the curve's geometry.
- **First order is all the home asks.** The home is Gauss–Newton at each node and its
  sensitivity the implicit function theorem, which reads a row's first order along each column;
  so a row with no Taylor form — an envelope's, a trace's, a traced curve's — is read there by its
  Jacobian (`RowKind::jets`, #377's seam, every higher order NaN), and refused anywhere a higher
  one would be read: in the body's other rows, in its rate rows, and over one formal.

### The gate, `tests/drawn_cauchy.rs`

| item | result |
|---|---|
| `bore.sv` against `housing.sv` (the apex the motion carries), 73 places at d = 0 … 3 | the two marches 6.6e-14 / 5.5e-13 / 1.1e-12 / 4.7e-12 apart at d = 0 / 1 / 2 / 3; each 3.5e-6 from the closed form at 3 |
| `bore.sv`'s `past` against `housing.sv`'s | refused at the waist at 14.52479 / 13.71964 / 13.92080 against 14.52479 / 13.71965 / 13.92079 on 16 / 32 / 64 pieces |
| `pitch.sv`'s pitch curve against `pitch_rise` | 1.9e-11 from the profile at the home (between nodes), 8.0e-10 from the envelope of the roller's centre at d = 6 |
| the three-lobed cam | a roller of 15 follows it (its parallel curve to 2.5e-6 at 15); one of 22 is refused at the hollow (u = 60°, radius 19.2) at 17.75 / 18.33 / 18.74 on 16 / 32 / 64 pieces |
| a spline's front, a traced circle's | the parallel curve (`d` off, square to the tangent, to 1e-7); the concentric circle of 20 − d |
| a contact on the pitch curve | its Jacobian in the point, the place and the profile's three heaviest columns against central differences |
| refusals | a point on the curve outside `initially`; the curve read by another relation; a surface as the curve; a curve given to a body over one formal |

### Findings

1. **A generated profile never undercuts its own roller.** The march from an envelope of a circle
   back by the circle's radius is regular by construction: where the pitch curve is convex the
   profile is less curved, and where it is concave more. The undercut the issue asks for is a
   *drawn* profile's, a hollow tighter than the roller; `pitch.sv` draws one by its formula.
2. **`conjugate_cams.sv` has no roller.** Its partner is generated by the lobe itself; read as a
   profile, its one concave stretch (about a roll of 0°, within 10° of it) is tightest at 0°,
   radius 69.4, so any roller smaller than that follows it. A follower there is a document of its
   own.
3. **A curve's constants are read when the body is compiled.** A trace's home and its pose on the
   sheet, and a plane's basis, ride in the row; its columns are live. A trace of a drawn instance
   whose pose moves far would want them refreshed as a contact's are.
4. **Cost.** `bore.sv`'s march to 3 is `housing.sv`'s: the envelope is evaluated once a node at
   the home, nowhere else. A contact's sensitivities carry every column of the curve's,
   as finding 1 of #370 says of any column.
