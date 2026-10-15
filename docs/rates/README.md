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

**Landed:** `rate(x over s)` and `initially { … }` (0.115, §6.5), a traced body that states a rate
integrated from its home in the plane and in space (`rates.rs`), and the gate `tests/rates.rs`.
The rule is `docs/contract/curves.md` ("Rates along a running formal") and
`docs/contract/language.md` ("Rates and `initially`").

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

### The gate, `tests/rates.rs`

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
