# The general sweep against the literature

The long tail of #684 (Tier 3: H–L) and #239 ask the same thing: what would an exact boundary of
any solid swept along any motion take? This note reads the most complete published answer — the
framework of Adsul, Machchhar and Sohoni (IIT Bombay) — maps it onto the code, and runs the
measurement #239 named as the first step: on the sweeps in the tree, how many contact branches,
how many folds, and where the sheet crosses itself. The measurement is `tests/sweep_census.rs`.

**In short.** The framework is complete for *simple* sweeps (no trimming) and was built for those;
for the rest it is an analysis — every trim is an ordinary transversal intersection once the regions
one function θ marks are cut out — and no implementation of that part beyond one decomposable
example was ever published. The code already holds the framework's local half, and holds θ twice
under other names (E3, S3). The census says branches are never the problem (two contacts a ring at
an instant, every case), a sweep cutting a blank as people write them is simple inside the blank,
undercut is the framework's *non-decomposable* sweep, and a sweep standing alone (the tumble) is
non-decomposable, θ < 0 over 40% of its contact set, though its tool and motion are an admitted
case's.

## The framework

Four papers:

1. *A procedural framework and mathematical analysis for solid sweeps*, arXiv:1204.0908, 2012.
2. *Local and global analysis of parametric solid sweeps*, CAGD 31, 2014 (arXiv:1305.7351): the
   funnel, trimming, θ.
3. *A computational framework for boundary representation of solid sweeps*, CAD&A, 2014
   (arXiv:1404.0119): the B-rep of a simple sweep, its topology and orientation.
4. *Incorporating sharp features in the general solid sweep framework*, CGF, 2016
   (arXiv:1405.7457): convex edges and vertices.

A tool `M` moves by `h(t) = (A(t), b(t))`, of class C², over `I = [t₀, t₁]`.

- **The funnel.** For a face `S(u, v)`, `f(u, v, t) = ⟨A N, v⟩`: the outward unit normal against
  the velocity. Its zero set `F` in `(u, v, t)` is the parameter space of the envelope faces the
  face makes, and `σ(u, v, t) = A S + b` carries it to the contact set. A point of the boundary is
  a contact, or on a **cap** — the tool at an end pose where `f ≤ 0` at `t₀` (`≥ 0` at `t₁`),
  bounded by the contact curve there (2, Proposition 8).
- **Simple.** A sweep is simple when every contact is on the boundary. The B-rep algorithm (3; 4,
  Algorithm 1) builds a simple sweep entity by entity from the tool's: vertices, co-edges, loops by
  the correspondence, faces over the funnel's components, each oriented by a determinant's sign.
- **Trims.** The trim set is the contacts the tool holds strictly inside at another time. `ℓ(p)`
  is the least time from `p`'s own at which it does. A sweep is **decomposable** when every piece
  of its roll shorter than some δ is simple (equivalently `inf ℓ > 0`). Then every trim curve is a
  transversal intersection of two pieces of the sheet from separated times, three at isolated
  points (2, Lemma 29): ordinary surface–surface intersection.
- **θ.** At a contact, `σ_t = l σ_u + m σ_v`, and `θ = l f_u + m f_v − f_t`. It is intrinsic, not
  of the chart (2, Theorem 35). It is the second derivative, at the contact, of the point's signed
  distance from the tool along its inverse trajectory, `⟨−σ̈ + 2Ȧ·V, N⟩ + κv²` (Theorem 39). θ > 0
  everywhere makes the sweep decomposable; θ < 0 anywhere makes it not (Theorem 33). Every
  contact with θ < 0 is trimmed (Proposition 42). θ = 0 is exactly where σ on the funnel stops
  being an immersion, the cusp locus (Lemma 37), and θ's gradient does not vanish there, so it is a
  curve found robustly.
- **Non-decomposable.** Each *singular* trim curve touches a θ = 0 curve tangentially at a point
  where `φ = 0`, φ the angle between that curve and the kernel of σ's Jacobian (Proposition 44).
  Away from that point it is again a transversal intersection (Proposition 45). So the region it
  bounds is found from θ and φ and cut out, and the rest intersected.
- **Sharp features** (4). At a convex edge the normals form a cone `cone(N₁, N₂)`. A point is in
  contact iff `⟨N₁, v⟩` and `⟨N₂, v⟩` differ in sign (§4.1). The edge's funnel is bounded by
  `⟨N₁, v⟩ = 0` and `⟨N₂, v⟩ = 0`, the co-edges it shares with the two faces' funnels, and its
  face is the edge's curve moved. That face is singular exactly where the velocity runs along the
  edge, and there every normal of the cone is in contact (Lemma 15). Its interior is never overrun
  from arbitrarily close times (Proposition 19). A sharp vertex sweeps an edge over the
  sub-intervals where two of its faces' `⟨Nᵢ, v⟩` differ in sign. Concave edges are assumed away:
  generically they add no point to the envelope, only intersections between the sheets of the
  faces either side.
- **The lifting theorem** (3, Theorem 11; 4, Theorem 17). For a simple sweep the correspondence `π`
  from the envelope to the tool is continuous, so entities adjacent on the envelope come from
  entities adjacent on the tool. It is a necessary condition, and drives the co-edge matching.
- **Assumptions.** General position throughout. Trim curves that reach the caps are skipped "for
  want of space"; the non-generic case "will need careful programming".

**What was built.** A pilot on ACIS, of simple sweeps: over 40 solids in (3), over 50 with sharp
edges in (4), each in half a minute to three minutes. (3) shows one sweep with trims, a capsule
along a helix, and calls the general case implemented; that sweep is decomposable, its trims
elementary ((2), Figure 8). (3)'s own conclusion puts "a more elaborate and full implementation
... for local and global intersections" in the pipeline, and (4) §8 says: "The extension to
non-simple sweeps is in progress and will require (i) scheduling of surface-surface intersections
and (ii) integration of θ." No later paper reports it. Machchhar's later swept-volume work is on
CNC machining simulation, with Plakhotnik and Elber (CAD, 2017), and on planar swept volumes
(CAGD, 2022).

## Mapped onto the code

| Theirs | Ours | State |
|---|---|---|
| the funnel `F`, σ | `SweepContacts::at_source_over` and `solid::contact_trace` read `F` as a graph `t(u, v)` over the tool (one contact time a point, E2); under a constant twist `F` is the characteristic times the roll (`constant_twist::Characteristic`); the census reads it instant by instant, as (2)'s seed surface does | built |
| faces evaluated by Newton from a seed surface | sheets fitted through exact nodes and judged (`sheet::settle`); the surface references (`docs/contract/analytic.md`) | built |
| caps, and trims that reach them | E1 keeps both out of the blank | class H |
| simple | what the admitted classes ask inside the blank: one contact a point (E2, S2), no fold (E3, S3), no self-crossing (E4, S4), no cap (E1) | built; the census finds every admitted case simple inside its blank |
| θ | E3's sign is `J f_t`, and `J f_t = ∓θ` exactly (below); S3's `(c′ × v)·n` vanishes exactly where θ does | built twice, as refusals |
| θ < 0 is trimmed | nothing: a branch wholly θ < 0 is fitted, split and judged | free to drop before the split |
| θ = 0, the cusp locus | E3 and S3 refuse it inside the blank | class J |
| decomposable, `ℓ`, elementary trims | the split trims crossings between placements; within one placement E2 and E4 refuse them | class I |
| singular trims, φ | none | class J |
| trimming by surface–surface intersection | `boolean::split`, the cells judged by the field | built |
| orientation (3, Theorem 12) | the field's judgement of the cells | not needed |
| the cone of normals | E's fan (`Piece::Corner`) under a constant twist; the generating class's crease fans | built |
| Lemma 15 | S1's refusal of a corner both faces meet at one point (a V wheel slid square) | refused |
| Proposition 19 | S3 asked of fan nodes | redundant beside S1: both vanish at Lemma 15's points alone |
| a sharp vertex's edge | none: a revolved tool has no vertex off its axis | prism tools (T1) |
| concave edges | T2 refuses one; a union's crease is trimmed by the split (#684 C) | T2 can follow C |
| the lifting theorem | E's faces cut at the fan's ends, each joined along the curve they share | fans only |

**E3 is θ.** Where each tool point has one contact time `t(u, v)`, the generated surface is
`σ̃(u, v) = σ(u, v, t(u, v))` and `t_u = −f_u/f_t`, `t_v = −f_v/f_t`. Then `σ̃_u = σ_u + σ_t t_u` and
`σ̃_v = σ_v + σ_t t_v`, and with `σ_t = l σ_u + m σ_v`:

`σ̃_u × σ̃_v = (1 − (l f_u + m f_v)/f_t) σ_u × σ_v = −(θ/f_t) σ_u × σ_v`.

A rigid motion keeps `|σ_u × σ_v| = |S_u × S_v|`, so E3's area factor is `J = ∓θ/f_t` (the sign
the chart's orientation) and the product E3 reads is `∓θ`. The factor alone passes through
infinity where `f_t = 0`, as `admission.rs` says, because that is θ divided by zero there. Under a
constant twist `f_t = 0` everywhere and `θ = l f_u + m f_v = df(v)`. The characteristic's tangent
`c′` spans `df`'s kernel in the tangent plane, so `(c′ × v)·n` vanishes exactly where θ does.

## The three questions

**Can their trim curves replace the split's surface–surface intersection?** No: they are
surface–surface intersections too. (2) reduces trimming to "guided parametric surface–surface
intersections". What θ adds is three things:

- where a sheet must stop: the sheet has a cusp edge along θ = 0, and no smooth sheet fits across
  it;
- what can be dropped before the split without asking: θ < 0 is always trimmed, and that is a
  quarter to two thirds of a turning tool's whole contact set in the census;
- that what is left meets transversally, except at points known in advance (φ = 0).

The field stays the referee of which cells are material, which is where their orientation theory
goes. θ narrows what the split is given; it does not replace the split.

**Does the lifting theorem give the face adjacencies directly?** Only for a simple sweep, and only
as a necessary condition: `π` is defined only where each envelope point has one preimage. Once
trims exist, the trimmed faces' adjacencies come from the intersections. Its use here is narrower
and still worth having. Some pieces of one sweep meet *tangentially* by construction:

- a face's sheet and its corner's fan, along `⟨Nᵢ, v⟩ = 0`;
- a sheet and its cap, along the contact curve at an end;
- two pieces of one roll's sheet, along the contact curve at their common time.

These should be joined along the curve the correspondence names, never found by intersection: a
tangential intersection is exactly what the split cannot do robustly ("a split robust to sheets
meeting tangentially", #239). E does this for fans already: the fan's ends are rows of the grid
exactly, and the sheet is cut there into faces. H needs it for caps.

**How does their cone of normals map onto class E?** One to one. E's fan stations are the cone's
normals turned from one face's normal to the other's. Each station's contact on the circle is `A cos
ψ + B sin ψ + C = 0` (ψ round the circle; the code's θ, not this note's) with that fan normal's
coefficients: §4.1's condition, read one normal of the cone at a time. The fan's ends, where the
walk runs on and off, are the co-edges `⟨N₁, v⟩ = 0` and `⟨N₂, v⟩ = 0`, made rows of the grid and
cut into faces: Theorem 17's adjacency, built rather than found. S1's sharp-edge refusal is Lemma
15's singular point (every normal of the fan in contact), and S3 on a fan node vanishes exactly
there. Proposition 19 holds in the census: on the tilted cylinder turned off its axis, every fan
sample overrun within a degree of its own time stands within 5% of a join (`α` or `1 − α` below
0.05); over all cases about half do, the rest within a degree, which a sampled band cannot call
infinitesimal. What (4) has that E lacks: sharp vertices (a prism's corners: T1's prisms), and fans
under motions other than a constant twist, which the generating class's crease fans already sweep.

## The census

`tests/sweep_census.rs` reads each sweep's contact set instant by instant: 97 instants over the
roll, 40 rings along each face, 720 stations round each ring. Each sign change of `f` round a
ring is bisected to a contact. Each convex corner's fan is in contact where its two faces' `f`
differ in sign. At each face contact θ is read by differences. Apart from θ, every contact is
asked whether the tool's own material field holds it strictly inside at any of 1081 times over
the roll (the trim set's definition), and how soon (`ℓ`). The census then splits the face samples:

- **local**: trimmed with θ < 0;
- **beside a fold**: trimmed with θ ≥ 0 within a degree of its own time, where a singular trim
  curve runs from θ = 0 to the crossing it ends on;
- **global**: trimmed from further away, a crossing between separated pieces of the sheet, or the
  tool at an end pose.

The census counts these over the whole sweep and inside the blank. Two checks run beside it:

- `theta_is_the_inverse_trajectorys_second_derivative` holds θ to a closed form. A ball turned
  about an axis half its radius from its centre gives `(1 ∓ 2 cos φ)/4` at latitude φ, toward and
  away from the axis, to 1e-4.
- θ < 0 and trimmed, read independently, agree. The column "θ < 0 kept" counts the exceptions:
  four samples in 290 000.

The whole sweep:

| case | roll | roots a ring at an instant | contact times a point | θ < 0 | kept | local | beside a fold | global (at an end) | least ℓ global (median) |
|---|---|---|---|---|---|---|---|---|---|
| torus, compound roll (`swept_tumble.sv`) | 150° | 2 | ≤ 1 | 39.9 | 51.1 | 39.9 | 0.4 | 8.6 (2.3) | 1.0° (14.7°) |
| torus, skew cradle roll (admitted on its post) | 150° | 2 | ≤ 1 | 39.9 | 51.1 | 39.9 | 0.4 | 8.6 (2.3) | 1.0° (14.7°) |
| lens (two sharp rims), skew cradle roll | 150° | 2 | ≤ 2 | 36.6 | 50.4 | 36.6 | 0.6 | 12.4 (2.1) | 1.0° (15.5°) |
| torus, crossed roll (a ring in contact at every time) | 120° | 2 | ≤ 1 | 50.0 | 49.3 | 50.0 | 0.7 | 0 | — |
| cylinder tumbled (a turn whose axis meets the tool's) | 120° | 2 | fixed | 66.7 | 25.1 | 66.7 | 8.2 | 0 | — |
| cylinder slid across (a slide) | 120° | 2 (its caps all round) | fixed | 0 | 100 | 0 | 0 | 0 | — |
| cylinder tilted 30°, turned off its axis | 120° | 2 | fixed | 64.6 | 35.4 | 64.6 | 0 | 0 | — |
| cylinder tilted 30°, roll | 120° | 2 | ≤ 1 | 24.4 | 75.2 | 24.4 | 0.5 | 0 | — |
| dumbbell, roll | 120° | 2 | ≤ 1 | 0 | 99.7 | 0 | 0.3 | 0 | — |
| thin plate (its sides), roll | 120° | 1 | ≤ 1 | 50.0 | 50.0 | 50.0 | 0 | 0 | — |
| torus turned across a ring (`solid_sweeps/turn.sv`) | 280° | 2 | fixed | 37.5 | 50.8 | 37.5 | 2.3 | 9.3 (1.9) | 1.0° (26.6°) |
| V wheel screwed along a rod (`solid_sweeps/vthread.sv`) | 528° | 2 | fixed | 33.3 | 66.7 | 33.3 | 0 | 0 | — |
| hypoid pinion's crown, bevel (admitted) | 49° | 2 | ≤ 1 | 25.6 | 70.8 | 25.6 | 0.2 | 3.4 (2.7) | 1.0° (9.7°) |
| hypoid pinion's crown, offset 20 (refused, its flank folds) | 51° | 2 | ≤ 2 | 31.8 | 66.4 | 31.8 | 0.1 | 1.7 (0.2) | 1.0° (10.1°) |
| hypoid pinion's crown, offset 30 (refused, undercut) | 50° | 2 | ≤ 2 | 25.5 | 73.2 | 25.5 | 0.2 | 1.1 (0) | 1.0° (7.3°) |

Inside the blank, where a case cuts one:

| case | face samples | θ < 0 | kept | local | beside a fold | global | least ℓ global (median) |
|---|---|---|---|---|---|---|---|
| torus, skew cradle roll (admitted) | 159 | 0 | 100 | 0 | 0 | 0 | — |
| lens, skew cradle roll | 128 | 0 | 100 | 0 | 0 | 0 | — |
| torus, crossed roll | 212 | 17.5 | 76.4 | 17.5 | 6.1 | 0 | — |
| torus turned across a ring | 82 | 0 | 100 | 0 | 0 | 0 | — |
| V wheel screwed along a rod | 1398 | 0 | 100 | 0 | 0 | 0 | — |
| hypoid pinion, bevel (admitted) | 2688 | 0 | 100 | 0 | 0 | 0 | — |
| hypoid pinion, offset 20 | 3950 | 8.9 | 85.7 | 8.9 | 0.4 | 5.0 | 1.1° (7.3°) |
| hypoid pinion, offset 30 | 4475 | 19.1 | 75.9 | 19.1 | 0.9 | 4.1 | 1.0° (5.5°) |

Percentages are of face samples. The table the tool prints also has the samples' counts and the
fans' (`cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core
sweep_census::the_census -- --ignored --nocapture`, about 40 s; `SOLVENT_CENSUS` a word of the
cases' names reads only those). Samples are uniform in instants and rings, not in area. A prism's
caps are not sampled, so the thin plate's row is its sides. Every column is sampled and says so.

**What it says.**

1. **Branches are not the problem.** No ring had more than two contacts at an instant, the
   compound roll included, as the closed form says for a revolution under any rigid motion; a
   prism's side has one. No tool point had more than two contact times over its roll, and fewer
   than one in two hundred had two. #239 feared "any number of roots" under a compound motion; on
   the motions in the tree it does not happen, read per instant or per tool point.
2. **A sweep standing alone is rarely simple once the tool turns.** Every case whose tool turns
   but one has θ < 0 on a quarter to two thirds of its contact samples, so as a whole each is
   non-decomposable (Theorem 33). The slide has none: a convex tool slid at a steady rate has
   `θ = κv² ≥ 0` (Theorem 39 with `Ȧ = 0` and `b̈ = 0`). Neither has the dumbbell, whose roll
   spins it about its own axis (nothing, to a revolution) and carries it round the spindle at six
   times its largest radius. The tumble's row is the admitted torus-on-a-post's row:
   `swept_tumble.sv` is that fixture's tool under that fixture's motion. What keeps it out is not
   its motion but that it stands alone, its whole envelope wanted.
3. **A sweep cutting a blank, as people write them, is simple inside the blank.** Every admitted
   case keeps all of its contact samples there. The rows are exactly what makes the blank's part
   of the sweep simple.
4. **Undercut is a non-decomposable sweep, and nothing more.** The hypoid pinion at offsets 20 and
   30 has θ < 0 on 9% and 19% of its contacts in the blank, and 4–5% trimmed from 5–7° away. That
   is a fold whose singular trim curve runs from θ = 0 to a transversal crossing: (2)'s Figures 12
   to 14.
5. **Caps are a few percent.** 2–3% of a whole sweep is trimmed by the tool at an end pose (class
   H). Inside an admitted blank, none (E1).
6. **The non-generic cases sit outside the framework.** A ring in contact at every time (the
   crossed roll) leaves 17.5% θ < 0 and 6% beside a fold inside the blank. A flat face the
   motion keeps in its plane has `f ≡ 0`: the slid cylinder's caps, every ring of them in contact
   all round. The framework assumes both away; the code refuses both by name (M2 under a roll,
   S1 under a constant twist) and should go on doing so.

## What it means for Tier 3

- **One function, θ.** It is E3's and S3's quantity already. Reading it as θ at every node of a
  sheet gives three things: the sign rows as they stand, the regions free to drop before a split
  (θ < 0), and the curves where a sheet must stop (θ = 0).
- **H, ends inside the blank.** The caps are the tool's faces at the end poses, cut along the
  contact curve there (`f(·, t₀) = 0`). Each cap is joined to the sheet along that curve by
  construction, since the two are tangent there; whatever else of the sheet crosses a cap does so
  transversally, and the split trims it.
- **I, several contacts a point (E2), and E4.** Where θ > 0 on the reach (decomposable), split
  the roll into pieces shorter than `inf ℓ`, so that each piece is simple. Fit each as a
  placement's sheet, joined end to end along the contact curve at their common time; the split
  already trims sheets of different placements. The census found no such case in the tree (every
  case with a second contact also folds), so this is cheap but may be rare.
- **J, undercut and folds.** Cut each sheet at θ = 0 (a robust curve, its gradient never zero),
  keep the θ > 0 pieces, and let the split find the singular trim curve. It is transversal but at
  the φ = 0 point, which can be computed beforehand and made a vertex. The hypoid's undercut is
  exactly this.
- **T2, concave corners.** (4) says a concave corner adds no point to the envelope, only an
  intersection between the sheets either side: the trim a union's crease already gets from the
  split (#684 C). Extend each face's sheet past the corner and let the split trim them.
- **K and L, nested motions and varying rates.** These are not separate problems. The funnel is
  defined for any C² motion, and the census read rolls, turns, slides, screws and the compound
  roll through one path. A rate law changes the swept set only through the path the tool takes
  in space, never through its speed along it: a single joint driven by any continuous law sweeps
  what the constant-rate joint sweeps over the law's range.
- **A sweep standing alone** (#239's tumble, retiring the field mesher). It needs J's machinery
  over most of its envelope (θ < 0 on 40% of the tumble) and H's caps. It is the hardest case in
  the corpus, and not a problem of its motion.

**Why the framework stopped where it did.** What its authors left "in progress" is where the
kernel's intersection meets tangency (caps, singular trim points) or a non-generic input; #239
records the same pieces failing here. The project has what the ACIS pilot did not: an exact
referee, the field, which makes a wrong trim visible.
