# General sweeps: the census (#239)

**Status (2026-10-13): measured; the decision is open.** Issue #239 asks whether the continuous
sweeps outside the three admitted classes (generating, constant twist, pocketed planar;
`docs/generating-sweeps.md`) can have an exact B-rep, or whether the language should be narrowed to
the admitted classes so the field mesher can be retired without one. Its first step was a
measurement before any construction: over the tumble and the removed negative controls, how many
contact branches per tool latitude, how many folds, and where sheets cross. This is that
measurement. Its numbers are held by `rust/gcs-core/tests/sweep_census.rs`, which prints the table
below and asserts the facts the conclusions rest on.

## What the census reads

For every face of the tool, the fan of normals every convex crease of the tool sweeps, and a
prism's two caps, the contact condition `f(u, v, t) = n·velocity` is sampled on a grid over the face
and the roll: 25 latitudes × 72 columns × 121 times (`GRID`). Nodes are stepped off the faces'
lines of symmetry by irrational fractions of a step, so a characteristic a symmetric fixture lays
along a meridian does not fall on a node. The zero set is read as marching cubes read a field.

- **Roots per tool point**: sign changes of `f` along the roll at one (u, v).
- **Pieces per latitude**: pieces of `{f = 0}` over (v, t) at one u (a ring of a revolution, a
  line across a prism); a **loop** is a piece that reaches neither roll limit.
- **Sheets**: pieces of `{f = 0}` over (u, v, t).
- **Folds**: where `g = f_t − a f_u − b f_v` changes sign along the zero set. Here (a, b) is the
  point's velocity in the moved tangent frame. `g` is the generated area factor times the rate of
  the contact condition, admission's E3 sign, and it changes only where the envelope itself folds.
- **Crossings**: contacts within the sampling's spacing of one another in space, whose tool points
  lie more than eight spacings apart, at more than 11° (a *steep* site: more than 26°).
- **Exposure**: whether the swept solid's own material field (`MaterialField`, the referee) puts
  a contact on the boundary. A point a ten-thousandth of the tool's size out along the tool's
  outward normal must read outside. The same test is applied to the tool at each roll limit (the
  end caps).

Every contact is a root refined by bisection on its grid edge. The whole census takes 4.4 s
(`cargo test -p gcs-core --test core sweep_census -- --nocapture`). `SWEEP_CENSUS_FINE=1` reads it
at twice the grid in every direction (24 s).

The cases are the swept stock the corpus writes, plus the negative controls exactly as the removed
general closure's tests wrote them (`git show 4bedee36~1:rust/gcs-core/tests/sweep_mesh/grazing.rs`,
`creases.rs`). The corpus's stock is `swept_tumble`, `swept_spring`, and `swept_groove`'s cutter.
`fixtures::tools::DUMBBELL` had drifted from the language (no `in` block, `on` for `union`) and is
brought up to date here.

## The census

| case | motion | twist | roots/point | pieces/latitude | loops | sheets | fold points | crossing sites (steep/exposed/self) | envelope exposed | ends exposed |
|---|---|---|---|---|---|---|---|---|---|---|
| swept_tumble | turn relative to a skew turn | varying | 1 | 2 | 0 | 3 (2 folded) | 6 (0 exposed) | 9/2/0/9 | 51% | 42% / 34% |
| swept_spring | screw | constant | 0 | 2 | 0 | 1 | 0 | 0 | 100% | 51% / 51% |
| swept_groove (cutter) | single turn | constant | 0 | 2 | 0 | 2 | 0 | 0 | 100% | 52% / 52% |
| tumbling cylinder | single turn | constant | 0 | 2 | 0 | 7 | 0 | 0 | 60% | 17% / 17% |
| tilted cylinder 30 | single turn | constant | 0 | 2 | 0 | 9 | 0 | 0 | 80% | 37% / 37% |
| tilted cylinder 85 | single turn | constant | 0 | 2 | 0 | 9 | 0 | 0 | 80% | 36% / 36% |
| tilted cylinder 5, slid | translation | constant | 0 | 2 | 0 | 4 | 0 | 0 | 100% | 50% / 50% |
| thin plate under a roll | turn relative to a turn | varying | 1 | 1 | 1 | 10 | 0 | 0 | 58% | 48% / 51% |
| sliding dumbbell | translation | constant | 0 | 2 | 0 | 6 | 0 | 0 | 100% | 52% / 52% |

"Twist constant" means `f` does not change over the roll at any tool point, so the contact set is
fixed on the tool: its **characteristic**. Roots per point are then 0, and the characteristic is
swept whole. Exposure shares count samples, not area.

At twice the grid every column agrees except the two that count samples along a curve. The tumble
shows 15 fold points (none exposed; one beside the boundary where a fold runs into the tool's last
pose) and 92 crossing sites (none steep, none exposed, all self). Every sheet count, root count,
piece count and exposure share stays within a point.

Per sheet (the census's detail lines), every sheet of every case is either nearly all boundary or
nearly all buried: at least 96% exposed, or at most 4%. The tumble's three sheets are 100% (no
fold), 1% (folds) and 4% (folds).

## Findings

1. **No case has several contacts per tool point.** At most one contact time per tool point lies
   within the roll, and at most two pieces of contact cross any latitude. For a relative rotation
   this is closed form: the contact condition at a tool point is one sinusoid in the source's angle
   (`motion/contact.rs`), so it has at most two roots per turn of that angle. The tumble's 150° of
   roll at ratio 0.25 spans 37.5° of it, and the plate's 120° at 1.05 spans 126°; within those
   stretches the census finds one root at most. A longer roll, or a sinusoid whose two roots lie
   close together, gives a point two contacts, and a branch where they merge. A motion nested deeper (a relative
   motion of a relative motion) adds harmonics, and `Family::normal_velocity` refuses it today.
   None of these cases has one. The difficulty the issue names first, "a general function of the
   roll, with any number of roots, branching and merging", does not arise for the motions these
   documents write.

2. **Six of the nine have a constant twist**: the single turns, the translations and the screw.
   Their envelope is the characteristic swept, with no time chart, no folds and no crossings.
   That is the constant-twist class's construction (`solid::constant_twist`, S1–S4), but the class
   is reached only through `Family::screw`, which needs both a turn and an advance. A single turn
   (zero pitch) and a translation (no turn) are its two limits. As cuts they are refused by M2,
   because their twist is constant, and not by anything in their geometry.

3. **Folds and crossings appear only in the tumble, and none of it is boundary.** Two of its three
   sheets fold, and those two are the buried ones (1% and 4% exposed). The exposed sheet is
   fold-free, with one contact per point. The near pairs the crossing test finds are all on one
   sheet. They are the layers of a folded sheet doubled back beside its folds: at twice the grid
   none is steeper than 26°, and none is exposed. No transversal crossing reaches the boundary in
   any case. The outer-arrangement problem ("the boundary is the outer part of an arrangement, not
   the sheets") is not exercised by any of them. Each sheet is boundary or buried as a whole, up to
   a few percent where a buried sheet runs up to a roll limit.

4. **What every case lacks is the end caps.** At each roll limit 17–52% of the tool is boundary,
   and 51–100% of the envelope is. Every admitted class keeps its caps out of the blank (E1), so
   none builds a cap. A swept *stock*, which is every case here, is the exposed sheets plus the
   tool at its two limit poses, trimmed against one another. A cut that stops inside its blank
   needs them too: `swept_groove`'s cut is refused by E1 (the ball at each end of its swing lies
   in the block) before M2 is asked. The cases are stock rather than cuts
   for this reason, not because their motions are exotic. The tumble's motion and torus are exactly
   `swept_torus`'s, whose removal is admitted and exported exactly as a cut of a post. Only as a
   stock is it refused.

5. **The removed closure's hard cases are easy here.** The tilted, slid and tumbling cylinders,
   the thin plate and the dumbbell were where the certified general closure failed. Under this
   reading they are single-sheet-per-face, fold-free, crossing-free and constant-twist (bar the
   plate). The plate's contact loop that "flickers open and closed" is its caps' sheets: each is a
   graph `t(u, v)` over the whole cap with one root a point, reaching neither roll limit (the one
   loop in the table). Its difficulty was the tracer following a curve per time, not the envelope.

## What it says about the three directions

- **Direction 3 (widen the classes one motion family at a time) is tractable** for every motion
  measured. The branch discovery it needs is closed form (one sinusoid) or absent (constant twist).
  The widening is two steps, each with rows of its own:
  1. *Constant twist at its limits*: a single turn and a translation, `S(s, t) = M(t)·c(s)` over the
     characteristic `c`, as for a screw. A face wholly in contact (a cap square to the turn's axis,
     a prism side along a slide) sweeps within its own surface and is its own piece; none occurs in
     these cases, but an untilted cylinder turned about its axis has one. Alone this admits a
     single turn's or a translation's cut whose ends clear the blank.
  2. *Ends*: the tool at both limit poses as two more pieces, beside the exposed sheets,
     split against one another and the cells judged by the field. This is the split-and-classify
     path the generating class already runs (`boolean::split`, `sweep::sector`), with the caps as
     sheets. Each sheet is admitted only if its exposed part is fold-free (E3) and does not cross
     itself (E4). A sheet the field reads as buried, as the tumble's folded two are, is dropped
     before the split, so in these cases no fold is split along. With step 1, this would cover every
     case here as a stock, and `swept_groove`'s cut, whose ends lie in the block.
- **Direction 1 (split a box by every branch)** reduces to step 2 for these cases: the arrangement
  holds at most ten sheets and two caps, and no exposed crossing. Its stated hard parts (branch
  discovery over the whole roll, a split robust to tangential meetings) are not met. The only
  tangential meeting is an exposed sheet with the tool at a limit pose, along that pose's contact
  curve, where the sheet ends. That is a seam to build, not a crossing to find.
- **Direction 2 (field-guided construction)** is not needed for any case measured. It stays the
  fallback for a motion whose contacts the closed forms do not give (nested relative motions).
- **Narrowing the language** would refuse `swept_tumble`, `swept_spring` (as a stock),
  `swept_groove` and the controls. The census suggests these are near the existing classes, not
  outside them.

## Open questions this does not settle

- The census is sampled. Its counts depend on the grid (fold points and crossing sites are curve
  samples; the rest stood still at twice the grid). An admission would read these as rows over the
  whole tool, as `admit_body` does, with witnesses.
- Caps are read for a convex profile (the plate's and the box's). Fans are read for convex creases
  only, and their folds are not read. A concave crease of the tool (the dumbbell's bar entering a
  ball) is where two faces' sheets meet; it is a seam and is not counted as a crossing.
- The cases are what the corpus and the old controls write: one sweep each, one motion step or a
  relative turn. A nested relative motion, a sweep of a sweep, or a roll long enough to reach a
  sinusoid's second root are not measured.
- What a stock's B-rep would be gated by is unchanged: zero disagreement with the field
  (`solid::agreement`).

## Decision

Open, for the issue's owner. The census favours direction 3 in the two steps above over narrowing
the language. It finds nothing in these cases that needs general branch discovery or an outer
arrangement.
