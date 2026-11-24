# Faces trimmed on nets singular outside them (#1013)

The three crossed Booleans of brep-bench's tumbling tori (`swept/tumbling-torus-crossed-union`,
`-difference`, `-intersection`) refuse. Their operands build and are valid. This file is steps 1–3
of [#1013](https://github.com/bmander/geomsolver/issues/1013): what the kernel cannot do, each thing
pinned by the smallest case that shows it, and those cases added to brep-bench. Step 4, building
them, is not begun.

## The capabilities missing

Each operand face of the tube road lies on a fitted net that runs on past the face: folded back
past the face's cuspidal edge, with a second layer near the face, a cusp, or an isolated singular
point of its parametrization just outside. The Boolean reads a B-spline face through its whole net.
It finds a point's foot on the nearest layer, finds a line's crossings over the whole net, and
marches a meeting over the whole net. Only after that does it place what it found against the
face's trim. Each of the four capabilities below fails at a different one of those readings. Each
was found and shown alone on a block under a Bézier patch with no sweep in it. Each case fails on
`main` (6836e76). Each builds to its closed form, by every Boolean asked of it, when the top face is put
on the patch cut down to its own part (`Net::segment`: the same surface, the same parameters,
nothing past the face). All four belong to the Boolean, not the sweep. The sweep can avoid them
by giving its faces nets that do not run on (the experiment branch's `Net::segment` of each face to
its pcurves' box, 8ad646d, or `claude/880-t1f-tube-faces`). But brep-bench's cases are about faces
whose surfaces do run on, and the same nets reach the Boolean from STEP files.

| # | capability | minimal case (gate, `tests/patch_layers.rs`) | fails on `main` with | how |
|---|---|---|---|---|
| T1 | **A meeting placed on a face by its own layer**: a stretch of a meeting on the net's other layer is not on the face, even where its foot on the nearest layer is | `nurbs/folded-patch-notched-at-its-rim`: patch `(s², v, 1/2 + 3s⁴/10 + v/10)`, covering itself exactly; the face `u ≥ 13/20`; a box whose side `y = 1/2` crosses the face's rim | "a bspline's pieces do not close" | the side meets the patch in one exact iso-curve over the whole net (`u` in [0, 1]). Its stretch on the folded-back layer (`u ≤ 7/20`) is given a pcurve by its foot, which lies on the face's layer. So the edge from the rim to `x = 1` is laid a second time on the face (edge 33, `u` from 0.65 to 1, over the true edge 32). The failure needs the side to cross the rim: a box that starts at `x = 0.2` builds. |
| T2 | **An edge's crossings with a face found on the face's own layer**: every crossing of a line with the net, each placed by its own parameters | `nurbs/patch-over-a-near-layer-pierced`: the same patch plus `s³/100`, its folded layer `x^{3/2}/50` beneath the face; a square hole through it | "a shell of the arrangement is kept on one side of a bspline (`top`) and dropped on another" | each upright edge of the hole crosses the face once and the other layer just beneath it. At `x = 0.7` only the other layer's crossing is found (z 0.6594, read `Out`) and the face's (z 0.6946) is lost. At `x = 0.4` both are found. With layers 1.8e-4 apart (`s³/1000`) the pair is read as one touch, `touching: Out`. It fails for every layer gap tried from `s³/10⁴` to `3s³/100`. |
| T3 | **A meeting ended at the face, or at a fold of its net**: no march over the net past the trim into its cusp | `nurbs/cuspidal-patch-sliced-through-its-cusp`: patch `(s², v, 1/2 + s³ + v/10)`, face `u ≥ 11/20` (a hundredth from the cusp); cut by `z = 11y/10` through the cusp | "a shell of the arrangement is kept on one side of a bspline and dropped on another" (or "a bspline's pieces do not close", cut otherwise) | a march runs on past the rim to the cusp, outside the face, and comes back along itself. Cut by a slab under the plane, it is laid as a closed curve of 69 points, from the rim to the cusp and back. Cut by `build::kept`, as the gate and the bench cut it, the march from the face's far edge goes to the cusp and returns to its own start (224 points). Another march, through the cusp onto the other layer, ends the true stretch's curve without its end on the rim, so the stretch dangles. This is the issue's "a march creeping across a fold of its sheet". With the face trimmed at `u = 13/20`, or with a steeper plane, it builds. It also depends on where the cutting block's faces lie, because they seed the trace: cut by a block standing on the plane at another place, it builds. |
| T4 | **A meeting ended short of a singular point of the net outside the face** | `nurbs/pinched-patch-sliced-beside-its-pinch`: patch `(s³, t³, 1/2 + st)`, singular at `u = v = 1/2` alone; face `u, v ≥ 11/20`; cut by `−x + y + z/10 = 13/250`, 0.002 from the singular point | "a shell of the arrangement is kept on one side of a bspline and dropped on another" (or "a bspline's pieces do not close", cut otherwise) | the march runs from the face's far edge, past the rim, through the singular point where the meeting turns sharply, and on into the opposite quadrant (170 points). Its fit takes five to ten rounds and leaves gaps, and from the rim point a second march goes to the singular point and back as a closed curve (79 points), so the face's stretch is laid off the meeting or twice. Another placement of the plane, `y ≈ 2x + 0.0005`, turns back at the point instead and lays the face's stretch twice (239 points, "closed"). This is the issue's "a march cycling through an isolated singular point". It is erratic: of 32 planes within 0.002 of the point, 10 fail; the net cut to the face builds all 32. |

None of the three rules on `claude/880-t1f-trimmed-meetings` fixes any of the four cases. All four
still refuse there, the same way. The seed filter acts on seeds, and T1's meeting is a closed form
with none. The fold and self-meeting rules end marches, but T3's march is ended in the wrong place,
and T4's fails in its fit.

### Struck or not yet shown

- **"A march stalled where the projection lands back on the point it stood on"** and **"a march
  creeping at the least step"** are what T3 and T4 look like from inside the march. They are not
  separate capabilities: a march that stays inside the face's trim never meets them.
- **"Where a march ends on a face's boundary"**: no minimal case needed it. In every case above the
  end on the rim was found once the stretch past the rim was not marched.
- **The radial order round an edge**: struck as the cause of the crossed union's leak on the
  experiment branch. Each of the 21 edges that join a kept side and a dropped side of the leaking
  shell is explained by a piece that spans both sides of the other operand, with no misordered
  edge needed. One piece of `A1` (`Sheet(1.0)`), face 8 of the complex, is in 18 of them. It uses meeting edges 236, 238 and 239 twice: round
  each, "8, a face of B, 8, a face of B". So those meetings are slits inside one piece that spans
  both sides of B, and the face was never parted along them. The debug line "every meeting edge
  used four times" counted face 8 twice. See *The crossed union* below.
- **Time** is not pinned by a case of its own. The minimal cases show it: on the whole net each
  Boolean takes 0.3–0.5 s against 0.02 s on the net cut to the face (T1, T2), and a box that does
  not reach the rim of the exactly doubled patch builds in 1.4 s against 0.03 s. The tori's
  144 s of face pairs are the same reading over nets of 940 × 850 poles. A bench case for the
  bench's own reads of such nets (the reverse torus's 407 s) would time the harness's STEP reader,
  not a kernel, so it is left to the harness. Note: `harness/mass` itself cannot read OCCT's
  STEP files of T1 and T3 ("a face whose loops bound no region of its surface", "a loop that does
  not join up on its surface"). The bench's reader has T1/T3's capability gap too.

## The crossed union

On `claude/880-t1f-trimmed-meetings` plus a classification report (each edge's uses, each face
with both sides in the leaking shell, each join of a kept and a dropped side with its edge's
radial order, and the shortest walk over the side graph from the outer cell), the union refuses
in 162 s.

- The leaking shell holds both sides of 41 faces. Twenty-one edges join a kept and a dropped side.
  Eighteen of them involve face 8, a piece of `A1`, read `kept|dropped` (outside B). Its
  neighbours across A's own edges 12 and 14 are inside B, and so are the B pieces at its meeting
  edges. The other three are edges of B's own seam between `B5` and `B8`, where a piece of `B8`
  (face 110) is read inside A beside a piece of `B5` read outside it.
- Face 8 runs on through meeting edges 236 (from `P = (4.2085, 0, 2.0702)` to
  `(3.9776, 0.7733, 1.9988)`), 238 and 239 (ending at `(2.4669, 0.9743, 1.7931)`). Those meetings
  were traced and laid, but the cut of `A1` does not close round them. At both far ends the
  meeting stops on the boundary of B's face `B8`, and `A1 × B8` is traced from no seeds.
- `B5`–`B8` are four faces of `Sheet(-1.0)`, classed as one surface (they share one meeting with
  `A1`). On this branch each carries its own segment of the sheet's net (8ad646d), so a march
  for one face ends at that face's net's edge ("a step left the surfaces": the `B7` march stops at
  `(2.4668, 0.9743, 1.7931)`, on the `B7`/`B8` seam). `B8`'s own seeds there lie on a known curve
  and are dropped, so the stretch on `B8` is never traced. That is a fifth gap, **T5: a meeting
  shared by the faces of one surface class is traced over each face's own net**. It arises
  only where faces of one class carry different nets. On `main` those faces carry one net, and the
  leak there is the subject of the next paragraph.
- `P` is where four branches of the meeting cross (A1 and B5 both regular there, area factors 0.70
  and 0.85, at `v = 1/2`, `y = 0`). Probably the sheets touch there, since both sweeps pass
  through the same pose at `t = 0`. The four branches are traced and end at `P`.

On `main` (6836e76) the same report is being read: the union, under full debug, was still tracing
face pairs (`A5`) after 40 minutes. The issue's account of it is a refusal after about 430 s with the
same message, "a shell of the arrangement is kept on one side of a bspline (`Sheet(1.0)`)".

## The cases

Each capability is a case in brep-bench's `nurbs/` with closed-form answers (mpmath; the volume of
T1 is exact), implemented for cad-kernel (refused or wrong there, as `main` is) and for OCCT:

| case | volume | area | OCCT |
|---|---|---|---|
| `nurbs/folded-patch-notched-at-its-rim` | 0.48658855 | 4.1267385566944163 | 0.48658855000000023, area 9.6e-8 high |
| `nurbs/patch-over-a-near-layer-pierced` | 0.54662685536654843 | 5.0938677482524649 | agrees to 1e-11 |
| `nurbs/cuspidal-patch-sliced-through-its-cusp` | 0.51031272172170669 | 4.2949491490074952 | 2.5e-8 low, area 4e-8 low |
| `nurbs/pinched-patch-sliced-beside-its-pinch` | 0.5244906492156999 | 4.591931211856204 | agrees to 2e-10 |

The gates are in `rust/gcs-core/tests/patch_layers.rs`. Each case is there twice. The control has
the net cut to the face and passes today. The gate has the whole net, is `#[ignore]`d with the
reason it fails today, and checks every reading against the closed forms and inclusion–exclusion:
all three Booleans for T1 and T2. For T3 and T4 the gate keeps each side of the plane by
`build::kept` (the sides summing to |A|), as the bench's implementation does, because those two
failures turn on the cutting block (see T3). `cargo test -p gcs-core --test core patch_layers --
--include-ignored` shows the four controls pass and the four gates fail, each with the message
above. `rust/cad-kernel/examples/patch_probe.rs` is the probe the cases were found with: any of the
four patches, trimmed anywhere, cut by a box or a plane, on the whole net or its part.
