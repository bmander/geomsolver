# Shells, written in Solvent

`shell` is not a feature of the kernel. It is a word of the standard library
(`rust/lib/std/rounds.sv`, `use std.rounds (shell)`), written over things the language says anyway:

- `A shell(t: 2mm) B` keeps of the body `B`'s faces are of only what lies within `t` of one of its
  faces not among `A`'s: the body hollowed to walls `t` thick, open through `A`'s faces.
- `shell(t: 2mm) X` keeps what lies within `t` of everything outside `X`: a sealed hollow.

Its body is a cavity, `{ c | c inside B }` with each kept face's wall `{ p | p distance(<= t) w }`
cut from it, cut from the body. Like a round, it acts on the body after the body's other terms.

The field reads every shell point by point. Most are also built exactly, held to that reading,
and exported by `solventc --step` and `--stl`, in one of two ways:

- **As a prism.** Where the body is a prism (lines and arcs in its section) less what is cut from
  it, the cavity is a prism too. Its section is the body's section with each edge moved in by `t`:
  a line along its normal, an arc about its centre. The corners meet where two edges meet at a
  convex corner, and are joined by an arc of radius `t` about a reflex one. Each cut from the body
  takes from the cavity what lies within `t` of it.
- **By its walls.** Any other body — a boss put on it, a turned one — is built by its walls: what
  lies within `t` of what is outside the body, built as a body grown is (`std.rounds.Grown`) about
  its faces turned inside out.

Read them in this order:

| File | What it shows |
|---|---|
| `tray.sv` | **The word.** A box open at its top, `tray.near shell(t: wall) tray`. The cavity runs out through the rim and lies on none of the box's faces. Built exactly: 11712 mm³. |
| `float.sv` | **Sealed.** `shell(t: wall) float`: every face closed, a hollow inside no face reaches; the body is two shells. Built exactly: 8427 mm³. |
| `drawer.sv` | **Open through a side.** `drawer.north` is the side its section's `north` edge sweeps: the section is moved out through that edge, the caps in. Built exactly: 5184 mm³. |
| `step.sv` | **A reflex corner.** An L sealed: within its concave edge the cavity is rounded by an arc of radius `wall` about the edge, not met square. Built exactly: 3600 − 8 (160 − π) mm³. |
| `polygons.sv` | **Corners of other angles.** A hex box open at its top (inner faces at 120°) and a sealed right-triangular wedge (acute corners, the section scaled about its incentre). Both built exactly. |
| `enclosure.sv` | **Arcs, and something in the hollow.** A rounded enclosure: each corner's arc moved in about its centre, the inner faces tangent as the outer ones are. A screw post on the floor is added to the shelled body by naming it first (`case := solid(enclosure)`), so the shell does not hollow the post. Built exactly. |
| `cup.sv` | **A whole circle.** A cylinder open at its top: its section one circle, moved in to one of radius 18. Built exactly: 2928π mm³. |
| `hopper.sv` | **Walls meeting the opening at a slant.** A hopper whose wall leans in over its open top ends in a rounded lip: past the wall's edge a point is nearer the rim than the wall's plane. A flared tray's walls lean out, and run straight up through the opening. Both built exactly. |
| `pocketed.sv` | **What is cut is shelled with the body.** A tray with a pocket in its top, open at the top, and a plate with a bore, sealed: the cavity less each cut grown by `wall`, so a wall stands round the pocket and under its floor, rounded within its edges, and a tube of wall round the bore. Both built exactly. |
| `plinth.sv` | **A boss put on the body.** A union is not hollowed term by term: the cavity reaches up from the slab into the boss, rounded by `wall` within the boss's foot. Built by its walls, exactly: 16500 less the slab's, the foot's 100 − 10π + 8/3 and the boss's cavities. |
| `chimney.sv` | **A boss open at its top.** `plinth.boss.near` left open: the opening is carried out by a prism of itself before the walls are built, so they run up through it. Built exactly. |
| `bosses.sv` | **Square and round.** A plate with a square boss and a round post, sealed: a quarter round along each edge of the square foot, one turned round the post's circle. Built exactly. |
| `can.sv` | **A turned body.** A rectangle turned about its own edge, sealed: no prism, so built by its walls, the band moved in and the caps mitred to it. Built exactly: 3576π mm³. |
| `refused.sv` | **What is not built**, read by the field and refused by an exact export, saying why: a boss standing on an opening (its wall's quarter round hangs below the opening), an L shelled thicker than half its arms (its walls meet), a pocket's faces left open (an opening is one of the stock's faces). |

`rust/gcs-core/tests/shell_examples.rs` holds each file to what its comments say: each exact
body's volume to its closed form to 1e-9, each refusal's words. `rust/gcs-cli/tests/cli.rs`
(`the_shells_export_and_measure_within_their_tolerance`) writes each exact body as STEP and STL
within 10 µm and measures it. `tests/shell.rs` holds the word's own cases, read point by point at
a lattice against their closed forms.
