# Solid sweeps, built exactly

A solid sweep is everything a tool passes through as a motion carries it: `sweep(tool over t in
(a, b), under: m)`, and most often `cut` from a blank, as a machine cuts. Any such sweep can be
read point by point (the material field), but its boundary is built exactly — written by
`solventc --step`/`--stl` and shown as a smooth surface — only for the classes a tool and a motion
fall into whose boundary has a known structure, each admitted by its own rows and refused by name
outside them ([docs/generating-sweeps.md](../../../docs/generating-sweeps.md)). The classes grow
one at a time (#684); each has its simplest case here, and every file's volume is a figure its
comments give.

Read them in this order:

| File | Class | What it shows |
|---|---|---|
| `ball.sv` | the canal | **A ball's sweep is its tube.** A ball swung a quarter turn about an upright axis, half sunk in a block: the tube about its centre's path with half a ball at each end, whatever the motion turns it by. Pappus. |
| `slide.sv` | constant twist: a slide | **A tool slid along a line.** A torus slid up a rod, clear at both ends: the wall is the wheel's characteristic extruded, here its tube's circle, and the groove the lens the rod and the tube share. Tilt it and the wall is no circle of it. |
| `turn.sv` | constant twist: a turn | **A tool turned partway about an axis.** A torus leaning across the turn swung over a curved block: the wall is its characteristic laid in a half-plane through the axis and revolved. Pappus over the torus's shadow. |
| `screw.sv` | constant twist: a screw | **A tool carried along a helix.** The same wheel screwed up the rod: the characteristic is read once on the wheel and carried along the helix, so the groove's section is the same at every height, turned. |
| `vthread.sv` | constant twist: a screw with sharp edges | **A tool with a sharp edge carried along a helix.** A ring with a 60° V rim screwed up the rod, as a thread is ground: each flank's characteristic meets the rim at its own point, and the wall runs between them round the rim, where every normal between the flanks is the wheel's. The groove's section, the same at every height, is read point by point from the V. |
| `rack.sv` | generating | **A tool rolled on a turning blank.** A rack tooth slid along the pitch line while the blank turns: one tooth space, its flanks involutes nobody draws. The tool's contact moves over it as it rolls, so each point of the wall is a contact at its own instant. |
| `conjugate.sv` | planar subtraction | **A tool always in mesh.** A plus turning against a wheel over a whole period of the pair: never clear of the blank, so the region its section sweeps in the plane is read exactly and the wheel is the blank less its prism. |
| `union.sv` | any class, member by member: a union | **A tool that is several tools.** A wheel of two beads that overlap, slid up a rod: where they meet the wheel's profile turns inward, a crease no one tool's sweep is built over, but a sweep of a union is its members' sweeps, so the groove is each bead's slid groove, both cut. Each bead's shadow is its tube's circle; the groove is what of the rod either covers. |

Larger cases of each class: the spiral bevel and hypoid pair (`spiral_bevel/`, generating), the
rack-cut spur gear (`generation/rack_cut_gear.sv`), the twist drill and the straight-fluted drill
(`twist_drill/`, a screw and a slide), the Wankel rotor (`wankel/`, the planar generating class,
whose pocket's inner envelope has no smaller case) and the swept spring (`swept_spring.sv`, a ball
on a screw).

Outside every class — a tumbling tool (`swept_tumble.sv`), a tool that stops inside the blank,
undercut — the sweep is read by its field and drawn by the page, and its export is refused, saying
which row it fails.

`rust/gcs-core/tests/solid_sweeps_examples.rs` holds each file to its class, to an exact build
whose STL agrees with the field, and to its volume; a file added here is named in this README and
read by a test there, or that file's last test fails.
