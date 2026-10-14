# A 1 hp e-bike hub motor

A direct-drive, outer-rotor, brushless permanent-magnet hub motor for a 26" rear wheel, rated at
one mechanical horsepower (746 W) from a 48 V pack. Every part is a Solvent component; the
assembly solves with no freedom left (`well`, DOF 0) and twelve clearance claims hold on the
solids.

`hub_motor.sv` is the assembly, `hub_motor.svd` the drawing (four sheets), `motor/` the parts:

| module | part | made as |
|---|---|---|
| `motor/dims.sv` | the dimension table: duty, electromagnetic sizing, every length | |
| `motor/axle.sv` | axle, key | 4140, turned, flats milled, keyway, Ø8 cable bore and radial port |
| `motor/carrier.sv` | stator carrier | 6061-T6 spider: hub keyed to the axle, web with six windows, rim |
| `motor/stator.sv` | lamination stack, windings, hall sensors | 100 × 0.35 mm M270-35A, bonded; 51 slots |
| `motor/rotor.sv` | rotor ring, magnets | 1018 tube (back iron and hub shell); 46 N42SH arc magnets |
| `motor/cover.sv` | side covers | A356-T6 castings, machined; spoke flange, bearing boss |
| `motor/parts.sv` | bearings, spacers, locknuts; drawing helpers | 6004-2RS; steel tube; M14 × 1.5 flanged nuts |

Each part file ends with a `preview { … }`: opened on its own, it draws the assembly's frame and
that part in it (the carrier on its axle, the brake-side cover on the ring's screw circle, and
the right-hand bearing, spacer and locknut for `motor/parts.sv`). Each solves with no freedom
left and builds the solid the assembly does; `use` leaves the block out.

## Rating

| | |
|---|---|
| output | 746 W (1 hp) continuous at 255 rpm: 32 km/h on a 26 × 2.1" tyre (2.09 m roll-out) |
| torque | 27.9 N·m rated; peak about 3× from a 40 A controller limit (use a torque arm) |
| supply | 48 V nominal, 18.1 A at rated load, 86 % efficient (≈120 W of loss) |
| topology | outer rotor, 46 poles / 51 slots, fractional-slot concentrated winding, star |
| air-gap shear | 11.9 kPa on a 103.1 mm gap radius and a 35 mm stack: air-cooled territory |
| cogging | lcm(51, 46) = 2346 cogs a turn: negligible |

The electromagnetic sizing is worked in the comments of `motor/dims.sv`. The flux density in the
gap is taken as 0.85 T from 3 mm of N42SH over a 0.6 mm gap. That is an estimate: confirm it with
a 2D FEA before tooling.

- Flux per pole: Φ = B_g · α · τ_p · L = 0.85 T · 0.80 · 14.1 mm · 35 mm = 0.335 mWb.
- Winding: 7 turns a tooth, 17 coils a phase (119 turns), 5 strands of Ø0.80 mm in hand.
  The winding factor is 0.94 (pitch 0.988, spread 0.955).
- Back-EMF at 255 rpm (97.8 Hz electrical): 16.5 V rms a phase, 28.5 V rms line to line, 40.3 V
  peak. That is under the pack's 48 V, which leaves the controller headroom.
- Current: 15.1 A rms a phase at rated torque, 6.0 A/mm² in the copper. The slot fill is 0.43
  (17.6 mm² of copper a coil side, two coil sides in each 81 mm² slot).
- Iron: the teeth run at 1.6 T, the stator yoke at 0.6 T (it is thicker than flux needs so it
  can take the press fit), and the rotor ring at 0.8 T.
- Hall sensors: three SS41s in the mouths of slots 0, 17 and 34. Seventeen slots are a third of
  a turn, and 17 slot pitches are exactly 240° electrical, so the three read the phases 120°
  apart.

## Stack-up along the axle (from the mid-plane, mm)

| station | what stands there |
|---|---|
| 0 … ±17.5 | the lamination stack and the magnets |
| ±17.5 … ±23.5 | the end turns |
| ±22 | the carrier's hub ends |
| ±23 … ±26 | each cover's spigot, inside the ring's bore |
| ±24 … ±36 | the 6004-2RS bearings, inner races on the axle's shoulders |
| ±26 | the rotor ring's ends, the covers' inner faces; spoke flanges ±26 … ±29 |
| ±36 … ±38 | the bearing lips |
| −38 … −52.4 | the ISO 6-bolt disc boss: M5 on Ø44, face 15.1 inboard of the dropout |
| +38 … +54 | the BSA freewheel thread, 1.375" × 24 TPI (cut, not drawn) |
| ±36 … ±60 | the spacers, inner race to the step |
| ±60 … ±67.5 | the flanged locknuts: 135 mm over locknuts |
| ±62 … ±95 | the axle's 10 mm flats, in the dropouts |

The spoke flanges carry 18 holes each (36 spokes, Ø2.6 for 14 g) on a 233 mm circle, staggered
half a pitch side to side. That is 2.2 mm clear of the ring for the spoke heads.

## How it is written

The motor is drawn in two views of one frame (see `motor/parts.sv`):

- **The end view** (`std.front`) is the electromagnetic section. One slot, one magnet and one
  hall sensor are drawn, and a `ring` turns each round: 51, 46 and 3 copies, with only the first
  of each solved for.
  - A slot is its walls' offsets from the two teeth's centrelines and the circles they stop on.
    Nothing is placed by a sine.
  - The lamination is the annulus less the 51 slot cutters.
  - The copper is the band the coils fill less the lamination. That one subtraction leaves a coil
    side in every slot and the end turns either side.
- **The half-section** (`std.side`) holds every turned profile: axle, carrier, covers, bearings,
  spacers and locknuts. Each is placed point by point by its station along the axle and its
  radius, and turned about the axle's centreline.
- **Features that are not turns** are prisms:
  - The axle's flats are cut sideways from the section.
  - The keyway is cut along the axle from the end view.
  - The cable port is drawn in the plan and swept straight up.
  - The holes (spoke, screw, window, tapped) are drawn round where they are seen round and swept
    through their part. The covers' screw holes are drawn on the very centres of the ring's tapped
    holes, which are handed in.

The claims at the end of `hub_motor.sv` are the fits the assembly depends on: the air gap, the end
turns clear of the covers, the hall sensors under the magnets and between the shoes, the carrier
clear of the bearings and covers, and the spacers clear of the lips.

```
make solventc
build/solventc rust/examples/hub_motor/hub_motor.sv              # solve, diagnose, judge the claims
build/solventc rust/examples/hub_motor/hub_motor.sv --where rotor.magnets.volume
build/solventc rust/examples/hub_motor/hub_motor.sv --stl lamination.stl --solid stator.lamination
build/solventc rust/examples/hub_motor/hub_motor.svd --sheet arrangement -o arrangement.svg
build/solventc rust/examples/hub_motor/motor/cover.sv          # one part, from its preview
```

The sheets are `arrangement` (end view and half-section), `axle`, `solids` (isometrics of the
parts) and `cover`. `tests/hub_motor.rs` holds the parts to closed forms computed from the table's
numbers, independently of the solver.

## What is not drawn

- Threads: the M14 axle ends, the freewheel thread and the tapped holes are drawn at their tap
  drills.
- The individual coils' shape; the slot liners and wedges.
- Seals, the hall sensors' leads and board, the cable and its strain relief.
- The torque arm and the wheel.
