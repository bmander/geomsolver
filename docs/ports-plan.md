# Ports: parts that say how they are joined: plan

**Status (2026-11-13): stages 1, 3 (in part), 4 (in part) and 5 built; stage 2 and fits not
yet.** It comes from the shaft-drive benchmark (`rust/examples/shaft_drive/`, #906, #904). The
question it answers is how a part says where and how it is joined, so that joining it costs a
document one statement that carries the intent. [Results](#results) says what the experiment
found; §2 onward is the design as it was proposed, and [Open questions](#open-questions) what is
still to decide.

## Results

**What was built.**
- **Stage 1, typed operands**, as P2 proposes: `WordDef::types`, a typed operand bound as a typed
  formal is, E103 at the use and E101 in the body (`tests/ports.rs`).
- **A rule the plan did not foresee, found by building it.** A port's frame must be the part's
  own, or a mate through the port has nothing to move. An instance stood in a frame of its own
  only where something outside read its `std` by its written path (#548), and a mate through a
  port reads it through a group. So **a definition that hands out a datum of its own in a group
  it declares now stands in a frame of its own** (`framed::exports_datum`). Without it, every
  mate conflicted with the instance standing at the world's frame.
- **`std.ports`**: the interfaces `Seat`, `Seated`, `Flat`, `RingSeat`, `Grooved`; the mates
  `seats_in` (frames made one, and the room the part takes cut from the seat's host), `abuts`
  (faces against each other) and `grooves` (a ring's groove cut in what it is fitted to); and
  `FrameOn` and `Station`, a port's frame on an axis.
- **`std.bearings` and `std.retaining`**: ISO 15 and DIN 471/472 rows for the sizes the drive
  uses, `Bearing`, `Ring471`, `Ring472` with their ports, and P6's bundles `OnShaft` and `InBore`.
- **Not built.**
  - Stage 2, the host rule: a word or component may still cut what any group it is handed names.
  - P4's fits: `std.fits` and its intervals. The shaft's and spindle's diameters read their
    bearings' bores from the catalogue instead.
  - Catalogue rows beyond those used.

**The experiment** (stage 5): the shaft drive's seven bearings and seven rings, rewritten.

| | statements for the joints | number rows in `dims.sv` | offsets tuned by eye | the design's own hardware file |
|---|---|---|---|---|
| before | about 60: 14 stations, 6 grooves and 6 cuts, a seat's 3, the sleeve groove's 3, 14 parts and 14 `seats` | 12 (sizes, a restated bore ×2, a restated seat, a groove's place) | 6 (`0.8mm`) | 131 lines |
| ports, the mates written out | 46: per joint a station, the bearing, `seats_in`, the ring, `abuts`, `grooves` | 0 | 0 | none |
| ports, the bundles (P6) | 21: per joint a station and an `OnShaft` or `InBore`, and 4 journals and an axis | 0 | 0 | none |

- **The geometry is the same.** All ten clearance claims hold, at the same margins to a
  thousandth of a millimetre. Every body a mate cuts writes an exact STL: the shell, the bearing
  cup, the hub, the spindle, the shaft.
- **What it did pay for.** A joint is about two statements instead of about nine. What remains is
  what a designer decides: which bearing, where its seat stands, which ring. The bore, the room,
  the groove and the ring's place are derived, and no number is stated twice.
- **What it did not pay for.**
  - Mates written out save little: 46 statements against about 60. The gain is in correctness,
    not length. The brevity comes from P6.
  - The bundles fix a convention, and a designer must know it: a station points the way its
    bearing went in, and the ring is on its trailing side.
- **What it cost, and what that turned out to be.**
  - At first, placing the planes before the solve took 25 s against 1.2 s. The solve itself is
    0.2 s either way.
  - Two causes, neither in the ports' idea:
    - The ports' frames were stated over their normals. Placement reads a normal off its plane's
      axes and never solves it, so every placing round ran to its iteration limit. Stated over
      `u` and `v` (a frame's `u` square to its axis and `v` turned a quarter from it), placing took
      6 s.
    - `elaborate`'s passes placed the same planes from the same input again and again: forty
      placings of twenty inputs. Placed once for each input (`views::PlaceMemo`), placing takes
      2.8 s.
  - The drive on ports now elaborates and solves in 13 s, where the drive before ports took 15 s
    on v0.1.19. The same memo takes the drive before ports to 9.8 s.
  - What is left is the claims: judging `clear` on the built solids is most of the full run's
    2 min 40 s, before ports and after.

## 1. Why

The shaft drive places seven deep-groove bearings and seven retaining rings. Here is everything
`drive.sv` and `dims.sv` say to put one bearing, the spindle's right 6804, on its shaft and in its
housing, with its ring:

```
bearing_od := 32mm                     // dims.sv: the spindle's 6804s, 20 × 32 × 7
b6804 := {d: 20mm, D: 32mm, B: 7mm, balls: 14, dw: 3.5mm}
r471_20 := {groove: 19mm, t: 1.2mm, band: 2.6mm, r_eye: 13.4mm, eye: 3.4mm, hole: 1.5mm}

right_bb := At(bb_line, std.origin, d: case_right + end_wall / 2)
right_clip := At(bb_line, std.origin, d: case_right + end_wall + 0.8mm)
right_spindle_groove := Groove(bb_line, right_clip.p, r: r471_20.groove / 2,
  r_out: spindle_d.x / 2 + 1mm, t: r471_20.t)
right_spindle_groove.body cut spindle
right_seat_set := { q | q in std.sets.Cylinder(bb_line, r: bearing_od / 2)
    std.origin distance(in: (…), along: bb_line) q }
construction right_seat := solid(right_seat_set)
right_seat cut shell
right_6804 := hardware.Bearing(b6804)
right_6804 seats(turn: 0deg, up: drive_side, east: std.x) right_bb.p
right_ring := hardware.Ring471(r471_20)
right_ring seats(turn: 0deg, up: drive_side, east: std.x) right_clip.p
```

That is fifteen statements for one design decision ("a 6804 carries the spindle's right end in
the shell, held by a ring"), and four of them break the Design Bible:

- **A fact said twice** (principle 2). `bearing_od := 32mm` restates `b6804.D`. Change the bearing and the
  housing's bore silently stays 32.
- **Numbers a designer would not state** (principle 3).
  - `0.8mm` puts the ring's groove where it looked right: it is a result disguised as an input.
  - The groove's diameter of 19 is copied from DIN 471's table, not looked up in it.
- **A position given by coordinates** (principle 3). Each station is an ordinate from the world's origin, where
  the designer's intent is "against the shell's end wall".
- **The material consequence is the parent's chore.** The bore in the shell and the groove in the
  spindle exist because of the bearing and the ring. Yet each is drawn, sized and cut by hand,
  away from the part that needs it.

Across the drive the glue is linear in the hardware: 53 of `drive.sv`'s 333 lines only place
bearings and rings. Solvent's repetition (`ring`, `repeat`) and its one-build-per-binding make
regular structure cheap. What stays dear is the *irregular joint*, and that is most of a machine.

The Design Bible asks that "a design says 'a groove for a #014 O-ring', and the library does the
arithmetic" (principle 4), that "each fact is stated once, and everything else is derived", and
that positions be "given as the shop would dimension the part" (principle 3). A joint is where
those principles currently fail.

## 2. What already works

This plan is mostly library, because the language nearly has what it needs. Each of these
elaborates and builds on v0.1.19:

1. **An interface can type geometry, not only numbers.** `interface Shoulder { at: plane, host:
   solid }` is accepted. A component's group of its own plane and its body, `face := {at: f,
   host: body}`, is accepted where an argument typed `Shoulder` is expected. Interfaces are
   structural: nothing says it is a `Shoulder` but having the members (primer 1.8).
2. **A component can expose such a group, and its members are read through the instance.**
   `a.journal` names `Shaft`'s `journal := {at: tip, axis: ax, d: d}`.
3. **A relation word reads its operands' members.** `s meets h := { s.at coincident h.at … }`
   applied as `a.journal meets b.journal` elaborates, and its rows are the solve's.
4. **A relation word's body may change a solid its operands carry.** `p seats_in h := { p.at
   coincident h.at; p.relief cut h.host }` with `pin.end seats_in b.face` builds `b.body` as a
   20 mm ball less a 5 mm one, its volume exact to the cubic millimetre.

The one thing rejected is **typing a word's operands**: `p: End seats_in h: Face := {…}` is
E001, "a statement relates two things with a word between them". An untyped word given the wrong
group fails inside its body ("no such entity: `x.relief`"), in the body's names rather than the
author's.

So ports exist today **by convention**. This plan makes the convention the language's:
- one grammar change, which gives a meaning to a construct the compiler rejects;
- one rule about what a mate may change;
- a standard library of ports, mates and tables.

## 3. The design

### P1. A port is a member group a component declares, typed by an interface

There is no `port` keyword. A port is what §2 shows: a group of the component's own entities,
numbers and solids, which an interface describes. The interfaces of mechanical joints live in a
new module, `std.ports`, written in Solvent:

```
// a cylindrical seat on a shaft: its frame's origin at the seat's end, its normal along the
// axis into the seat; the axis; the seat's circle, drawn in the frame; the body it is turned in
interface Journal { at: plane, axis: line, seat: circle, host: solid }
// a cylindrical seat in a housing, the same way round
interface Bore { at: plane, axis: line, seat: circle, host: solid }
// a flat face square to an axis: a shoulder, a housing's end wall, a bearing's side
interface Face { at: plane, host: solid }
```

**A port's frame is a plane**, because a plane is a frame (bible, Frames). Frames are the unit
of placement, and a mate is a constraint between the origins, axes or planes of two frames
(bible, Terms). A bearing declares its ports in its own frame, from its own numbers:

```
component Bearing(s: BearingSize) {
  …                                   // rings, balls: as hardware.sv draws them
  bore := {at: inner_frame, axis: axis_line, seat: bore_circle, host: inner_ring}   // a Bore
  od := {at: outer_frame, axis: axis_line, seat: od_circle, host: outer_ring}       // a Journal
  left := {at: left_frame, host: inner_ring}                 // Faces: its two sides
  right := {at: right_frame, host: inner_ring}
}
```

A part typed this way needs nothing new to be read: `right_6804.od.seat` is a circle like any
other.

### P2. A mate is a relation word over ports, its operands typed

The grammar change is one: **a defined word's operands may be typed by an interface**.

```
s: Journal fitted(fit) h: Bore := { … }
```

- **A use given the wrong group is E103 at the operand**, naming the interface and the missing
  member ("`spindle.end` is no `Journal`: it has no `seat`"). Inside the body there is nothing left
  to fail.
- An untyped operand stays legal and means what it means today.
- Typing is structural, as for a component's formals (primer 1.8). Inside the body an operand
  holds only what its interface declares.

The mates of `std.ports` are words, each a few relations written in Solvent:

```
// a journal fitted in a bore: one axis (a point of one on the other, the two parallel, since
// two lines are not said collinear), and one diameter
s: Journal fitted(fit) h: Bore := {
  s.at.origin coincident h.axis
  s.axis parallel(sense: same) h.axis
  s.seat equal h.seat                 // the nominal; the fit's interval is P4's
}
// two faces against each other: one plane, facing each other
a: Face abuts b: Face := {
  a.at.origin coincident b.at
  a.at.normal parallel(sense: opposite) b.at.normal
}
```

`seats` (in `std.place`) is the mate of a part's whole frame at a point. These are mates of its
ports. Each mate is a constraint between frames, as the bible defines one. When a joint
conflicts, its culprit reads as the word (`right_6804.bore fitted(k6) spindle.end`), as a defined
word's does today.

### P3. A port may open its host to the part it is mated with

Section 2's fourth case lets any relation word cut any solid its operands name. P3 gives that
power a rule:

- **A mate may change a solid only through a port that names it as its `host`.** Naming the host
  is the port's consent to be cut, or added to, by what it is mated with.
- Nothing else may reach into another part's body. A component that cuts its argument's solid
  outside a port stays refused (E109's rule: a definition states no fact about its inputs alone).

What a mate adds or takes is the region a port carries:
- a ring's `groove`, which its mate cuts from the journal's host;
- a bearing's `seat`, which its mate cuts from the bore's host.

So the material a joint needs is defined once, in the part that needs it, from its own numbers.
The cut itself is a fact of the parent's body, and its place is irrelevant: a body is its stock,
plus `union`, minus what `cut`s it, wherever those statements stand (solids.md). So **the part
stays a function of its arguments**. The bearing never reaches outside its scope. The mate,
stated in the parent, says what the joint does to both parts. A component that bundles a joint
(P6) does the same in its own body, over ports it is given.

**Caching survives.** A part's binding is still built once and placed (#566). A host cut by a mate
is the parent's solid, built from the part's placed body and the regions cut from it. This is how
`head.plug cut top.body` and `sleeve_groove cut sun.body` build today.

### P4. A fit is a table, and a toleranced diameter is a set

- **The numbers come from a table.** `fitted(k6)` reads ISO 286 from `std.fits`, a module of
  Solvent tables like `std.ports`. It gives:
  - the bore's and journal's deviation intervals for the nominal size;
  - an `H7` hole, whose nominal size is the bearing's catalogue diameter.
- **The solve reads the nominal.** The journal's diameter is the bore's.
- **The interval belongs to claims.** A tolerance is a set (bible, principle 3: a toleranced
  dimension is an interval), so a claim such as `clear` holds for every acceptable part.
- **Its arrival can wait.** That worst-case judging is the bible's and not this plan's. Until it
  arrives, a fit's interval is recorded, reported and not yet judged.
- **One number for the shaft.** The designer states one thing, the fit class, and the journal's
  diameter is derived from the bearing. `Shaft` leaves its seat's diameter an unbound numeric
  formal, so its seat circle's radius is an unknown of the caller, and the mate's `equal`
  determines it. That is the existing rule: an argument left unbound becomes an unknown of the
  caller, and a parent may fix it by a relation on the instance's outputs.

Catalogues come the same way. `std.bearings` is a table of ISO 15 sizes by designation, and
`std.retaining` a table of DIN 471 and DIN 472. Each is a module of groups, read by name. A design
says `Bearing(std.bearings.s6804)`, and the 32 mm housing bore is never typed.

### P5. A position follows from what the part is against

The stations of §1 disappear when a part's axial position is a mate rather than an ordinate:

- The bearing's `left` face `abuts` the housing's end wall's inner `Face`.
- The ring's face abuts the bearing's `right` face.
- The ring's `groove` is cut wherever that leaves it.

The 0.8 mm guess becomes the ring's own number, the axial play its table gives. These are
positions "as the shop would dimension the part", against features, never from the world's
origin.

**A face a port needs may be made by the mate.**
- A shaft has no groove until a ring is fitted to it. The ring's `seat` port, typed
  `interface RingSeat { at: plane, axis: line, groove: solid }`, carries the groove as a region
  in the ring's own frame, and `clip.seat retains shaft` cuts it from the journal's host.
- A part's feature that exists only because of a joint is defined by the part that causes it, and
  appears in its host when the two are joined.

### P6. A joint of joints is a component

The standard library can then hold the arrangements a machine designer reaches for: a locating
bearing, a floating bearing, a shaft held at both ends. Each is a component whose formals are
ports:

```
// a bearing carrying `shaft` in `housing`, located against `wall` and held by a ring
component LocatingBearing(size: BearingSize, ring: RingSize, shaft: Journal, housing: Bore,
                          wall: Face) {
  b := Bearing(size)
  clip := Ring471(ring)
  b.bore fitted(k6) shaft
  b.od fitted(H7) housing
  b.left abuts wall
  clip.face abuts b.right
  clip.seat retains shaft
}
```

Its rows place everything it makes relative to its inputs, which is the definition rule (E108,
E109). The two cuts reach the shaft's and the housing's bodies only through the ports it was
given (P3). The drive's right spindle bearing becomes:

```
right := LocatingBearing(std.bearings.s6804, ring: std.retaining.din471_20,
                         shaft: spindle.right_end, housing: shell.right_bore,
                         wall: shell.right_wall)
```

That is one statement for one design decision. It carries intent the fifteen statements did not:
the bearing, its fits, what locates it and what holds it. The spindle's right journal diameter,
the shell's bore, the groove and both stations are derived.

**This is where the cost of a joint falls.** It is the cost of a design decision, not of its
consequences. A machine's text then grows with the decisions in it: an irregular assembly of n
joints still needs about n statements, one per joint, and that is the information it holds. The
statements no longer carry its geometry. Regular structure keeps the costs it already has: O(1)
text for a `ring` and one build per binding.

## 4. What this plan does not add

- **No `port`, `assembly`, `connector` or `mate` keyword.** A port is an interface-typed group,
  and a mate is a relation word (bible: Solvent shrinks; there is no assembly level).
- **No graph of connections.** A port is not connected to one other port. Mates are relations,
  and a port may take several: a journal both fitted and abutted.
- **No order.** Which mate is written first changes nothing: a cut by a mate is a fact of a body.
- **No second spelling.** A mate written out by hand, as §2's cases are, elaborates to the same
  rows and the same solids as the word. The word is a library definition, not a privileged
  path. The evaluator recognises nothing by a word's name.
- **No kernel feature.** Every solid a mate makes is a set already lowered today: a cylinder's
  band, a bore.

## 5. Conformance with the Design Bible

| principle | how this plan keeps it |
|---|---|
| 2: Solvent shrinks | One grammar change, typed operands on a word, which gives meaning to E001. Everything else is library: `std.ports`, `std.fits`, `std.bearings`, `std.retaining`. |
| 2: a constraint is a relation word | Every mate is a word between its operands: `b.bore fitted(k6) shaft`. |
| 2: each fact stated once | A bearing's size lives in its catalogue row; the bore, the journal, the groove and the stations are derived. |
| 2: two spellings, one result | A mate written out by hand and its word give identical rows and solids. |
| 3: numbers a designer states | Fit classes, catalogue designations and abutments replace a restated diameter, a table copied by hand and a guessed offset. |
| 3: no cartesian placement | A part is placed against features, not by ordinates from the world's origin. |
| 3: a tolerance is a set | A fit is an interval, judged by claims for every acceptable part once worst-case claims arrive. |
| 4: functional components | A part reaches only its own scope. A cut into another part's body is a mate's, and only through a port that consents (P3). |
| 4: frames are fundamental | A port's frame is a plane; a mate relates two frames. |
| 4: knowledge in the library | ISO 286, ISO 15 and DIN 471/472 are Solvent tables. |
| 1: refuse with a reason | A wrong port is E103 at the operand, naming the interface and the missing member. |

## 6. Stages

Each stage lands with its tests (`rust/gcs-core/tests/ports.rs`), its spec and primer rows, its
contract paragraph (language.md, relations.md) and a changelog entry.

1. **Typed word operands.**
   - Parse `s: I word(…) h: J := …`, check uses structurally, and give E103 at the operand.
   - Inside the body an operand holds only its interface's members, as a typed formal does
     (E101 otherwise).
   - Gate: a word given the wrong group is refused at the use, naming the missing member, and a
     word given the right one elaborates to the rows the untyped word gives.
2. **P3, the host rule.**
   - A word or a component may `cut`, `union` or `common` a solid only where it is its own, or a
     port's `host` reached through the port's interface.
   - Gate: §2's fourth case builds as it does. The same cut reached through an untyped group, or
     a component cutting an argument's solid directly, is refused naming the rule.
3. **`std.ports` and `std.fits`.**
   - The interfaces of P1, the mates `fitted`, `abuts`, `retains`, and ISO 286 for a first set of
     classes (H7, k6, j6, m6, h6).
   - Gate: a shaft fitted in a bore solves with its diameter derived, and `--where` reports the
     fit's interval.
4. **Catalogues.**
   - `std.bearings` (ISO 15, the 618/619/60/62 series) and `std.retaining` (DIN 471/472), each
     row held by its table's claims (d < D, a groove under the journal).
   - `hardware.sv`'s `Bearing` and `Ring471`/`Ring472` move to the library with their ports.
5. **The shaft drive rewritten on ports.**
   - Gate: the same solved positions to 1e-9, the same ten clearance claims, and the text the
     joints take measured before and after.
   - This stage is the experiment the bible asks for before new syntax is exposed (#461): if the
     drive's joints do not get shorter and clearer, stages 1–4 are reconsidered.

## Open questions

1. **Is `host` a convention or a name the language knows?** P3 needs the elaborator to know which
   member a port consents with.
   - A member named `host` is the smallest change, but it makes a name meaningful.
   - The alternative is a member typed `solid` in an interface marked as a port. That spends a
     keyword.
   - This plan prefers the name, documented in `std.ports`.
2. **Should a mate be able to *add* material to a host** (a boss for a press-fit pin), or only
   take it away? Taking away covers every joint the shaft drive needs.
3. **Worst-case claims over fits** belong to the bible's tolerance work (principle 3, a design is a
   set of acceptable parts). Until then, should a fit's interval be reported on `--where`, or held
   back until it can be judged?
4. **Joints that move.** `turns`, `spins` and `slides` (`std.motion`) relate frames over a running
   parameter. A revolute mate between ports, `b.bore turns_on shaft`, is `fitted` plus a joint's
   value. Should it be in stage 3, or wait for a mechanism that needs it?
