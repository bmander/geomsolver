// The V-twin's dimension table, passed to components as `vtwin_dims`.
//
// A 90° V-twin *oscillating-cylinder* engine — a "wobbler" — run on shop air.  Each cylinder
// rocks on a bolt through the frame plate; its piston rod is rigid to the piston and its eye
// rides the one crank pin both banks share; and the rocking is the whole of the valve gear: a
// port drilled from the cylinder's face into the top of its bore sweeps across an intake port
// and an exhaust port in the plate, so the cylinder is fed while it is driven and vents while it
// returns.  No valves, no timing, three moving parts a bank.
//
// Every number is stated here once, with its unit; `use components.dims` puts the table in scope for
// root code that draws from it; a component receives it as `dims.D`.  Bank R's top dead centre is at
// crank angle `alphaR`, clockwise from top.  The parts are printed, with hardware-store metal
// where a printed part would wear, leak or be loaded in tension: a 5/16" steel rod is the
// crankshaft (7.94 mm — a press fit in a 608 skateboard bearing's 8 mm bore), a 1/4" clevis pin
// with a hairpin cotter is the crank pin, a 1/4"-20 hex bolt with its head trapped in the
// cylinder's face wall is each pivot, with a compression spring and a nylon-insert nut behind the
// plate; an O-ring seals each piston, #8-32 set screws in trapped nuts hold the disc and the
// flywheel to the shaft, and a 1/4" NPT brass coupling epoxied into the inlet boss takes the
// air line's quick-release plug.

use hardware            // what the fasteners and fittings measure, by name

// -- the engine ---------------------------------------------------------------------------
D := 16mm          // bore
R := 10mm          // crank throw: the stroke is 2R
L := 46mm          // piston rod: pin centre to the piston's crown
H := 30mm          // crank axis to a cylinder's pivot, along the bank
V := 90deg         // the included angle of the V
alphaR := V / 2    // bank R leans this far clockwise of vertical; bank L the same the other way
alphaL := -V / 2
theta0 := 180deg   // where the crank *starts*: a seed, read by nothing but seeds.  The crank
                        // angle itself is the drawing's one freedom, `crank.theta` — a name no
                        // statement defines, so the solver answers for it and a drag turns it
swing := asin(R / H)   // how far a cylinder rocks either side of its bank: 19.5°

// -- the cylinder -----------------------------------------------------------------------------
// Width and top follow bore plus wall; the mouth follows the skirt at bottom dead centre.
// Cylinder computes those dimensions locally. The head is shared with the port layout.
wall := 4mm
head := R + L + 2mm    // the bore's closed end: the crown at top dead centre, and 2 to spare
fwA := trapz + traph + 3mm  // bank A: bearing wall, head slot, then 3mm to the bore
fwB := fwA + rw        // bank B's is a rod thicker, so its rod rides the pin beside A's
zA := fwA + D / 2      // each rod's mid-plane, off the plate's face
zB := fwB + D / 2

// -- the piston and its rod ------------------------------------------------------------------
ph := 14mm             // piston, crown to skirt: its skirt is at the mouth at bottom dead centre
clr := 0.3mm           // piston to bore, a side — the O-ring seals it
rt := 5mm              // the rod's width in the plane of swing
rw := 6mm              // the rod's thickness along the crank axis
reye := 6.5mm          // the rod's eye, outside: 3.3 of wall round the pin
// The O-ring: a #014 (1/2" bore, 1/16" section).  A moving seal wants 10–20% squeeze on its
// section: the groove's bottom is the bore less twice 88% of the section, and the ring's own
// 12.4 bore stretches 4% onto it, which keeps it seated.  The groove is a third wider than the
// section, so the ring can roll a little rather than drag.
oring := hardware.oring014_cs
// Piston and throttle derive their grooves locally from their ring and `seal_dims`.
groove := 4mm                       // the groove's top, below the crown

// -- the ports ------------------------------------------------------------------------------
// The cylinder's port is `a` from the pivot on its axis; the plate's two ports sit on the arc it
// sweeps, `beta` either side of the bank.  A port is open while the two circles overlap.  The
// two ports are a port's width apart from the rock's end, so a port stays open through
// mid-stroke instead of closing again before it: `beta + dport / a` reaches `swing`.
a := head - 1mm - H    // the port's centre, 1 below the bore's end: 27 from the pivot
dport := 3.5mm
beta := 16deg          // the ports' own angle on the arc is dport / a, 7.4°
rpl := sqrt(H^2 + a^2 + 2 * H * a * cos(beta))   // every port's radius from the crank axis:
                            // the banks are one bank turned a quarter turn, so all four share it

// -- the pivot: a 1/4"-20 hex bolt, head trapped in the cylinder, nut behind the plate --------
// The head slides into a slot cut into the face wall from the cylinder's side, `trapz` behind
// the face, with the shank out through a hole in the face.  The bolt's tension then pulls the
// head against the wall between the slot and the face — plastic in compression — and the slot,
// a head's width across the flats, stops it turning.  (A pocket opening on the face would not
// do: the tension pulls the head *toward* the face, and nothing would hold the cylinder on.)
rstud := hardware.hexbolt14_d / 2       // the bolt's shank
boltaf := hardware.hexbolt14_af         // its head, across flats — the slot is this wide
boltac := hardware.hexbolt14_ac         // and across corners: how far the slot must reach past the axis
boltH := hardware.hexbolt14_h           // the head's height
trapz := 4mm           // the face to the slot: the wall the head bears on
traph := boltH + 0.6mm  // the slot height follows the selected bolt head
trapfit := hardware.fit14       // the hole through the wall the shank is located by; the plate's is the running fit
studclr := hardware.clearance14         // the plate's hole for the shank: it is the pivot's bearing
wsh := hardware.washer14_t              // a 1/4" flat washer
nutH := hardware.nylock14_h             // a 1/4"-20 nylon-insert nut
spring := 12mm         // the spring's working length between the plate and the washer

// -- the frame plate ------------------------------------------------------------------------
// One printed part: the plate the cylinders bear on, with the plenum channel inside it, the
// bearing boss and the foot on its back, the inlet boss on its top edge.  Printed foot down.
tp := 14mm             // the plate: thick enough to carry the plenum on its mid-plane
fx := 56mm             // its half-width
fy0 := -42mm           // its bottom edge, below the crank axis
fy1 := 66mm            // its top edge
fch := 26mm            // the chamfer off each top corner, each way
footd := 44mm          // the foot runs this far back from the plate's back face
footh := 8mm
shafthole := 8.5mm     // the shaft's clearance through the plate

// -- the crank train ------------------------------------------------------------------------
dshaft := hardware.rod516_d     // steel rod
rshaft := dshaft / 2
rbrg := hardware.brg608_od / 2  // 608 bearing: 22 outside, 8 bore, 7 wide — two, in the boss
wbrg := hardware.brg608_w
boss := brgpocket + 1.5mm  // bearing pocket plus the material left against the plate
brgpocket := 2 * wbrg + 0.5mm   // the pocket the two sit in, from the boss's back
rdisc := 18mm          // the crank disc, in front of the plate, clear of the cylinder mouths
zdisc := 1.4mm         // its clearance off the plate's face
tdisc := zA - rw / 2 - wsh - zdisc   // its thickness: rod A's near face, less a washer
dpin := hardware.clevis14_d     // the crank pin: a 1/4" × 1-1/4" clevis pin, its head in a pocket in
                            // the disc's back, the rods on its shank, a hairpin cotter outboard
rpin := dpin / 2
pinclr := dpin + 0.15mm  // diametral clearance in the disc and each rod eye
pinhead := hardware.clevis14_head_d     // the clevis pin's head
pinheadH := hardware.clevis14_head_t
pinpocket := pinheadH + 0.7mm  // clevis head plus recess clearance; shared with the side view
pingrip := hardware.clevis14_grip_114   // under the head to the cotter hole
dhub := 8mm            // the disc's and the flywheel's bore for the shaft
grub := hardware.screw832_clearance     // a #8-32 set screw's clearance hole, rim to bore
nutaf := hardware.nut832_af             // a #8-32 nut, across flats — the pocket it is trapped in
nutac := hardware.nut832_ac             // and across corners
nutT := hardware.nut832_t               // its thickness
nutin := 4mm           // the pocket starts this far out from the bore
rfw := 32mm            // the flywheel, behind the boss
wfw := 12mm
zfw := tp + boss + 4mm // its near face, behind the plate's front face

// -- the manifold and the throttle ----------------------------------------------------------
// The plenum runs inward of the ports, with short radial feeds to the two intakes. Keeping
// it on the ports' radius would also join the left exhaust. The inlet stands on the plate's
// top edge: a boss holding
// the brass coupling, and across the passage between the coupling and the plenum a rotary barrel
// throttle — a cross-drilled barrel that turns its hole out of line with the passage, an O-ring
// either side of the hole to seal it in its bore, a third behind the boss to retain it, and its
// lever on the front.
wch := 4mm             // the plenum channel, and the passage
rman := rpl - dport / 2 - wall - wch / 2  // plenum centreline, leaving wall to the exhaust
bossw := 24mm          // the inlet boss, across
bossz := 20mm          // and deep, centred on the plate's mid-plane
bossh := 98mm          // its top, above the crank axis
Ty := 72mm             // the throttle barrel's centre, above the crank axis
rbar := 5mm            // the barrel
lev := 22mm            // the throttle lever
levw := 4mm            // its width, and the hub's height off the boss
hubr := 4mm
throttle := 35deg      // the lever's angle off full open; 90 is shut
tor := hardware.oring010_cs     // a #010 O-ring (1/4" bore, 1/16" section)
torz := 5.5mm          // the two seals' grooves, either side of the cross-hole
tback := 4mm           // the barrel runs this far past the boss's back
tretain := 1.5mm       // the retaining ring's groove, behind the boss's back face
cpl := hardware.npt14_cpl_af    // the 1/4" NPT brass coupling: across flats, and its length
cpll := hardware.npt14_cpl_l
cplin := 18mm          // how deep it is set into the boss
cplhole := 16.5mm      // the boss's hole for it, epoxied
cplbore := hardware.npt14_drill // its bore, near enough: the tap drill for 1/4" NPT

// Component inputs: shared dimensions, selected hardware, and seal proportions.
// Each part derives its private sizes from these; extra root-only values stay above.
vtwin_dims := group(
  D: D, H: H, L: L, R: R, Ty: Ty,
  V: V, a: a, alphaL: alphaL, alphaR: alphaR, beta: beta,
  boltH: boltH, boltac: boltac, boltaf: boltaf, boss: boss, bossh: bossh,
  bossw: bossw, bossz: bossz, brgpocket: brgpocket, clr: clr, cpl: cpl,
  cplhole: cplhole, cplin: cplin, cpll: cpll, dhub: dhub, dport: dport,
  fch: fch, footd: footd, footh: footh, fwA: fwA, fwB: fwB,
  fx: fx, fy0: fy0, fy1: fy1, groove: groove, grub: grub,
  head: head, hubr: hubr, lev: lev, levw: levw, mplug_body_d: hardware.mplug_body_d,
  mplug_body_l: hardware.mplug_body_l, mplug_nose_d: hardware.mplug_nose_d, mplug_nose_l: hardware.mplug_nose_l, nutH: nutH, nutT: nutT,
  nutaf: nutaf, nutin: nutin, oring: oring, ph: ph, pinclr: pinclr,
  pingrip: pingrip, pinhead: pinhead, pinheadH: pinheadH, pinpocket: pinpocket, rbar: rbar,
  rbrg: rbrg, rdisc: rdisc, reye: reye, rfw: rfw, rman: rman,
  rpin: rpin, rpl: rpl, rshaft: rshaft, rstud: rstud, rt: rt,
  rw: rw, seal: hardware.seal_dims, shafthole: shafthole, spring: spring, studclr: studclr,
  swing: swing, tback: tback, tdisc: tdisc, theta0: theta0, throttle: throttle,
  tor: tor, torz: torz, tp: tp, trapfit: trapfit, traph: traph,
  trapz: trapz, tretain: tretain, wall: wall, wbrg: wbrg, wch: wch,
  wfw: wfw, wsh: wsh, zA: zA, zB: zB, zdisc: zdisc,
  zfw: zfw
)
