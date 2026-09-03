// The V-twin's dimension table: one module every view reads (§14.4).
//
// A 90° V-twin *oscillating-cylinder* engine — a "wobbler" — run on shop air.  Each cylinder
// rocks on a bolt through the frame plate; its piston rod is rigid to the piston and its eye
// rides the one crank pin both banks share; and the rocking is the whole of the valve gear: a
// port drilled from the cylinder's face into the top of its bore sweeps across an intake port
// and an exhaust port in the plate, so the cylinder is fed while it is driven and vents while it
// returns.  No valves, no timing, three moving parts a bank.
//
// Every number is stated here once, with its unit; `use vtwin.dims` puts the table in scope for
// whichever file draws from it, so a bore is `D` in every view.  Bank R's top dead centre is at
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
param D = 16mm          // bore
param R = 10mm          // crank throw: the stroke is 2R
param L = 46mm          // piston rod: pin centre to the piston's crown
param H = 30mm          // crank axis to a cylinder's pivot, along the bank
param V = 90deg         // the included angle of the V
param alphaR = V / 2    // bank R leans this far clockwise of vertical; bank L the same the other way
param alphaL = -V / 2
param theta0 = 180deg   // where the crank *starts*: a seed, read by nothing but seeds.  The crank
                        // angle itself is the drawing's one freedom, `crank.theta` — a name no
                        // statement defines, so the solver answers for it and a drag turns it
param swing = asin(R / H)   // how far a cylinder rocks either side of its bank: 19.5°

// -- the cylinder -----------------------------------------------------------------------------
param wall = 4mm
param hw = D / 2 + wall     // the body's half-width in the plane of swing
param cb = 22mm             // the mouth of the bore, from the crank axis along the bank
param head = R + L + 2mm    // the bore's closed end: the crown at top dead centre, and 2 to spare
param ct = head + wall      // the body's top
param fwA = 12mm            // bank A's face wall, the face to the bore: the pivot bolt's head sits
                            // in a slot in it (`trapz`, `traph`), 3 of wall left to the bore
param fwB = fwA + rw        // bank B's is a rod thicker, so its rod rides the pin beside A's
param tcylA = fwA + D + wall
param tcylB = fwB + D + wall
param zA = fwA + D / 2      // each rod's mid-plane, off the plate's face
param zB = fwB + D / 2

// -- the piston and its rod ------------------------------------------------------------------
param ph = 14mm             // piston, crown to skirt: its skirt is at the mouth at bottom dead centre
param clr = 0.3mm           // piston to bore, a side — the O-ring seals it
param rt = 5mm              // the rod's width in the plane of swing
param rw = 6mm              // the rod's thickness along the crank axis
param reye = 6.5mm          // the rod's eye, outside: 3.3 of wall round the pin
// The O-ring: a #014 (1/2" bore, 1/16" section).  A moving seal wants 10–20% squeeze on its
// section: the groove's bottom is the bore less twice 88% of the section, and the ring's own
// 12.4 bore stretches 4% onto it, which keeps it seated.  The groove is a third wider than the
// section, so the ring can roll a little rather than drag.
param oring = oring014_cs
param grooveb = 12.9mm      // the groove's bottom diameter: the bore less twice the squeezed section
param groovew = 2.4mm       // the groove's width: `oring_groove_w` sections
param groove = 4mm                       // the groove's top, below the crown

// -- the ports ------------------------------------------------------------------------------
// The cylinder's port is `a` from the pivot on its axis; the plate's two ports sit on the arc it
// sweeps, `beta` either side of the bank.  A port is open while the two circles overlap.  The
// two ports are a port's width apart from the rock's end, so a port stays open through
// mid-stroke instead of closing again before it: `beta + dport / a` reaches `swing`.
param a = head - 1mm - H    // the port's centre, 1 below the bore's end: 27 from the pivot
param dport = 3.5mm
param beta = 16deg          // the ports' own angle on the arc is dport / a, 7.4°
param rpl = sqrt(H^2 + a^2 + 2 * H * a * cos(beta))   // every port's radius from the crank axis:
                            // the banks are one bank turned a quarter turn, so all four share it

// -- the pivot: a 1/4"-20 hex bolt, head trapped in the cylinder, nut behind the plate --------
// The head slides into a slot cut into the face wall from the cylinder's side, `trapz` behind
// the face, with the shank out through a hole in the face.  The bolt's tension then pulls the
// head against the wall between the slot and the face — plastic in compression — and the slot,
// a head's width across the flats, stops it turning.  (A pocket opening on the face would not
// do: the tension pulls the head *toward* the face, and nothing would hold the cylinder on.)
param rstud = hexbolt14_d / 2       // the bolt's shank
param boltaf = hexbolt14_af         // its head, across flats — the slot is this wide
param boltac = hexbolt14_ac         // and across corners: how far the slot must reach past the axis
param boltH = hexbolt14_h           // the head's height
param trapz = 4mm           // the face to the slot: the wall the head bears on
param traph = 5mm           // the slot's height, the head and 0.6 to spare
param trapw = boltaf + 0.3mm    // the slot's width: the head's flats, and a little
param trapd = boltac / 2 + 0.3mm    // the slot runs this far past the axis, so the head centres on it
param trapfit = fit14       // the hole through the wall the shank is located by; the plate's is the running fit
param studclr = clearance14         // the plate's hole for the shank: it is the pivot's bearing
param wsh = washer14_t              // a 1/4" flat washer
param nutH = nylock14_h             // a 1/4"-20 nylon-insert nut
param spring = 12mm         // the spring's working length between the plate and the washer

// -- the frame plate ------------------------------------------------------------------------
// One printed part: the plate the cylinders bear on, with the plenum channel inside it, the
// bearing boss and the foot on its back, the inlet boss on its top edge.  Printed foot down.
param tp = 14mm             // the plate: thick enough to carry the plenum on its mid-plane
param fx = 56mm             // its half-width
param fy0 = -42mm           // its bottom edge, below the crank axis
param fy1 = 66mm            // its top edge
param fch = 26mm            // the chamfer off each top corner, each way
param footd = 44mm          // the foot runs this far back from the plate's back face
param footh = 8mm
param shafthole = 8.5mm     // the shaft's clearance through the plate

// -- the crank train ------------------------------------------------------------------------
param dshaft = rod516_d     // steel rod
param rshaft = dshaft / 2
param rbrg = brg608_od / 2  // 608 bearing: 22 outside, 8 bore, 7 wide — two, in the boss
param wbrg = brg608_w
param boss = 16mm           // the bearing boss behind the plate
param brgpocket = 2 * wbrg + 0.5mm   // the pocket the two sit in, from the boss's back
param rdisc = 18mm          // the crank disc, in front of the plate, clear of the cylinder mouths
param zdisc = 1.4mm         // its clearance off the plate's face
param tdisc = zA - rw / 2 - wsh - zdisc   // its thickness: rod A's near face, less a washer
param dpin = clevis14_d     // the crank pin: a 1/4" × 1-1/4" clevis pin, its head in a pocket in
                            // the disc's back, the rods on its shank, a hairpin cotter outboard
param rpin = dpin / 2
param pinclr = 6.5mm        // the hole for it, in the disc and in each eye
param pinhead = clevis14_head_d     // the clevis pin's head
param pinheadH = clevis14_head_t
param pinpocket = 3mm       // the pocket in the disc's back the head sits in
param pinpocketd = 10.2mm
param pingrip = clevis14_grip_114   // under the head to the cotter hole
param dhub = 8mm            // the disc's and the flywheel's bore for the shaft
param grub = screw832_clearance     // a #8-32 set screw's clearance hole, rim to bore
param nutaf = nut832_af             // a #8-32 nut, across flats — the pocket it is trapped in
param nutac = nut832_ac             // and across corners
param nutT = nut832_t               // its thickness
param nutin = 4mm           // the pocket starts this far out from the bore
param rfw = 32mm            // the flywheel, behind the boss
param wfw = 12mm
param zfw = tp + boss + 4mm // its near face, behind the plate's front face

// -- the manifold and the throttle ----------------------------------------------------------
// Both intake ports are one radius from the crank axis, so a plenum arc concentric with the
// crank, inside the plate, joins them.  The inlet stands on the plate's top edge: a boss holding
// the brass coupling, and across the passage between the coupling and the plenum a rotary barrel
// throttle — a cross-drilled barrel that turns its hole out of line with the passage, an O-ring
// either side of the hole to seal it in its bore, a third behind the boss to retain it, and its
// lever on the front.
param wch = 4mm             // the plenum channel, and the passage
param bossw = 24mm          // the inlet boss, across
param bossz = 20mm          // and deep, centred on the plate's mid-plane
param bossh = 98mm          // its top, above the crank axis
param Ty = 72mm             // the throttle barrel's centre, above the crank axis
param rbar = 5mm            // the barrel
param barbore = 10.2mm      // its bore in the boss
param dhole = wch           // its cross-hole
param lev = 22mm            // the throttle lever
param levw = 4mm            // its width, and the hub's height off the boss
param hubr = 4mm
param throttle = 35deg      // the lever's angle off full open; 90 is shut
param tor = oring010_cs     // a #010 O-ring (1/4" bore, 1/16" section)
param torgb = 6.9mm         // its groove's bottom diameter, sized as the piston's is
param torw = 2.4mm
param torz = 5.5mm          // the two seals' grooves, either side of the cross-hole
param tback = 4mm           // the barrel runs this far past the boss's back
param tretain = 1.5mm       // the retaining ring's groove, behind the boss's back face
param cpl = npt14_cpl_af    // the 1/4" NPT brass coupling: across flats, and its length
param cpll = npt14_cpl_l
param cplin = 18mm          // how deep it is set into the boss
param cplhole = 16.5mm      // the boss's hole for it, epoxied
param cplbore = npt14_drill // its bore, near enough: the tap drill for 1/4" NPT
