// The V-twin's dimension table: one module every view reads (§14.4).
//
// A 90° V-twin *oscillating-cylinder* engine — a "wobbler" — run on shop air.  Each cylinder
// rocks on a stud through the frame plate; its piston rod is rigid to the piston and its eye
// rides the one crank pin both banks share; and the rocking is the whole of the valve gear: a
// port drilled from the cylinder's face into the top of its bore sweeps across an intake port
// and an exhaust port in the plate, so the cylinder is fed while it is driven and vents while it
// returns.  No valves, no timing, three moving parts a bank.
//
// Every number is stated here once, with its unit; `use vtwin.dims` puts the table in scope for
// whichever file draws from it, so a bore is `D` in every view.  Bank R's top dead centre is at
// crank angle `alphaR`, clockwise from top.  The parts are printed, with
// hardware-store metal where a printed part would wear or leak: a 5/16" steel rod is the
// crankshaft (7.94 mm — a press fit in a 608 skateboard bearing's 8 mm bore), 3/16" rod the
// crank pin, 1/4"-20 threaded rod the pivot studs with a compression spring and a nylon-insert
// nut each, an O-ring on each piston, and a 1/4" NPT brass coupling epoxied into the inlet boss
// for the air line's quick-release plug.

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

// -- the cylinder and its piston -------------------------------------------------------------
param wall = 4mm
param hw = D / 2 + wall     // the body's half-width in the plane of swing
param cb = 22mm             // the mouth of the bore, from the crank axis along the bank
param head = R + L + 2mm    // the bore's closed end: the crown at top dead centre, and 2 to spare
param ct = head + wall      // the body's top
param ph = 14mm             // piston, crown to skirt: its skirt is at the mouth at bottom dead centre
param clr = 0.3mm           // piston to bore, a side — the O-ring seals it
param groove = 4mm          // the O-ring groove, below the crown
param oring = 2.6mm         // the O-ring's section (a #112: 1/2" bore, 3/32" section)
param rt = 5mm              // the rod's width in the plane of swing
param rw = 6mm              // the rod's thickness along the crank axis
param reye = 5mm            // the rod's eye, outside

// -- the ports ------------------------------------------------------------------------------
// The cylinder's port is `a` from the pivot on its axis; the plate's two ports sit on the arc it
// sweeps, `beta` either side of the bank.  A port is open while the two circles overlap.  The
// two ports are a port's width apart from the rock's end, so a port stays open through
// mid-stroke instead of closing again before it: `beta + dport / a` reaches `swing`.
param a = head - 1mm - H    // the port's centre, 1 below the bore's end: 27 from the pivot
param dport = 3.5mm
param beta = 16deg          // the ports' own angle on the arc is dport / a, 7.4°

// -- the frame plate ------------------------------------------------------------------------
// One printed part: the plate the cylinders bear on, with the plenum channel inside it, the
// bearing boss and the foot on its back, the inlet boss on its top edge.  Printed foot down.
param tp = 14mm             // the plate: thick enough to carry the plenum on its mid-plane
param fx = 56mm             // its half-width
param fy0 = -42mm           // its bottom edge, below the crank axis
param fy1 = 66mm            // its top edge
param fch = 26mm            // the chamfer off each top corner, each way
param rstud = 0.125"        // the pivot stud: 1/4"-20 threaded rod, through the plate
param footd = 44mm          // the foot runs this far back from the plate's back face
param footh = 8mm

// -- the crank train ------------------------------------------------------------------------
param dshaft = 0.3125"      // steel rod
param rshaft = dshaft / 2
param rbrg = 11mm           // 608 bearing: 22 outside, 8 bore, 7 wide — two, in the boss
param wbrg = 7mm
param boss = 16mm           // the bearing boss behind the plate
param rdisc = 18mm          // the crank disc, in front of the plate, clear of the cylinder mouths
param tdisc = 10mm
param zdisc = 1.5mm         // its clearance off the plate's face
param dpin = 0.1875"        // steel rod, pressed into the disc
param rpin = dpin / 2
param fwA = 8mm             // bank A's face wall, the face to the bore: the stud threads into it
param fwB = fwA + rw        // bank B's is a rod thicker, so its rod rides the pin beside A's
param tcylA = fwA + D + wall
param tcylB = fwB + D + wall
param zA = fwA + D / 2      // each rod's mid-plane, off the plate's face
param zB = fwB + D / 2
param rfw = 32mm            // the flywheel, behind the boss
param wfw = 12mm
param zfw = tp + boss + 4mm // its near face, behind the plate's front face

// -- the manifold and the throttle ----------------------------------------------------------
// Both intake ports are one radius from the crank axis — the two banks are one bank turned a
// quarter turn — so a plenum arc concentric with the crank, inside the plate, joins them.  The
// inlet stands on the plate's top edge: a boss holding the brass coupling, and across the
// passage between the coupling and the plenum a rotary barrel throttle — a cross-drilled
// barrel that turns its hole out of line with the passage, its lever on the front.
param rpl = sqrt(H^2 + a^2 + 2 * H * a * cos(beta))   // the intake ports' radius from the crank axis
param wch = 4mm             // the plenum channel, and the passage
param bossw = 24mm          // the inlet boss, across
param bossz = 20mm          // and deep, centred on the plate's mid-plane
param bossh = 98mm          // its top, above the crank axis
param Ty = 72mm             // the throttle barrel's centre, above the crank axis
param rbar = 5mm            // the barrel
param dhole = wch           // its cross-hole
param lev = 22mm            // the throttle lever
param tau = 35deg           // the lever's angle off full open; 90 is shut
param cpl = 0.625"          // the 1/4" NPT brass coupling: across flats, and its length
param cpll = 1 1/8"
param cplin = 18mm          // how deep it is set into the boss
param cplbore = 0.4375"     // its bore, near enough: the tap drill for 1/4" NPT
