// The engine's dimension table: one module every view reads (§14.4).
//
// An inline four: 80 bore, 90 stroke, 150 rod, cylinders on a 90 pitch, a pent-roof head with two
// overhead cams driven by a belt at half speed.  Each number is stated here once, with its unit,
// and `use engine.dims` puts the table in scope for whichever file draws from it — so a view and
// the parts drawn in it cannot disagree about a bore.

param D = 80mm          // bore
param R = 45mm          // crank throw: the stroke is 2R
param L = 150mm         // connecting rod, centre to centre
param P = 90mm          // bore pitch along the crank axis
param ch = 32mm         // compression height: piston pin to crown
param ph = 62mm         // piston height, crown to skirt

// -- the four-stroke cycle ----------------------------------------------------------------
// Cylinder 1's angle in its 720° cycle: 0 is top dead centre firing, so 0–180 is the power
// stroke, 180–360 exhaust, 360–540 intake, 540–720 compression.  The crank's *turn* — all the
// geometry sees — is the cycle angle modulo one revolution.  400 is forty degrees into the
// intake stroke: the intake valve opening on its lobe, the exhaust just shut.
param cycle = 400deg
param theta = cycle - 360deg * floor(cycle / 360deg)

// The valve timing, in crank degrees: the intake opens before top dead centre and closes after
// bottom, the exhaust opens before bottom and closes after top.  Everything about the cams —
// how far each nose stands out, where each lobe points at any moment, how far each valve is
// off its seat — follows from these four numbers and the cycle angle.
param ivo = 12deg
param ivc = 48deg
param evo = 48deg
param evc = 12deg
param idur = 180deg + ivo + ivc              // the intake valve is open this much of the crank's turn
param edur = 180deg + evo + evc
param icenter = 450deg + (ivc - ivo) / 2     // the intake lobe's centre, in the cycle
param ecenter = 270deg + (evc - evo) / 2

param deck = R + L + ch + 1mm      // crank axis to deck: the piston clears the deck by 1 at TDC
param rj = 25mm         // main journal
param rp = 20mm         // crank pin
param rbig = 30mm       // big end
param rsmall = 16mm     // small end
param rpin = 11mm       // piston pin
param pinlen = 28mm     // a crank pin's length along the axis: the rod's big end and its clearance

// the block: half-widths at the deck and at the pan rail, the rail and the sump below the axis
param hw = 75mm
param kw = 105mm
param rail = -40mm
param sump = -130mm
param wall = 170mm      // the cylinder wall runs this far below the deck
param front = -25mm     // the block's front face along the crank axis
param back = 4 * P + 25mm   // and its rear face
param bulk = 14mm       // a crankcase bulkhead's thickness, at each main bearing
param rmb = rj + 2mm    // the main bearing shell, outside: the bore in the bulkhead
param wmb = 22mm        // a main bearing's length along the axis
param capd = 14mm       // the bearing cap's depth below the shell

// the head: a separate casting, standing on its gasket
param gasket = 2mm      // the head gasket: the head's face stands this far off the deck
param head = 190mm      // deck to the top of the head
param rcamj = 13mm      // a camshaft journal
param wcamb = 16mm      // a cam bearing's length along the axis
param camcap = 4mm      // the bearing cap's wall round the journal

// the crank's rear end, past the block: a seal journal, the flange, the flywheel
param rseal = 22mm
param rflange = 60mm
param wflange = 12mm
param rfw = 140mm       // the flywheel
param wfw = 30mm

// the valvetrain: a tangent cam (base circle, nose circle, straight flanks) on a flat follower,
// valves inclined `va` either side of the bore axis in a pent roof
param rb = 17mm         // cam base circle
param rn = 7mm          // cam nose circle
// A tangent cam's duration is its geometry: the flat follower leaves the base circle where the
// nose circle's support equals the base's, `dn cos(β) + rn = rb`, a quarter of the duration
// either side of the nose (the cam turns at half speed).  So each lobe's nose distance is what
// its valve's duration asks for, and the lift is what that leaves: dn + rn - rb.
param dn_i = (rb - rn) / cos(idur / 4)
param dn_e = (rb - rn) / cos(edur / 4)
param lift_i = dn_i + rn - rb
param lift_e = dn_e + rn - rb
param va = 20deg        // valve inclination from the bore axis
param vs = 19mm         // valve seat centre off the bore axis, across the engine
param stem = 100mm      // seat to follower face
param div = 28mm        // intake head
param dev = 24mm        // exhaust head
param roof = deck + (D / 2) * tan(va)                             // the ridge of the pent roof
param camx = vs + (stem + rb) * sin(va)                            // cam centre off the bore axis
param camh = deck + (D / 2 - vs) * tan(va) + (stem + rb) * cos(va) // cam centre above the crank

// the timing drive: the cam pulleys are twice the crank's, since a cam turns at half speed
param rcp = 30mm
param rcam = 2 * rcp
