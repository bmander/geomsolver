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
param theta = 40deg     // crank angle of cylinder 1, after top dead centre

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
param head = 190mm      // deck to the top of the head

// the valvetrain: a tangent cam (base circle, nose circle, straight flanks) on a flat follower,
// valves inclined `va` either side of the bore axis in a pent roof
param rb = 17mm         // cam base circle
param rn = 7mm          // cam nose circle
param dn = 19mm         // nose centre from the cam centre: the lift is dn + rn - rb = 9
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
