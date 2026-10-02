// The engine's dimension table, passed to components as `engine_dims`.
//
// An inline four: 80 bore, 90 stroke, 150 rod, cylinders on a 90 pitch, a pent-roof head with two
// overhead cams driven by a belt at half speed.  Each number is stated here once, with its unit,
// and `use engine.dims` exports it to root code, which supplies it explicitly to each part
// and view. Components read the values through their `dims` argument.

D := 80mm          // bore
R := 45mm          // crank throw: the stroke is 2R
L := 150mm         // connecting rod, centre to centre
P := 90mm          // bore pitch along the crank axis
ch := 32mm         // compression height: piston pin to crown
ph := 62mm         // piston height, crown to skirt

// -- the four-stroke cycle ----------------------------------------------------------------
// Cylinder 1's angle in its 720° cycle: 0 is top dead centre firing, so 0–180 is the power
// stroke, 180–360 exhaust, 360–540 intake, 540–720 compression.  The crank's *turn* — all the
// geometry sees — is the cycle angle modulo one revolution.  400 is forty degrees into the
// intake stroke: the intake valve opening on its lobe, the exhaust just shut.
cycle := 400deg
theta := cycle - 360deg * floor(cycle / 360deg)

// The valve timing, in crank degrees: the intake opens before top dead centre and closes after
// bottom, the exhaust opens before bottom and closes after top.  Everything about the cams —
// how far each nose stands out, where each lobe points at any moment, how far each valve is
// off its seat — follows from these four numbers and the cycle angle.
ivo := 12deg
ivc := 48deg
evo := 48deg
evc := 12deg
idur := 180deg + ivo + ivc              // the intake valve is open this much of the crank's turn
edur := 180deg + evo + evc
icenter := 450deg + (ivc - ivo) / 2     // the intake lobe's centre, in the cycle
ecenter := 270deg + (evc - evo) / 2

deck := R + L + ch + 1mm      // crank axis to deck: the piston clears the deck by 1 at TDC
rj := 25mm         // main journal
rp := 20mm         // crank pin
rbig := 30mm       // big end
rsmall := 16mm     // small end
rpin := 11mm       // piston pin
pinlen := 28mm     // a crank pin's length along the axis: the rod's big end and its clearance

// the block: half-widths at the deck and at the pan rail, the rail and the sump below the axis
hw := 75mm
kw := 105mm
rail := -40mm
sump := -130mm
wall := 170mm      // the cylinder wall runs this far below the deck
front := -25mm     // the block's front face along the crank axis
back := 4 * P + 25mm   // and its rear face
bulk := 14mm       // a crankcase bulkhead's thickness, at each main bearing
rmb := rj + 2mm    // the main bearing shell, outside: the bore in the bulkhead
wmb := 22mm        // a main bearing's length along the axis
capd := 14mm       // the bearing cap's depth below the shell

// the head: a separate casting, standing on its gasket
gasket := 2mm      // the head gasket: the head's face stands this far off the deck
head := 190mm      // deck to the top of the head
rcamj := 13mm      // a camshaft journal
wcamb := 16mm      // a cam bearing's length along the axis
camcap := 4mm      // the bearing cap's wall round the journal

// the crank's rear end, past the block: a seal journal, the flange, the flywheel
rseal := 22mm
rflange := 60mm
wflange := 12mm
rfw := 140mm       // the flywheel
wfw := 30mm

// the valvetrain: a tangent cam (base circle, nose circle, straight flanks) on a flat follower,
// valves inclined `va` either side of the bore axis in a pent roof
rb := 17mm         // cam base circle
rn := 7mm          // cam nose circle
// A tangent cam's duration is its geometry: the flat follower leaves the base circle where the
// nose circle's support equals the base's, `dn cos(β) + rn = rb`, a quarter of the duration
// either side of the nose (the cam turns at half speed).  So each lobe's nose distance is what
// its valve's duration asks for, and the lift is what that leaves: dn + rn - rb.
dn_i := (rb - rn) / cos(idur / 4)
dn_e := (rb - rn) / cos(edur / 4)
lift_i := dn_i + rn - rb
lift_e := dn_e + rn - rb
va := 20deg        // valve inclination from the bore axis
vs := 19mm         // valve seat centre off the bore axis, across the engine
stem := 100mm      // seat to follower face
div := 28mm        // intake head
dev := 24mm        // exhaust head
roof := deck + (D / 2) * tan(va)                             // the ridge of the pent roof
camx := vs + (stem + rb) * sin(va)                            // cam centre off the bore axis
camh := deck + (D / 2 - vs) * tan(va) + (stem + rb) * cos(va) // cam centre above the crank

// the timing drive: the cam pulleys are twice the crank's, since a cam turns at half speed
rcp := 30mm
rcam := 2 * rcp

// Explicit design inputs shared by the components.
engine_dims := group(
  D: D, L: L, P: P, R: R, back: back,
  bulk: bulk, camcap: camcap, camh: camh, camx: camx, capd: capd,
  ch: ch, cycle: cycle, deck: deck, dev: dev, div: div,
  dn_e: dn_e, dn_i: dn_i, ecenter: ecenter, edur: edur, evc: evc,
  evo: evo, front: front, gasket: gasket, head: head, hw: hw,
  icenter: icenter, idur: idur, ivc: ivc, ivo: ivo, kw: kw,
  lift_e: lift_e, lift_i: lift_i, ph: ph, pinlen: pinlen, rail: rail,
  rb: rb, rbig: rbig, rcam: rcam, rcamj: rcamj, rcp: rcp,
  rflange: rflange, rfw: rfw, rj: rj, rmb: rmb, rn: rn,
  roof: roof, rp: rp, rpin: rpin, rseal: rseal, rsmall: rsmall,
  stem: stem, sump: sump, theta: theta, va: va, vs: vs,
  wall: wall, wcamb: wcamb, wflange: wflange, wfw: wfw, wmb: wmb
)
