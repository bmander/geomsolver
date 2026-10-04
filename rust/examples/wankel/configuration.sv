// The engine's numbers, stated once: Mazda's 13B, a rotor 105 mm from its centre to each apex,
// riding a 15 mm eccentric, 80 mm wide (K = R / e = 7; 654 cm³ a chamber). Every other length
// is derived from these in `wankel.sv`, and no profile is typed in: the housing's bore is where
// the apex goes, and the rotor what the housing never touches.
unit mm

generating_radius := 105mm    // R: the rotor's centre to an apex
eccentricity := 15mm          // e: the shaft's crank, the rotor's centre from the shaft's
width := 80mm                 // W: the rotor's, and the housing's between its side plates

// The rotor one machines: its apexes and flanks this far in from the envelope's, so that it turns
// in the bore without touching it (`wankel.sv` claims as much over a whole turn).
clearance := 0.5mm
