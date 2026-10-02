// The pair's configuration: the teeth, the shafts and the crown, as a designer states
// them (design.sv adds the proportions), and the fabrication allowances a shop needs.
// Machine settings belong outside this model.
pinion_teeth := 24
gear_teeth := 48
mean_module := 2mm
// The shafts: the angle between them, and the offset, the length of their common
// perpendicular. Zero offset is a bevel pair with a common apex; anything else a
// hypoid, whose pinion's pitch cone is solved against the gear's (pitch/pinion.sv).
shaft_angle := 90deg
offset := 25mm
// Added to the crown tooth's inner flank pressure angle and taken from its
// outer flank's. An offset works the two sides of a tooth at different
// effective pressure angles, and a hypoid balances them by this split rather
// than accept undercut on one side (docs/generating-sweeps-plan.md, Phase 1).
pressure_shift := 12.5deg
// The crown's spiral angle at the mean point; the pinion's is this plus the
// offset angle, the angle the pinion's pitch generator turns from the gear's in
// the pitch plane, which the layout solves.
spiral_angle := 25deg
// The normal backlash: the clearance between the pair's flanks, measured along their
// normal with the other flanks touching. Each member's teeth are thinned by half of it, a
// quarter on each flank (crown/section.sv); zero is the conjugate pair.
backlash := 0.05mm
// The tip relief: every tooth's tip edges chamfered this far down the tooth by a
// semi-topping cut beside the generating crown (crown/relief.sv); zero leaves them sharp.
tip_relief := 0.2mm
// The end relief: the tips' toe and heel edges, where the tip cone meets the toe and heel
// spheres, chamfered this far along each (blank/ends.sv); zero leaves them sharp.
end_relief := 0.2mm
