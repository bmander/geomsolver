// The pair as a designer states it: the teeth, the shafts and the crown, then the allowances
// a shop needs. design.sv adds the proportions; machine settings belong outside this model.
pinion_teeth := 24
gear_teeth := 48
mean_module := 2mm
// The shafts: the angle between them, and the offset, the length of their common
// perpendicular. Zero offset is a bevel pair with a common apex; any other, a hypoid.
shaft_angle := 90deg
offset := 25mm
// Added to the crown tooth's inner flank pressure angle and taken from its outer flank's: an
// offset works a tooth's two sides at different effective pressure angles, and a hypoid
// balances them rather than accept undercut on one (docs/generating-sweeps-plan.md, Phase 1).
pressure_shift := 12.5deg
// The crown's spiral angle at the mean point. The pinion's adds the offset angle, which the
// layout solves for.
spiral_angle := 25deg
// The allowances, each zero for the conjugate, sharp-edged pair. The normal backlash: the
// flank clearance with the other flanks touching, a quarter of it off each crown flank
// (crown/section.sv). The tip relief: the tips' flank edges chamfered this far down the tooth
// (crown/relief.sv). The end relief: the tips' toe and heel edges chamfered this far each way
// (blank/ends.sv).
backlash := 0.05mm
tip_relief := 0.2mm
end_relief := 0.2mm
