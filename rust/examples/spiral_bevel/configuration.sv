// Reference-pair dimensions. Manufacturing choices belong outside this model.
param pinion_teeth = 24
param gear_teeth = 48
param mean_module = 2mm
// The shafts: the angle between them, and the offset, the length of their common
// perpendicular. Zero offset is a bevel pair with a common apex; anything else a
// hypoid, whose pinion's pitch cone is solved against the gear's (pitch/pinion.sv).
param shaft_angle = 90deg
param offset = 25mm
// Added to the crown tooth's inner flank pressure angle and taken from its
// outer flank's. An offset works the two sides of a tooth at different
// effective pressure angles, and a hypoid balances them by this split rather
// than accept undercut on one side (docs/generating-sweeps-plan.md, Phase 1).
param pressure_shift = 12.5deg
// The crown's spiral angle at the mean point; the pinion's is this plus the
// offset angle, the angle the pinion's pitch generator turns from the gear's in
// the pitch plane, which the layout solves.
param spiral_angle = 30deg
