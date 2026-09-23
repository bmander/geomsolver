// Reference-pair dimensions. Manufacturing choices belong outside this model.
param pinion_teeth = 24
param gear_teeth = 48
param mean_module = 2mm
// The pinion slides around the crown by this angle: its axis turns about the
// crown's normal at the mean point, so zero is a bevel pair with a common apex
// and anything else a hypoid, its axes about mean_distance * sin(offset_angle)
// apart.
param offset_angle = 6deg
// Added to the crown tooth's inner flank pressure angle and taken from its
// outer flank's. An offset works the two sides of a tooth at different
// effective pressure angles, and a hypoid balances them by this split rather
// than accept undercut on one side (docs/generating-sweeps-plan.md, Phase 1).
param pressure_shift = 0deg
// The crown's spiral angle at the mean point; the pinion's is this plus the
// offset angle.
param spiral_angle = 35deg
