// Reference-pair dimensions. Manufacturing choices belong outside this model.
param pinion_teeth = 24
param gear_teeth = 48
param mean_module = 2mm
// The pinion slides around the crown by this angle: its axis turns about the
// crown's normal at the mean point, so zero is a bevel pair with a common apex
// and anything else a hypoid, its axes about mean_distance * sin(offset_angle)
// apart.
param offset_angle = 6deg
